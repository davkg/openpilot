import pytest

from cereal import car
from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized_class
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import V_CRUISE_INITIAL
from openpilot.selfdrive.car.tests.test_cruise_speed import TestVCruiseHelper

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


# TODO: test pcmCruise and pcmCruiseSpeed
@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(False, True)])
class TestCustomAccIncrements(TestVCruiseHelper):
  def setup_method(self):
    TestVCruiseHelper.setup_method(self)
    self.params = Params()
    self.reset_custom_params()

  def reset_custom_params(self) -> None:
    """Reset to default custom ACC parameters"""
    self.params.put_bool("CustomAccIncrementsEnabled", False)
    self.params.put("CustomAccShortPressIncrement", 1)
    self.params.put("CustomAccLongPressIncrement", 5)
    self.v_cruise_helper.read_custom_set_speed_params()

  def press_button_short(self, button_type: car.CarState.ButtonEvent.Type) -> None:
    """Simulate a short button press (press + release)"""
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

  def press_button_long(self, button_type: car.CarState.ButtonEvent.Type) -> None:
    """Simulate a long button press (50+ frames)"""
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    # Hold for 50 frames to trigger long press
    CS.buttonEvents = []
    for _ in range(50):
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

  def set_custom_increments(self, enabled: bool, short_inc: int, long_inc: int) -> None:
    """Set custom ACC increment parameters"""
    self.params.put_bool("CustomAccIncrementsEnabled", enabled)
    self.params.put("CustomAccShortPressIncrement", short_inc)
    self.params.put("CustomAccLongPressIncrement", long_inc)
    self.v_cruise_helper.read_custom_set_speed_params()

  def test_default_behavior_when_disabled(self):
    """Test that default increments are used when custom ACC is disabled"""
    self.set_custom_increments(enabled=False, short_inc=5, long_inc=10)
    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph

    # Short press should increment by 1 (default)
    self.press_button_short(ButtonType.accelCruise)
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + 1

  @pytest.mark.parametrize("increment", (1, 2, 3, 4, 5, 6, 7, 8, 9, 10))
  def test_custom_short_press_increments(self, increment):
    """Test custom short press increments (1-10)"""
    self.set_custom_increments(enabled=True, short_inc=increment, long_inc=5)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_short(ButtonType.accelCruise)

    if increment in (5, 10):
      # Should round to nearest increment
      expected_speed = ((initial_speed // increment) + 1) * increment
    else:
      expected_speed = initial_speed + increment

    assert self.v_cruise_helper.v_cruise_kph == expected_speed

  @pytest.mark.parametrize("increment", (1, 5, 10))
  def test_custom_long_press_increments(self, increment):
    """Test custom long press increments (1, 5, 10)"""
    self.set_custom_increments(enabled=True, short_inc=1, long_inc=increment)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_long(ButtonType.accelCruise)

    if increment in (5, 10):
      # Should round to nearest increment
      expected_speed = ((initial_speed // increment) + 1) * increment
    else:
      expected_speed = initial_speed + increment

    assert self.v_cruise_helper.v_cruise_kph == expected_speed

  @pytest.mark.parametrize("button_type", [ButtonType.accelCruise, ButtonType.decelCruise])
  def test_accel_decel_symmetry(self, button_type):
    """Test that acceleration and deceleration work symmetrically"""
    self.set_custom_increments(enabled=True, short_inc=3, long_inc=5)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    # vEgo matches set speed so the decel-jump target (ego+10) is above set speed and won't fire
    CS = car.CarState(vEgo=50 * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    expected_change = 3 if button_type == ButtonType.accelCruise else -3
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + expected_change

  def test_rounding_behavior(self):
    """Test rounding behavior for 5 and 10 increments"""
    test_cases = [
      (47, 5, 50),   # 47 -> 50 (round up to next 5)
      (45, 5, 50),   # 45 -> 50 (already at 5, increment by 5)
      (43, 10, 50),  # 43 -> 50 (round up to next 10)
      (40, 10, 50),  # 40 -> 50 (already at 10, increment by 10)
    ]

    for initial, increment, expected in test_cases:
      self.set_custom_increments(enabled=True, short_inc=increment, long_inc=increment)
      self.reset_cruise_speed_state()
      self.enable(initial * CV.KPH_TO_MS, False, False)

      self.press_button_short(ButtonType.accelCruise)
      assert self.v_cruise_helper.v_cruise_kph == expected

  def test_invalid_values_fallback(self):
    """Test that invalid values fallback to safe defaults"""
    # Test invalid short increment
    self.set_custom_increments(enabled=True, short_inc=-1, long_inc=5)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_short(ButtonType.accelCruise)
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + 1  # Should fallback to 1

    # Test invalid long increment
    self.reset_cruise_speed_state()
    self.set_custom_increments(enabled=True, short_inc=1, long_inc=99)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_long(ButtonType.accelCruise)
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + 10  # Should fallback to 10


@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(False, True)])
class TestDecelTapJump(TestVCruiseHelper):
  """A decel tap jumps the set speed to (current speed + 10) rounded to the nearest 5
  (display units), floored at DECEL_JUMP_MIN_SPEED, when the result would be lower than
  the current set speed. Holds step by 5 per tick and never jump."""

  def setup_method(self):
    TestVCruiseHelper.setup_method(self)
    # ensure default increments regardless of test order
    params = Params()
    params.put_bool("CustomAccIncrementsEnabled", False)
    params.put("CustomAccShortPressIncrement", 1)
    params.put("CustomAccLongPressIncrement", 5)
    self.v_cruise_helper.read_custom_set_speed_params()

  @staticmethod
  def _to_ms(speed_disp: float, is_metric: bool) -> float:
    return speed_disp * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)

  def _tap(self, button_type: car.CarState.ButtonEvent.Type, v_ego: float, is_metric: bool) -> float:
    """Single press+release tap; return the resulting set speed."""
    CS = car.CarState(vEgo=v_ego, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=is_metric)
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=is_metric)
    return self.v_cruise_helper.v_cruise_kph

  @pytest.mark.parametrize("is_metric, expected_kph", [
    (True, [40, 39, 38]),           # jumps to 40 kph, then steps -1 kph per tap
    (False, [64.0, 62.4, 60.8]),    # jumps to 40 mph, then steps -1 mph per tap
  ])
  def test_first_decel_tap_jumps_to_ego_plus_10(self, is_metric, expected_kph):
    """ego ~31, set ~70: first decel tap jumps to ego+10 rounded, then steps down by 1."""
    self.enable(self._to_ms(70, is_metric), False, False)
    speeds = [self._tap(ButtonType.decelCruise, self._to_ms(31, is_metric), is_metric) for _ in range(3)]
    assert speeds == pytest.approx(expected_kph)

  @pytest.mark.parametrize("v_ego_mph, expected_kph", [
    (0, 40.0),    # naive target 10 -> floored to 25 mph
    (12, 40.0),   # naive target 20 -> floored to 25 mph
    (17, 40.0),   # naive target 25 -> already at the floor
    (20, 48.0),   # naive target 30 -> above the floor, unchanged
  ])
  def test_decel_jump_floored_at_25_mph(self, v_ego_mph, expected_kph):
    """The decel tap jump never lowers the set speed below 25 mph."""
    self.enable(self._to_ms(50, False), False, False)
    result = self._tap(ButtonType.decelCruise, self._to_ms(v_ego_mph, False), False)
    assert result == pytest.approx(expected_kph)

  @pytest.mark.parametrize("is_metric", [True, False])
  def test_decel_tap_never_raises_set_speed(self, is_metric):
    """When ego+10 isn't below the set speed, the decel tap falls back to the
    normal step instead of jumping the set speed up."""
    self.enable(self._to_ms(40, is_metric), False, False)
    initial = self.v_cruise_helper.v_cruise_kph
    result = self._tap(ButtonType.decelCruise, self._to_ms(60, is_metric), is_metric)
    assert result < initial

  @pytest.mark.parametrize("is_metric", [True, False])
  def test_accel_tap_unchanged(self, is_metric):
    """Accel tap is unaffected by the decel jump."""
    self.enable(self._to_ms(50, is_metric), False, False)
    initial = self.v_cruise_helper.v_cruise_kph
    result = self._tap(ButtonType.accelCruise, self._to_ms(31, is_metric), is_metric)
    assert result > initial

  def test_decel_hold_does_not_jump(self):
    """A decel hold steps by 5 per long-press tick and never triggers the jump."""
    self.enable(70 * CV.KPH_TO_MS, False, False)
    initial = self.v_cruise_helper.v_cruise_kph
    CS = car.CarState(vEgo=31 * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    CS.buttonEvents = []
    for _ in range(50):  # one long-press tick
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    assert self.v_cruise_helper.v_cruise_kph == initial - 5
