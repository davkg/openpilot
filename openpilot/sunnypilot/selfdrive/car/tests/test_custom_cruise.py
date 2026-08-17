from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized, parameterized_class
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import V_CRUISE_INITIAL
from openpilot.selfdrive.car.tests.test_cruise_speed import TestVCruiseHelper

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


# TODO: test pcmCruise and pcmCruiseSpeed
@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(False, True)])
class TestCustomAccIncrements(TestVCruiseHelper):
  def setup_method(self):
    TestVCruiseHelper.openpilot_setup_method(self)
    self.params = Params()
    self.reset_custom_params()

  def reset_custom_params(self) -> None:
    """Reset to default custom ACC parameters"""
    self.params.put_bool("CustomAccIncrementsEnabled", False, block=True)
    self.params.put("CustomAccShortPressIncrement", 1, block=True)
    self.params.put("CustomAccLongPressIncrement", 5, block=True)
    self.v_cruise_helper.read_custom_set_speed_params()

  def press_button_short(self, button_type: car.CarState.ButtonEvent.Type, v_ego: float = 0.0) -> None:
    """Simulate a short button press (press + release)"""
    CS = car.CarState(vEgo=v_ego, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

  def press_button_long(self, button_type: car.CarState.ButtonEvent.Type, v_ego: float = 0.0) -> None:
    """Simulate a long button press (50+ frames)"""
    CS = car.CarState(vEgo=v_ego, cruiseState={"available": True})
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
    self.params.put_bool("CustomAccIncrementsEnabled", enabled, block=True)
    self.params.put("CustomAccShortPressIncrement", short_inc, block=True)
    self.params.put("CustomAccLongPressIncrement", long_inc, block=True)
    self.v_cruise_helper.read_custom_set_speed_params()

  def test_default_behavior_when_disabled(self):
    """Test that default increments are used when custom ACC is disabled"""
    self.set_custom_increments(enabled=False, short_inc=5, long_inc=10)
    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph

    # Short press should increment by 1 (default)
    self.press_button_short(ButtonType.accelCruise)
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + 1

  @parameterized.expand((1, 2, 3, 4, 5, 6, 7, 8, 9, 10))
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

  @parameterized.expand((1, 5, 10))
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

  @parameterized.expand([ButtonType.accelCruise, ButtonType.decelCruise])
  def test_accel_decel_symmetry(self, button_type):
    """Test that acceleration and deceleration work symmetrically"""
    self.set_custom_increments(enabled=True, short_inc=3, long_inc=5)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    # ego near the set speed so the decel-tap jump doesn't apply and we test the increment itself
    self.press_button_short(button_type, v_ego=50 * CV.KPH_TO_MS)

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
class TestDecelJump(TestVCruiseHelper):
  """A decel tap jumps the set speed down to (current speed + 10) instead of stepping."""

  def setup_method(self):
    TestVCruiseHelper.openpilot_setup_method(self)

  @staticmethod
  def _to_ms(display, is_metric):
    return display * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)

  def _tap(self, button_type, v_ego, is_metric):
    CS = car.CarState(vEgo=v_ego, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=is_metric)
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=is_metric)
    return self.v_cruise_helper.v_cruise_kph

  def test_first_decel_tap_jumps_to_ego_plus_10_imperial(self):
    # ego 31 mph -> +10 = 41 -> rounds to 40 mph
    self.enable(self._to_ms(70, False), False, False)
    result = self._tap(ButtonType.decelCruise, self._to_ms(31, False), False)
    self.assertAlmostEqual(result, 40.0 * round(CV.MPH_TO_KPH, 1), places=1)

  def test_first_decel_tap_jumps_to_ego_plus_10_metric(self):
    # ego 50 kph -> +10 = 60 -> already a multiple of 5
    self.enable(self._to_ms(110, True), False, False)
    result = self._tap(ButtonType.decelCruise, self._to_ms(50, True), True)
    self.assertAlmostEqual(result, 60.0, places=1)

  def test_decel_jump_floored(self):
    """The jump never lowers the set speed below the floor."""
    self.enable(self._to_ms(50, False), False, False)
    result = self._tap(ButtonType.decelCruise, self._to_ms(5, False), False)
    self.assertAlmostEqual(result, 25.0 * round(CV.MPH_TO_KPH, 1), places=1)

  def test_decel_tap_never_raises_set_speed(self):
    """When ego+10 isn't below the set speed, fall back to the normal step down."""
    for is_metric in (True, False):
      TestVCruiseHelper.openpilot_setup_method(self)
      self.enable(self._to_ms(40, is_metric), False, False)
      initial = self.v_cruise_helper.v_cruise_kph
      result = self._tap(ButtonType.decelCruise, self._to_ms(60, is_metric), is_metric)
      self.assertLess(result, initial)

  def test_accel_tap_unchanged(self):
    """Accel tap is unaffected by the decel jump."""
    self.enable(self._to_ms(50, True), False, False)
    initial = self.v_cruise_helper.v_cruise_kph
    result = self._tap(ButtonType.accelCruise, self._to_ms(31, True), True)
    self.assertGreater(result, initial)
    self.assertFalse(self.v_cruise_helper.decel_jump_fired)

  def test_decel_jump_fired_flag(self):
    """decel_jump_fired is True only on the frame the jump applies, then resets."""
    self.enable(self._to_ms(70, True), False, False)
    CS = car.CarState(vEgo=self._to_ms(31, True), cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    self.assertFalse(self.v_cruise_helper.decel_jump_fired)  # press, no jump yet
    CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    self.assertTrue(self.v_cruise_helper.decel_jump_fired)   # release: jump fires
    CS.buttonEvents = []
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    self.assertFalse(self.v_cruise_helper.decel_jump_fired)  # one-shot

  def test_decel_hold_does_not_jump(self):
    """A decel hold steps normally and never triggers the jump."""
    self.enable(70 * CV.KPH_TO_MS, False, False)
    initial = self.v_cruise_helper.v_cruise_kph
    CS = car.CarState(vEgo=31 * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    self.assertFalse(self.v_cruise_helper.decel_jump_fired)
    CS.buttonEvents = []
    for _ in range(50):  # one long-press tick
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
      self.assertFalse(self.v_cruise_helper.decel_jump_fired)
    self.assertEqual(self.v_cruise_helper.v_cruise_kph, initial - 5)
