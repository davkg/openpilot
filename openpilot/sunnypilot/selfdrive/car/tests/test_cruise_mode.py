from opendbc.car.structs import car
from openpilot.common.parameterized import parameterized_class
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


class FakeParams:
  """In-memory Params stand-in so the DEC-param read/write stays isolated from the real
  store (and from other test workers) and is synchronous for deterministic assertions."""
  def __init__(self):
    self._store: dict[str, bool] = {}

  def get_bool(self, key: str) -> bool:
    return self._store.get(key, False)

  def put_bool(self, key: str, value: bool) -> None:
    self._store[key] = bool(value)

  put_bool_nonblocking = put_bool


@parameterized_class(('openpilot_longitudinal',), [(True,)])
class TestCruiseHelper(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(openpilotLongitudinalControl=self.openpilot_longitudinal)
    self.cruise_helper = CruiseHelper(self.CP)
    self.cruise_helper.params = FakeParams()
    self.cruise_helper.experimental_mode_switched = False
    self.events = Events()

  @property
  def _dec(self) -> bool:
    return self.cruise_helper.params.get_bool("DynamicExperimentalControl")

  def _set_state(self, experimental_mode: bool, dec: bool) -> None:
    self.cruise_helper._experimental_mode = experimental_mode
    self.cruise_helper.params.put_bool("ExperimentalMode", experimental_mode)
    self.cruise_helper.params.put_bool("DynamicExperimentalControl", dec)

  def reset(self):
    for _ in range(2):
      CS = car.CarState(cruiseState={"available": False})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=False)]
      self._set_state(False, False)
      self.cruise_helper.experimental_mode_switched = False
      self.cruise_helper.update(CS, self.events, False)

  def _hold(self, button) -> None:
    """Long-press and release a button once, advancing one step in the mode cycle."""
    experimental_mode = self.cruise_helper._experimental_mode
    for i in range(DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=button, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, self.events, experimental_mode)
    # release to rearm for the next hold
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button, pressed=False)]
    self.cruise_helper.update(CS, self.events, self.cruise_helper._experimental_mode)


  def test_button_hold_cycles_three_states(self) -> None:
    """Each hold advances acc -> experimental(DEC off) -> experimental(DEC on) -> acc, looping."""
    self.reset()

    # (ExperimentalMode, DynamicExperimentalControl) after each successive hold, twice round
    expected = [(True, False), (True, True), (False, False)] * 2
    for exp_em, exp_dec in expected:
      self._hold(ButtonType.lkas)
      assert self.cruise_helper._experimental_mode is exp_em
      assert self._dec is exp_dec

  def test_button_hold_continues_no_extra_toggle(self) -> None:
    """Continuing to hold past the first switch must not advance the cycle again."""
    self.reset()

    for i in range(3 * DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, self.events, self.cruise_helper._experimental_mode)

    # only the single acc -> experimental(DEC off) transition fired
    assert self.cruise_helper._experimental_mode is True
    assert self._dec is False

  def test_button_short_press_no_toggle(self) -> None:
    for experimental_mode in (True, False):
      self.cruise_helper = CruiseHelper(self.CP)
      self.cruise_helper.params = FakeParams()
      self.reset()
      self._set_state(experimental_mode, False)

      for i in range(DISTANCE_LONG_PRESS - 1):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=True)] if i == 0 else []
        self.cruise_helper.update(CS, self.events, experimental_mode)

      assert self.cruise_helper._experimental_mode == experimental_mode
      assert self._dec is False
      assert self.cruise_helper.experimental_mode_switched is False

  def test_release_allows_retoggle(self) -> None:
    """Release rearms the debounce so the next hold advances the cycle."""
    self.reset()

    self._hold(ButtonType.lkas)
    assert self.cruise_helper._experimental_mode is True
    assert self._dec is False
    assert self.cruise_helper.experimental_mode_switched is False  # cleared on release

    self._hold(ButtonType.lkas)
    assert self.cruise_helper._experimental_mode is True
    assert self._dec is True

  def test_gap_adjust_no_longer_toggles_mode(self) -> None:
    """The distance button is free for personality now; holding it must not change mode."""
    self.reset()

    for i in range(3 * DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, self.events, False)

    assert self.cruise_helper._experimental_mode is False
    assert self._dec is False
    assert self.cruise_helper.experimental_mode_switched is False
