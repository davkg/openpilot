from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.parameterized import parameterized_class
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName


class FakeParams:
  """In-memory Params stand-in so the ExperimentalMode read/write stays isolated from the real
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
    self.cruise_helper.params = FakeParams()  # ty: ignore[invalid-assignment]
    self.cruise_helper.experimental_mode_switched = False
    self.events = Events()

  def _set_state(self, experimental_mode: bool) -> None:
    self.cruise_helper._experimental_mode = experimental_mode
    self.cruise_helper.params.put_bool("ExperimentalMode", experimental_mode)

  def reset(self):
    for _ in range(2):
      CS = car.CarState(cruiseState={"available": False})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=False)]
      self._set_state(False)
      self.cruise_helper.experimental_mode_switched = False
      self.cruise_helper.update(CS, 0, self.events, False)

  def _hold(self, button) -> None:
    """Long-press and release a button once, toggling experimental mode."""
    experimental_mode = self.cruise_helper._experimental_mode
    for i in range(DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=button, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, 0, self.events, experimental_mode)
    # release to rearm for the next hold
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button, pressed=False)]
    self.cruise_helper.update(CS, 0, self.events, self.cruise_helper._experimental_mode)


  def test_button_hold_toggles(self) -> None:
    """Each hold flips acc <-> experimental."""
    self.reset()

    for expected in (True, False) * 2:
      self._hold(ButtonType.lkas)
      assert self.cruise_helper._experimental_mode is expected
      assert self.cruise_helper.params.get_bool("ExperimentalMode") is expected

  def test_button_hold_continues_no_extra_toggle(self) -> None:
    """Continuing to hold past the first switch must not toggle again."""
    self.reset()

    for i in range(3 * DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, 0, self.events, self.cruise_helper._experimental_mode)

    # only the single acc -> experimental transition fired
    assert self.cruise_helper._experimental_mode is True

  def test_button_short_press_no_toggle(self) -> None:
    for experimental_mode in (True, False):
      self.cruise_helper = CruiseHelper(self.CP)
      self.cruise_helper.params = FakeParams()  # ty: ignore[invalid-assignment]
      self.reset()
      self._set_state(experimental_mode)

      for i in range(DISTANCE_LONG_PRESS - 1):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=True)] if i == 0 else []
        self.cruise_helper.update(CS, 0, self.events, experimental_mode)

      assert self.cruise_helper._experimental_mode == experimental_mode
      assert self.cruise_helper.experimental_mode_switched is False

  def test_release_allows_retoggle(self) -> None:
    """Release rearms the debounce so the next hold toggles again."""
    self.reset()

    self._hold(ButtonType.lkas)
    assert self.cruise_helper._experimental_mode is True
    assert self.cruise_helper.experimental_mode_switched is False  # cleared on release

    self._hold(ButtonType.lkas)
    assert self.cruise_helper._experimental_mode is False

  def test_gap_adjust_no_longer_toggles_mode(self) -> None:
    """The distance button is free for personality now; holding it must not change mode."""
    self.reset()

    for i in range(3 * DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, 0, self.events, False)

    assert self.cruise_helper._experimental_mode is False
    assert self.cruise_helper.experimental_mode_switched is False

  def test_set_speed_step_chimes(self) -> None:
    """card's set-speed step maps to the directional chime; no step, no chime."""
    for step, expected in ((1, {EventNameSP.cruiseStepUp}), (-1, {EventNameSP.cruiseStepDown}), (0, set())):
      self.reset()
      CS = car.CarState(cruiseState={"available": True})
      self.events = EventsSP()
      self.cruise_helper.update(CS, step, self.events, False)
      assert set(self.events.events) & {EventNameSP.cruiseStepUp, EventNameSP.cruiseStepDown} == expected

  def test_set_speed_step_no_chime_when_cruise_unavailable(self) -> None:
    self.reset()
    CS = car.CarState(cruiseState={"available": False})
    self.events = EventsSP()
    self.cruise_helper.update(CS, 1, self.events, False)
    assert EventNameSP.cruiseStepUp not in self.events.events
