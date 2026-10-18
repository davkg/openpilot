from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.parameterized import parameterized_class
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS
from openpilot.sunnypilot.selfdrive.controls.lib.follow_time import MIN_FOLLOW_TIME, MAX_FOLLOW_TIME, follow_time_bars, step_follow_time

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName


class FakeParams:
  """In-memory Params stand-in so the ExperimentalMode/LongFollowTime read/write stays isolated from the
  real store (and from other test workers) and is synchronous for deterministic assertions."""
  def __init__(self):
    self._store: dict[str, bool | float] = {}

  def get(self, key: str):
    return self._store.get(key)

  def put(self, key: str, value) -> None:
    self._store[key] = value

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
    """The distance button is free for follow time now; holding it must not change mode."""
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

  def _tap(self, button) -> EventsSP:
    events = EventsSP()
    for pressed in (True, False):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=button, pressed=pressed)]
      self.cruise_helper.update(CS, 0, events, False)
    return events

  def test_follow_time_taps(self) -> None:
    """Distance tap steps 0.1 s closer, LKAS tap 0.1 s farther, silently (no alert or chime)."""
    self.reset()
    self.cruise_helper.follow_time = 1.2

    events = self._tap(ButtonType.gapAdjustCruise)
    assert self.cruise_helper.follow_time == 1.1
    assert self.cruise_helper.params.get("LongFollowTime") == 1.1
    assert not events.events

    events = self._tap(ButtonType.lkas)
    events = self._tap(ButtonType.lkas)
    assert self.cruise_helper.follow_time == 1.3
    assert not events.events

  def test_follow_time_clamps(self) -> None:
    self.reset()
    for start, button in ((MIN_FOLLOW_TIME, ButtonType.gapAdjustCruise), (MAX_FOLLOW_TIME, ButtonType.lkas)):
      self.cruise_helper.follow_time = start
      self._tap(button)
      assert self.cruise_helper.follow_time == start

  def test_lkas_hold_does_not_step_follow_time(self) -> None:
    self.reset()
    self.cruise_helper.follow_time = 1.2
    self._hold(ButtonType.lkas)
    assert self.cruise_helper._experimental_mode is True
    assert self.cruise_helper.follow_time == 1.2

  def test_lkas_tap_ignored_with_mads(self) -> None:
    self.reset()
    self.cruise_helper.follow_time = 1.2
    self.cruise_helper._lkas_steps_follow_time = False
    self._tap(ButtonType.lkas)
    assert self.cruise_helper.follow_time == 1.2

  def test_follow_time_stays_on_grid(self) -> None:
    """Repeated float steps must not drift: 0.7 + 3 x 0.1 has to land exactly on the 1.0 bar threshold."""
    t = MIN_FOLLOW_TIME
    for _ in range(3):
      t = step_follow_time(t, 1)
    assert t == 1.0
    assert follow_time_bars(t) == 2

  def test_follow_time_bars(self) -> None:
    for t, bars in ((0.7, 1), (0.9, 1), (1.0, 2), (1.2, 2), (1.3, 3), (1.6, 3), (1.7, 4), (2.0, 4)):
      assert follow_time_bars(t) == bars, t
