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
  """In-memory Params stand-in so the LongFollowTime read/write stays isolated from the
  real store (and from other test workers) and is synchronous for deterministic assertions."""
  def __init__(self):
    self._store: dict[str, bool | float] = {}

  def get(self, key: str):
    return self._store.get(key)

  def put(self, key: str, value) -> None:
    self._store[key] = value

  def get_bool(self, key: str) -> bool:
    return bool(self._store.get(key, False))

  def put_bool(self, key: str, value: bool) -> None:
    self._store[key] = bool(value)

  put_bool_nonblocking = put_bool


@parameterized_class(('openpilot_longitudinal',), [(True,)])
class TestCruiseHelper(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(openpilotLongitudinalControl=self.openpilot_longitudinal)
    self.cruise_helper = CruiseHelper(self.CP)
    self.cruise_helper.params = FakeParams()  # ty: ignore[invalid-assignment]
    self.events = Events()

  def reset(self):
    for _ in range(2):
      CS = car.CarState(cruiseState={"available": False})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=False)]
      self.cruise_helper.update(CS, 0, self.events)

  def _hold(self, button) -> None:
    """Long-press and release a button once."""
    for i in range(DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=button, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, 0, self.events)
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button, pressed=False)]
    self.cruise_helper.update(CS, 0, self.events)


  def test_set_speed_step_chimes(self) -> None:
    """card's set-speed step maps to the directional chime; no step, no chime."""
    for step, expected in ((1, {EventNameSP.cruiseStepUp}), (-1, {EventNameSP.cruiseStepDown}), (0, set())):
      self.reset()
      CS = car.CarState(cruiseState={"available": True})
      self.events = EventsSP()
      self.cruise_helper.update(CS, step, self.events)
      assert set(self.events.events) & {EventNameSP.cruiseStepUp, EventNameSP.cruiseStepDown} == expected

  def test_set_speed_step_no_chime_when_cruise_unavailable(self) -> None:
    self.reset()
    CS = car.CarState(cruiseState={"available": False})
    self.events = EventsSP()
    self.cruise_helper.update(CS, 1, self.events)
    assert EventNameSP.cruiseStepUp not in self.events.events

  def _tap(self, button) -> EventsSP:
    events = EventsSP()
    for pressed in (True, False):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=button, pressed=pressed)]
      self.cruise_helper.update(CS, 0, events)
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
