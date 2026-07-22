from cereal import car, custom
from openpilot.common.parameterized import parameterized_class
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS, CRUISE_LONG_PRESS
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName


def _csp(decel_jump_fired: bool = False) -> custom.CarStateSP:
  msg = custom.CarStateSP.new_message()
  msg.decelJumpFired = decel_jump_fired
  return msg


class FakeParams:
  """In-memory Params stand-in so the DEC-param read/write stays isolated from the real
  store (and from other xdist workers) and is synchronous for deterministic assertions."""
  def __init__(self):
    self._store: dict[str, bool] = {}

  def get_bool(self, key: str) -> bool:
    return self._store.get(key, False)

  def put_bool(self, key: str, value: bool) -> None:
    self._store[key] = bool(value)

  put_bool_nonblocking = put_bool


@parameterized_class(('openpilot_longitudinal',), [(True,)])
class TestCruiseHelper:
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
      self.cruise_helper.update(CS, _csp(), self.events, False)

  def _hold(self, button) -> None:
    """Long-press and release a button once, advancing one step in the mode cycle."""
    experimental_mode = self.cruise_helper._experimental_mode
    for i in range(DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=button, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, _csp(), self.events, experimental_mode)
    # release to rearm for the next hold
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button, pressed=False)]
    self.cruise_helper.update(CS, _csp(), self.events, self.cruise_helper._experimental_mode)

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
      self.cruise_helper.update(CS, _csp(), self.events, self.cruise_helper._experimental_mode)

    # only the single acc -> experimental(DEC off) transition fired
    assert self.cruise_helper._experimental_mode is True
    assert self._dec is False

  def test_button_short_press_no_toggle(self) -> None:
    for experimental_mode in (True, False):
      self.setup_method()
      self.reset()
      self._set_state(experimental_mode, False)

      for i in range(DISTANCE_LONG_PRESS - 1):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=True)] if i == 0 else []
        self.cruise_helper.update(CS, _csp(), self.events, experimental_mode)

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

  def test_cruise_hold_chimes_on_each_step(self) -> None:
    """Holding inc/dec raises a directional chime event on every long-press step."""
    for button, event, other in ((ButtonType.accelCruise, EventNameSP.cruiseStepUp, EventNameSP.cruiseStepDown),
                                 (ButtonType.decelCruise, EventNameSP.cruiseStepDown, EventNameSP.cruiseStepUp)):
      self.setup_method()
      events_sp = EventsSP()
      for i in range(3 * CRUISE_LONG_PRESS):
        events_sp.clear()
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=button, pressed=True)] if i == 0 else []
        self.cruise_helper.update(CS, _csp(), events_sp, False)

        on_step = (i + 1) % CRUISE_LONG_PRESS == 0
        assert events_sp.has(event) == on_step
        assert events_sp.has(other) is False

  def test_cruise_short_tap_no_chime(self) -> None:
    """A short inc/dec tap (released before the long-press threshold) is silent."""
    for button in (ButtonType.accelCruise, ButtonType.decelCruise):
      self.setup_method()
      events_sp = EventsSP()
      for i in range(CRUISE_LONG_PRESS - 1):
        events_sp.clear()
        CS = car.CarState(cruiseState={"available": True})
        if i == 0:
          CS.buttonEvents = [ButtonEvent(type=button, pressed=True)]
        elif i == 5:
          CS.buttonEvents = [ButtonEvent(type=button, pressed=False)]
        else:
          CS.buttonEvents = []
        self.cruise_helper.update(CS, _csp(), events_sp, False)
        assert len(events_sp) == 0

  def test_decel_jump_fired_chimes(self) -> None:
    """When card signals decelJumpFired, CruiseHelper raises cruiseStepDown."""
    self.setup_method()
    events_sp = EventsSP()
    CS = car.CarState(cruiseState={"available": True})
    self.cruise_helper.update(CS, _csp(decel_jump_fired=True), events_sp, False)
    assert events_sp.has(EventNameSP.cruiseStepDown)
    assert not events_sp.has(EventNameSP.cruiseStepUp)

  def test_decel_jump_not_fired_no_chime(self) -> None:
    """No chime when decelJumpFired is unset."""
    self.setup_method()
    events_sp = EventsSP()
    CS = car.CarState(cruiseState={"available": True})
    self.cruise_helper.update(CS, _csp(decel_jump_fired=False), events_sp, False)
    assert len(events_sp) == 0
