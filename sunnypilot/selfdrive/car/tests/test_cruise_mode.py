from cereal import car, custom
from openpilot.common.parameterized import parameterized_class
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS, CRUISE_LONG_PRESS
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName


@parameterized_class(('openpilot_longitudinal',), [(True,)])
class TestCruiseHelper:
  def setup_method(self):
    self.CP = car.CarParams(openpilotLongitudinalControl=self.openpilot_longitudinal)
    self.cruise_helper = CruiseHelper(self.CP)
    self.cruise_helper.experimental_mode_switched = False
    self.events = Events()

  def reset(self):
    for _ in range(2):
      CS = car.CarState(cruiseState={"available": False})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=False)]
      self.cruise_helper._experimental_mode = False
      self.cruise_helper.experimental_mode_switched = False
      self.cruise_helper.update(CS, self.events, False)


  def test_gap_adjust_cruise_long_press_toggle_mode(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode
        toggled_mode = not experimental_mode if pressed else experimental_mode

        for i in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        # mode should be toggled
        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

        # keep holding button after switching mode
        for _ in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)]
          self.cruise_helper.update(CS, self.events, toggled_mode)

        # mode should not be toggled
        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

  def test_gap_adjust_cruise_short_press_toggle_mode(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode

        for i in range(DISTANCE_LONG_PRESS - 1):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        # mode should not be toggled
        assert self.cruise_helper._experimental_mode == experimental_mode
        assert self.cruise_helper.experimental_mode_switched is False

  def test_lkas_long_press_toggle_mode(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode
        toggled_mode = not experimental_mode if pressed else experimental_mode

        for i in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

        # keep holding button after switching mode — should not toggle again
        for _ in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=pressed)]
          self.cruise_helper.update(CS, self.events, toggled_mode)

        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

  def test_lkas_short_press_no_toggle(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode

        for i in range(DISTANCE_LONG_PRESS - 1):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        assert self.cruise_helper._experimental_mode == experimental_mode
        assert self.cruise_helper.experimental_mode_switched is False

  def test_lkas_release_allows_retoggle(self) -> None:
    self.reset()
    self.cruise_helper._experimental_mode = False

    for i in range(DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, self.events, False)
    assert self.cruise_helper._experimental_mode is True
    assert self.cruise_helper.experimental_mode_switched is True

    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=False)]
    self.cruise_helper.update(CS, self.events, True)
    assert self.cruise_helper.experimental_mode_switched is False

    for i in range(DISTANCE_LONG_PRESS):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.lkas, pressed=True)] if i == 0 else []
      self.cruise_helper.update(CS, self.events, True)
    assert self.cruise_helper._experimental_mode is False
    assert self.cruise_helper.experimental_mode_switched is True

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
        self.cruise_helper.update(CS, events_sp, False)

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
        self.cruise_helper.update(CS, events_sp, False)
        assert len(events_sp) == 0
