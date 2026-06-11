"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import car, custom
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

DISTANCE_LONG_PRESS = 50
CRUISE_LONG_PRESS = 50  # frames; must match CRUISE_LONG_PRESS in selfdrive/car/cruise.py

# holding inc/dec -> directional chime raised on each set-speed step
CRUISE_STEP_EVENTS = {
  ButtonType.accelCruise: EventNameSP.cruiseStepUp,
  ButtonType.decelCruise: EventNameSP.cruiseStepDown,
}


def next_experimental_dec_state(experimental_mode: bool, dec: bool) -> tuple[bool, bool]:
  """Advance the 3-state mode cycle on a mode-button hold (LKAS/distance) or on-screen tap.

  The state is the (ExperimentalMode, DynamicExperimentalControl) param pair:
    acc (chill)            : (False, *)     -> next: experimental, DEC off
    experimental, DEC off  : (True, False)  -> next: experimental, DEC on
    experimental, DEC on   : (True, True)   -> next: acc (chill)
  Returns the (ExperimentalMode, DynamicExperimentalControl) values for the next state. Returning
  to acc also clears DEC so the settings toggle reflects the inactive state rather than showing on.
  """
  if not experimental_mode:
    return True, False
  if not dec:
    return True, True
  return False, False


class CruiseHelper:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0, ButtonType.lkas: 0,
                                ButtonType.accelCruise: 0, ButtonType.decelCruise: 0}
    self._experimental_mode = False
    self.experimental_mode_switched = False

  def update(self, CS, CS_SP, events, experimental_mode) -> None:
    if self.CP.openpilotLongitudinalControl:
      if CS.cruiseState.available:
        self.update_button_frame_counts(CS)

        # toggle experimental mode once on distance button hold or on LKAS press
        self.update_experimental_mode(CS, events, experimental_mode)

        # audible feedback on each set-speed step while holding inc/dec, plus
        # on a DEC tap that triggers the (vEgo+10) jump (signaled via CS_SP)
        self.update_cruise_step_chime(CS_SP, events)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_experimental_mode(self, CS, events, experimental_mode) -> None:
    lkas_long_pressed = self.button_frame_counts[ButtonType.lkas] >= DISTANCE_LONG_PRESS
    gap_adjust_long_pressed = self.button_frame_counts[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS

    if (lkas_long_pressed or gap_adjust_long_pressed) and not self.experimental_mode_switched:
      dec = self.params.get_bool("DynamicExperimentalControl")
      self._experimental_mode, new_dec = next_experimental_dec_state(experimental_mode, dec)
      self.params.put_bool_nonblocking("ExperimentalMode", self._experimental_mode)
      self.params.put_bool_nonblocking("DynamicExperimentalControl", new_dec)
      events.add(EventNameSP.experimentalModeSwitched)
      self.experimental_mode_switched = True

    if any(be.type == ButtonType.lkas and not be.pressed for be in CS.buttonEvents):
      self.experimental_mode_switched = False
    if any(be.type == ButtonType.gapAdjustCruise and not be.pressed for be in CS.buttonEvents):
      self.experimental_mode_switched = False

  def update_cruise_step_chime(self, CS_SP, events) -> None:
    # holding inc/dec steps the set speed every CRUISE_LONG_PRESS frames (see
    # VCruiseHelper._update_v_cruise_non_pcm); chime on each step so the hold can
    # be counted by ear instead of watched. ET.WARNING gates this to when openpilot
    # is engaged, which is the only time the set speed actually moves.
    for button, event in CRUISE_STEP_EVENTS.items():
      count = self.button_frame_counts[button]
      if count > 0 and count % CRUISE_LONG_PRESS == 0:
        events.add(event)

    # DEC tap that triggered the (vEgo+10) jump-down: card sets decelJumpFired on
    # the frame the jump applies; chime so the jump can be heard rather than read.
    if CS_SP.decelJumpFired:
      events.add(EventNameSP.cruiseStepDown)
