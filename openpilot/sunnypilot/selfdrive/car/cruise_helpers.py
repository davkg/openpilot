"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.follow_time import read_follow_time, step_follow_time

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

DISTANCE_LONG_PRESS = 50


class CruiseHelper:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()

    self.button_frame_counts = {ButtonType.lkas: 0}
    self.follow_time = read_follow_time(self.params)
    self._lkas_steps_follow_time = not self.params.get_bool("Mads")  # MADS owns the LKAS tap when enabled

  def update(self, CS, set_speed_step, events) -> None:
    if self.CP.openpilotLongitudinalControl:
      if CS.cruiseState.available:
        # before the frame counts update, which clears the LKAS hold length on release
        self.update_follow_time(CS)
        self.update_button_frame_counts(CS)

        # audible feedback on each long-press set-speed step and on the decel-tap jump
        self.update_cruise_step_chime(set_speed_step, events)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_follow_time(self, CS) -> None:
    # distance tap steps closer, LKAS tap steps farther; an LKAS hold does nothing.
    # deliberately silent: no alert or chime, the value is logged via selfdriveStateSP.followTime
    direction = 0
    for be in CS.buttonEvents:
      if be.pressed:
        continue
      if be.type == ButtonType.gapAdjustCruise:
        direction -= 1
      elif be.type == ButtonType.lkas and self._lkas_steps_follow_time and self.button_frame_counts[ButtonType.lkas] < DISTANCE_LONG_PRESS:
        direction += 1

    follow_time = step_follow_time(self.follow_time, direction)
    if follow_time != self.follow_time:
      self.follow_time = follow_time
      self.params.put("LongFollowTime", follow_time)

  def update_cruise_step_chime(self, set_speed_step, events) -> None:
    # card decides when the set speed stepped (VCruiseHelper.set_speed_step, via CarStateSP)
    if set_speed_step > 0:
      events.add(EventNameSP.cruiseStepUp)
    elif set_speed_step < 0:
      events.add(EventNameSP.cruiseStepDown)
