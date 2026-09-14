"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath
from openpilot.sunnypilot.selfdrive.controls.lib.lane_centering import get_lane_centering_visual_direction

LANE_LINE_COLORS_SP = {
  UIStatus.LAT_ONLY: rl.Color(0, 255, 64, 255),
  UIStatus.LONG_ONLY: rl.Color(0, 255, 64, 255),
}

# Lane centering: the ego lane line we are being biased toward is tinted this color
LANE_CENTERING_COLOR = rl.Color(65, 145, 255, 255)


class ModelRendererSP:
  def __init__(self):
    self.rainbow_path = RainbowPath()

    self._lane_centering_enabled = False
    self._lane_center_offset = 0.0
    self._lane_centering_e2e_authority = 1.0
    self._lane_centering_pause_on_signal = True

  def update_lane_centering_params(self) -> None:
    params = ui_state.params
    self._lane_centering_enabled = params.get_bool("LaneCentering")
    self._lane_center_offset = float(params.get("LaneCenterOffset", return_default=True))
    self._lane_centering_e2e_authority = float(params.get("LaneCenteringE2EAuthority", return_default=True))
    self._lane_centering_pause_on_signal = bool(params.get("LaneCenteringPauseOnSignal", return_default=True))

  def lane_centering_direction(self) -> int:
    """1 to tint the right ego lane line, -1 the left, 0 for no tint."""
    if not self._lane_centering_enabled:
      return 0

    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      return 0
    CS = sm["carState"]

    # Prefer the correction actually applied by controlsd over the instantaneous raw one, so the
    # tint follows the smoothed output and does not flicker (also drops out on driver override).
    applied_correction = None
    if sm.recv_frame["controlsState"] >= ui_state.started_frame:
      applied_correction = sm["controlsState"].desiredCurvature - sm["modelV2"].action.desiredCurvature

    return get_lane_centering_visual_direction(
      sm["modelV2"], CS.vEgo, self._lane_center_offset, self._lane_centering_e2e_authority,
      True, ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY), self._lane_centering_pause_on_signal,
      bool(CS.leftBlinker or CS.rightBlinker), applied_correction)
