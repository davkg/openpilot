"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app

CALIBRATED = log.LiveCalibrationData.Status.calibrated

# CAMERA_OBJECT_TRACKS marker config
NUM_OBJECT_SLOTS = 10
MARKER_Z_M = 1.2
MIN_RADIUS_PX = 12.0
MAX_RADIUS_PX = 36.0
NEAR_RANGE_M = 5.0
FAR_RANGE_M = 100.0
EGO_LANE_HALF_W = 1.5
ADJACENT_LANE_OUTER = 4.5

# Lane-band colors (RGB; alpha modulated per-frame by per-slot filter)
COLOR_EGO   = (255, 255, 255)  # white
COLOR_LEFT  = (0,   200, 255)  # cyan
COLOR_RIGHT = (255,  80, 220)  # magenta
COLOR_FAR   = (160, 160, 160)  # dim grey


def _marker_color_for(y_rel: float) -> tuple[int, int, int]:
  abs_y = abs(y_rel)
  if abs_y < EGO_LANE_HALF_W:
    return COLOR_EGO
  if abs_y <= ADJACENT_LANE_OUTER:
    return COLOR_LEFT if y_rel > 0 else COLOR_RIGHT
  return COLOR_FAR


def _marker_radius_for(d_rel: float) -> float:
  d = max(NEAR_RANGE_M, min(FAR_RANGE_M, d_rel))
  t = (d - NEAR_RANGE_M) / (FAR_RANGE_M - NEAR_RANGE_M)
  return MAX_RADIUS_PX - t * (MAX_RADIUS_PX - MIN_RADIUS_PX)


class ModelRendererSP:
  def __init__(self):
    self.rainbow_path = RainbowPath()
    self.chevron_metrics = ChevronMetrics()
    self._camera_marker_alpha = [
      FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps) for _ in range(NUM_OBJECT_SLOTS)
    ]
    self._camera_marker_font: rl.Font = gui_app.font(FontWeight.MEDIUM)

  def update_camera_object_markers(self, sm) -> None:
    if not getattr(ui_state, 'adjacent_vehicle_markers', False):
      for f in self._camera_marker_alpha:
        f.update(0.0)
      return

    tracks = sm['cameraObjectTracksSP'].tracks
    seen = [False] * NUM_OBJECT_SLOTS
    for t in tracks:
      if 0 <= t.slot < NUM_OBJECT_SLOTS:
        self._camera_marker_alpha[t.slot].update(1.0 if t.valid else 0.0)
        seen[t.slot] = True
    for i, was_seen in enumerate(seen):
      if not was_seen:
        self._camera_marker_alpha[i].update(0.0)

  def draw_camera_object_markers(self) -> None:
    if not getattr(ui_state, 'adjacent_vehicle_markers', False):
      return

    sm = ui_state.sm
    if sm['liveCalibration'].calStatus != CALIBRATED:
      return

    tracks = sm['cameraObjectTracksSP'].tracks
    if len(tracks) == 0:
      return

    debug = getattr(ui_state, 'adjacent_vehicle_markers_debug', False)

    for t in tracks:
      if not (0 <= t.slot < NUM_OBJECT_SLOTS):
        continue
      alpha = self._camera_marker_alpha[t.slot].x
      if alpha < 0.01:
        continue

      # Mirror the lead-chevron sign/offset convention from _update_leads
      point = self._map_to_screen(t.dRel, -t.yRel + self._camera_offset, MARKER_Z_M + self._path_offset_z)
      if point is None:
        continue

      r, g, b = _marker_color_for(t.yRel)
      a = int(255 * max(0.0, min(1.0, alpha)))
      color = rl.Color(r, g, b, a)
      radius = _marker_radius_for(t.dRel)
      rl.draw_circle(int(point[0]), int(point[1]), radius, color)

      if debug and t.valid:
        label_pos = rl.Vector2(point[0] + radius + 2, point[1] - 12)
        rl.draw_text_ex(self._camera_marker_font, f"#{int(t.objectId)}", label_pos, 18, 0, color)
