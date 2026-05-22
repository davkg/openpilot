"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
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
# Device-frame z is positive-down in _map_to_screen, so negative values lift
# the marker above the path. ~-0.8 m puts it around mid-rear of a sedan.
MARKER_Z_M = -0.8
MIN_RADIUS_PX = 1.0
MAX_RADIUS_PX = 30.0
NEAR_RANGE_M = 5.0
FAR_RANGE_M = 100.0
EGO_LANE_HALF_W = 1.5
ADJACENT_LANE_OUTER = 4.5

# Lane-band colors (RGB; alpha modulated per-frame by per-slot filter)
COLOR_EGO   = (255, 255, 255)  # white
COLOR_LEFT  = (0,   200, 255)  # cyan
COLOR_RIGHT = (255,  80, 220)  # magenta
COLOR_FAR   = (160, 160, 160)  # dim grey

# Paced-track highlight (checkerboard staggering)
PACED_RING_GAP_PX = 6.0       # gap from dot edge to inner edge of ring
PACED_RING_THICKNESS_PX = 3.0
PACED_PULSE_PERIOD_S = 0.8    # full pulse cycle
PACED_PULSE_MIN = 0.4         # alpha floor of the pulse so the ring never fully vanishes


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
    self._paced_pulse_frame = 0

  def render_camera_object_markers(self, sm) -> None:
    # Show markers if the debug toggle is on OR the checkerboard controller is actively
    # pacing (so the driver can see which adjacent car is triggering the bias even when
    # the debug overlay is off).
    paced_object_id = int(sm['longitudinalPlanSP'].checkerboard.pacingObjectId)
    if not (getattr(ui_state, 'adjacent_vehicle_markers', False) or paced_object_id != 0):
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

    self._paced_pulse_frame += 1

    if sm['liveCalibration'].calStatus != CALIBRATED or len(tracks) == 0:
      return

    debug = getattr(ui_state, 'adjacent_vehicle_markers_debug', False)

    # Pulse phase for the paced-track ring (sinusoid clamped to [PACED_PULSE_MIN, 1]).
    period_frames = max(1.0, PACED_PULSE_PERIOD_S * gui_app.target_fps)
    pulse_phase = 0.5 + 0.5 * math.sin(2.0 * math.pi * self._paced_pulse_frame / period_frames)
    pulse = PACED_PULSE_MIN + (1.0 - PACED_PULSE_MIN) * pulse_phase

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

      # Paced-track highlight: pulsing white ring around the controller's chosen target.
      if paced_object_id != 0 and t.valid and int(t.objectId) == paced_object_id:
        ring_alpha = int(255 * max(0.0, min(1.0, alpha * pulse)))
        rl.draw_ring(rl.Vector2(point[0], point[1]),
                     radius + PACED_RING_GAP_PX,
                     radius + PACED_RING_GAP_PX + PACED_RING_THICKNESS_PX,
                     0.0, 360.0, 24,
                     rl.Color(255, 255, 255, ring_alpha))

      if debug and t.valid:
        label_pos = rl.Vector2(point[0] + radius + 2, point[1] - 12)
        rl.draw_text_ex(self._camera_marker_font, f"#{int(t.objectId)}", label_pos, 18, 0, color)
