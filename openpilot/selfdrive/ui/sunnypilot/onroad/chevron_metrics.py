"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import pyray as rl
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

# Render at bottom left instead of moving around with lead indicator
STATIC_ANCHOR_X = 360
STATIC_ANCHOR_Y_FROM_BOTTOM = 60


class ChevronOptions:
  OFF = 0
  DISTANCE_ONLY = 1
  SPEED_ONLY = 2
  TTC_ONLY = 3
  ALL = 4


class ChevronMetrics:
  def __init__(self):
    self._lead_status_alpha: float = 0.0
    self._font = gui_app.font(FontWeight.MONO_SEMI_BOLD)

  def update_alpha(self, has_lead: bool):
    """Update the alpha value for fade in/out animation"""
    if not has_lead:
      self._lead_status_alpha = max(0.0, self._lead_status_alpha - 0.05)
    else:
      self._lead_status_alpha = min(1.0, self._lead_status_alpha + 0.1)

  def should_render(self) -> bool:
    """Check if dev UI should be rendered"""
    return ui_state.chevron_metrics != ChevronOptions.OFF and self._lead_status_alpha > 0.0

  def _draw_lead(self, lead_data, lead_vehicle, v_ego: float, rect: rl.Rectangle, cam_d_rel: float = 0.0, desired_dist: float = 0.0):
    """Draw lead vehicle status information (distance, speed, TTC)"""
    if not self.should_render():
      return

    d_rel = lead_data.dRel
    v_rel = lead_data.vRel

    chevron_x = rect.x + STATIC_ANCHOR_X
    chevron_y = rect.y + rect.height - STATIC_ANCHOR_Y_FROM_BOTTOM
    sz = 0.0

    text_lines = self._build_text_lines(d_rel, v_rel, v_ego, cam_d_rel, desired_dist)
    if not text_lines:
      return

    self._render_text_lines(text_lines, chevron_x, chevron_y, sz, rect)

  @staticmethod
  def _build_text_lines(d_rel: float, v_rel: float, v_ego: float, cam_d_rel: float = 0.0, desired_dist: float = 0.0) -> list[str]:
    """Build text lines based on chevron info setting"""
    text_lines = []

    # Distance
    if ui_state.chevron_metrics == ChevronOptions.DISTANCE_ONLY or ui_state.chevron_metrics == ChevronOptions.ALL:
      val = max(0.0, d_rel)
      text_lines.append(f"{val:.1f} m")
      # if cam_d_rel > 0:
      #   text_lines.append(f"{val:.0f} m ({cam_d_rel:.0f} m)")
      # else:
      #   text_lines.append(f"{val:.0f} m")

    # Time to collision
    if ui_state.chevron_metrics == ChevronOptions.TTC_ONLY or ui_state.chevron_metrics == ChevronOptions.ALL:
      val = (d_rel / v_ego) if (d_rel > 0 and v_ego > 0) else 0.0
      ttc_text = f"{val:.1f} s" if (0 < val < 200) else "---"
      text_lines.append(ttc_text)

    # MPC desired follow distance
    if desired_dist > 0:
      desired_dist_s = desired_dist / v_ego if v_ego > 0 else 0.0
      if 0 < desired_dist_s < 200:
        text_lines.append(f"mpc {desired_dist:.1f} m | {desired_dist_s:.1f} s")
      else:
        text_lines.append(f"mpc {desired_dist:.1f} m")

    # Speed
    if ui_state.chevron_metrics == ChevronOptions.SPEED_ONLY:
      multiplier = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
      val = max(0.0, (v_rel + v_ego) * multiplier)
      unit = "km/h" if ui_state.is_metric else "mph"
      text_lines.append(f"{val:.0f} {unit}")

    return text_lines

  def _render_text_lines(self, text_lines: list[str], chevron_x: float, chevron_y: float,
                         sz: float, rect: rl.Rectangle):
    """Render text lines with proper centering and positioning"""
    font_size = 56
    line_height = 70
    margin = 20

    text_y = chevron_y + sz + 15
    total_height = len(text_lines) * line_height

    # Adjust Y position if text would go off screen
    if text_y + total_height > rect.y + rect.height - margin:
      y_max = min(chevron_y, rect.y + rect.height - margin)
      text_y = y_max - 15 - total_height
      text_y = max(rect.y + margin, text_y)

    alpha = int(255 * self._lead_status_alpha)
    text_color = rl.Color(255, 255, 255, alpha)
    shadow_color = rl.Color(0, 0, 0, int(200 * self._lead_status_alpha))

    for i, line in enumerate(text_lines):
      y = int(text_y + (i * line_height))
      if y + line_height > rect.y + rect.height - margin:
        break

      # Measure actual text width for proper centering
      text_size = measure_text_cached(self._font, line, font_size, 0)
      text_width = text_size.x

      # Left align the text
      x = int(chevron_x)
      x = int(np.clip(x, rect.x + margin, rect.x + rect.width - text_width - margin))

      # Draw shadow
      rl.draw_text_ex(self._font, line, rl.Vector2(x + 2, y + 2), font_size, 0, shadow_color)
      # Draw text
      rl.draw_text_ex(self._font, line, rl.Vector2(x, y), font_size, 0, text_color)

  def draw_lead_status(self, sm, radar_state, rect, lead_vehicles):
    lead_one = radar_state.leadOne
    has_lead_one = lead_one.present if lead_one else False

    self.update_alpha(has_lead_one)

    if not self.should_render():
      return

    v_ego = sm['carState'].vEgo
    # Single-lead distance from CAMERA_LEAD; 0 = no lead / out of range.
    cam_d_rel = sm['carStateSP'].cameraLeadDistance
    desired_dist = sm['longitudinalPlanSP'].desiredFollowDistance

    if has_lead_one:
      self._draw_lead(lead_one, lead_vehicles[0], v_ego, rect, cam_d_rel, desired_dist)
