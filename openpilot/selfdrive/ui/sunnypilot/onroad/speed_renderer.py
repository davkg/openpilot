"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG, FONT_SIZES, COLORS


class SpeedRenderer:
  def __init__(self):
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

  def update(self) -> None:
    car_state = ui_state.sm['carState']
    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen and not ui_state.true_v_ego_ui else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def render(self, rect: rl.Rectangle) -> None:
    if ui_state.hide_v_ego_ui:
      return

    # Current speed sits directly under the set-speed box in the bottom-right cluster
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    set_speed_x = rect.x + rect.width - 30 - 2 * set_speed_width - 24
    center_x = set_speed_x + set_speed_width / 2
    set_speed_bottom = rect.y + rect.height - 410 + UI_CONFIG.set_speed_height

    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_y = set_speed_bottom + 20
    rl.draw_text_ex(self._font_bold, speed_text, rl.Vector2(center_x - speed_text_size.x / 2, speed_y),
                    FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    rl.draw_text_ex(self._font_medium, unit_text,
                    rl.Vector2(center_x - unit_text_size.x / 2, speed_y + speed_text_size.y - 10),
                    FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
