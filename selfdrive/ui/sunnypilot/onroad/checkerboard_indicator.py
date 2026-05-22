"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

CheckerboardState = custom.LongitudinalPlanSP.Checkerboard.CheckerboardState

PACING_COLOR = rl.Color(255, 165, 0, 255)  # amber, distinct from SCC green / SLA grey


class CheckerboardIndicator(Widget):
  def __init__(self):
    super().__init__()
    self.pacing = False
    self.pacing_side = 0  # +1 = left adjacent, -1 = right adjacent
    self.font = gui_app.font(FontWeight.BOLD)

  def update(self):
    sm = ui_state.sm
    if sm.updated["longitudinalPlanSP"]:
      cb = sm["longitudinalPlanSP"].checkerboard
      self.pacing = cb.state == CheckerboardState.pacing
      self.pacing_side = int(cb.pacingSide)

  def _draw_icon(self, rect_center_x, rect_height, x_offset, y_offset, name):
    font_size = 36
    padding_v = 5
    padding_h = 20

    sz = measure_text_cached(self.font, name, font_size)
    box_width = int(sz.x + padding_h * 2)
    box_height = int(sz.y + padding_v * 2)

    screen_y = rect_height / 4 + y_offset
    box_x = rect_center_x + x_offset - box_width / 2
    box_y = screen_y - box_height / 2

    rl.draw_rectangle_rounded(rl.Rectangle(box_x, box_y, box_width, box_height), 0.2, 10, PACING_COLOR)
    text_pos_x = box_x + (box_width - sz.x) / 2
    text_pos_y = box_y + (box_height - sz.y) / 2
    rl.draw_text_ex(self.font, name, rl.Vector2(text_pos_x, text_pos_y), font_size, 0, rl.Color(0, 0, 0, 255))

  def _render(self, rect: rl.Rectangle):
    if not self.pacing:
      return

    if self.pacing_side > 0:
      label = "CB - LEFT"
    elif self.pacing_side < 0:
      label = "CB - RIGHT"
    else:
      label = "CB"
    self._draw_icon(rect.x, rect.height, 110, 300, label)
