"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui.accel_graph import ACCEL_LIMIT
from openpilot.selfdrive.ui.ui_state import ui_state


BAR_WIDTH = 28.0
TICK_EXTENSION = 12.0
ZERO_TICK_EXTENSION = 20.0
ACCEL_COLOR = rl.Color(0, 245, 0, 200)
DECEL_COLOR = rl.Color(245, 0, 0, 200)
TICK_COLOR = rl.Color(255, 255, 255, 140)
ZERO_LINE_COLOR = rl.Color(255, 255, 255, 200)


class RocketFuel:
  def __init__(self):
    self.vc_accel = 0.0

  def render(self, rect: rl.Rectangle, sm) -> None:
    if not ui_state.rocket_fuel:
      return

    vc_accel0 = sm['carState'].aEgo

    # Smooth the acceleration
    self.vc_accel = self.vc_accel + (vc_accel0 - self.vc_accel) / 5.0

    # Linear scale matching the developer UI accel graph: full half-height maps to ACCEL_LIMIT.
    half_h = rect.height / 2.0
    mid_y = rect.y + half_h

    clamped = max(-ACCEL_LIMIT, min(ACCEL_LIMIT, self.vc_accel))
    bar_h = abs(clamped) / ACCEL_LIMIT * half_h

    if clamped > 0:
      bar_y = mid_y - bar_h
      color = ACCEL_COLOR
    else:
      bar_y = mid_y
      color = DECEL_COLOR

    if bar_h > 0:
      rl.draw_rectangle(int(rect.x), int(bar_y), int(BAR_WIDTH), int(bar_h), color)

    # Tick marks at every 1 m/s² so the bar's scale is readable; zero is a thicker center line.
    tick_x0 = rect.x
    for level in range(-int(ACCEL_LIMIT), int(ACCEL_LIMIT) + 1):
      y = mid_y - (level / ACCEL_LIMIT) * half_h
      if level == 0:
        rl.draw_line_ex(rl.Vector2(tick_x0, y), rl.Vector2(tick_x0 + BAR_WIDTH + ZERO_TICK_EXTENSION, y),
                        4.0, ZERO_LINE_COLOR)
      else:
        rl.draw_line_ex(rl.Vector2(tick_x0, y), rl.Vector2(tick_x0 + BAR_WIDTH + TICK_EXTENSION, y),
                        2.0, TICK_COLOR)
