"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Iterable

import pyray as rl


ACCEL_LIMIT = 4.0  # m/s^2, clamp for vertical axis
GRID_LEVEL = 2.0   # m/s^2, faint horizontal gridlines at ±this value

BG_COLOR = rl.Color(0, 0, 0, 100)
ZERO_LINE_COLOR = rl.Color(255, 255, 255, 80)
GRID_COLOR = rl.Color(255, 255, 255, 40)
TRACE_COLOR = rl.Color(0, 255, 0, 220)


class AccelGraph:
  def render(self, rect: rl.Rectangle, history: Iterable[float]) -> None:
    rl.draw_rectangle(int(rect.x), int(rect.y), int(rect.width), int(rect.height), BG_COLOR)

    mid_y = rect.y + rect.height / 2.0
    half_h = rect.height / 2.0

    def accel_to_y(a: float) -> float:
      clamped = max(-ACCEL_LIMIT, min(ACCEL_LIMIT, a))
      return mid_y - (clamped / ACCEL_LIMIT) * half_h

    rl.draw_line_ex(rl.Vector2(rect.x, mid_y), rl.Vector2(rect.x + rect.width, mid_y), 1.0, ZERO_LINE_COLOR)
    for level in (GRID_LEVEL, -GRID_LEVEL):
      y = accel_to_y(level)
      rl.draw_line_ex(rl.Vector2(rect.x, y), rl.Vector2(rect.x + rect.width, y), 1.0, GRID_COLOR)

    samples = list(history)
    n = len(samples)
    if n < 2:
      return

    # Newest sample on the right edge; older samples extend back to the left.
    step = rect.width / (n - 1)
    prev_pt = rl.Vector2(rect.x, accel_to_y(samples[0]))
    for i in range(1, n):
      pt = rl.Vector2(rect.x + i * step, accel_to_y(samples[i]))
      rl.draw_line_ex(prev_pt, pt, 4.0, TRACE_COLOR)
      prev_pt = pt
