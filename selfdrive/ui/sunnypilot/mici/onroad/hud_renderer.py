"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.exp_button import DEC_RING_ACC_COLOR, DEC_RING_BLENDED_COLOR, DecState
from openpilot.selfdrive.ui.ui_state import ui_state

# Filled disc behind the ~50px mici wheel
DEC_RING_RADIUS = 35
# Seconds the disc flashes to full after any state change, so a toggle made while disengaged is still seen.
DEC_RING_FLASH_SECONDS = 2.0

# Darker colors for the DEC-off states
DEC_RING_BLENDED_DARK = rl.Color(180, 90, 0, 200)
DEC_RING_ACC_DARK = rl.Color(0, 122, 144, 200)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self._ring_state: tuple[bool, bool] | None = None
    self._ring_flash_end = 0.0

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)

  def _draw_wheel_ring(self, cx: int, cy: int, cy_rest: int, alpha: float) -> None:
    # DEC enabled: orange = e2e, teal = acc
    # DEC disabled: dark orange = e2e, dark teal = acc
    dec = ui_state.sm["longitudinalPlanSP"].dec
    experimental = ui_state.sm["selfdriveState"].experimentalMode
    live = dec.active
    blended = (dec.state == DecState.blended) if live else experimental
    if live:
      base = DEC_RING_BLENDED_COLOR if blended else DEC_RING_ACC_COLOR
    else:
      base = DEC_RING_BLENDED_DARK if blended else DEC_RING_ACC_DARK

    # Flash to full whenever the displayed state changes, so a toggle made while disengaged is visible
    now = time.monotonic()
    state = (live, blended)
    if self._ring_state is not None and state != self._ring_state:
      self._ring_flash_end = now + DEC_RING_FLASH_SECONDS
    self._ring_state = state
    flash = max(0.0, (self._ring_flash_end - now) / DEC_RING_FLASH_SECONDS) * 255
    eff_alpha = max(alpha, flash)

    # While flashing, pin the disc to the wheel's resting center so a disengaged flash isn't
    # slid off the bottom edge
    draw_y = cy_rest if flash > 0.0 else cy
    color = rl.Color(base.r, base.g, base.b, int(base.a * eff_alpha / 255))
    rl.draw_circle(cx, draw_y, DEC_RING_RADIUS, color)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
