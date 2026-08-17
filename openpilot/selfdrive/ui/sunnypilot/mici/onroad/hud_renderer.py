"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.exp_button import DEC_RING_ACC_COLOR, DEC_RING_BLENDED_COLOR, DecState
from openpilot.selfdrive.ui.ui_state import ui_state

# Filled disc behind the ~50px mici wheel
DEC_RING_RADIUS = 35

# Darker colors for the DEC-off states
DEC_RING_BLENDED_DARK = rl.Color(180, 90, 0, 200)
DEC_RING_ACC_DARK = rl.Color(0, 122, 144, 200)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)

  def _draw_wheel_ring(self, cx: int, cy: int, alpha: float) -> None:
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

    color = rl.Color(base.r, base.g, base.b, int(base.a * alpha / 255))
    rl.draw_circle(cx, cy, DEC_RING_RADIUS, color)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
