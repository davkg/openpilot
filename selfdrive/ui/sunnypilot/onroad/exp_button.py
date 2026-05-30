import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState

# Ring colors reuse the experimental/chill gradient palette of the mode toggle (exp_mode_button.py)
# so the DEC ring reads as "experimental" vs "chill" at a glance.
DEC_RING_BLENDED_COLOR = rl.Color(255, 155, 63, 200)  # orange = experimental (e2e / blended)
DEC_RING_ACC_COLOR = rl.Color(20, 255, 171, 200)      # teal = chill (acc)
DEC_RING_WIDTH = 10


class ExpButtonSP(ExpButton):
  """Mode button that also surfaces Dynamic Experimental Control (DEC) state.

  When DEC is arbitrating (experimental mode on + DEC enabled, i.e. dec.active), a colored ring is
  drawn around the button and the icon tracks DEC's live choice: experimental icon for blended
  (e2e), steering wheel for acc (chill). This lets the three modes be told apart at a glance:
    - no ring + wheel icon          -> chill only (experimental mode off)
    - no ring + experimental icon   -> experimental only (DEC off)
    - ring (orange/teal) + matching icon -> experimental + DEC active (live acc<->blended)
  With DEC inactive the button behaves exactly like the upstream ExpButton.
  """
  def __init__(self, button_size: int, icon_size: int):
    super().__init__(button_size, icon_size)
    self._dec_active: bool = False
    self._dec_blended: bool = False

  def _update_state(self) -> None:
    super()._update_state()
    dec = ui_state.sm["longitudinalPlanSP"].dec
    self._dec_active = dec.active
    self._dec_blended = dec.state == DecState.blended

  def _held_or_actual_mode(self) -> bool:
    base = super()._held_or_actual_mode()  # resolves tap-hold and clears expired holds
    # Preserve the upstream tap feedback while a user-initiated hold is active.
    if self._hold_end_time is not None:
      return base
    # While DEC arbitrates, show its live choice rather than the static ExperimentalMode param.
    if self._dec_active:
      return self._dec_blended
    return base

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    if self._dec_active:
      center = rl.Vector2(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2)
      radius = self._rect.width / 2
      color = DEC_RING_BLENDED_COLOR if self._dec_blended else DEC_RING_ACC_COLOR
      rl.draw_ring(center, radius - DEC_RING_WIDTH, radius, 0, 360, 64, color)
