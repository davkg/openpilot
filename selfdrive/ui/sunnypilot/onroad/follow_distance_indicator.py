import pyray as rl
from cereal import messaging, car

from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG, COLORS
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget

# Bar geometry
BAR_WIDTH = 60
BAR_HEIGHT = 40
BAR_GAP = 8
BAR_COUNT = 3
INDICATOR_HEIGHT = BAR_COUNT * BAR_HEIGHT + (BAR_COUNT - 1) * BAR_GAP  # 88 px
MARGIN_RIGHT_OF_BUTTON = 24  # gap between right edge of exp button and left edge of background box

# Background box around the bars
BG_PAD_X = 15
BG_PAD_Y = 15
BG_ROUNDNESS = 0.35

_ACTIVE_COLOR = rl.Color(255, 255, 255, 255)
_INACTIVE_COLOR = rl.Color(255, 255, 255, 30)


class FollowDistanceRenderer(Widget):
  """Draws a vertically stacked 1-3 bar follow-distance/personality indicator.

  aggressive → 1 bar, standard → 2 bars, relaxed → 3 bars (lit from bottom up).
  Positioned at the bottom-left of the screen, just to the right of the
  experimental-mode button.
  Hidden when OP does not have longitudinal control.
  """

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._personality_raw: int = 1  # 0=aggressive, 1=standard, 2=relaxed
    self._active_bars: int = 2  # 1..3
    self._longitudinal_control: bool = False

    if car_params := self._params.get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def _get_personality_raw(self) -> int:
    if ui_state.started and ui_state.sm.valid["selfdriveState"]:
      personality = ui_state.sm["selfdriveState"].personality
      return personality.raw

    return int(self._params.get("LongitudinalPersonality", return_default=True))

  def update(self) -> None:
    if ui_state.sm.updated["carParams"]:
      self._longitudinal_control = ui_state.sm["carParams"].openpilotLongitudinalControl

    self._personality_raw = self._get_personality_raw()
    # aggressive=0 → 1 bar, standard=1 → 2 bars, relaxed=2 → 3 bars
    self._active_bars = self._personality_raw + 1

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._longitudinal_control:
      return

    button_x = rect.x + UI_CONFIG.border_size * 2
    button_y = rect.y + rect.height - UI_CONFIG.border_size * 2 - UI_CONFIG.button_size
    button_center_y = button_y + UI_CONFIG.button_size / 2

    bg_width = BAR_WIDTH + 2 * BG_PAD_X
    bg_height = INDICATOR_HEIGHT + 2 * BG_PAD_Y
    bg_x = button_x + UI_CONFIG.button_size + MARGIN_RIGHT_OF_BUTTON
    bg_y = button_center_y - bg_height / 2
    bar_x = bg_x + BG_PAD_X
    bar_y_start = bg_y + BG_PAD_Y

    rl.draw_rectangle_rounded(rl.Rectangle(bg_x, bg_y, bg_width, bg_height), BG_ROUNDNESS, 10, COLORS.BLACK_TRANSLUCENT)

    # Light bars from the bottom up: index 0 is the bottom bar.
    for i in range(BAR_COUNT):
      y = bar_y_start + (BAR_COUNT - 1 - i) * (BAR_HEIGHT + BAR_GAP)
      color = _ACTIVE_COLOR if i < self._active_bars else _INACTIVE_COLOR
      rl.draw_rectangle_rounded(
        rl.Rectangle(bar_x, y, BAR_WIDTH, BAR_HEIGHT),
        0.5,
        6,
        color,
      )
