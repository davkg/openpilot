import pyray as rl
from cereal import messaging, car

from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG, COLORS
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget

# Bar geometry
BAR_WIDTH = 60
BAR_HEIGHT = 24
BAR_GAP = 8
BAR_COUNT = 3
INDICATOR_WIDTH = BAR_COUNT * BAR_WIDTH + (BAR_COUNT - 1) * BAR_GAP  # 132 px
MARGIN_ABOVE_BOX = 24  # gap between top of set-speed box and bottom of bars

# Colors per personality for active bars, keyed by .raw int (capnp enums are not reliably hashable)
_ACTIVE_COLORS = {
  0: rl.Color(255, 100, 50, 255),   # aggressive  → orange-red
  1: rl.Color(22, 127, 64, 255),    # standard    → green (UIStatus.ENGAGED)
  2: rl.Color(255, 255, 255, 255),  # relaxed     → white
}
_INACTIVE_COLOR = COLORS.DARK_GREY


class FollowDistanceRenderer(Widget):
  """Draws a horizontal 1-3 bar follow-distance/personality indicator.

  aggressive → 1 bar, standard → 2 bars, relaxed → 3 bars.
  Centered horizontally between the set-speed box left edge and the
  speed-limit sign right edge. Positioned just above both boxes.
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

    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial

    # Horizontal center between set-speed left edge and speed-limit sign right edge:
    #   set_speed_left  = rect.x + rect.width - 30 - 2*set_speed_width - 24
    #   sign_right      = rect.x + rect.width - 30
    #   center_x        = (set_speed_left + sign_right) / 2
    #                   = rect.x + rect.width - 30 - set_speed_width - 12
    center_x = rect.x + rect.width - 30 - set_speed_width - 12

    # Vertical: just above the top of the set-speed / speed-limit boxes
    set_speed_top_y = rect.y + rect.height - 410
    bar_y = set_speed_top_y - BAR_HEIGHT - MARGIN_ABOVE_BOX

    active_color = _ACTIVE_COLORS.get(self._personality_raw, COLORS.WHITE)
    bar_x_start = center_x - INDICATOR_WIDTH / 2

    for i in range(BAR_COUNT):
      x = bar_x_start + i * (BAR_WIDTH + BAR_GAP)
      color = active_color if i < self._active_bars else _INACTIVE_COLOR
      rl.draw_rectangle_rounded(
        rl.Rectangle(x, bar_y, BAR_WIDTH, BAR_HEIGHT),
        0.5,
        6,
        color,
      )
