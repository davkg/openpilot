import numpy as np
import pyray as rl
from openpilot.cereal import messaging
from opendbc.car.structs import car

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG, COLORS
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.follow_time import get_follow_time, read_follow_time
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

FONT_SIZE = 90
UNIT_FONT_SIZE = 40
UNIT_OVERLAP = 18  # pulls the unit up into the value's line padding
MARGIN_RIGHT_OF_BUTTON = 24  # gap between right edge of exp button and left edge of background box

# Background box around the value and its "sec" unit line
BG_WIDTH = 190
BG_HEIGHT = 130
BG_ROUNDNESS = 0.35

# How far e2e binds below ACC, shown as a bar rather than a switch so near-ties can't flicker.
# The bar fills over BLEND_WIDTH m/s^2 of margin.
BLEND_WIDTH = 0.3
BLEND_RC = 0.1  # s, light smoothing so lead swaps don't jump the bar
BAR_WIDTH = 130
BAR_HEIGHT = 14
BAR_MARGIN_BOTTOM = 18  # bar bottom to box bottom
VALUE_SHIFT_UP = 14  # makes room for the bar under the value


class FollowDistanceRenderer(Widget):
  """Draws the longitudinal follow time in a box: the value (e.g. "1.2") with a smaller "sec" below it,
  styled like the speed indicator's unit.

  Positioned at the bottom-left of the screen, just to the right of the
  experimental-mode button.
  Hidden when OP does not have longitudinal control.

  In experimental mode with long active, a bar under the value fills as the e2e model brakes below ACC.
  """

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._follow_time: float = read_follow_time(self._params)
    self._longitudinal_control: bool = False
    self._e2e_binding = FirstOrderFilter(0.0, BLEND_RC, 1 / gui_app.target_fps)

    if car_params := self._params.get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def update(self) -> None:
    if ui_state.sm.updated["carParams"]:
      self._longitudinal_control = ui_state.sm["carParams"].openpilotLongitudinalControl

    if ui_state.started and ui_state.sm.valid["selfdriveStateSP"]:
      self._follow_time = get_follow_time(ui_state.sm["selfdriveStateSP"])

    e2e_binding = 0.0
    if ui_state.started and ui_state.sm["carControl"].longActive and ui_state.sm["selfdriveState"].experimentalMode:
      lp_sp = ui_state.sm["longitudinalPlanSP"]
      e2e_binding = np.clip((min(lp_sp.mpcAccel, lp_sp.cruiseAccel) - lp_sp.e2eAccel) / BLEND_WIDTH, 0.0, 1.0)
    self._e2e_binding.update(e2e_binding)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._longitudinal_control:
      return

    button_x = rect.x + UI_CONFIG.border_size * 2
    button_y = rect.y + rect.height - UI_CONFIG.border_size * 2 - UI_CONFIG.button_size
    button_center_y = button_y + UI_CONFIG.button_size / 2

    bg_x = button_x + UI_CONFIG.button_size + MARGIN_RIGHT_OF_BUTTON
    bg_y = button_center_y - BG_HEIGHT / 2
    rl.draw_rectangle_rounded(rl.Rectangle(bg_x, bg_y, BG_WIDTH, BG_HEIGHT), BG_ROUNDNESS, 10, COLORS.BLACK_TRANSLUCENT)

    center_x = bg_x + BG_WIDTH / 2

    value_text = f"{self._follow_time:.1f}"
    value_size = measure_text_cached(self._font_bold, value_text, FONT_SIZE)
    value_y = button_center_y - value_size.y / 2 - VALUE_SHIFT_UP
    rl.draw_text_ex(self._font_bold, value_text, rl.Vector2(center_x - value_size.x / 2, value_y), FONT_SIZE, 0, COLORS.WHITE)

    bar_x = center_x - BAR_WIDTH / 2
    bar_y = bg_y + BG_HEIGHT - BAR_MARGIN_BOTTOM - BAR_HEIGHT
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, bar_y, BAR_WIDTH, BAR_HEIGHT), 1.0, 10, COLORS.BORDER_TRANSLUCENT)
    fill_width = BAR_WIDTH * float(self._e2e_binding.x)
    if fill_width >= 1:
      rl.draw_rectangle_rounded(rl.Rectangle(bar_x, bar_y, fill_width, BAR_HEIGHT), 1.0, 10, COLORS.WHITE)

    # value_text = f"{self._follow_time:.1f}"
    # unit_text = tr("sec")
    # value_size = measure_text_cached(self._font_bold, value_text, FONT_SIZE)
    # unit_size = measure_text_cached(self._font_medium, unit_text, UNIT_FONT_SIZE)

    # # center the overlapped value + unit pair in the box; need to manually adjust y offset to center
    # value_y = button_center_y - (value_size.y + unit_size.y - UNIT_OVERLAP) / 2 - 4
    # rl.draw_text_ex(self._font_bold, value_text, rl.Vector2(center_x - value_size.x / 2, value_y), FONT_SIZE, 0, COLORS.WHITE)
    # rl.draw_text_ex(self._font_medium, unit_text, rl.Vector2(center_x - unit_size.x / 2, value_y + value_size.y - UNIT_OVERLAP),
    #                 UNIT_FONT_SIZE, 0, COLORS.WHITE_TRANSLUCENT)
