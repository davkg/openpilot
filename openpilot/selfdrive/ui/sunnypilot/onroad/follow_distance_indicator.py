import numpy as np
import pyray as rl
from openpilot.cereal import messaging
from opendbc.car.structs import car

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.hud_renderer import COLORS
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.follow_time import get_follow_time, read_follow_time
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

FONT_SIZE = 90
UNIT_FONT_SIZE = 40
UNIT_OVERLAP = 18  # pulls the unit up into the value's line padding
CORNER_MARGIN = 60  # box inset from the left and bottom screen edges; the compass stacks above it

# Background box around the value and its "sec" unit line
BG_WIDTH = 190
BG_HEIGHT = 150
BG_ROUNDNESS = 0.35

# Two-sided bar under the value, filling out from the center by how far one candidate binds below the other.
# Left: e2e binds below ACC (gas nudges it up). Right: the lead MPC binds below e2e (the follow time is the limit).
E2E_WIDTH = 0.3  # m/s^2 of margin for a full left side
MPC_WIDTH = 0.15  # m/s^2 of margin for a full right side
BLEND_RC = 0.3  # s, one filter on the signed fill, so a side swap passes through empty
# Right side in steady following only: while closing, MPC binding means braking early, not a long gap
VREL_GATE = (0.5, 1.5)  # m/s, |vRel| fade-out
VREL_RC = 1.0  # s, so the gate can't chatter
BAR_WIDTH = 130
BAR_HEIGHT = 16
BAR_MARGIN_BOTTOM = 25  # bar bottom to box bottom
VALUE_SHIFT_UP = 18  # makes room for the bar under the value; with BAR_MARGIN_BOTTOM, centers digits + bar in the box


class FollowDistanceRenderer(Widget):
  """Draws the longitudinal follow time in a box: the value (e.g. "1.2") with a smaller "sec" below it,
  styled like the speed indicator's unit.

  Positioned in the bottom-left corner of the screen, under the compass.
  Hidden when OP does not have longitudinal control.

  With long active, a bar under the value fills left when the model limits accel below ACC, and right when
  the follow time limits it behind a lead.
  """

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._follow_time: float = read_follow_time(self._params)
    self._longitudinal_control: bool = False
    self._binding = FirstOrderFilter(0.0, BLEND_RC, 1 / gui_app.target_fps)  # -1 e2e side .. +1 MPC side
    self._v_rel = FirstOrderFilter(0.0, VREL_RC, 1 / gui_app.target_fps)
    self._lead_present = False

    if car_params := self._params.get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def update(self) -> None:
    if ui_state.sm.updated["carParams"]:
      self._longitudinal_control = ui_state.sm["carParams"].openpilotLongitudinalControl

    if ui_state.started and ui_state.sm.valid["selfdriveStateSP"]:
      self._follow_time = get_follow_time(ui_state.sm["selfdriveStateSP"])

    lead = ui_state.sm["radarState"].leadOne
    if lead.present:
      if not self._lead_present:
        self._v_rel.x = lead.vRel  # a new lead may already be closing
      self._v_rel.update(lead.vRel)
    self._lead_present = lead.present

    binding = 0.0
    if ui_state.started and ui_state.sm["carControl"].longActive:
      lp_sp = ui_state.sm["longitudinalPlanSP"]
      e2e_binding = np.clip((min(lp_sp.mpcAccel, lp_sp.cruiseAccel) - lp_sp.e2eAccel) / E2E_WIDTH, 0.0, 1.0)
      mpc_binding = 0.0
      if lead.present:
        steady = np.interp(abs(self._v_rel.x), VREL_GATE, [1.0, 0.0])
        mpc_binding = steady * np.clip((min(lp_sp.e2eAccel, lp_sp.cruiseAccel) - lp_sp.mpcAccel) / MPC_WIDTH, 0.0, 1.0)
      binding = mpc_binding - e2e_binding  # at most one is nonzero
    self._binding.update(binding)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._longitudinal_control:
      return

    bg_x = rect.x + CORNER_MARGIN
    bg_y = rect.y + rect.height - CORNER_MARGIN - BG_HEIGHT
    rl.draw_rectangle_rounded(rl.Rectangle(bg_x, bg_y, BG_WIDTH, BG_HEIGHT), BG_ROUNDNESS, 10, COLORS.BLACK_TRANSLUCENT)

    center_x = bg_x + BG_WIDTH / 2

    value_text = f"{self._follow_time:.1f}"
    value_size = measure_text_cached(self._font_bold, value_text, FONT_SIZE)
    value_y = bg_y + BG_HEIGHT / 2 - value_size.y / 2 - VALUE_SHIFT_UP
    rl.draw_text_ex(self._font_bold, value_text, rl.Vector2(center_x - value_size.x / 2, value_y), FONT_SIZE, 0, COLORS.WHITE)

    bar_x = center_x - BAR_WIDTH / 2
    bar_y = bg_y + BG_HEIGHT - BAR_MARGIN_BOTTOM - BAR_HEIGHT
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, bar_y, BAR_WIDTH, BAR_HEIGHT), 1.0, 10, COLORS.BORDER_TRANSLUCENT)
    fill_width = BAR_WIDTH / 2 * abs(float(self._binding.x))
    if fill_width >= 1:
      fill_x = center_x if self._binding.x > 0 else center_x - fill_width
      rl.draw_rectangle_rounded(rl.Rectangle(fill_x, bar_y, fill_width, BAR_HEIGHT), 1.0, 10, COLORS.WHITE)
      # square off the center end so the fill reads as anchored there
      cap = min(fill_width, BAR_HEIGHT / 2)
      cap_x = center_x if self._binding.x > 0 else center_x - cap
      rl.draw_rectangle_rec(rl.Rectangle(cap_x, bar_y, cap, BAR_HEIGHT), COLORS.WHITE)

    # value_text = f"{self._follow_time:.1f}"
    # unit_text = tr("sec")
    # value_size = measure_text_cached(self._font_bold, value_text, FONT_SIZE)
    # unit_size = measure_text_cached(self._font_medium, unit_text, UNIT_FONT_SIZE)

    # # center the overlapped value + unit pair in the box; need to manually adjust y offset to center
    # value_y = bg_y + BG_HEIGHT / 2 - (value_size.y + unit_size.y - UNIT_OVERLAP) / 2 - 4
    # rl.draw_text_ex(self._font_bold, value_text, rl.Vector2(center_x - value_size.x / 2, value_y), FONT_SIZE, 0, COLORS.WHITE)
    # rl.draw_text_ex(self._font_medium, unit_text, rl.Vector2(center_x - unit_size.x / 2, value_y + value_size.y - UNIT_OVERLAP),
    #                 UNIT_FONT_SIZE, 0, COLORS.WHITE_TRANSLUCENT)
