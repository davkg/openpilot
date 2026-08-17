"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntEnum

from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise_sub_layouts.speed_limit_settings import SpeedLimitSettingsLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.t_follow_curve import parse_t_follow_curve
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
from openpilot.system.ui.sunnypilot.widgets.list_view import (
  toggle_item_sp, option_item_sp, simple_button_item_sp, multiple_button_item_sp, button_item_sp,
)
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.confirm_dialog import alert_dialog
from openpilot.system.ui.widgets.scroller_tici import Scroller


class PanelType(IntEnum):
  CRUISE = 0
  SLA = 1


ICBM_DESC = tr_noop("When enabled, sunnypilot will attempt to manage the built-in cruise control buttons " +
                    "by emulating button presses for limited longitudinal control.")
ICMB_UNAVAILABLE = tr_noop("Intelligent Cruise Button Management is currently unavailable on this platform.")
ICMB_UNAVAILABLE_LONG_AVAILABLE = tr_noop("Disable the sunnypilot Longitudinal Control (alpha) toggle to allow Intelligent Cruise Button Management.")
ICMB_UNAVAILABLE_LONG_UNAVAILABLE = tr_noop("sunnypilot Longitudinal Control is the default longitudinal control for this platform.")

ACC_ENABLED_DESCRIPTION = tr_noop("Enable custom Short & Long press increments for cruise speed increase/decrease.")
ACC_NOLONG_DESCRIPTION = tr_noop("This feature can only be used with sunnypilot longitudinal control enabled.")
ACC_PCMCRUISE_DISABLED_DESCRIPTION = tr_noop("This feature is not supported on this platform due to vehicle limitations.")
ONROAD_ONLY_DESCRIPTION = tr_noop("Start the vehicle to check vehicle compatibility.")


class CruiseLayout(Widget):
  def __init__(self):
    super().__init__()
    self._current_panel = PanelType.CRUISE
    self._speed_limit_layout = SpeedLimitSettingsLayout(lambda: self._set_current_panel(PanelType.CRUISE))

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):

    self.icbm_toggle = toggle_item_sp(
      title=tr("Intelligent Cruise Button Management (ICBM) (Alpha)"),
      description="",
      param="IntelligentCruiseButtonManagement")

    self.scc_v_toggle = toggle_item_sp(
      title=tr("Smart Cruise Control - Vision"),
      description=tr("Use vision path predictions to estimate the appropriate speed to drive through turns ahead."),
      param="SmartCruiseControlVision")

    self.scc_m_toggle = toggle_item_sp(
      title=tr("Smart Cruise Control - Map"),
      description=tr("Use map data to estimate the appropriate speed to drive through turns ahead."),
      param="SmartCruiseControlMap")

    self.custom_acc_toggle = toggle_item_sp(
      title=tr("Custom ACC Speed Increments"),
      description="",
      param="CustomAccIncrementsEnabled",
      callback=self._on_custom_acc_toggle)

    self.custom_acc_short_increment = option_item_sp(
      title=tr("Short Press Increment"),
      param="CustomAccShortPressIncrement",
      min_value=1, max_value=10, value_change_step=1,
      inline=True)

    self.custom_acc_long_increment = option_item_sp(
      title=tr("Long Press Increment"),
      param="CustomAccLongPressIncrement",
      value_map={1: 1, 2: 5, 3: 10},
      min_value=1, max_value=3, value_change_step=1,
      inline=True)

    self.sla_settings_button = simple_button_item_sp(
      button_text=lambda: tr("Speed Limit"),
      button_width=800,
      callback=lambda: self._set_current_panel(PanelType.SLA)
    )

    self.dec_toggle = toggle_item_sp(
      title=tr("Enable Dynamic Experimental Control"),
      description=tr("Enable toggle to allow the model to determine when to use sunnypilot ACC or sunnypilot End to End Longitudinal."),
      param="DynamicExperimentalControl")

    self.t_follow_toggle = toggle_item_sp(
      title=tr("Custom Follow Distance (T_FOLLOW)"),
      description=tr("Set a speed-dependent follow time per driving personality as \"mph:seconds\" pairs " +
                     "(e.g. 20:1.3, 40:1.4, 60:1.6). When off, the stock follow distances are used."),
      param="LongTFollowCustomEnabled",
      callback=self._refresh_t_follow)

    self.t_follow_mode = multiple_button_item_sp(
      title=lambda: tr("Follow Distance Mode"),
      description=lambda: tr("Simple: one follow time per personality. " +
                             "Advanced: speed-dependent \"mph:seconds\" curves."),
      buttons=[lambda: tr("Simple"), lambda: tr("Advanced")],
      param="LongTFollowMode",
      callback=self._refresh_t_follow,
      inline=False)

    # Simple mode: single follow-time slider per personality.
    self.t_follow_simple_rows = [
      self._make_t_follow_simple_row(tr("Relaxed"), "LongTFollowSimpleRelaxed"),
      self._make_t_follow_simple_row(tr("Standard"), "LongTFollowSimpleStandard"),
      self._make_t_follow_simple_row(tr("Aggressive"), "LongTFollowSimpleAggressive"),
    ]

    # Advanced mode: manual "mph:seconds" curve string per personality.
    self.t_follow_rows = [
      self._make_t_follow_row(tr("Relaxed"), "LongTFollowCurveRelaxed"),
      self._make_t_follow_row(tr("Standard"), "LongTFollowCurveStandard"),
      self._make_t_follow_row(tr("Aggressive"), "LongTFollowCurveAggressive"),
    ]

    items = [
      self.icbm_toggle,
      self.dec_toggle,
      self.t_follow_toggle,
      self.t_follow_mode,
      *self.t_follow_simple_rows,
      *self.t_follow_rows,
      self.scc_v_toggle,
      self.scc_m_toggle,
      self.custom_acc_toggle,
      self.custom_acc_short_increment,
      self.custom_acc_long_increment,
      self.sla_settings_button,
    ]
    return items

  def _render(self, rect):
    if self._current_panel == PanelType.SLA:
      self._speed_limit_layout.render(rect)
    else:
      self._scroller.render(rect)

  def show_event(self):
    self._set_current_panel(PanelType.CRUISE)
    self._scroller.show_event()
    self.icbm_toggle.show_description(True)
    self.custom_acc_toggle.show_description(True)

  def _set_current_panel(self, panel: PanelType):
    self._current_panel = panel
    if panel == PanelType.SLA:
      self._speed_limit_layout.show_event()

  def _update_state(self):
    super()._update_state()

    if ui_state.CP is not None and ui_state.CP_SP is not None:
      has_icbm = ui_state.has_icbm
      has_long = ui_state.has_longitudinal_control

      if ui_state.CP_SP.intelligentCruiseButtonManagementAvailable and not has_long:
        self.icbm_toggle.action_item.set_enabled(ui_state.is_offroad())
        self.icbm_toggle.set_description(tr(ICBM_DESC))
      else:
        ui_state.params.remove("IntelligentCruiseButtonManagement")
        self.icbm_toggle.action_item.set_enabled(False)

        long_desc = ICMB_UNAVAILABLE
        if has_long:
          if ui_state.CP.alphaLongitudinalAvailable:
            long_desc += " " + ICMB_UNAVAILABLE_LONG_AVAILABLE
          else:
            long_desc += " " + ICMB_UNAVAILABLE_LONG_UNAVAILABLE

        new_desc = "<b>" + tr(long_desc) + "</b>\n\n" + tr(ICBM_DESC)
        if self.icbm_toggle.description != new_desc:
          self.icbm_toggle.set_description(new_desc)
          self.icbm_toggle.show_description(True)

      if has_long or has_icbm:
        self.custom_acc_toggle.action_item.set_enabled(((has_long and not ui_state.CP.pcmCruise) or has_icbm) and ui_state.is_offroad())
        self.dec_toggle.action_item.set_enabled(has_long)
        self.scc_v_toggle.action_item.set_enabled(True)
        self.scc_m_toggle.action_item.set_enabled(True)
        self.t_follow_toggle.action_item.set_enabled(has_long)
        self.t_follow_mode.action_item.set_enabled(has_long)
        for row in self.t_follow_simple_rows:
          row.action_item.set_enabled(has_long)
      else:
        ui_state.params.remove("CustomAccIncrementsEnabled")
        ui_state.params.remove("DynamicExperimentalControl")
        ui_state.params.remove("SmartCruiseControlVision")
        ui_state.params.remove("SmartCruiseControlMap")
        ui_state.params.remove("LongTFollowCustomEnabled")
        self.custom_acc_toggle.action_item.set_enabled(False)
        self.dec_toggle.action_item.set_enabled(False)
        self.scc_v_toggle.action_item.set_enabled(False)
        self.scc_m_toggle.action_item.set_enabled(False)
        self.t_follow_toggle.action_item.set_enabled(False)
        self.t_follow_mode.action_item.set_enabled(False)
        for row in self.t_follow_simple_rows:
          row.action_item.set_enabled(False)

    else:
      has_icbm = has_long = False
      self.icbm_toggle.action_item.set_enabled(False)
      self.icbm_toggle.set_description(tr(ONROAD_ONLY_DESCRIPTION))

    show_custom_acc_desc = False

    if ui_state.is_offroad():
      new_custom_acc_desc = tr(ONROAD_ONLY_DESCRIPTION)
      show_custom_acc_desc = True
    else:
      if has_long or has_icbm:
        if has_long and ui_state.CP.pcmCruise:
          new_custom_acc_desc = tr(ACC_PCMCRUISE_DISABLED_DESCRIPTION)
          show_custom_acc_desc = True
        else:
          new_custom_acc_desc = tr(ACC_ENABLED_DESCRIPTION)
      else:
        new_custom_acc_desc = tr(ACC_NOLONG_DESCRIPTION)
        show_custom_acc_desc = True
        self.custom_acc_toggle.action_item.set_state(False)

    if self.custom_acc_toggle.description != new_custom_acc_desc:
      self.custom_acc_toggle.set_description(new_custom_acc_desc)
      if show_custom_acc_desc:
        self.custom_acc_toggle.show_description(True)

    self._on_custom_acc_toggle(self.custom_acc_toggle.action_item.get_state())
    self._refresh_t_follow()

  def _on_custom_acc_toggle(self, state):
    self.custom_acc_short_increment.set_visible(state)
    self.custom_acc_long_increment.set_visible(state)
    self.custom_acc_short_increment.action_item.set_enabled(self.custom_acc_toggle.action_item.enabled)
    self.custom_acc_long_increment.action_item.set_enabled(self.custom_acc_toggle.action_item.enabled)

  def _make_t_follow_simple_row(self, label, param):
    # use_float_scaling stores the param as seconds ("1.30"); the control works in hundredths
    # internally, so 0.80-3.00 s maps to 80-300 in steps of 10 (0.10 s).
    return option_item_sp(
      title=label,
      param=param,
      min_value=80, max_value=300, value_change_step=10,
      use_float_scaling=True,
      label_callback=lambda v: f"{v / 100:.2f}s",
      inline=True)

  def _make_t_follow_row(self, label, param):
    return button_item_sp(
      title=lambda: f"{label}: {ui_state.params.get(param, return_default=True)}",
      button_text=tr("Edit"),
      callback=lambda: self._edit_t_follow(label, param))

  def _edit_t_follow(self, label, param):
    def _on_confirm(result, text):
      if result != DialogResult.CONFIRM:
        return
      if parse_t_follow_curve(text) is None:
        gui_app.push_widget(alert_dialog(tr("Invalid curve. Use \"mph:seconds\" pairs with increasing speeds and 0.8-3.0s")))
      else:
        ui_state.params.put(param, text)

    dialog = InputDialogSP(
      title=f"{label} - T_FOLLOW",
      sub_title=tr("mph:seconds pairs, e.g. 20:1.3, 40:1.4, 60:1.6"),
      current_text=ui_state.params.get(param, return_default=True),
      callback=_on_confirm)
    dialog.show()

  def _refresh_t_follow(self, *_):
    enabled = bool(self.t_follow_toggle.action_item.get_state())
    advanced = int(ui_state.params.get("LongTFollowMode", return_default=True)) == 1
    self.t_follow_mode.set_visible(enabled)
    for row in self.t_follow_simple_rows:
      row.set_visible(enabled and not advanced)
    for row in self.t_follow_rows:
      row.set_visible(enabled and advanced)
