from openpilot.cereal import log

from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle, BigMultiToggle, BigToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.mici.layouts.settings.initial_cruise_speed import InitialCruiseSpeedButton

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

# Coarse presets for the lane centering settings. These params are floats shared with the 3X menu and
# sunnylink, which both offer the full range, so a value set there snaps to the nearest preset here.
LANE_CENTER_OFFSET = ((-0.1, "-0.1m"), (0.0, "0m"), (0.1, "+0.1m"))
LANE_CENTERING_E2E_AUTHORITY = ((0.0, "0%"), (0.5, "50%"), (1.0, "100%"))


def _nearest_label(presets, value) -> str:
  return min(presets, key=lambda p: abs(p[0] - float(value)))[1]


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._personality_toggle = BigMultiParamToggle("following distance", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"],
                                                   description="Standard is recommended.\n" +
                                                               "Aggressive follows closer, with firmer gas and braking.\n" +
                                                               "Relaxed leaves more space.\n" +
                                                               "Use the steering wheel distance button on supported cars.")
    self._initial_cruise_speed = InitialCruiseSpeedButton()
    self._initial_cruise_speed.set_visible(lambda: ui_state.CP is not None and ui_state.has_longitudinal_control and not ui_state.CP.pcmCruise)
    is_metric_toggle = BigToggle("use metric units", initial_state=ui_state.params.get_bool("IsMetric"), toggle_callback=self._on_metric_toggle)
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled",
                                 description="Warn when you drift across a detected lane line.\n" +
                                             "Only above 31 mph (50 km/h), with no turn signal.")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM", description="Monitor the driver even when sunnypilot is not engaged.")
    record_front = BigParamControl("record & upload cabin camera", "RecordFront",
                                   description_icon=gui_app.texture("icons_mici/settings/device/cameras.png", 64, 64),
                                   toggle_callback=restart_needed_callback, description="Upload cabin camera data to help improve driver monitoring.")
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", description_icon=gui_app.texture("icons_mici/microphone.png", 64, 64),
                                 toggle_callback=restart_needed_callback,
                                 description="Record microphone audio while driving.\n" +
                                             "Audio is included in dashcam videos in sunnylink.")
    self._long_active_with_gas = BigParamControl("long active with accel pedal", "LongitudinalActiveWithGas",
                                                 toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable sunnypilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback,
                                       description="Enable to use sunnypilot driver assistance.\n" +
                                                   "Disable to use your car's stock driver assistance.")


    self._lane_centering_toggle = BigParamControl("lane centering", "LaneCentering",
                                                  toggle_callback=self._update_lane_centering_visible)
    self._lane_centering_pause_toggle = BigToggle("pause with blinker",
                                                  initial_state=bool(ui_state.params.get("LaneCenteringPauseOnSignal", return_default=True)),
                                                  toggle_callback=self._on_lane_centering_pause_on_signal)
    self._lane_center_offset_toggle = BigMultiToggle("center offset", [label for _, label in LANE_CENTER_OFFSET],
                                                     select_callback=self._on_lane_center_offset)
    self._lane_centering_e2e_authority_toggle = BigMultiToggle("e2e override", [label for _, label in LANE_CENTERING_E2E_AUTHORITY],
                                                               select_callback=self._on_lane_centering_e2e_authority)

    self._scroller.add_widgets([
      self._personality_toggle,
      self._initial_cruise_speed,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      self._long_active_with_gas,
      self._lane_centering_toggle,
      self._lane_centering_pause_toggle,
      self._lane_center_offset_toggle,
      self._lane_centering_e2e_authority_toggle,
      enable_openpilot,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("LongitudinalActiveWithGas", self._long_active_with_gas),
      ("OpenpilotEnabledToggle", enable_openpilot),
      ("LaneCentering", self._lane_centering_toggle),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))
    record_mic.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _on_metric_toggle(self, state: bool):
    ui_state.params.put_bool("IsMetric", state, block=True)
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    self._initial_cruise_speed.refresh()

    if ui_state.CP is not None:
      # replaced by the button-stepped follow time (LongFollowTime)
      self._personality_toggle.set_visible(False)
      self._long_active_with_gas.set_visible(ui_state.has_longitudinal_control)
      if not ui_state.has_longitudinal_control:
        ui_state.params.remove("LongitudinalActiveWithGas")

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

    # these three aren't param-bound (bool default is on, the other two map floats to presets)
    self._lane_centering_pause_toggle.set_checked(bool(ui_state.params.get("LaneCenteringPauseOnSignal", return_default=True)))
    self._lane_center_offset_toggle.set_value(
      _nearest_label(LANE_CENTER_OFFSET, ui_state.params.get("LaneCenterOffset", return_default=True)))
    self._lane_centering_e2e_authority_toggle.set_value(
      _nearest_label(LANE_CENTERING_E2E_AUTHORITY, ui_state.params.get("LaneCenteringE2EAuthority", return_default=True)))
    self._update_lane_centering_visible(ui_state.params.get_bool("LaneCentering"))

  def _update_lane_centering_visible(self, enabled: bool):
    self._lane_centering_pause_toggle.set_visible(enabled)
    self._lane_center_offset_toggle.set_visible(enabled)
    self._lane_centering_e2e_authority_toggle.set_visible(enabled)

  @staticmethod
  def _on_lane_centering_pause_on_signal(state: bool):
    ui_state.params.put_bool("LaneCenteringPauseOnSignal", state, block=True)

  @staticmethod
  def _on_lane_center_offset(label: str):
    ui_state.params.put("LaneCenterOffset", next(v for v, lb in LANE_CENTER_OFFSET if lb == label), block=True)

  @staticmethod
  def _on_lane_centering_e2e_authority(label: str):
    ui_state.params.put("LaneCenteringE2EAuthority", next(v for v, lb in LANE_CENTERING_E2E_AUTHORITY if lb == label), block=True)
