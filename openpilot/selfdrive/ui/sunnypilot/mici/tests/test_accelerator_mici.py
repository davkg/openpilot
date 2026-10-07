"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The mici models panel's Jetlink setting: the provisioning line, the icon state,
the link toggle and the accelerator's default big model. From zoompilot's
test_mici_settings.py, converted from pytest to unittest.
"""
import os
import unittest
from unittest import mock

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")

from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.test import OpenpilotTestCase

# the window's own params, for whatever it reads while it comes up; every test
# then runs under its own prefix
_window_prefix = OpenpilotPrefix()


def setUpModule():
  """A hidden raylib window, once for the module: widgets need textures."""
  import pyray as rl
  from openpilot.system.ui.lib.application import gui_app
  _window_prefix.__enter__()
  rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
  gui_app.init_window("test_accelerator_mici", fps=30)


def tearDownModule():
  from openpilot.system.ui.lib.application import gui_app
  gui_app.close()
  _window_prefix.__exit__(None, None, None)


class MiciTest(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    from openpilot.common.params import Params
    from openpilot.selfdrive.ui.ui_state import ui_state
    self.params = Params()
    ui_state.params = self.params


def jetlink_status(**fields):
  """jetlink's snapshot as the UI's params pass takes it, nothing to show unless a field says so."""
  from jetlink.openpilot import Status
  base = {'enabled': False, 'mode': 'off', 'transport': 'USB', 'present': False, 'port': None, 'ready': False,
          'reason': None, 'progress': None, 'model': None, 'default_model': None}
  return Status(**{**base, **fields})


def render(widget):
  """Drive one frame through Widget.render, which calls _update_state."""
  import pyray as rl
  widget.render(rl.Rectangle(0, 0, 800, 600))


class TestAcceleratorProgressRenders(MiciTest):
  """the models panel's provisioning line; the layout sweep above runs with no progress set"""

  STAGES = ['download', 'connect', 'upload', 'build', 'failed']

  def _info(self, stage=None, frac=0.0, msg='', drops=0, mode='usb', **fields):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.jetlink
    progress = {'stage': stage, 'frac': frac, 'msg': msg, 'drops': drops} if stage else None
    ui_state.jetlink = jetlink_status(present=True, mode=mode, progress=progress, **fields)
    try:
      from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import _model_info
      return _model_info()
    finally:
      ui_state.jetlink = saved

  def test_every_stage_gives_a_line(self):
    params = self.params
    for stage in self.STAGES:
      with self.subTest(stage=stage):
        self._test_every_stage_gives_a_line(params, stage)

  def _test_every_stage_gives_a_line(self, params, stage):
    active, header, info = self._info(stage, 0.42)
    assert active and header and info

  def test_a_percentage_is_shown_while_working(self):
    _, _, info = self._info('download', 0.42)
    assert '42%' in info

  def test_the_line_is_jetlinks_message(self):
    cases = [
      # a join has nothing to measure, and "getting ready" alone does not separate an
      # unplugged Jetson from one six seconds from ready
      ('connect', 0.0, 'waiting for jetlink', 0, 'usb', 'waiting for jetlink'),
      ('download', 0.45, 'downloading', 0, 'usb', 'downloading 45%'),
      # jetlink counts the drops; the card names the cable, and on iOS the phone app too
      ('connect', 0.0, 'reconnecting', 2, 'usb', 'reconnecting, check cable (2 drops)'),
      ('connect', 0.0, 'waiting for jetlink', 3, 'ios', 'waiting for jetlink, check cable or app (3 drops)'),
    ]
    for stage, frac, msg, drops, mode, shown in cases:
      with self.subTest(msg=msg, drops=drops, mode=mode):
        assert self._info(stage, frac, msg, drops, mode)[2] == shown

  def test_the_stand_in_is_named_while_the_pick_is_not_ready(self):
    assert self._info(enabled=True, model='ResAction Preview', standin='Cinque Terre V3')[1:] == \
      ('big model', 'cinque terre v3 for now')

  def test_failure_says_so_rather_than_showing_100_percent(self):
    _, _, info = self._info('failed', 1.0)
    assert '100%' not in info

  def test_ready_falls_back_to_the_normal_line(self):
    # 'ready' is the steady state: the panel must go back to naming the model,
    # not sit on a finished progress bar forever.
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts import models as models_layout
    ready = self._info('ready', 1.0)
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.jetlink
    ui_state.jetlink = jetlink_status(present=True)
    try:
      assert ready == models_layout._model_info()
    finally:
      ui_state.jetlink = saved

  def test_the_panel_draws_with_progress_set(self):
    params = self.params
    for stage in self.STAGES:
      with self.subTest(stage=stage):
        self._test_the_panel_draws_with_progress_set(params, stage)

  def _test_the_panel_draws_with_progress_set(self, params, stage):
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici

    saved = ui_state.jetlink
    ui_state.jetlink = jetlink_status(present=True, progress={'stage': stage, 'frac': 0.5, 'msg': ''})
    try:
      layout = ModelsLayoutMici()
      render(layout)
      render(layout)
      for item in layout._scroller.items:
        render(item)
      render(layout.current_model_info)
    finally:
      ui_state.jetlink = saved


class TestAcceleratorIconState(MiciTest):
  """chestnut_state for an off-board accelerator comes from jetlink's snapshot and
  modeld's acceleratorState, not a USB id the comma never enumerates. a fitted
  chestnut keeps upstream's path"""

  class FakeSM:
    def __init__(self, big=False, alive=False, recv=0):
      self.recv_frame = {"modelV2": recv}
      self.alive = {"modelV2": alive}
      self.big = big

    def __getitem__(self, name):
      if name == "deviceState":
        return type("DS", (), {"chestnutPresent": False})()
      assert name == "modelV2"
      return type("M", (), {"big": self.big})()

  @staticmethod
  def _view(present=True, ready=False, progress=None, state='none'):
    """The link on, and modeld's acceleratorState by name."""
    return jetlink_status(enabled=True, mode='usb', present=present, ready=ready, progress=progress), state

  def _state(self, view, sm=None, started=False):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state._accelerator_state_name
    ui_state.sm, ui_state.started, ui_state.started_frame = sm or self.FakeSM(), started, 0  # ty: ignore[invalid-assignment]
    ui_state.jetlink, ui_state._accelerator_state_name = view
    try:
      ui_state._update_chestnut_state()
      return ui_state.chestnut_state
    finally:
      ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state._accelerator_state_name = saved

  def test_offroad_states(self):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    assert self._state(self._view(present=False)) == ChestnutState.DISCONNECTED
    assert self._state(self._view(ready=True)) == ChestnutState.READY
    assert self._state(self._view()) == ChestnutState.UNCOMPILED
    assert self._state(self._view(progress={'stage': 'build', 'frac': 0.3})) == ChestnutState.LOADING
    assert self._state(self._view(progress={'stage': 'failed', 'frac': 1.0})) == ChestnutState.FAILED
    assert self._state(self._view(ready=True, progress={'stage': 'connect', 'frac': 0.0})) == ChestnutState.LOADING
    assert self._state(self._view(ready=True, progress={'stage': 'failed', 'frac': 1.0})) == ChestnutState.FAILED
    assert self._state(self._view(ready=True, progress={'stage': 'ready', 'frac': 1.0})) == ChestnutState.READY

  def test_absent_accelerator_does_not_pulse_onroad(self):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    view = self._view(present=False, ready=True, state='retrying')
    assert self._state(view, self.FakeSM(alive=True, recv=1), started=True) == ChestnutState.DISCONNECTED

  def test_onroad_states(self):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    driving = self.FakeSM(alive=True, recv=1)
    assert self._state(self._view(ready=True, state='joining'), driving, started=True) == ChestnutState.LOADING
    assert self._state(self._view(ready=True, state='retrying'), driving, started=True) == ChestnutState.LOADING
    assert self._state(self._view(ready=True, state='running'), driving, started=True) == ChestnutState.ACTIVE
    assert self._state(self._view(ready=True, state='unavailable'), driving, started=True) == ChestnutState.FAILED
    assert self._state(self._view(ready=False, state='none'), driving, started=True) == ChestnutState.UNCOMPILED
    # nothing from modeld yet is loading, not failed
    assert self._state(self._view(ready=True), self.FakeSM(), started=True) == ChestnutState.LOADING
    # a big frame is proof, whatever the status field says
    big = self.FakeSM(big=True, alive=True, recv=1)
    assert self._state(self._view(ready=True, state='unavailable'), big, started=True) == ChestnutState.ACTIVE


  @staticmethod
  def _usb(present):
    from contextlib import ExitStack
    from unittest import mock
    from openpilot.selfdrive.ui import ui_state as module
    stack = ExitStack()
    stack.enter_context(mock.patch.object(module, 'read_int', return_value=1))
    stack.enter_context(mock.patch.object(module, 'get_usb_state', return_value=[]))
    stack.enter_context(mock.patch("openpilot.sunnypilot.jetlink_adapter.status", return_value=jetlink_status(present=present)))
    return stack

  def test_a_present_accelerator_is_not_an_unknown_usb_device(self):
    import time
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink
    try:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown = True, time.monotonic() - 11.0, False
      with self._usb(present=True):
        ui_state.update_params()  # builds the view
        ui_state.usb_connected_ts = time.monotonic() - 11.0
        ui_state.update_params()  # decides
      assert ui_state.jetlink_view is not None
      assert ui_state.usb_unknown is False
      with self._usb(present=False):
        ui_state.update_params()
        ui_state.usb_connected_ts = time.monotonic() - 11.0
        ui_state.update_params()
      assert ui_state.jetlink_view is None
      assert ui_state.usb_unknown is True
    finally:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink = saved

  def test_an_accelerator_recognised_after_the_grace_period_clears_unknown(self):
    """the Jetson configures the gadget ~25 s after the UI starts, after the one-shot
    usb_unknown decision; presence arriving later must still clear it"""
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink
    try:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown = True, None, True
      with self._usb(present=False):
        ui_state.update_params()
        assert ui_state.usb_unknown is True
      with self._usb(present=True):
        ui_state.update_params()
        assert ui_state.usb_unknown is False
    finally:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink = saved


class TestAcceleratorLinkToggle(MiciTest):
  """off, usb or ios in one param, and the control is hidden on a device it means nothing to"""

  PARAM = "JetlinkLink"

  @staticmethod
  def _accelerators(installed=False, **fields):
    """ui_state's jetlink snapshot for the block: None, no jetlink here, unless
    it is installed or a field says there is something to show."""
    from unittest import mock
    from openpilot.selfdrive.ui.ui_state import ui_state
    status = jetlink_status(**fields) if installed or any(fields.values()) else None
    return mock.patch.object(ui_state, "jetlink", status)

  def _meaningful(self, **accelerators) -> bool:
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import link_toggle_meaningful
    with self._accelerators(**accelerators):
      return link_toggle_meaningful()

  def test_hidden_on_a_plain_device(self):
    params = self.params
    params.remove(self.PARAM)
    assert not self._meaningful()

  def test_shown_when_an_accelerator_is_attached(self):
    params = self.params
    params.remove(self.PARAM)
    assert self._meaningful(present=True)

  def test_shown_wherever_the_package_is_installed(self):
    params = self.params
    # with the link off there is no gadget for a Jetson to enumerate, so present()
    # alone would hide the toggle that turns the link on
    params.remove(self.PARAM)
    assert self._meaningful(installed=True)

  def test_the_value_line_says_what_each_mode_is_for(self):
    params = self.params
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    assert toggle.get_value() == "off"
    for index, value in enumerate(("off", "usb", "iOS")):
      params.put(self.PARAM, index, block=True)
      toggle.refresh()
      assert toggle.get_value() == value

  def test_shown_when_ready_with_the_hardware_out_of_the_car(self):
    params = self.params
    # the engine is cached and the link may be on, so modeld will still try it at
    # the next ignition. this is the case the off position exists for
    params.remove(self.PARAM)
    assert self._meaningful(ready=True)

  def test_shown_when_the_backend_has_a_complaint(self):
    params = self.params
    params.remove(self.PARAM)
    assert self._meaningful(reason="no gadget")

  def test_shown_once_the_user_has_turned_it_on(self):
    params = self.params
    params.put(self.PARAM, 1, block=True)
    assert self._meaningful()

  def test_hidden_when_off_with_nothing_attached(self):
    params = self.params
    params.put(self.PARAM, 0, block=True)
    assert not self._meaningful()

  def test_absent_reads_as_off(self):
    params = self.params
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import link_mode

    params.remove(self.PARAM)
    assert link_mode() == "off"
    for index, mode in enumerate(("off", "usb", "ios")):
      params.put(self.PARAM, index, block=True)
      assert link_mode() == mode

  def test_tap_cycles_off_usb_ios(self):
    params = self.params
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle
    from openpilot.system.ui.lib.application import MousePos

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    assert toggle._mode == "off"
    # the small model is the model manager's: the toggle never touches the runner cache
    with mock.patch.object(params, "remove", wraps=params.remove) as remove, \
         mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=True):
      for index in (1, 2, 0):
        toggle._handle_mouse_release(MousePos(0, 0))
        assert params.get(self.PARAM) == index
    assert "ModelRunnerTypeCache" not in {c.args[0] for c in remove.call_args_list}

  def test_refresh_follows_the_param(self):
    params = self.params
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    params.put(self.PARAM, 2, block=True)
    toggle.refresh()
    assert toggle._mode == "ios"

  def test_link_toggle_cannot_change_after_ignition(self):
    params = self.params
    from unittest import mock
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle
    from openpilot.system.ui.lib.application import MousePos

    params.put(self.PARAM, 2, block=True)
    toggle = AcceleratorLinkToggle()
    with mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=False):
      # drawn disabled, as the model buttons beside it are onroad: a refused
      # tap used to animate with nothing changing
      assert not toggle.enabled
      render(toggle)
      toggle._handle_mouse_release(MousePos(0, 0))
      assert params.get(self.PARAM) == 2
      assert toggle._mode == "ios", "the pills must not show a mode the param does not have"
    with mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=True):
      assert toggle.enabled
      toggle._handle_mouse_release(MousePos(0, 0))
      assert params.get(self.PARAM) == 0

  def test_layout_hides_the_toggle_until_it_means_something(self):
    params = self.params
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici

    params.remove(self.PARAM)
    with self._accelerators():
      layout = ModelsLayoutMici()
      assert not layout.link_toggle.is_visible
      assert layout.link_toggle in layout._scroller.items
    with self._accelerators(present=True):
      layout = ModelsLayoutMici()
      assert layout.link_toggle.is_visible
      render(layout)
      render(layout)
      render(layout.link_toggle)

  def test_refresh_spins_beside_the_toggle_until_both_catalogs_are_stamped(self):
    params = self.params
    # sunnypilot's refresh, as upstream: the model manager restamps each catalog it
    # refetches, the big-model one extended for the accelerator or not
    import time
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
    from openpilot.selfdrive.ui.sunnypilot.model_info import MODEL_SYNC_KEYS

    params.put(self.PARAM, 1, block=True)
    for key in MODEL_SYNC_KEYS:
      params.put(key, 1, block=True)
    with self._accelerators(present=True), \
         mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=True):
      layout = ModelsLayoutMici()
      button = layout.refresh_btn

      def shown():
        render(layout)
        return button.get_value(), button.enabled

      assert layout.link_toggle.is_visible and button in layout._scroller.items
      assert shown() == ("", True)
      layout._refresh_models()
      deadline = time.monotonic() + 2.0
      while any(params.get(key) for key in MODEL_SYNC_KEYS):
        assert time.monotonic() < deadline, "the refresh never zeroed the sync keys"
        time.sleep(0.005)
      assert shown() == ("fetching...", False)
      params.put(MODEL_SYNC_KEYS[0], 2, block=True)
      assert shown() == ("fetching...", False)
      params.put(MODEL_SYNC_KEYS[1], 2, block=True)
      assert shown() == ("", True)


class TestDefaultBigModelMici(MiciTest):
  """The big-models button names whose default an empty slot runs: the chestnut's
  model in the tree when a board is fitted, else the accelerator's."""

  def _big_models_value(self, board):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import gui_app

    pushed = []
    with mock.patch.object(gui_app, "push_widget", lambda w: pushed.append(w)), \
         mock.patch.object(ui_state, "chestnut_present", board), \
         mock.patch.object(ui_state, "jetlink", jetlink_status(present=True, default_model="Cinque Terre V3 Model")):
      layout = ModelsLayoutMici()
      render(layout)
      layout._show_folders()
      buttons = pushed[-1]._scroller.items
      for button in buttons:
        render(button)
    return buttons[1].get_value().removesuffix(" (active)")

  def test_a_fitted_chestnut_names_the_in_tree_model(self):
    params = self.params
    from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL
    params.remove("ModelManager_ActiveBundleChestnut")
    assert self._big_models_value(True) == f"{DEFAULT_BIG_MODEL} (Default)".lower()

  def test_without_a_chestnut_the_accelerator_names_its_default(self):
    params = self.params
    params.remove("ModelManager_ActiveBundleChestnut")
    assert self._big_models_value(False) == "cinque terre v3 model (default)"


if __name__ == '__main__':
  unittest.main()
