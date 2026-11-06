import time
from unittest import mock

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.modeld_v2 import big_model_loader
from openpilot.sunnypilot.modeld_v2.big_model_loader import LOAD_ATTEMPTS, LOAD_TIMEOUT, BigModelLoader, in_control

MOD = "openpilot.sunnypilot.modeld_v2.big_model_loader"


def make_sm(enabled=False, mads=False, alive=True, valid=True):
  sm = mock.MagicMock()
  sm.all_alive.return_value = alive
  sm.all_valid.return_value = valid
  sm.__getitem__.side_effect = lambda k: {'carControl': mock.Mock(enabled=enabled), 'carControlSP': mock.Mock(mads=mock.Mock(enabled=mads))}[k]
  return sm


class TestInControl(OpenpilotTestCase):
  def test_engaged_either_way(self):
    assert not in_control(make_sm())
    assert in_control(make_sm(enabled=True))
    assert in_control(make_sm(mads=True))

  def test_stale_services_count_as_in_control(self):
    assert in_control(make_sm(alive=False))
    assert in_control(make_sm(valid=False))


class TestBigModelLoader(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = Params()
    self.params.put_bool("ChestnutLoading", True, block=True)
    self.model = mock.Mock()
    self.state = mock.Mock(big=False, sends=7)
    p = mock.patch(f"{MOD}.wait_for_chestnut")
    self.wait = p.start()
    self.addCleanup(p.stop)

  def loader(self, build=None):
    loader = BigModelLoader(build or (lambda: self.model), self.params)
    loader.small = mock.Mock()
    return loader

  def update(self, loader, **kwargs):
    return loader.update(make_sm(**kwargs), self.state)

  def test_start_warms_the_small_model_then_loads(self):
    loader, small = self.loader(), mock.Mock()
    loader.thread = mock.Mock()
    with mock.patch("tinygrad.helpers.PARALLEL") as parallel:
      loader.start(small)
    small.warmup.assert_called_once()
    loader.thread.start.assert_called_once()
    assert parallel.value == 0

  def wait_for(self, key, value):
    # modeld writes these without blocking its frame
    deadline = time.monotonic() + 2.
    while self.params.get(key) != value and time.monotonic() < deadline:
      time.sleep(0.01)
    assert self.params.get(key) == value, key

  def test_swaps_only_when_nothing_is_in_control(self):
    loader = self.loader()
    loader._load()
    self.model.warmup.assert_called_once()
    assert self.params.get_bool("ChestnutBigReady")

    assert self.update(loader, enabled=True) is None
    assert self.params.get_bool("ChestnutLoading")
    loader.small.reset_history.assert_not_called()

    assert self.update(loader) is self.model
    loader.small.reset_history.assert_called_once()
    assert self.state.big and self.state.sends == 0
    self.wait_for("ChestnutBigReady", False)
    self.wait_for("ChestnutActive", True)
    self.wait_for("ChestnutLoading", False)

    assert self.update(loader) is None  # once

  def test_not_loaded_yet(self):
    assert self.update(self.loader()) is None
    assert self.params.get_bool("ChestnutLoading")
    assert self.params.get("ChestnutActive") is None

  def test_waits_out_the_gps_hold(self):
    self.params.put_bool("ChestnutGpsHoldActive", True, block=True)
    def sleep(_):
      assert self.wait.call_count == 0
      self.params.remove("ChestnutGpsHoldActive")
    with mock.patch(f"{MOD}.time.sleep", side_effect=sleep) as slept:
      self.loader()._load()
    slept.assert_called_once()
    self.wait.assert_called_once()

  def test_retries_once_then_stays_small(self):
    build = mock.Mock(side_effect=RuntimeError)
    loader = self.loader(build)
    loader._load()
    assert build.call_count == LOAD_ATTEMPTS

    assert self.update(loader) is None
    self.wait_for("ChestnutActive", False)
    self.wait_for("ChestnutLoading", False)
    assert not self.params.get_bool("ChestnutBigReady")
    assert not self.params.get_bool("OnroadCycleRequested")

  def test_retry_recovers(self):
    build = mock.Mock(side_effect=[RuntimeError, self.model])
    loader = self.loader(build)
    loader._load()
    assert self.update(loader) is self.model

  def test_a_wedged_load_times_out(self):
    loader = self.loader()
    loader.attempt_start = time.monotonic() - LOAD_TIMEOUT - 1
    assert self.update(loader) is None
    self.wait_for("ChestnutActive", False)
    self.wait_for("ChestnutLoading", False)

  def test_a_late_finish_after_giving_up_is_ignored(self):
    loader = self.loader()
    loader.attempt_start = time.monotonic() - LOAD_TIMEOUT - 1
    self.update(loader)
    self.wait_for("ChestnutLoading", False)
    loader._load()
    assert not self.params.get_bool("ChestnutBigReady")
    assert self.update(loader) is None

  def test_timeout_starts_after_the_hold(self):
    loader = self.loader()
    assert loader.attempt_start is None
    with mock.patch.object(big_model_loader.time, "monotonic", return_value=1e9):
      assert self.update(loader) is None
    assert self.params.get_bool("ChestnutLoading")
