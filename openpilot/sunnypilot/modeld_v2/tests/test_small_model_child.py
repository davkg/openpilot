import os
import signal
import subprocess
import sys
import time
from unittest import mock

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.modeld_v2.small_model_child import PARENT_ENV, SmallModelChild, in_control, is_small_child

MOD = "openpilot.sunnypilot.modeld_v2.small_model_child"


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


class TestIsSmallChild(OpenpilotTestCase):
  def test_modeld_itself(self):
    with mock.patch.dict(os.environ):
      os.environ.pop(PARENT_ENV, None)
      assert not is_small_child()

  def test_the_child(self):
    with mock.patch.dict(os.environ, {PARENT_ENV: str(os.getppid())}), mock.patch(f"{MOD}.ctypes.CDLL"):
      assert is_small_child()

  def test_exits_if_modeld_is_already_gone(self):
    with mock.patch.dict(os.environ, {PARENT_ENV: str(os.getppid() + 1)}), mock.patch(f"{MOD}.ctypes.CDLL"), self.assertRaises(SystemExit):
      is_small_child()


class TestSpawn(OpenpilotTestCase):
  def test_child_gets_its_parent_and_stays_off_the_egpu(self):
    with mock.patch(f"{MOD}.subprocess.Popen") as popen:
      SmallModelChild("some.module")
    args, kwargs = popen.call_args
    assert args[0] == [sys.executable, "-m", "some.module"]
    assert kwargs["env"][PARENT_ENV] == str(os.getpid())
    assert kwargs["env"]["DEV"] == "QCOM"

  def test_a_real_child_is_killed_on_hand_over(self):
    # a real process, end to end except for the module it runs
    params = Params()
    child = SmallModelChild.__new__(SmallModelChild)
    child.proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    self.addCleanup(child.proc.kill)
    with mock.patch(f"{MOD}.messaging.SubMaster", return_value=make_sm()):
      child.hand_over(params, True)
    assert child.proc.returncode == -signal.SIGKILL
    assert params.get_bool("ChestnutActive")


class TestHandOver(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = Params()
    self.params.put_bool("ChestnutLoading", True, block=True)
    self.child = SmallModelChild.__new__(SmallModelChild)
    self.child.proc = mock.Mock()
    self.child.proc.poll.return_value = None
    self.child.proc.wait.return_value = -9

  def wait_for(self, key, value):
    # written without blocking
    deadline = time.monotonic() + 2.
    while self.params.get(key) != value and time.monotonic() < deadline:
      time.sleep(0.01)
    assert self.params.get(key) == value, key

  def hand_over(self, sm, big_ready=True):
    with mock.patch(f"{MOD}.messaging.SubMaster", return_value=sm):
      self.child.hand_over(self.params, big_ready)

  def test_swaps_as_soon_as_nothing_is_in_control(self):
    self.hand_over(make_sm())
    self.child.proc.kill.assert_called_once()
    self.child.proc.wait.assert_called_once()
    assert self.params.get_bool("ChestnutActive")
    self.wait_for("ChestnutBigReady", False)

  def test_waits_while_in_control_then_swaps(self):
    sm = make_sm()

    def still_small(_):
      self.child.proc.kill.assert_not_called()
      assert self.params.get("ChestnutActive") is None
    sm.update.side_effect = still_small

    with mock.patch(f"{MOD}.in_control", side_effect=[True, True, False]):
      self.hand_over(sm)
    assert sm.update.call_count == 2
    self.child.proc.kill.assert_called_once()
    assert self.params.get_bool("ChestnutActive")

  def test_a_dead_child_takes_over_even_while_in_control(self):
    self.child.proc.poll.return_value = -11
    self.hand_over(make_sm(enabled=True))
    assert self.params.get_bool("ChestnutActive")

  def test_load_failure_stays_small_without_a_cycle(self):
    self.hand_over(make_sm(), big_ready=False)
    self.child.proc.kill.assert_not_called()
    self.child.proc.wait.assert_called_once()  # until the child dies, then modeld runs its own small model
    self.wait_for("ChestnutActive", False)
    self.wait_for("ChestnutLoading", False)
    assert not self.params.get_bool("OnroadCycleRequested")
