"""
Runs the small model in a child process while modeld loads the big model, so a 3X has a model ~10 s after boot instead of
waiting out the chestnut GPS hold and a 28-45 s load. The load can't share a process with the frame loop: it stalled frames
~2 s on Python's lock, and tinygrad's process-wide Context flags made the frame raise (route 00000187--c14022e911).

modeld stops the child and takes over only while nothing is in control: the models' plans disagree, and the swap costs a few
frames and starts the big model from an empty history.
"""
import ctypes
import os
import signal
import subprocess
import sys

from openpilot.cereal import messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

PARENT_ENV = "MODELD_SMALL_CHILD_OF"
PR_SET_PDEATHSIG = 1
IN_CONTROL_SERVICES = ('carState', 'carControl', 'carControlSP')


def in_control(sm) -> bool:
  # a late or invalid service counts as in control, or a dead card would have the swap land on stale controls
  if not (sm.all_alive(list(IN_CONTROL_SERVICES)) and sm.all_valid(list(IN_CONTROL_SERVICES))):
    return True
  return bool(sm['carControl'].enabled or sm['carControlSP'].mads.enabled)


def is_small_child() -> bool:
  """Called first in modeld. In the child, also ties its life to modeld's: two processes must never publish modelV2."""
  parent = os.environ.get(PARENT_ENV)
  if parent is None:
    return False
  if sys.platform == "linux":
    ctypes.CDLL(None).prctl(PR_SET_PDEATHSIG, signal.SIGKILL)
  if os.getppid() != int(parent):  # modeld died before the prctl
    sys.exit(0)
  return True


class SmallModelChild:
  def __init__(self, module: str):
    # DEV: never probe the eGPU, modeld owns it (its USB lock)
    env = os.environ | {PARENT_ENV: str(os.getpid()), "DEV": "QCOM"}
    self.proc = subprocess.Popen([sys.executable, "-m", module], cwd=BASEDIR, env=env)

  def hand_over(self, params: Params, big_ready: bool) -> None:
    """Blocks until modeld should run the frame loop itself. The small model drives on without a big model, as an onroad
    cycle would restart the stack mid-drive: modeld only takes over if the child dies."""
    if big_ready:
      self._wait_until_free(params)
      # while the small model still runs: selfdrived reads the swap from these, and holds off the errors its gap in modelV2 raises.
      # A write can take over a second on a drive (route 0000018a--7641dcb651), so neither can wait for after the kill
      params.put_bool("ChestnutActive", True, block=True)
      params.put_bool("ChestnutLoading", False, block=True)
      self.proc.kill()
    else:
      cloudlog.error("big model load gave up, staying on the small model")
      params.put_bool("ChestnutActive", False)
      params.put_bool("ChestnutLoading", False)

    cloudlog.warning(f"small model child exited: {self.proc.wait()}")

  def _wait_until_free(self, params: Params) -> None:
    sm = messaging.SubMaster(list(IN_CONTROL_SERVICES))
    params.put_bool("ChestnutBigReady", True)
    while self.proc.poll() is None and in_control(sm):
      sm.update(100)
    params.put_bool("ChestnutBigReady", False)
