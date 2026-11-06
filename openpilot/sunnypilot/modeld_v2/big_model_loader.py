"""
Loads the big model behind a small model that is already driving, so a 3X has a model ~10 s after boot instead of
waiting out the chestnut GPS hold and a 28-45 s load. The swap happens only while nothing is in control: the models'
plans disagree, and a swap costs a frame or two and starts the big model from an empty history.
"""
import faulthandler
import os
import threading
import time
from collections.abc import Callable
from typing import Any

from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.helpers import wait_for_chestnut

LOAD_TIMEOUT = 60.  # per attempt, counted once the GPS hold has ended
LOAD_ATTEMPTS = 2
IN_CONTROL_SERVICES = ('carState', 'carControl', 'carControlSP')  # both modelds subscribe to all of these


def in_control(sm) -> bool:
  # a late or invalid service counts as in control, or a dead card would have the swap land on stale controls
  if not (sm.all_alive(list(IN_CONTROL_SERVICES)) and sm.all_valid(list(IN_CONTROL_SERVICES))):
    return True
  return bool(sm['carControl'].enabled or sm['carControlSP'].mads.enabled)


class BigModelLoader:
  def __init__(self, build: Callable, params: Params):
    self.build = build
    self.params = params
    self.small: Any = None
    self.model: Any = None
    self.failed = False
    self.done = False
    self.attempt_start: float | None = None
    self.thread = threading.Thread(target=self._load, daemon=True)

  def start(self, small) -> None:
    """Before modeld goes realtime: a thread started after inherits SCHED_FIFO on modeld's core and preempts the frame loop."""
    # TEMP bring-up: where routes 167/168's -11 dies (first small frames, stuck ~10 s). Drop this commit once parked tests are clean.
    if COMMA_HARDWARE:
      fault_log = open("/data/modeld_fault.log", "a")
      fault_log.write(f"--- modeld pid {os.getpid()}\n")
      fault_log.flush()
      faulthandler.enable(fault_log)
      faulthandler.dump_traceback_later(8, file=fault_log)

    self.small = small
    small.warmup()  # its first run stalls ~1 s
    from tinygrad.helpers import PARALLEL
    PARALLEL.value = 0  # no compile pool: it would fork from this process while the small model is running frames
    self.thread.start()

  def _load(self) -> None:
    params = Params()
    # the chestnut is unpowered during the boot GPS hold (chestnut_gps_hold), so loading now would time out
    while params.get_bool("ChestnutGpsHoldActive"):
      time.sleep(0.1)

    for attempt in range(LOAD_ATTEMPTS):
      self.attempt_start = time.monotonic()
      try:
        wait_for_chestnut()
        model = self.build()
        model.warmup()
        self.model = model
        if not self.done:
          params.put_bool("ChestnutBigReady", True, block=True)
        return
      except Exception:
        cloudlog.exception(f"big model load failed, attempt {attempt + 1}/{LOAD_ATTEMPTS}")
    self.failed = True

  def update(self, sm, chestnut_state) -> Any:
    """Once a frame, on modeld's thread: the big model on the frame it should take over, else None."""
    if self.done:
      return None

    timed_out = self.model is None and self.attempt_start is not None and time.monotonic() - self.attempt_start > LOAD_TIMEOUT
    if self.failed or timed_out:
      # the small model drives on; no onroad cycle, it would restart the stack mid-drive
      cloudlog.error(f"big model load gave up, staying on the small model (timed out: {timed_out})")
      self.done = True
      self.params.put_bool("ChestnutActive", False)
      self.params.put_bool("ChestnutLoading", False)
      return None

    if self.model is None or in_control(sm):
      return None

    cloudlog.warning("swapping to the big model")
    self.done = True
    self.params.put_bool("ChestnutBigReady", False)
    self.params.put_bool("ChestnutActive", True)
    self.params.put_bool("ChestnutLoading", False)
    self.small.reset_history()  # a later fallback must not resume from a history minutes old
    chestnut_state.big = True
    chestnut_state.sends = 0  # read the metrics on the next send: without them chestnutGpuState is invalid
    return self.model
