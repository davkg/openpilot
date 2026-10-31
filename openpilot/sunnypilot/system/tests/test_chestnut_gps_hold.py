import importlib.util
import os
import re
from pathlib import Path
from unittest import mock

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.system import chestnut_gps_hold
from openpilot.sunnypilot.system.chestnut_gps_hold import DRIVE_TIMEOUT, GpsHold

MOD = "openpilot.sunnypilot.system.chestnut_gps_hold"


class TestGpsHold(OpenpilotTestCase):
  def test_fix_ends_hold(self):
    assert GpsHold().done(True, "park", 0.)

  def test_parked_never_times_out(self):
    hold = GpsHold()
    assert not any(hold.done(False, gear, t) for t in range(0, 3600, 10) for gear in ("park", "reverse", "neutral", "unknown"))

  def test_timeout_counts_from_first_forward_gear(self):
    hold = GpsHold()
    assert not hold.done(False, "park", 0.)
    assert not hold.done(False, "drive", 500.)
    assert not hold.done(False, "park", 500. + DRIVE_TIMEOUT - 1)  # keeps counting once started
    assert hold.done(False, "park", 500. + DRIVE_TIMEOUT)


class TestHoldDaemon(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = Params()
    self.params.put_bool("ChestnutGpsHold", True, block=True)
    self.writes = []
    patches = [
      mock.patch(f"{MOD}.HARDWARE.get_device_type", return_value="tizi"),
      mock.patch(f"{MOD}.cable_connected", return_value=True),
      mock.patch(f"{MOD}.chestnut_compiled", return_value=True),
      mock.patch(f"{MOD}.sudo_write", side_effect=lambda val, path: self.writes.append(val)),
      mock.patch(f"{MOD}.sudo_read", return_value=""),
      mock.patch(f"{MOD}.signal.signal"),
    ]
    for p in patches:
      p.start()
      self.addCleanup(p.stop)

  def _sm(self, update):
    sm = mock.MagicMock()
    sm.__getitem__.side_effect = lambda k: sm.data[k]
    sm.data = {'gpsLocation': mock.Mock(hasFix=False), 'carState': mock.Mock(gearShifter="park")}
    sm.update.side_effect = lambda *_: update(sm)
    return sm

  def test_holds_until_fix_then_restores(self):
    def update(sm):
      assert self.params.get_bool("ChestnutGpsHoldActive")
      sm.data['gpsLocation'].hasFix = True
    with mock.patch(f"{MOD}.messaging.SubMaster", return_value=self._sm(update)):
      chestnut_gps_hold.main()
    assert self.writes == ["0", "1"]
    assert not self.params.get_bool("ChestnutGpsHoldActive")

  def test_restores_on_error(self):
    def update(sm):
      raise RuntimeError
    with mock.patch(f"{MOD}.messaging.SubMaster", return_value=self._sm(update)), self.assertRaises(RuntimeError):
      chestnut_gps_hold.main()
    assert self.writes == ["0", "1"]
    assert not self.params.get_bool("ChestnutGpsHoldActive")

  def test_skipped_when_toggled_off(self):
    self.params.put_bool("ChestnutGpsHold", False, block=True)
    chestnut_gps_hold.main()
    assert self.writes == []
    assert not self.params.get_bool("ChestnutGpsHoldActive")

  def test_skipped_without_chestnut_cable(self):
    with mock.patch(f"{MOD}.cable_connected", return_value=False):
      chestnut_gps_hold.main()
    assert self.writes == []


OPENPILOT = Path(__file__).resolve().parents[3]
CHESTNUT_CHECK = re.compile(r"\bchestnut_present\(\)|\bchestnutPresent\b")
# Every enumerated-only chestnut check (chestnut_present() / deviceState.chestnutPresent), reviewed against the hold:
# while it runs, a cabled chestnut is unpowered and reads absent. A check that decides something for the drive must also
# count cable_connected(), as get_active_source and the modelds' CHESTNUT do.
REVIEWED_CHESTNUT_CHECKS = {
  "common/hardware/usb.py": 1,  # publishes deviceState.chestnutPresent
  "selfdrive/modeld/helpers.py": 3,  # the def; wait_for_chestnut (after the hold); reproject_expected (or cable_connected)
  "selfdrive/modeld/modeld.py": 1,  # CHESTNUT (or cable_connected)
  "selfdrive/selfdrived/selfdrived.py": 1,  # bigModelFailed, only once the big model was active
  "selfdrive/ui/ui_state.py": 2,  # display; onroad also counts usb_connected
  "sunnypilot/modeld_v2/modeld.py": 1,  # CHESTNUT (or cable_connected)
  "sunnypilot/models/fetcher.py": 1,  # catalog refresh slot, display only
  "sunnypilot/models/helpers.py": 2,  # get_active_source, _tizi_with_chestnut (both or cable_connected)
  "sunnypilot/models/manager.py": 1,  # model manager slot, display only
}


class TestUpstreamSeams(OpenpilotTestCase):
  """Tripwires for upstream merges: the hold leans on code it doesn't own."""

  def test_chestnut_checks_reviewed(self):
    found = {}
    for d, dirs, files in os.walk(OPENPILOT):
      dirs[:] = [x for x in dirs if x not in ("tests", "test", "third_party", "__pycache__")]
      for f in files:
        if f.endswith(".py") and (n := len(CHESTNUT_CHECK.findall((Path(d) / f).read_text()))):
          found[str((Path(d) / f).relative_to(OPENPILOT))] = n
    changed = {k: (REVIEWED_CHESTNUT_CHECKS.get(k), found.get(k)) for k in found.keys() | REVIEWED_CHESTNUT_CHECKS.keys()
               if found.get(k) != REVIEWED_CHESTNUT_CHECKS.get(k)}
    assert not changed, f"chestnut checks changed (reviewed, now), review them against the hold: {changed}"

  def test_held_chestnut_is_the_active_slot(self):
    from openpilot.sunnypilot.models.helpers import get_active_source
    with (mock.patch("openpilot.sunnypilot.models.helpers.chestnut_present", return_value=False),
          mock.patch("openpilot.sunnypilot.models.helpers.cable_connected", return_value=True)):
      assert get_active_source() == "chestnut"

  def test_modelds_wait_out_the_hold(self):
    for path in ("selfdrive/modeld/modeld.py", "sunnypilot/modeld_v2/modeld.py"):
      src = (OPENPILOT / path).read_text()
      assert 'get_bool("ChestnutGpsHoldActive")' in src and "wait_for_chestnut()" in src, path

  def test_tinygrad_usb_scan_is_clearable(self):
    # wait_for_chestnut clears this cached scan, the one the AMD USB device opens from
    from tinygrad.runtime.support.usb import USB3
    assert hasattr(USB3.list_devices, "cache_clear")
    assert "USB3.list_devices(" in Path(importlib.util.find_spec("tinygrad.runtime.ops_amd").origin).read_text()

  def test_qcomgpsd_runs_offroad_during_the_hold(self):
    from openpilot.system.manager import process_config
    self.params = Params()
    self.params.put_bool("ChestnutGpsHoldActive", True, block=True)
    with mock.patch.object(process_config, "ublox_available", return_value=False):
      assert process_config.qcomgps(False, self.params, None)
    assert "chestnut_gps_hold" in process_config.managed_processes

  def test_selfdrived_ignores_the_exited_hold(self):
    # one-shot: once it exits, processNotRunning would block engagement
    assert "'chestnut_gps_hold'" in (OPENPILOT / "selfdrive/selfdrived/selfdrived.py").read_text()
