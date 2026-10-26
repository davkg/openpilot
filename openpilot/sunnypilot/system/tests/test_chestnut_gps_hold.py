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
