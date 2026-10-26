#!/usr/bin/env python3
"""
Keeps the chestnut unpowered at boot until the 3X has a GPS fix. Its USB 3 link costs the 3X's GNSS ~8 dB:
enough to block acquisition but not tracking, so a fix taken before the chestnut comes back survives it.
modeld waits out the hold before loading the big model.
"""
import signal
import sys
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware import HARDWARE
from openpilot.common.hardware.usb import cable_connected
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.utils import sudo_read, sudo_write
from openpilot.selfdrive.modeld.helpers import chestnut_compiled

VBUS_PATH = "/sys/kernel/debug/regulator/smb2-vbus/enable"
# counted from the first forward gear, so time parked (e.g. in a garage) doesn't use it up
DRIVE_TIMEOUT = 120.
FORWARD_GEARS = ("drive", "sport", "low", "brake", "eco", "manumatic")


class GpsHold:
  def __init__(self):
    self.drive_start: float | None = None

  def done(self, has_fix: bool, gear: str, now: float) -> bool:
    if has_fix:
      return True
    if self.drive_start is None and gear in FORWARD_GEARS:
      self.drive_start = now
    return self.drive_start is not None and now - self.drive_start >= DRIVE_TIMEOUT


def write_vbus(on: bool) -> None:
  try:
    sudo_write("1" if on else "0", VBUS_PATH)
  except OSError:
    cloudlog.exception("chestnut gps hold: VBUS write failed")


def main() -> None:
  params = Params()
  if HARDWARE.get_device_type() != "tizi" or not params.get_bool("ChestnutGpsHold") or not (cable_connected() and chestnut_compiled()):
    return

  signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
  params.put_bool("ChestnutGpsHoldActive", True, block=True)
  sm = messaging.SubMaster(['gpsLocation', 'carState'])
  hold = GpsHold()
  st = time.monotonic()
  try:
    write_vbus(False)
    cloudlog.event("chestnut gps hold start", vbus=sudo_read(VBUS_PATH))
    while not hold.done(sm['gpsLocation'].hasFix, str(sm['carState'].gearShifter), time.monotonic()):
      sm.update(1000)
  finally:
    # power the chestnut before modeld is let go
    write_vbus(True)
    params.remove("ChestnutGpsHoldActive")
    cloudlog.event("chestnut gps hold end", fix=sm['gpsLocation'].hasFix, duration=round(time.monotonic() - st, 1),
                   drive_time=None if hold.drive_start is None else round(time.monotonic() - hold.drive_start, 1))


if __name__ == "__main__":
  main()
