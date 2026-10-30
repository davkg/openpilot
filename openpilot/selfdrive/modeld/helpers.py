import functools
import sys
import time
from pathlib import Path

from openpilot.common.hardware import AGNOS, HARDWARE
from openpilot.common.hardware.usb import CHESTNUT_USB_IDS, CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, cable_connected, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def load_oob(path, chestnut=False):
  from tinygrad import Context
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    from tinygrad_repo.examples.openpilot.helpers import load_pickle
    return load_pickle(path, out_of_band=True)

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def chestnut_usb_visible() -> bool:
  # tinygrad caches its USB scan for the life of the process, and modeld_v2 already scanned at startup (importing
  # tinygrad.nn.onnx picks a default device, which probes AMD over USB). With the boot GPS hold the chestnut was
  # unpowered then, so the cached scan is empty and the load fails "AMD:0 does not exist". Rescan once it's here.
  from tinygrad.runtime.support.usb import USB3
  USB3.list_devices.cache_clear()
  return any(USB3.list_devices(*usb_id) for usb_id in CHESTNUT_USB_IDS)

def wait_for_chestnut(timeout: float = 10.) -> None:
  # the cable is detected before chestnut enumerates
  st = time.monotonic()
  while not (chestnut_present() and chestnut_usb_visible()):
    if time.monotonic() - st > timeout:
      raise TimeoutError("chestnut did not enumerate")
    time.sleep(0.1)

def chestnut_compiled() -> bool:
  path = modeld_pkl_path(chestnut=True)
  return path.is_file() and all(
    (MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl').is_file() for size in ('1344x760', '1928x1208'))


@functools.cache
def reproject_expected() -> bool:
  """reprojectd serves the 3X cameras as a comma 4's for the big model: a 3X with a chestnut and the big model in place.
  The chestnut test is modeld's, so the two always agree on the frames. Decided once per process, as modeld does: the
  manager starts reprojectd and reprojectcalibd on it, so a chestnut plugged in later takes a reboot."""
  return HARDWARE.get_device_type() == "tizi" and (chestnut_present() or cable_connected()) and chestnut_compiled()
