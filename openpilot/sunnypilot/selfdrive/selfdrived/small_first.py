"""
selfdrived's events while a small model drives ahead of the big one (modeld_v2/big_model_loader.py).

A modelV2 published while ChestnutLoading is still set can only be that small model. ChestnutLoading then stays set for
the UI and hardwared, but stops holding engagement off and stops hiding lag and comm errors once the model is up.
"""
import time
from typing import NamedTuple

from openpilot.cereal import custom, log

SETTLE_S = 5.  # bounds the wait for the services downstream of the first model frame
SWAP_NO_ENTRY_S = 5.  # about what the big model's history takes to fill: 99 frames at 20 Hz


class Flags(NamedTuple):
  hold: bool = False  # as a native big model load: no entry, and comm errors held
  no_entry: bool = False  # the seconds after a swap
  chime: bool = False  # the end of those seconds: the big model drives, engage now
  offer: bool = False  # the big model is ready while engaged: disengage to switch
  switched: bool = False  # the swap's tick: hold the errors its dropped frames raise


class SmallFirstEvents:
  def __init__(self):
    self.up_t: float | None = None
    self.released = False
    self.loading_prev = True
    self.swap_t: float | None = None
    self.offered = False

  def update(self, sm, params, loading: bool, in_control: bool, events, events_sp) -> Flags:
    """One selfdrived tick: adds its events, and returns what selfdrived acts on itself (hold, switched)."""
    if not loading and (self.up_t is None or (not self.loading_prev and self.swap_t is None)):
      return Flags()  # nothing to do: no chestnut, a native load's end, or after the swap

    small_up = sm.seen['modelV2'] and sm.alive['modelV2'] and not sm['modelV2'].big
    localizer_ok = not sm.seen['deviceMotion'] or sm['deviceMotion'].inputsOK
    flags = self.step(loading, params.get_bool("ChestnutBigReady"), params.get("ChestnutActive"), small_up,
                      sm.all_checks() and localizer_ok, in_control, time.monotonic())
    if flags.no_entry:
      events.add(log.OnroadEvent.EventName.bigModelLoading)
    if flags.chime:
      events_sp.add(custom.OnroadEventSP.EventName.bigModelReady)
    if flags.offer:
      events_sp.add(custom.OnroadEventSP.EventName.bigModelAvailable)
    return flags

  def step(self, loading: bool, big_ready: bool, big_active: bool | None, small_up: bool, checks_ok: bool,
           in_control: bool, now: float) -> Flags:
    if loading and small_up and self.up_t is None:
      self.up_t = now
    if self.up_t is None:
      return Flags(hold=loading)  # a native big model load

    self.released = self.released or checks_ok or now - self.up_t >= SETTLE_S

    switched = self.loading_prev and not loading and big_active is True
    self.loading_prev = loading
    if switched:
      self.swap_t = now

    no_entry = chime = False
    if self.swap_t is not None:
      no_entry = now < self.swap_t + SWAP_NO_ENTRY_S
      if not no_entry:
        chime = True
        self.swap_t = None

    offer = big_ready and in_control and not self.offered
    self.offered = self.offered or offer

    return Flags(loading and not self.released, no_entry, chime, offer, switched)
