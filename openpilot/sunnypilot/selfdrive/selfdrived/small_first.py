"""
When selfdrived holds as it does through a native big model load, while a small model drives ahead of the big one
(modeld_v2/small_model_child.py).

A modelV2 published while ChestnutLoading is still set can only be that small model. ChestnutLoading then stays set for
the UI and hardwared, but the hold ends once the model is up. The swap to the big model holds again for a few seconds.
"""
import time

from openpilot.cereal import custom

SETTLE_S = 5.  # bounds the wait for the services downstream of the first model frame
# covers the swap's 0.3-1.2 s without a model. Kept short: a set press refused here still lets the panda allow controls, and
# one accepted within its ~2 s to drop them ends in Controls Mismatch (route 00000190--68f82fd3ed)
SWAP_HOLD_S = 2.


class SmallFirstEvents:
  def __init__(self):
    self.up_t: float | None = None
    self.released = False
    self.loading_prev = True
    self.swap_t: float | None = None
    self.offered = False

  def update(self, sm, params, loading: bool, in_control: bool, events_sp) -> bool:
    """One selfdrived tick: adds the disengage to switch prompt, and returns whether to hold."""
    if not loading and (self.up_t is None or (not self.loading_prev and self.swap_t is None)):
      return False  # nothing to do: no chestnut, a native load's end, or after the swap

    small_up = sm.seen['modelV2'] and sm.alive['modelV2'] and not sm['modelV2'].big
    localizer_ok = not sm.seen['deviceMotion'] or sm['deviceMotion'].inputsOK
    hold, offer = self.step(loading, params.get_bool("ChestnutBigReady"), params.get("ChestnutActive"), small_up,
                            sm.all_checks() and localizer_ok, in_control, time.monotonic())
    if offer:
      events_sp.add(custom.OnroadEventSP.EventName.bigModelAvailable)
    return hold

  def step(self, loading: bool, big_ready: bool, big_active: bool | None, small_up: bool, checks_ok: bool,
           in_control: bool, now: float) -> tuple[bool, bool]:
    if loading and small_up and self.up_t is None:
      self.up_t = now
    if self.up_t is None:
      return loading, False  # a native big model load

    self.released = self.released or checks_ok or now - self.up_t >= SETTLE_S

    if self.loading_prev and not loading and big_active is True:
      self.swap_t = now
    self.loading_prev = loading

    if self.swap_t is not None and now >= self.swap_t + SWAP_HOLD_S:
      self.swap_t = None

    offer = big_ready and in_control and not self.offered
    self.offered = self.offered or offer

    return (loading and not self.released) or self.swap_t is not None, offer
