from unittest import mock

from openpilot.cereal import custom, log
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.selfdrived.small_first import SETTLE_S, SWAP_NO_ENTRY_S, SmallFirstEvents


def tick(events, now, loading=True, big_ready=False, big_active=None, small_up=True, checks_ok=True, in_control=False):
  return events.step(loading, big_ready, big_active, small_up, checks_ok, in_control, now)


class TestSmallFirstEvents(OpenpilotTestCase):
  def test_native_load_is_untouched(self):
    events = SmallFirstEvents()
    assert tick(events, 0., small_up=False).hold
    # the native load clears ChestnutLoading before the model publishes
    flags = tick(events, 1., loading=False, big_active=True, small_up=False)
    assert flags == type(flags)()
    # a big model publishing afterwards never reads as small-first
    assert tick(events, 2., loading=False, big_active=True, small_up=False) == type(flags)()

  def test_holds_until_the_small_model_and_its_services_are_up(self):
    events = SmallFirstEvents()
    assert tick(events, 0., small_up=False).hold
    assert tick(events, 8., checks_ok=False).hold
    assert not tick(events, 9.).hold
    assert not tick(events, 9.1, checks_ok=False).hold  # released for good: later comm errors are real

  def test_hold_is_bounded(self):
    events = SmallFirstEvents()
    assert tick(events, 8., checks_ok=False).hold
    assert not tick(events, 8. + SETTLE_S, checks_ok=False).hold

  def test_swap_blocks_entry_then_chimes(self):
    events = SmallFirstEvents()
    tick(events, 0.)
    flags = tick(events, 40., loading=False, big_active=True, small_up=False)
    assert flags.switched and flags.no_entry and not flags.chime
    flags = tick(events, 40. + SWAP_NO_ENTRY_S - 0.01, loading=False, big_active=True, small_up=False)
    assert flags.no_entry and not flags.switched and not flags.chime
    flags = tick(events, 40. + SWAP_NO_ENTRY_S, loading=False, big_active=True, small_up=False)
    assert flags.chime and not flags.no_entry
    assert not tick(events, 50., loading=False, big_active=True, small_up=False).chime

  def test_failed_load_neither_chimes_nor_blocks(self):
    events = SmallFirstEvents()
    tick(events, 0.)
    for now in (40., 42., 50.):
      flags = tick(events, now, loading=False, big_active=False)
      assert not (flags.switched or flags.no_entry or flags.chime or flags.hold)

  def test_offer_shows_once_and_only_while_engaged(self):
    events = SmallFirstEvents()
    tick(events, 0.)
    assert not tick(events, 1., big_ready=True).offer
    assert tick(events, 2., big_ready=True, in_control=True).offer
    assert not tick(events, 3., big_ready=True, in_control=True).offer


class TestSelfdrivedTick(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.events = SmallFirstEvents()
    self.e, self.e_sp = mock.Mock(), mock.Mock()
    self.params = mock.Mock()
    self.params.get_bool.return_value = False
    self.params.get.return_value = None
    self.sm = mock.MagicMock()
    self.sm.__getitem__.return_value = mock.Mock(big=False, inputsOK=True)

  def update(self, loading=True, in_control=False):
    return self.events.update(self.sm, self.params, loading, in_control, self.e, self.e_sp)

  def test_no_chestnut_reads_nothing(self):
    assert self.update(loading=False) == type(self.update(loading=False))()
    self.params.get.assert_not_called()
    self.params.get_bool.assert_not_called()

  def test_swap_raises_no_entry_then_chime(self):
    self.update()
    self.params.get.return_value = True
    assert self.update(loading=False).switched
    self.e.add.assert_called_once_with(log.OnroadEvent.EventName.bigModelLoading)
    self.events.swap_t = 0.  # long past
    self.update(loading=False)
    self.e_sp.add.assert_called_once_with(custom.OnroadEventSP.EventName.bigModelReady)

  def test_offer_raised(self):
    self.params.get_bool.return_value = True
    self.update(in_control=True)
    self.e_sp.add.assert_called_once_with(custom.OnroadEventSP.EventName.bigModelAvailable)
