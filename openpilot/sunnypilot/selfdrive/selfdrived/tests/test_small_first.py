from unittest import mock

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.selfdrived.small_first import SETTLE_S, SWAP_HOLD_S, SmallFirstEvents


def tick(events, now, loading=True, big_ready=False, big_active=None, small_up=True, checks_ok=True, in_control=False):
  return events.step(loading, big_ready, big_active, small_up, checks_ok, in_control, now)


def hold(events, now, **kwargs):
  return tick(events, now, **kwargs)[0]


def offer(events, now, **kwargs):
  return tick(events, now, **kwargs)[1]


class TestSmallFirstEvents(OpenpilotTestCase):
  def test_native_load_holds_only_while_loading(self):
    events = SmallFirstEvents()
    assert hold(events, 0., small_up=False)
    assert hold(events, 30., small_up=False)
    # the native load clears ChestnutLoading before the model publishes, and that model never reads as small-first
    assert not hold(events, 31., loading=False, big_active=True, small_up=False)
    assert events.swap_t is None

  def test_holds_until_the_small_model_and_its_services_are_up(self):
    events = SmallFirstEvents()
    assert hold(events, 0., small_up=False)
    assert hold(events, 8., checks_ok=False)
    assert not hold(events, 9.)
    assert not hold(events, 9.1, checks_ok=False)  # released for good: later comm errors are real

  def test_hold_is_bounded(self):
    events = SmallFirstEvents()
    assert hold(events, 8., checks_ok=False)
    assert not hold(events, 8. + SETTLE_S, checks_ok=False)

  def test_swap_holds_for_a_few_seconds(self):
    events = SmallFirstEvents()
    assert not hold(events, 0.)
    assert hold(events, 40., loading=False, big_active=True, small_up=False)
    assert hold(events, 40. + SWAP_HOLD_S - 0.01, loading=False, big_active=True, small_up=False)
    assert not hold(events, 40. + SWAP_HOLD_S, loading=False, big_active=True, small_up=False)
    assert not hold(events, 50., loading=False, big_active=True, small_up=False)

  def test_chimes_at_the_end_of_the_swap_and_not_when_the_small_model_comes_up(self):
    # as selfdrived does: if self.big_model_loading and not hold and not loading
    events = SmallFirstEvents()
    chimes = []
    held = False
    for now, loading, small_up in [(0., True, False), (9., True, True), (40., False, False), (44., False, True), (45., False, True), (46., False, True)]:
      h = hold(events, now, loading=loading, big_active=None if loading else True, small_up=small_up)
      if held and not h and not loading:
        chimes.append(now)
      held = h
    assert chimes == [45.]

  def test_failed_load_does_not_hold(self):
    events = SmallFirstEvents()
    tick(events, 0.)
    for now in (40., 42., 50.):
      assert not hold(events, now, loading=False, big_active=False)

  def test_offer_shows_once_and_only_while_engaged(self):
    events = SmallFirstEvents()
    tick(events, 0.)
    assert not offer(events, 1., big_ready=True)
    assert offer(events, 2., big_ready=True, in_control=True)
    assert not offer(events, 3., big_ready=True, in_control=True)


class TestSelfdrivedTick(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.events = SmallFirstEvents()
    self.e_sp = mock.Mock()
    self.params = mock.Mock()
    self.params.get_bool.return_value = False
    self.params.get.return_value = None
    self.sm = mock.MagicMock()
    self.sm.__getitem__.return_value = mock.Mock(big=False, inputsOK=True)

  def update(self, loading=True, in_control=False):
    return self.events.update(self.sm, self.params, loading, in_control, self.e_sp)

  def test_no_chestnut_reads_nothing(self):
    assert not self.update(loading=False)
    self.params.get.assert_not_called()
    self.params.get_bool.assert_not_called()

  def test_swap_holds_then_stops_reading_params(self):
    self.update()
    self.params.get.return_value = True
    assert self.update(loading=False)
    self.events.swap_t = -SWAP_HOLD_S  # long past
    assert not self.update(loading=False)
    self.params.get.reset_mock()
    assert not self.update(loading=False)
    self.params.get.assert_not_called()

  def test_offer_raised(self):
    self.params.get_bool.return_value = True
    self.update(in_control=True)
    self.e_sp.add.assert_called_once_with(custom.OnroadEventSP.EventName.bigModelAvailable)
