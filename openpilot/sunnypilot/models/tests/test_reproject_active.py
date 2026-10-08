from unittest import mock

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
import openpilot.sunnypilot.models.helpers as helpers

Runner = custom.ModelManagerSP.Runner


class TestReprojectActive(OpenpilotTestCase):
  """reproject_active gates reprojectd, reprojectcalibd and calibrationd's hold: it must be true only when the running
  modeld reads the "reproject" frames, else calibration holds for a fit that never comes."""

  def check(self, runner, stock_expected: bool, tizi_chestnut: bool) -> bool:
    with mock.patch.object(helpers, 'get_active_model_runner', return_value=runner), \
         mock.patch.object(helpers, 'reproject_expected', return_value=stock_expected), \
         mock.patch.object(helpers, '_tizi_with_chestnut', return_value=tizi_chestnut):
      return helpers.reproject_active()

  def test_stock_runner_follows_stock_modeld(self):
    self.assertTrue(self.check(Runner.stock, True, True))
    self.assertFalse(self.check(Runner.stock, False, True))  # big model not compiled: stock modeld reads camerad

  def test_picked_big_model(self):
    self.assertTrue(self.check(Runner.tinygrad, False, True))
    self.assertFalse(self.check(Runner.tinygrad, True, False))

  def test_other_runners(self):
    self.assertFalse(self.check(Runner.snpe, True, True))
