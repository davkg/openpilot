from types import SimpleNamespace
from unittest import mock

from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
from openpilot.sunnypilot.selfdrive.controls.lib.launch_gain import LaunchGain, LAUNCH_GAIN, FADE_BP, GAIN_RATE


def bundle(name):
  return {"internalName": name, "minimumSelectorVersion": REQUIRED_JSON_VERSION}


def slot_params(qcom=None, chestnut=None):
  params = mock.MagicMock()
  params.get.side_effect = {"ModelManager_ActiveBundle": qcom, "ModelManager_ActiveBundleChestnut": chestnut}.get
  return params


def sm(v_ego, big=True, should_stop=False):
  return {'carState': SimpleNamespace(vEgo=v_ego), 'modelV2': SimpleNamespace(big=big, action=SimpleNamespace(shouldStop=should_stop))}


class TestLaunchGain(OpenpilotTestCase):
  def settled(self, lg, v_ego, big=True, a=1.0, seconds=2.):
    for _ in range(round(seconds / DT_MDL)):
      out = lg.update(sm(v_ego, big), a)
    return out

  def test_listed_model_scales_launch(self):
    lg = LaunchGain(slot_params(qcom=bundle("TSFDOM"), chestnut=bundle("LSMV2")))
    self.assertAlmostEqual(self.settled(lg, 2.), LAUNCH_GAIN['LSMV2'])
    self.assertAlmostEqual(self.settled(lg, 2., a=0.2), 0.2 * LAUNCH_GAIN['LSMV2'])  # a hesitation stays small

  def test_follows_the_model_that_drives(self):
    lg = LaunchGain(slot_params(qcom=bundle("TSFDOM"), chestnut=bundle("LSMV2")))
    self.assertAlmostEqual(self.settled(lg, 2., big=False), 1.0)  # the small model drives stock

  def test_unlisted_or_missing_model_is_stock(self):
    for params in (slot_params(chestnut=bundle("SOMETHING_NEW")), slot_params()):
      self.assertAlmostEqual(self.settled(LaunchGain(params), 2.), 1.0)

  def test_fades_with_speed(self):
    lg = LaunchGain(slot_params(chestnut=bundle("LSMV2")))
    mid = self.settled(lg, sum(FADE_BP) / 2)
    self.assertAlmostEqual(mid, (1. + LAUNCH_GAIN['LSMV2']) / 2)
    self.assertAlmostEqual(self.settled(lg, FADE_BP[1] + 1.), 1.0)

  def test_braking_and_stopping_untouched(self):
    lg = LaunchGain(slot_params(chestnut=bundle("LSMV2")))
    self.settled(lg, 2.)
    self.assertEqual(lg.update(sm(2.), -0.8), -0.8)
    self.assertEqual(lg.update(sm(2., should_stop=True), 0.5), 0.5)

  def test_model_handover_eases(self):
    lg = LaunchGain(slot_params(qcom=bundle("TSFDOM"), chestnut=bundle("LSMV2")))
    self.settled(lg, 2.)
    first = lg.update(sm(2., big=False), 1.0)  # big model falls back: one frame later the gain has barely moved
    self.assertAlmostEqual(first, LAUNCH_GAIN['LSMV2'] - GAIN_RATE * DT_MDL)
    self.assertAlmostEqual(self.settled(lg, 2., big=False), 1.0)
