from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper, CRUISE_LONG_PRESS

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


class TestSpeedNudgeButtons(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SpeedNudge", True, block=True)
    self.helper = VCruiseHelper(car.CarParams(pcmCruise=False), custom.CarParamsSP(pcmCruiseSpeed=True))
    for _ in range(2):
      self.helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=False)
    self.helper.initialize_v_cruise(car.CarState(vEgo=30.), 40)
    self.set_speed = self.helper.v_cruise_kph

  def step(self, events=(), is_metric=False, **cs):
    CS = car.CarState(**{"cruiseState": {"available": True}, "vEgo": 25., **cs})
    CS.buttonEvents = [ButtonEvent(type=t, pressed=p) for t, p in events]
    self.helper.update_v_cruise(CS, enabled=True, is_metric=is_metric)

  def tap(self, btn, **kw):
    self.step([(btn, True)], **kw)
    self.step([(btn, False)], **kw)

  def test_short_tap_nudges(self):
    self.tap(ButtonType.accelCruise)
    self.tap(ButtonType.accelCruise)
    self.tap(ButtonType.decelCruise)
    self.assertAlmostEqual(self.helper.speed_nudge, CV.MPH_TO_MS, places=5)
    self.assertEqual(self.helper.v_cruise_kph, self.set_speed)
    self.assertEqual(self.helper.set_speed_step, 0)

  def test_metric_step(self):
    self.tap(ButtonType.decelCruise, is_metric=True)
    self.assertAlmostEqual(self.helper.speed_nudge, -CV.KPH_TO_MS, places=5)

  def test_long_press_steps_set_speed(self):
    self.step([(ButtonType.accelCruise, True)])
    for _ in range(CRUISE_LONG_PRESS):
      self.step()
    self.step([(ButtonType.accelCruise, False)])
    self.assertGreater(self.helper.v_cruise_kph, self.set_speed)
    self.assertEqual(self.helper.speed_nudge, 0.)

  def test_set_while_overriding_keeps_stock_meaning(self):
    self.tap(ButtonType.decelCruise, gasPressed=True, vEgo=35.)
    self.assertEqual(self.helper.speed_nudge, 0.)
    self.assertAlmostEqual(self.helper.v_cruise_kph, 35. * CV.MS_TO_KPH, places=1)

  def test_resume_in_standstill_is_not_a_nudge(self):
    self.tap(ButtonType.accelCruise, cruiseState={"available": True, "standstill": True})
    self.assertEqual(self.helper.speed_nudge, 0.)

  def test_disabled_steps_set_speed(self):
    self.params.put_bool("SpeedNudge", False, block=True)
    self.helper.read_custom_set_speed_params()
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.speed_nudge, 0.)
    self.assertGreater(self.helper.v_cruise_kph, self.set_speed)
