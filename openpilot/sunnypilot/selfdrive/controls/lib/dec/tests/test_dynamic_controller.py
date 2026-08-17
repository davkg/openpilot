from typing import Any

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController

class MockLeadOne:
  def __init__(self, present=0.0, dRel=0.0):
    self.present = present
    self.dRel = dRel

class MockRadarState:
  def __init__(self, present=0.0, dRel=0.0):
    self.leadOne = MockLeadOne(present=present, dRel=dRel)

class MockCarState:
  def __init__(self, vEgo=0.0, vCruise=0.0, standstill=False):
    self.vEgo = vEgo
    self.vCruise = vCruise
    self.standstill = standstill

class MockModelData:
  def __init__(self, valid=True, desired_accel=0.0, orientation_rate_z=0.0, velocity_x=0.0):
    size = 33 if valid else 10  # incomplete if invalid
    self.position = type("Pos", (), {"x": [0.0] * size})()
    self.orientation = type("Ori", (), {"x": [0.0] * size})()
    self.action = type("Action", (), {"desiredAcceleration": desired_accel})()
    # orientationRate.z * velocity.x -> predicted lateral accel along the path (turn detection)
    self.orientationRate = type("OriRate", (), {"z": [orientation_rate_z] * size})()
    self.velocity = type("Vel", (), {"x": [velocity_x] * size})()

class MockSelfDriveState:
  def __init__(self, experimentalMode=False):
    self.experimentalMode = experimentalMode

class MockParams:
  def get_bool(self, name):
    return True

# Fake Kalman Filter that always returns a given value
class FakeKalman:
  def __init__(self, value=1.0):
    self.value = value
  def add_data(self, v): pass
  def get_value(self): return self.value
  def get_confidence(self): return 1.0
  def reset_data(self): pass


class TestDynamicExperimentalController(OpenpilotTestCase):
  def setUp(self):
    class CP:
      radarUnavailable = False

    class MPC:
      crash_cnt = 0
      a_solution = [0.0] * 13  # MPC accel trajectory; a_solution[0] = current accel (0 = not braking)

    self.CP = CP()
    self.mpc = MPC()
    self.sm: dict[str, Any] = {
      'carState': MockCarState(vEgo=10.0, vCruise=20.0),
      'radarState': MockRadarState(present=1.0),
      'modelV2': MockModelData(valid=True),
      'selfdriveState': MockSelfDriveState(experimentalMode=True),
    }

  def _controller(self, radarless=False):
    self.CP.radarUnavailable = radarless
    return DynamicExperimentalController(self.CP, self.mpc, params=MockParams())

  def _run(self, controller, n=12):
    for _ in range(n):
      controller.update(self.sm)

  # ---- upstream coverage ----

  def test_initial_mode_is_acc(self):
    self.assertEqual(self._controller().mode(), "acc")

  def test_standstill_triggers_blended(self):
    controller = self._controller()
    self.sm['carState'].standstill = True
    self._run(controller, 10)
    self.assertEqual(controller.mode(), "blended")

  def test_emergency_blended_on_fcw(self):
    controller = self._controller()
    self.mpc.crash_cnt = 1  # simulate FCW
    self._run(controller, 2)
    self.assertEqual(controller.mode(), "blended")

  def test_radarless_slowdown_triggers_blended(self):
    controller = self._controller(radarless=True)
    controller._slow_down_filter = FakeKalman(value=1.0)
    controller._v_ego_kph = 35.0
    self.sm['radarState'] = MockRadarState(present=0.0)  # no close lead
    self.sm['modelV2'] = MockModelData(valid=False)      # incomplete trajectory
    self._run(controller, 3)
    self.assertEqual(controller.mode(), "blended")

  # ---- close-lead gate ----

  def test_radarless_close_lead_forces_acc(self):
    # A close lead keeps ACC for routine/gentle following (low-urgency slowdown).
    controller = self._controller(radarless=True)
    # Close lead: present and well within the close-lead distance (v_ego 10 m/s -> ~76 m gate)
    self.sm['radarState'] = MockRadarState(present=1.0, dRel=15.0)
    # Mild slowdown (urgency below the 0.7 emergency threshold) -> close-lead gate takes priority
    controller._slow_down_filter = FakeKalman(value=0.5)
    self.sm['modelV2'] = MockModelData(valid=False)
    self._run(controller, 10)
    self.assertEqual(controller.mode(), "acc")

  def test_radarless_close_lead_high_urgency_stays_acc(self):
    # Regression: a close lead's trajectory naturally shortens the endpoint, manufacturing phantom
    # high urgency even in routine following (which over-fired blended in undulating traffic). The
    # urgency emergency is gated on no-close-lead, so a close lead + high urgency stays ACC.
    controller = self._controller(radarless=True)
    self.sm['radarState'] = MockRadarState(present=1.0, dRel=15.0)
    controller._slow_down_filter = FakeKalman(value=1.0)
    self.sm['modelV2'] = MockModelData(valid=False)
    self._run(controller, 10)
    self.assertEqual(controller.mode(), "acc")

  def test_radarless_distant_lead_allows_blended(self):
    # A lead beyond the close-lead distance must NOT suppress blended (model sees further).
    controller = self._controller(radarless=True)
    self.sm['radarState'] = MockRadarState(present=1.0, dRel=140.0)
    controller._slow_down_filter = FakeKalman(value=1.0)
    self.sm['modelV2'] = MockModelData(valid=False)
    self._run(controller, 3)
    self.assertEqual(controller.mode(), "blended")

  # ---- e2e-decel trigger ----

  def test_radarless_e2e_decel_triggers_blended(self):
    # The e2e model's desiredAcceleration anticipating a slowdown (no close lead) should engage blended
    # even when the trajectory-endpoint shortfall hasn't tripped yet.
    controller = self._controller(radarless=True)
    self.sm['carState'].vEgo = 15.0
    self.sm['radarState'] = MockRadarState(present=0.0)
    controller._slow_down_filter = FakeKalman(value=0.0)
    self.sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)  # e2e wants to brake
    self._run(controller)
    self.assertEqual(controller.mode(), "blended")

  def test_radarless_low_speed_close_lead_e2e_decel_stays_acc(self):
    # Below E2E_DECEL_OVERRIDE_MIN_SPEED, a close lead slowing gently stays ACC -- the MPC handles
    # low-speed close-lead slowdowns confidently and the gate keeps stop-n-go responsive.
    controller = self._controller(radarless=True)
    self.sm['carState'].vEgo = 10.0  # below the override speed
    self.sm['radarState'] = MockRadarState(present=1.0, dRel=30.0)  # close lead
    controller._slow_down_filter = FakeKalman(value=0.0)
    self.sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)
    self._run(controller)
    self.assertEqual(controller.mode(), "acc")

  def test_radarless_highspeed_close_lead_e2e_decel_blended(self):
    # Above the override speed, e2e-decel beats the close-lead gate: a close lead the e2e model
    # anticipates slowing (e.g. coasting toward a stop at highway speed) engages blended.
    controller = self._controller(radarless=True)
    self.sm['carState'].vEgo = 20.0
    self.sm['radarState'] = MockRadarState(present=1.0, dRel=30.0)
    controller._slow_down_filter = FakeKalman(value=0.0)
    self.sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)  # e2e -0.5 << MPC (0.0)
    self._run(controller)
    self.assertEqual(controller.mode(), "blended")

  def test_radarless_highspeed_mpc_already_braking_stays_acc(self):
    # The MPC is ALREADY braking harder than e2e, so min(e2e, mpc) would add nothing. The margin
    # gate keeps DEC in ACC rather than flipping mode for no benefit.
    self.mpc.a_solution = [-1.2] * 13
    controller = self._controller(radarless=True)
    self.sm['carState'].vEgo = 20.0
    self.sm['radarState'] = MockRadarState(present=1.0, dRel=30.0)
    controller._slow_down_filter = FakeKalman(value=0.0)
    self.sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)
    self._run(controller)
    self.assertEqual(controller.mode(), "acc")

  def test_radarless_low_speed_no_lead_e2e_decel_blended(self):
    # No speed gate: with no close lead, e2e-decel engages at any speed (e.g. a low-speed approach
    # to a stop sign / red light). Stop-n-go stays ACC via the close-lead gate, not a speed gate.
    controller = self._controller(radarless=True)
    self.sm['carState'].vEgo = 8.0
    self.sm['radarState'] = MockRadarState(present=0.0)
    controller._slow_down_filter = FakeKalman(value=0.0)
    self.sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)
    self._run(controller)
    self.assertEqual(controller.mode(), "blended")

  # ---- upcoming-turn trigger ----

  def test_radarless_upcoming_turn_triggers_blended(self):
    # A sharp curve ahead (predicted lat accel > TURN_LAT_ACC_ENGAGE) engages blended early so the
    # e2e model's curve easing leads the reactive e2e-decel trigger. No lead, no e2e decel.
    controller = self._controller(radarless=True)
    self.sm['carState'].vEgo = 20.0
    self.sm['radarState'] = MockRadarState(present=0.0)
    controller._slow_down_filter = FakeKalman(value=0.0)
    # 20 m/s * 0.1 rad/s = 2.0 m/s^2 predicted lat accel (> 1.5 engage)
    self.sm['modelV2'] = MockModelData(valid=True, orientation_rate_z=0.1, velocity_x=20.0)
    self._run(controller)
    self.assertEqual(controller.mode(), "blended")
    self.assertEqual(controller.reason(), "upcomingTurn")

  def test_radarless_turn_clears_returns_to_acc(self):
    # Once the predicted curve clears (below TURN_LAT_ACC_RELEASE) and nothing else requests
    # blended, DEC falls back to ACC.
    controller = self._controller(radarless=True)
    self.sm['carState'].vEgo = 20.0
    self.sm['radarState'] = MockRadarState(present=0.0)
    controller._slow_down_filter = FakeKalman(value=0.0)
    self.sm['modelV2'] = MockModelData(valid=True, orientation_rate_z=0.1, velocity_x=20.0)
    self._run(controller)
    self.assertEqual(controller.mode(), "blended")

    # Straighten out: predicted lat accel ~0 -> turn releases, mode returns to ACC
    self.sm['modelV2'] = MockModelData(valid=True, orientation_rate_z=0.0, velocity_x=20.0)
    self._run(controller, 20)
    self.assertEqual(controller.mode(), "acc")
    self.assertEqual(controller.reason(), "none")
