import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController

class MockLeadOne:
  def __init__(self, status=0.0, dRel=0.0):
    self.status = status
    self.dRel = dRel

class MockRadarState:
  def __init__(self, status=0.0, dRel=0.0):
    self.leadOne = MockLeadOne(status=status, dRel=dRel)

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

@pytest.fixture
def default_sm():
  sm = {
    'carState': MockCarState(vEgo=10.0, vCruise=20.0),
    'radarState': MockRadarState(status=1.0),
    'modelV2': MockModelData(valid=True),
    'selfdriveState': MockSelfDriveState(experimentalMode=True),
  }
  return sm

@pytest.fixture
def mock_cp():
  class CP:
    radarUnavailable = False
  return CP()

@pytest.fixture
def mock_mpc():
  class MPC:
    crash_cnt = 0
    a_solution = [0.0] * 13  # MPC accel trajectory; a_solution[0] = current accel (0 = not braking)
  return MPC()

# Fake Kalman Filter that always returns a given value
class FakeKalman:
  def __init__(self, value=1.0):
    self.value = value
  def add_data(self, v): pass
  def get_value(self): return self.value
  def get_confidence(self): return 1.0
  def reset_data(self): pass

def test_initial_mode_is_acc(mock_cp, mock_mpc):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  assert controller.mode() == "acc"

def test_standstill_triggers_blended(mock_cp, mock_mpc, default_sm):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  default_sm['carState'].standstill = True
  for _ in range(10):
    controller.update(default_sm)
  assert controller.mode() == "blended"

def test_emergency_blended_on_fcw(mock_cp, mock_mpc, default_sm):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  mock_mpc.crash_cnt = 1  # simulate FCW
  for _ in range(2):
    controller.update(default_sm)
  assert controller.mode() == "blended"

def test_radarless_slowdown_triggers_blended(mock_cp, mock_mpc, default_sm):
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  # No close lead, so the trajectory-shortfall path is free to select blended
  default_sm['radarState'] = MockRadarState(status=0.0)
  # Force conditions to simulate slowdown
  controller._slow_down_filter = FakeKalman(value=1.0)  # Ensure urgency triggers slowdown
  controller._v_ego_kph = 35.0
  default_sm['modelV2'] = MockModelData(valid=False)  # Incomplete trajectory

  for _ in range(3):
    controller.update(default_sm)

  assert controller.mode() == "blended"


def test_radarless_close_lead_forces_acc(mock_cp, mock_mpc, default_sm):
  # A close lead keeps ACC for routine/gentle following (low-urgency slowdown).
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  # Close lead: present and well within the close-lead distance (v_ego 10 m/s -> ~76 m gate)
  default_sm['radarState'] = MockRadarState(status=1.0, dRel=15.0)
  # Mild slowdown (urgency below the 0.7 emergency threshold) -> close-lead gate takes priority
  controller._slow_down_filter = FakeKalman(value=0.5)
  default_sm['modelV2'] = MockModelData(valid=False)

  for _ in range(10):
    controller.update(default_sm)

  assert controller.mode() == "acc"


def test_radarless_e2e_decel_triggers_blended(mock_cp, mock_mpc, default_sm):
  # The e2e model's desiredAcceleration anticipating a slowdown (no close lead) should engage blended
  # even when the trajectory-endpoint shortfall hasn't tripped yet.
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  default_sm['carState'].vEgo = 15.0
  default_sm['radarState'] = MockRadarState(status=0.0)
  controller._slow_down_filter = FakeKalman(value=0.0)   # isolate: no slow-down trigger
  default_sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)  # e2e wants to brake

  for _ in range(12):
    controller.update(default_sm)

  assert controller.mode() == "blended"


def test_radarless_low_speed_close_lead_e2e_decel_stays_acc(mock_cp, mock_mpc, default_sm):
  # Below E2E_DECEL_OVERRIDE_MIN_SPEED (15 m/s), a close lead slowing gently stays ACC -- the MPC
  # handles low-speed close-lead slowdowns confidently and the close-lead gate keeps stop-n-go responsive.
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  default_sm['carState'].vEgo = 10.0   # below the 15 m/s override speed
  default_sm['radarState'] = MockRadarState(status=1.0, dRel=30.0)   # close lead
  controller._slow_down_filter = FakeKalman(value=0.0)
  default_sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)

  for _ in range(12):
    controller.update(default_sm)

  assert controller.mode() == "acc"


def test_radarless_highspeed_close_lead_e2e_decel_blended(mock_cp, mock_mpc, default_sm):
  # Above 15 m/s, e2e-decel overrides the close-lead gate: a close lead the e2e model anticipates
  # slowing (e.g. coasting toward a stop at highway speed) engages blended for early braking.
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  default_sm['carState'].vEgo = 20.0   # above the 15 m/s override speed
  default_sm['radarState'] = MockRadarState(status=1.0, dRel=30.0)   # close lead
  controller._slow_down_filter = FakeKalman(value=0.0)
  default_sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)  # e2e -0.5 << MPC (0.0)

  for _ in range(12):
    controller.update(default_sm)

  assert controller.mode() == "blended"


def test_radarless_highspeed_mpc_already_braking_stays_acc(mock_cp, mock_mpc, default_sm):
  # Above 15 m/s with a close lead the e2e model wants to slow -- but the MPC is ALREADY braking harder
  # than e2e, so min(e2e,mpc) would add nothing. The margin gate keeps DEC in ACC (no pointless flip).
  mock_cp.radarUnavailable = True
  mock_mpc.a_solution = [-1.2] * 13   # MPC already braking harder than e2e (-0.5)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  default_sm['carState'].vEgo = 20.0
  default_sm['radarState'] = MockRadarState(status=1.0, dRel=30.0)   # close lead
  controller._slow_down_filter = FakeKalman(value=0.0)
  default_sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)

  for _ in range(12):
    controller.update(default_sm)

  assert controller.mode() == "acc"


def test_radarless_low_speed_no_lead_e2e_decel_blended(mock_cp, mock_mpc, default_sm):
  # No speed gate: with no close lead, e2e-decel engages at any speed (e.g. a low-speed approach
  # to a stop sign / red light). Stop-n-go stays ACC via the close-lead gate, not a speed gate.
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  default_sm['carState'].vEgo = 8.0
  default_sm['radarState'] = MockRadarState(status=0.0)   # no lead
  controller._slow_down_filter = FakeKalman(value=0.0)
  default_sm['modelV2'] = MockModelData(valid=True, desired_accel=-0.5)

  for _ in range(12):
    controller.update(default_sm)

  assert controller.mode() == "blended"


def test_radarless_close_lead_high_urgency_stays_acc(mock_cp, mock_mpc, default_sm):
  # Regression: a close lead's trajectory naturally shortens the endpoint, manufacturing phantom
  # high urgency even in routine following (which over-fired blended in undulating traffic). The
  # urgency emergency is gated on no-close-lead, so a close lead + high urgency stays ACC.
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  # Close lead present, and the slowdown urgency is high (> 0.7) -- close-lead gate wins
  default_sm['radarState'] = MockRadarState(status=1.0, dRel=15.0)
  controller._slow_down_filter = FakeKalman(value=1.0)
  default_sm['modelV2'] = MockModelData(valid=False)

  for _ in range(10):
    controller.update(default_sm)

  assert controller.mode() == "acc"


def test_radarless_distant_lead_allows_blended(mock_cp, mock_mpc, default_sm):
  # A lead beyond the close-lead distance must NOT suppress blended (model sees further).
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  # Lead present but far (well beyond the ~76 m gate at v_ego 10 m/s)
  default_sm['radarState'] = MockRadarState(status=1.0, dRel=140.0)
  controller._slow_down_filter = FakeKalman(value=1.0)
  default_sm['modelV2'] = MockModelData(valid=False)

  for _ in range(3):
    controller.update(default_sm)

  assert controller.mode() == "blended"


def test_radarless_upcoming_turn_triggers_blended(mock_cp, mock_mpc, default_sm):
  # A sharp curve ahead (predicted lat accel > TURN_LAT_ACC_ENGAGE) engages blended early so the
  # e2e model's curve easing leads the reactive e2e-decel trigger. No lead, no e2e decel.
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  default_sm['carState'].vEgo = 20.0
  default_sm['radarState'] = MockRadarState(status=0.0)   # no lead
  controller._slow_down_filter = FakeKalman(value=0.0)    # no slow-down trigger
  # 20 m/s * 0.1 rad/s = 2.0 m/s^2 predicted lat accel (> 1.5 engage)
  default_sm['modelV2'] = MockModelData(valid=True, desired_accel=0.0, orientation_rate_z=0.1, velocity_x=20.0)

  for _ in range(12):
    controller.update(default_sm)

  assert controller.mode() == "blended"
  assert controller.reason() == "upcomingTurn"


def test_radarless_turn_clears_returns_to_acc(mock_cp, mock_mpc, default_sm):
  # Once the predicted curve clears (lat accel below TURN_LAT_ACC_RELEASE) and nothing else requests
  # blended, DEC falls back to ACC.
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  default_sm['carState'].vEgo = 20.0
  default_sm['radarState'] = MockRadarState(status=0.0)
  controller._slow_down_filter = FakeKalman(value=0.0)
  default_sm['modelV2'] = MockModelData(valid=True, orientation_rate_z=0.1, velocity_x=20.0)
  for _ in range(12):
    controller.update(default_sm)
  assert controller.mode() == "blended"

  # Straighten out: predicted lat accel ~0 -> turn releases, mode returns to ACC
  default_sm['modelV2'] = MockModelData(valid=True, orientation_rate_z=0.0, velocity_x=20.0)
  for _ in range(20):
    controller.update(default_sm)
  assert controller.mode() == "acc"
  assert controller.reason() == "none"
