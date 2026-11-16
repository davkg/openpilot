from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.speed_nudge import SpeedNudge, NUDGE_ACCEL_MAX, DROP_TIME, EXPIRE_TIME

MPH = CV.MPH_TO_MS


class Road:
  """The car follows the e2e candidate exactly; the model holds whatever speed it has (accel a_model)."""
  def __init__(self, v_ego=25., a_model=0., yaw_rate=0., lane_change=log.LaneChangeState.off, lead=None, plan_drop=0.):
    self.v_ego = v_ego
    self.a_model = a_model
    self.yaw_rate = yaw_rate
    self.lane_change = lane_change
    self.lead = lead or SimpleNamespace(present=False, dRel=0., vRel=0.)
    self.plan_drop = plan_drop
    self.taps = 0.
    self.nudge = SpeedNudge()
    self.e2e = 0.

  def sm(self):
    n = ModelConstants.IDX_N
    plan_v = [self.v_ego - self.plan_drop * i / (n - 1) for i in range(n)]
    md = SimpleNamespace(velocity=SimpleNamespace(x=plan_v), orientationRate=SimpleNamespace(z=[self.yaw_rate] * n),
                         meta=SimpleNamespace(laneChangeState=self.lane_change))
    return {'carStateSP': SimpleNamespace(speedNudge=self.taps), 'carState': SimpleNamespace(vEgo=self.v_ego), 'modelV2': md,
            'radarState': SimpleNamespace(leadOne=self.lead), 'controlsState': SimpleNamespace(curvature=0.)}

  def tap(self, n=1, step=MPH):
    # taps come at least a few frames apart (press + release); more in one frame reads as card restarting
    for i in range(abs(n)):
      self.taps += step if n > 0 else -step
      if i < abs(n) - 1:
        self.run(0.2)

  def run(self, seconds, reset_state=False, move=True):
    lifts = []
    for _ in range(round(seconds / DT_MDL)):
      self.e2e = self.nudge.update(self.sm(), self.a_model, reset_state)
      lifts.append(self.nudge.lift)
      if move:
        self.v_ego += self.e2e * DT_MDL
    return lifts


class TestSpeedNudge(OpenpilotTestCase):
  def test_one_tap_arrives_in_about_2_5s(self):
    road = Road()
    road.run(1.)
    road.tap()
    road.run(DT_MDL)
    t = DT_MDL
    while road.nudge.active and t < 10.:
      road.run(DT_MDL)
      t += DT_MDL
    assert 2.0 < t < 3.5, t
    assert abs(road.v_ego - (25. + MPH)) < 0.05
    road.run(2.)
    assert road.nudge.lift == 0.
    assert abs(road.v_ego - (25. + MPH)) < 0.05  # holds: the model keeps the new speed

  def test_taps_stack_and_cap(self):
    road = Road()
    road.run(0.5)
    road.tap(5)
    lifts = road.run(15.)
    assert max(lifts) <= NUDGE_ACCEL_MAX + 1e-6
    assert abs(road.v_ego - (25. + 5 * MPH)) < 0.05

  def test_tap_down(self):
    road = Road()
    road.run(0.5)
    road.tap(-1)
    lifts = road.run(8.)
    assert min(lifts) < -0.1
    assert abs(road.v_ego - (25. - MPH)) < 0.05

  def test_tap_down_under_model_braking(self):
    # the model already brakes harder than the nudge's ceiling: nothing to add
    road = Road(a_model=-1.0)
    road.run(0.5)
    road.tap(-1)
    assert all(lift == 0. for lift in road.run(0.5))

  def test_model_already_faster(self):
    road = Road(a_model=0.5)
    road.run(0.5)
    road.tap()
    assert all(lift == 0. for lift in road.run(2.))

  def test_model_braking_drops_nudge(self):
    road = Road(a_model=-0.5)
    road.run(0.5)
    road.tap()
    lifts = road.run(DROP_TIME + 0.2, move=False)
    assert all(lift == 0. for lift in lifts)
    assert not road.nudge.active
    road.a_model = 0.  # the model stops braking: the dropped nudge does not come back
    assert all(lift == 0. for lift in road.run(2.))

  def test_planned_slowdown_drops_nudge(self):
    road = Road(plan_drop=4.)
    road.run(0.5)
    road.tap()
    assert all(lift == 0. for lift in road.run(DROP_TIME + 0.2, move=False))
    assert not road.nudge.active

  def test_closing_lead_drops_nudge(self):
    road = Road(lead=SimpleNamespace(present=True, dRel=30., vRel=-3.))  # 10 s to contact
    road.run(0.5)
    road.tap()
    assert all(lift == 0. for lift in road.run(DROP_TIME + 0.2, move=False))
    assert not road.nudge.active

  def test_steady_lead_still_nudges(self):
    road = Road(lead=SimpleNamespace(present=True, dRel=60., vRel=0.))
    road.run(0.5)
    road.tap()
    assert max(road.run(2.)) > 0.1

  def test_curve_ahead_drops_nudge(self):
    road = Road(yaw_rate=1.5 / 25.)  # 1.5 m/s^2 at 25 m/s
    road.run(0.5)
    road.tap()
    assert all(lift == 0. for lift in road.run(DROP_TIME + 0.2, move=False))
    assert not road.nudge.active

  def test_lane_change_drops_nudge(self):
    road = Road(lane_change=log.LaneChangeState.laneChangeStarting)
    road.run(0.5)
    road.tap()
    assert all(lift == 0. for lift in road.run(DROP_TIME + 0.2, move=False))
    assert not road.nudge.active

  def test_bend_mid_nudge_lets_go_gradually(self):
    road = Road()
    road.run(0.5)
    road.tap(3)
    road.run(2.)
    lift = road.nudge.lift
    assert lift > 0.2
    road.yaw_rate = 2. / road.v_ego
    lifts = road.run(0.2, move=False)
    assert lifts[-1] < lift and lifts[0] > 0.  # falls at the rate limit, not a step

  def test_low_speed(self):
    road = Road(v_ego=3.)
    road.run(0.5)
    road.tap()
    assert all(lift == 0. for lift in road.run(DROP_TIME + 0.2, move=False))

  def test_reset_clears(self):
    road = Road()
    road.run(0.5)
    road.tap(3)
    road.run(1.)
    assert road.nudge.lift > 0.
    road.run(DT_MDL, reset_state=True)
    assert road.nudge.lift == 0. and not road.nudge.active
    assert all(lift == 0. for lift in road.run(2.))  # a tap before the reset does not come back

  def test_expires(self):
    road = Road()
    road.run(0.5)
    road.tap()
    road.run(EXPIRE_TIME + 0.5, move=False)  # the car never responds
    assert not road.nudge.active

  def test_no_nudge_on_start_or_resync(self):
    road = Road()
    road.taps = 7 * MPH  # plannerd starting while card already counted taps
    assert all(lift == 0. for lift in road.run(1.))
    road.taps = 0.  # card restarted
    assert all(lift == 0. for lift in road.run(1.))
