import itertools
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized_class

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  get_safe_obstacle_distance, get_stopped_equivalence_factor, get_STOP_DISTANCE,
)
from openpilot.sunnypilot.selfdrive.controls.lib.follow_time import MIN_FOLLOW_TIME, DEFAULT_FOLLOW_TIME, MAX_FOLLOW_TIME
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver


def desired_follow_distance(v_ego, v_lead, t_follow=DEFAULT_FOLLOW_TIME, stop_distance=None):
  if stop_distance is None:
    stop_distance = get_STOP_DISTANCE(t_follow)
  return get_safe_obstacle_distance(v_ego, t_follow, stop_distance) - get_stopped_equivalence_factor(v_lead)

def run_following_distance_simulation(v_lead, t_end=100.0, e2e=False, follow_time=DEFAULT_FOLLOW_TIME):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=float(v_lead),
    lead_relevancy=True,
    initial_distance_lead=100,
    speed_lead_values=[v_lead],
    breakpoints=[0.],
    e2e=e2e,
    follow_time=follow_time,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1,2] - output[-1,1]


@parameterized_class(("e2e", "follow_time", "speed"), itertools.product(
                      [True, False], # e2e
                      [MIN_FOLLOW_TIME, 0.9, DEFAULT_FOLLOW_TIME, 1.5, MAX_FOLLOW_TIME], # follow_time
                      [0,10,35])) # speed
class TestFollowingDistance(OpenpilotTestCase):
  def test_following_distance(self):
    v_lead = float(self.speed)
    simulation_steady_state = run_following_distance_simulation(v_lead, e2e=self.e2e, follow_time=self.follow_time)
    correct_steady_state = desired_follow_distance(v_lead, v_lead, self.follow_time)
    err_ratio = 0.2 if self.e2e else 0.1
    abs_err_margin = 0.5 if v_lead > 0.0 else 1.15
    self.assertAlmostEqual(simulation_steady_state, correct_steady_state,
                           delta=err_ratio * correct_steady_state + abs_err_margin)
