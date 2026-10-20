"""
Cruise-driven lead car for plant.py.

Drives the lead with openpilot's own cruise accel law rather than stepping its
velocity straight to the maneuver's speed_lead values. The lead's per-step
v_cruise (typically the maneuver's interpolated speed_lead_values) is turned
into a jerk-limited acceleration, so the lead accelerates and brakes the way a
car actually does instead of taking corner-y linear-interp velocity steps.

The lead has no lead of its own, so its behavior is shaped purely by the cruise
schedule.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN, J_CRUISE_BP, J_CRUISE_VALS

# the planner's former chill-mode accel limit, kept so the lead accelerates like a car rather than at ACCEL_MAX
LEAD_A_MAX_VALS = [1.6, 1.2, 0.8, 0.6]


class MpcLead:
  def __init__(self, v0=0.0, a0=0.0, x0=0.0, dt=DT_MDL):
    self.dt = dt
    self.v = float(v0)
    self.a = float(a0)
    self.x = float(x0)

  def step(self, v_cruise):
    a_max = np.interp(self.v, J_CRUISE_BP, LEAD_A_MAX_VALS)
    j_max = np.interp(self.v, J_CRUISE_BP, J_CRUISE_VALS)
    a_cmd = float(np.clip(np.clip(float(v_cruise) - self.v, A_CRUISE_MIN, a_max), self.a - j_max * self.dt, self.a + j_max * self.dt))
    self.x = self.x + self.v * self.dt + 0.5 * a_cmd * self.dt * self.dt
    self.v = max(0.0, self.v + a_cmd * self.dt)
    self.a = a_cmd
    return self.v, self.a, self.x
