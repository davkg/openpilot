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
from opendbc.car import structs

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel

# Only steerRatio/wheelbase are read, and only to convert steering angle into
# lateral accel. The lead is driven straight ahead, so any sane values do.
_LEAD_CP = structs.CarParams(steerRatio=15.0, wheelbase=2.7)


class MpcLead:
  def __init__(self, v0=0.0, a0=0.0, x0=0.0, dt=DT_MDL):
    self.dt = dt
    self.v = float(v0)
    self.a = float(a0)
    self.x = float(x0)

  def step(self, v_cruise):
    a_cmd = get_cruise_accel(False, float(v_cruise), self.v, self.a, 0.0, _LEAD_CP, self.dt,
                             accel_coast=0.0, allow_throttle=True)
    self.x = self.x + self.v * self.dt + 0.5 * a_cmd * self.dt * self.dt
    self.v = max(0.0, self.v + a_cmd * self.dt)
    self.a = a_cmd
    return self.v, self.a, self.x
