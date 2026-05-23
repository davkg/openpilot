"""
MPC-driven lead car for plant.py.

Wraps a separate LongitudinalMpc as a "virtual driver" for the lead.
The lead's per-step v_cruise (typically the maneuver's interpolated
speed_lead_values) is fed into the MPC, which produces a realistic
jerk-limited acceleration profile -- no corner-y linear-interp velocity
changes.

The lead's MPC sees no leads of its own, so its behavior is shaped
purely by the cruise schedule.
"""
import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc


_STANDARD = log.LongitudinalPersonality.standard


class MpcLead:
  def __init__(self, v0=0.0, a0=0.0, x0=0.0, dt=DT_MDL):
    self.dt = dt
    self.mpc = LongitudinalMpc(dt=dt)
    self.mpc.mode = 'acc'
    self.v = float(v0)
    self.a = float(a0)
    self.x = float(x0)
    self.mpc.set_weights(personality=_STANDARD, v_ego=self.v)
    self.mpc.set_cur_state(self.v, self.a)
    self._radar = messaging.new_message('radarState').radarState

  def step(self, v_cruise):
    self.mpc.set_weights(personality=_STANDARD, v_ego=self.v)
    self.mpc.set_cur_state(self.v, self.a)
    self.mpc.update(self._radar, float(v_cruise), personality=_STANDARD)
    # a_solution[1] is the planned accel one MPC step ahead -- what the lead
    # will execute over the next dt.
    a_cmd = float(self.mpc.a_solution[1])
    self.x = self.x + self.v * self.dt + 0.5 * a_cmd * self.dt * self.dt
    self.v = max(0.0, self.v + a_cmd * self.dt)
    self.a = a_cmd
    return self.v, self.a, self.x
