"""
Speed nudge: a short RES+/SET- tap moves the driving speed 1 mph/kph (card counts the taps into
CarStateSP.speedNudge) instead of the set speed.

The e2e model has no set-speed input and barely defends its own pace: pushed off it, it pulls back about
0.01 m/s^2 per mph, and behind a lead it matches the lead's speed while ignoring the gap. So a short pull
sticks: the model holds the new speed, or the closer gap, as it does after a gas tap. Nothing persists
once the target is reached.

A tap sets a target speed; the e2e candidate is lifted to a jerk-limited arrival at it,
  e2e = a_model + lift,  lift -> gain * fade * max(0, floor - a_model),  floor = sqrt(2 * NUDGE_JERK * gap)
and the planner's min() still caps it at the lead MPC and the set speed. A tap down mirrors it as a
ceiling. The model's slowdowns win: the lift lets go as the model brakes or plans to slow, and in bends,
and the nudge is dropped once held off for DROP_TIME. Same approach (floor, gain, fade, never handing
added speed back) as zoompilot's Set Speed Assist, docs/zoompilot/e2e-set-speed.md.
"""
import math

import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants

LaneChangeState = log.LaneChangeState

NUDGE_JERK = 0.15  # m/s^3, eases into the target: 1 mph arrives in ~2.5 s (peak 0.37 m/s^2)
NUDGE_ACCEL_MAX = 0.6  # m/s^2, cap for stacked taps
NUDGE_RISE = 0.8  # m/s^3, the lift builds at about upstream's highway cruise jerk
LIFT_FALL = 2.4  # m/s^3, slowest the lift lets go

# The model's acceleration wanders +-0.15 m/s^2 in cruise; below this band it is slowing for something
GAIN_BP = [-0.3, -0.1]  # m/s^2
# A planned slowdown, read through GAIN_BP as its mean decel over HORIZON; a closing lead counts by time to contact
HORIZON = 10.0  # s
LEAD_TTC_BP = [15., 25.]  # s
# No lift where the road bends harder than this, now or ahead, at the current speed
FADE_LAT_BP = [0.6, 1.0]  # m/s^2
# Stop-and-go stays the model's
MIN_SPEED, FULL_SPEED = 5., 8.  # m/s

GATE_OPEN = 0.5  # gain * fade under this counts as held off
DROP_TIME = 1.0  # s held off before the rest of the nudge is dropped
EXPIRE_TIME = 15.0  # s after the last tap
RESYNC = 1.0  # m/s, a jump in the tap sum over ~2 taps in one model frame is card restarting, not taps

T_IDXS = np.asarray(ModelConstants.T_IDXS)
HORIZON_MASK = T_IDXS <= HORIZON


def planned_slowdown(v_ego: float, plan_v: np.ndarray, lead) -> float:
  """m/s the model is set to shed over the horizon: down to the plan's lowest speed, or a closing lead's
  closing speed as it nears contact."""
  drop = v_ego - float(np.min(plan_v[HORIZON_MASK]))
  if lead.present and lead.vRel < 0.:
    ttc = max(lead.dRel, 0.) / -lead.vRel
    drop = max(drop, -lead.vRel * float(np.interp(ttc, LEAD_TTC_BP, [1., 0.])))
  return drop


class SpeedNudge:
  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.last_sum: float | None = None
    self.lift = 0.
    self._idle()

  def _idle(self) -> None:
    self.v_target: float | None = None
    self.direction = 0
    self.age = 0.
    self.held_off = 0.
    self.floor = 0.
    self.gain = 0.
    self.fade = 0.

  @property
  def active(self) -> bool:
    return self.v_target is not None

  def update(self, sm, a_model: float, reset_state: bool) -> float:
    """Return the e2e candidate with the nudge applied."""
    nudge_sum = float(sm['carStateSP'].speedNudge)
    delta = 0. if self.last_sum is None else nudge_sum - self.last_sum
    self.last_sum = nudge_sum
    if abs(delta) > RESYNC:
      delta = 0.

    # a pedal, a disengagement or long control off: the driver owns the speed, the nudge is gone
    if reset_state:
      self._idle()
      self.lift = 0.
      return a_model

    v_ego = sm['carState'].vEgo
    if delta != 0.:
      self.v_target = (self.v_target if self.v_target is not None else v_ego) + delta
      self.direction = int(np.sign(self.v_target - v_ego))
      self.age = 0.
      self.held_off = 0.

    target_lift = 0.
    if self.v_target is not None:
      self.age += self.dt
      gap = self.v_target - v_ego
      if gap * self.direction <= 0. or self.age > EXPIRE_TIME:
        self._idle()  # arrived (or overshot), or stale
      elif self.direction > 0:
        target_lift = self._lift_up(sm, a_model, v_ego, gap)
      else:
        # slowing down is always allowed: a ceiling, so the model's own braking still passes under it
        self.floor = -min(NUDGE_ACCEL_MAX, math.sqrt(2. * NUDGE_JERK * -gap))
        self.gain = self.fade = 1.
        target_lift = min(0., self.floor - a_model)

    self._rate_limit(target_lift)
    return a_model + self.lift

  def _lift_up(self, sm, a_model: float, v_ego: float, gap: float) -> float:
    md = sm['modelV2']
    plan_v = np.asarray(md.velocity.x, dtype=float)
    yaw_rate = np.asarray(md.orientationRate.z, dtype=float)
    if len(plan_v) != ModelConstants.IDX_N or len(yaw_rate) != ModelConstants.IDX_N or \
       not (np.isfinite(plan_v).all() and np.isfinite(yaw_rate).all()):
      self._idle()
      return 0.

    self.floor = min(NUDGE_ACCEL_MAX, math.sqrt(2. * NUDGE_JERK * gap))
    plan_fade = float(np.interp(-planned_slowdown(v_ego, plan_v, sm['radarState'].leadOne) / HORIZON, GAIN_BP, [0., 1.]))
    self.gain = (min(float(np.interp(a_model, GAIN_BP, [0., 1.])), plan_fade) *
                 float(np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.])))
    # road geometry rather than the plan's speeds, which flicker; the measured curvature covers a bend the path reads low
    kappa_ahead = float(np.max((np.abs(yaw_rate) / np.maximum(plan_v, 1.))[HORIZON_MASK]))
    lat = max(abs(sm['controlsState'].curvature), kappa_ahead) * v_ego ** 2
    self.fade = float(np.interp(lat, FADE_LAT_BP, [1., 0.]))
    if md.meta.laneChangeState != LaneChangeState.off:
      self.fade = 0.

    gate = self.gain * self.fade
    self.held_off = self.held_off + self.dt if gate < GATE_OPEN else 0.
    if self.held_off >= DROP_TIME:
      self._idle()
      return 0.
    return gate * max(0., self.floor - a_model)

  def _rate_limit(self, target: float) -> None:
    rise, fall = NUDGE_RISE * self.dt, LIFT_FALL * self.dt
    if target >= 0. and self.lift >= 0.:
      # when the model starts to brake the gain drops, and the lift drops with it at once
      release = (1. - self.gain) * self.lift if self.active else 0.
      self.lift = float(np.clip(target, self.lift - max(fall, release), self.lift + rise))
    elif target <= 0. and self.lift <= 0.:
      self.lift = float(np.clip(target, self.lift - rise, self.lift + fall))
    else:
      self.lift = float(np.clip(0., self.lift - fall, self.lift + fall))  # changing direction: let go first

  def fill(self, msg) -> None:
    msg.active = self.active
    msg.vTarget = float(self.v_target or 0.)
    msg.floor = float(self.floor)
    msg.gain = float(self.gain)
    msg.fade = float(self.fade)
    msg.lift = float(self.lift)
