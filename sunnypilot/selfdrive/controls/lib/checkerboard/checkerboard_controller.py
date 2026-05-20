"""
Checkerboard staggering — read adjacent-vehicle tracks from
`cameraObjectTracksSP` and propose a slow-only v_cruise bias to break
door-to-door pacing with an adjacent-lane car. The output participates in
`LongitudinalPlannerSP.update_targets`' min() arbitration like SCC/SLA.

Forward FOV only (no rear-blind-spot handling); slow-only authority (we
never propose to exceed v_cruise); slew-limited; hysteresis on entry/exit.
"""
import cereal.messaging as messaging
from cereal import custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

CheckerboardState = custom.LongitudinalPlanSP.Checkerboard.CheckerboardState

# Lane band (m, |y_rel|) — only adjacent lanes are interesting; ego-lane handled by lead logic
EGO_LANE_HALF_W = 1.5
ADJACENT_LANE_OUTER = 4.5

# Forward window (m, d_rel) for tracks to enter pacing logic
LONG_WINDOW_MIN = -5.0
LONG_WINDOW_MAX = 60.0

# Pacing zone (m, d_rel) — longitudinal range where ego is "door-to-door"-ish with an adjacent car
PACING_ZONE_MIN = -3.0
PACING_ZONE_MAX = 5.0

# Hysteresis on pacing-zone occupancy
ENTER_TICKS = 3   # consecutive ticks in zone to engage
EXIT_TICKS = 8    # consecutive ticks out of zone to disengage

# Aggression → absolute v_cruise delta cap (m/s)
MPH_TO_MS = 0.44704
DELTA_CAPS_MS = [
  1.0 * MPH_TO_MS,
  2.0 * MPH_TO_MS,
  3.0 * MPH_TO_MS,
  5.0 * MPH_TO_MS,
]

# Aggression → T_FOLLOW delta cap (s). Always additive (only lengthens follow distance).
T_FOLLOW_DELTA_CAPS_S = [0.10, 0.20, 0.30, 0.40]

V_BIAS_TAU_S = 0.2   # slew time constant for the v_cruise bias output
T_FOLLOW_TAU_S = 0.5  # slower slew on T_FOLLOW (felt more strongly than a v_cruise nudge)


class CheckerboardController:
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.0
  t_follow_delta: float = 0.0  # phase 4 will drive this

  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.state = CheckerboardState.disabled

    self._enabled = self.params.get_bool("CheckerboardStaggeringEnabled")
    self._aggression = int(self.params.get("CheckerboardStaggeringAggression", return_default=True))
    self._delta_cap = DELTA_CAPS_MS[max(0, min(3, self._aggression))]
    self._t_follow_cap = T_FOLLOW_DELTA_CAPS_S[max(0, min(3, self._aggression))]

    self._pacing_ticks_in = 0
    self._pacing_ticks_out = 0
    self._is_pacing = False

    self._v_bias_filter = FirstOrderFilter(0.0, V_BIAS_TAU_S, DT_MDL)
    self._t_follow_filter = FirstOrderFilter(0.0, T_FOLLOW_TAU_S, DT_MDL)

    self.is_enabled = False
    self.is_active = False

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self._enabled = self.params.get_bool("CheckerboardStaggeringEnabled")
      try:
        self._aggression = int(self.params.get("CheckerboardStaggeringAggression", return_default=True))
      except (TypeError, ValueError):
        self._aggression = 1
      self._delta_cap = DELTA_CAPS_MS[max(0, min(3, self._aggression))]
      self._t_follow_cap = T_FOLLOW_DELTA_CAPS_S[max(0, min(3, self._aggression))]

  @staticmethod
  def _eligible_for_pacing(t) -> bool:
    """Track is in an adjacent lane and within the forward window."""
    if not t.valid:
      return False
    if not (LONG_WINDOW_MIN <= t.dRel <= LONG_WINDOW_MAX):
      return False
    abs_y = abs(t.yRel)
    return EGO_LANE_HALF_W <= abs_y <= ADJACENT_LANE_OUTER

  @staticmethod
  def _in_pacing_zone(t) -> bool:
    return PACING_ZONE_MIN <= t.dRel <= PACING_ZONE_MAX

  def _step_hysteresis(self, any_in_zone: bool) -> None:
    if not self._is_pacing:
      if any_in_zone:
        self._pacing_ticks_in += 1
        if self._pacing_ticks_in >= ENTER_TICKS:
          self._is_pacing = True
          self._pacing_ticks_out = 0
      else:
        self._pacing_ticks_in = 0
    else:
      if not any_in_zone:
        self._pacing_ticks_out += 1
        if self._pacing_ticks_out >= EXIT_TICKS:
          self._is_pacing = False
          self._pacing_ticks_in = 0
      else:
        self._pacing_ticks_out = 0

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool,
             v_ego: float, a_ego: float, v_cruise_setpoint: float) -> None:
    self.frame += 1
    self._update_params()

    # Inactive paths — clear state, output unset so the planner's min() ignores us
    if not self._enabled or not long_enabled or long_override:
      self._is_pacing = False
      self._pacing_ticks_in = 0
      self._pacing_ticks_out = 0
      self._v_bias_filter.update(0.0)
      self._t_follow_filter.update(0.0)
      self.state = CheckerboardState.overriding if long_override else CheckerboardState.disabled
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = 0.0
      self.t_follow_delta = 0.0
      self.is_enabled = self._enabled and long_enabled
      self.is_active = False
      return

    # Find adjacent-lane tracks within the forward window
    tracks = sm['cameraObjectTracksSP'].tracks
    any_in_zone = any(self._eligible_for_pacing(t) and self._in_pacing_zone(t) for t in tracks)

    self._step_hysteresis(any_in_zone)

    target_bias = -self._delta_cap if self._is_pacing else 0.0
    target_t_follow = self._t_follow_cap if self._is_pacing else 0.0
    self._v_bias_filter.update(target_bias)
    self._t_follow_filter.update(target_t_follow)
    v_bias = self._v_bias_filter.x
    self.t_follow_delta = max(0.0, self._t_follow_filter.x)

    if abs(v_bias) > 0.01:
      self.state = CheckerboardState.pacing
      # Slow-only: bias must be negative; output the (cruise - |bias|) target
      self.output_v_target = max(0.0, v_cruise_setpoint + v_bias)
      self.output_a_target = a_ego  # let MPC choose accel; we only constrain v
      self.is_active = True
    else:
      self.state = CheckerboardState.enabled
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = 0.0
      self.is_active = False

    self.is_enabled = True
