"""
Checkerboard staggering — read adjacent-vehicle tracks from
`carStateSP.cameraTracks` and propose a slow-only v_cruise bias to break
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
EGO_LANE_HALF_W = 2.0
ADJACENT_LANE_OUTER = 4.5

# Forward window (m, d_rel) for tracks to enter pacing logic
LONG_WINDOW_MIN = -10.0
LONG_WINDOW_MAX = 70.0

# Pacing zone (m, d_rel) — longitudinal range where ego is "door-to-door"-ish with an adjacent car.
# +3 m roughly = ego's front bumper aligned with adjacent car's rear bumper (empirical at the corrected
# LONG_DIST scale 0.209/-16.9; measured on route 000000cf seg20 ~16s / PlotJuggler 1216s, 2026-06-11).
PACING_ZONE_MIN = -5.0
PACING_ZONE_MAX = 3.0

# Hysteresis on pacing-zone occupancy
ENTER_TICKS = 3   # consecutive ticks in zone to engage
EXIT_TICKS = 8    # consecutive ticks out of zone to disengage

# Traffic density gate — in dense traffic there's no room to checkerboard, so suppress engagement.
# Counts valid adjacent-lane tracks (both sides) within the density window. >= threshold → crowded.
DENSITY_WINDOW_MIN = -15.0
DENSITY_WINDOW_MAX = 50.0
DENSITY_THRESHOLD = 3

# Relative-motion gate — suppress engagement when an adjacent car is actively passing
# (i.e. relative longitudinal velocity is high). True pacing has |v_dot| ≈ 0.
# Hysteresis: tighter threshold to enter, looser to stay engaged through small fluctuations.
V_DOT_GATE_ENGAGE_MS = 0.5   # m/s (~1 mph); below this to start pacing
V_DOT_GATE_HOLD_MS = 1.0     # m/s (~2 mph); above this disengages even while paced
TRACK_CACHE_TTL_FRAMES = 20  # drop cached prior d_rel after this many ticks (~1 sec at DT_MDL)

# Per-objectId dwell required (s) before a track counts as pacing. The camera-side
# tracker can briefly report ~0 relative motion for ~1 s after a track is born,
# falsely satisfying the v_dot gate for fast-passing cars; requiring continuous
# zone occupancy for this long lets the v_dot estimate settle before we engage.
DWELL_REQUIRED_S = 6.0

# Ego-stability gate — pacing is a "both cars cruising" scenario. If ego itself is
# actively accelerating/decelerating, the situation is dynamic (merging, catching up,
# braking into traffic) and pacing intent doesn't apply.
EGO_STABLE_A_MAX_MS2 = 0.3   # m/s² (~0.7 mph/s); |a_ego| must be below this to engage

# Minimum ego speed (m/s) for engagement. Below this, surface streets are usually too
# busy/dynamic for checkerboarding to be practical.
MPH_TO_MS = 0.44704
V_EGO_MIN_MS = 50.0 * MPH_TO_MS

# Aggression → absolute v_cruise delta cap (m/s)
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
  pacing_side: int = 0  # +1 = pacing a left-adjacent car (left-lane only for now), 0 = not pacing

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

    # object_id → (d_rel, frame) for per-track relative-velocity estimation
    self._track_cache: dict[int, tuple[float, int]] = {}

    # object_id → frame when track first entered the (lane band + pacing zone). Cleared
    # when track exits either. Drives the per-track DWELL_REQUIRED_S gate.
    self._zone_entry_frame: dict[int, int] = {}

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

  def _eligible_for_pacing(self, t) -> bool:
    """Track is in the left-adjacent lane and within the forward window.

    Left-lane only (y_rel positive = left of ego, per DBC): right-side pacing wants a
    speed-up, which our slow-only authority can't express via the planner's min-arbitration,
    so it's left to the driver.
    """
    if not t.valid:
      return False
    if not (LONG_WINDOW_MIN <= t.dRel <= LONG_WINDOW_MAX):
      return False
    if t.yRel < 0:
      return False
    abs_y = abs(t.yRel)
    if not (EGO_LANE_HALF_W <= abs_y <= ADJACENT_LANE_OUTER):
      return False
    return True

  @staticmethod
  def _in_pacing_zone(t) -> bool:
    return PACING_ZONE_MIN <= t.dRel <= PACING_ZONE_MAX

  def _has_dwelled(self, t) -> bool:
    """True when this objectId has been continuously in the (lane band + pacing zone)
    for at least DWELL_REQUIRED_S seconds — gates out newly-appearing tracks whose
    v_dot estimate hasn't yet settled."""
    entry = self._zone_entry_frame.get(int(t.objectId))
    if entry is None:
      return False
    return (self.frame - entry) * DT_MDL >= DWELL_REQUIRED_S

  def _v_dot_for(self, t) -> float:
    """Per-track relative longitudinal velocity (m/s). Returns +inf if no prior history,
    which effectively rejects engagement until two consecutive ticks have been seen for
    this object_id.
    """
    prev = self._track_cache.get(int(t.objectId))
    if prev is None:
      return float('inf')
    d_prev, frame_prev = prev
    dframes = self.frame - frame_prev
    if dframes <= 0:
      return 0.0
    return (t.dRel - d_prev) / (dframes * DT_MDL)

  @staticmethod
  def _in_density_window(t) -> bool:
    """Either adjacent lane, within the density-counting forward window."""
    if not t.valid:
      return False
    if not (DENSITY_WINDOW_MIN <= t.dRel <= DENSITY_WINDOW_MAX):
      return False
    abs_y = abs(t.yRel)
    return EGO_LANE_HALF_W <= abs_y <= ADJACENT_LANE_OUTER

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
      self._zone_entry_frame.clear()
      self._v_bias_filter.update(0.0)
      self._t_follow_filter.update(0.0)
      self.state = CheckerboardState.overriding if long_override else CheckerboardState.disabled
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = 0.0
      self.t_follow_delta = 0.0
      self.pacing_side = 0
      self.is_enabled = self._enabled and long_enabled
      self.is_active = False
      return

    tracks = sm['carStateSP'].cameraTracks

    # ── Outer gates (whole-scene, not per-track) ──────────────────────────────────────
    # Ego must be cruising, not actively accelerating/braking.
    ego_stable = abs(a_ego) < EGO_STABLE_A_MAX_MS2
    # Highway-only: below ~50 mph, surface streets are too busy for staggering to be practical.
    v_ego_ok = v_ego >= V_EGO_MIN_MS
    # Adjacent-lane traffic density. Once already pacing, the hysteresis will slew us out
    # naturally via EXIT_TICKS rather than slamming off.
    crowded = sum(1 for t in tracks if self._in_density_window(t)) >= DENSITY_THRESHOLD

    # ── Per-track zone-dwell tracking ─────────────────────────────────────────────────
    # Record the first frame each objectId appears in (lane band + pacing zone); drop
    # entries whose track is no longer there. Drives DWELL_REQUIRED_S gate below.
    in_zone_ids: set[int] = set()
    for t in tracks:
      if not t.valid or not self._in_pacing_zone(t):
        continue
      abs_y = abs(t.yRel)
      if EGO_LANE_HALF_W <= abs_y <= ADJACENT_LANE_OUTER:
        oid = int(t.objectId)
        in_zone_ids.add(oid)
        self._zone_entry_frame.setdefault(oid, self.frame)
    self._zone_entry_frame = {oid: f for oid, f in self._zone_entry_frame.items() if oid in in_zone_ids}

    # ── Per-track gates ───────────────────────────────────────────────────────────────
    # Relative-motion threshold (hysteresis): tighter to enter, looser to hold engagement
    # through small fluctuations.
    v_dot_gate = V_DOT_GATE_HOLD_MS if self._is_pacing else V_DOT_GATE_ENGAGE_MS

    def _is_pacing_track(t) -> bool:
      return (self._eligible_for_pacing(t)             # validity, forward window, lane band, side rule
              and self._in_pacing_zone(t)              # d_rel in door-to-door range
              and self._has_dwelled(t)                 # been in zone long enough for v_dot to settle
              and abs(self._v_dot_for(t)) < v_dot_gate)  # relative motion small

    pacing_tracks = [t for t in tracks if _is_pacing_track(t)] if (ego_stable and not crowded and v_ego_ok) else []
    any_in_zone = bool(pacing_tracks)

    # Left-lane only, so the side is always left when pacing. Sticky through EXIT_TICKS
    # hold-out; only cleared when fully disengaged below.
    if pacing_tracks:
      self.pacing_side = 1

    # Update per-track velocity cache for next tick (after the gate check).
    for t in tracks:
      if t.valid:
        self._track_cache[int(t.objectId)] = (t.dRel, self.frame)
    # Prune stale entries
    cutoff = self.frame - TRACK_CACHE_TTL_FRAMES
    self._track_cache = {oid: v for oid, v in self._track_cache.items() if v[1] >= cutoff}

    self._step_hysteresis(any_in_zone)

    if not self._is_pacing:
      self.pacing_side = 0

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
