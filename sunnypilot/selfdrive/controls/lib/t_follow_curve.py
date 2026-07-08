"""
User-adjustable, speed-dependent T_FOLLOW curves.

Each longitudinal personality has an editable "mph:seconds, ..." string in params that defines a
follow-time curve over ego speed. The planner reads these once at construction and hands the
parsed curves to the MPC, which interpolates them per frame.
"""
import numpy as np

from cereal import log
from openpilot.common.constants import CV

MIN_T_FOLLOW = 0.8  # s
MAX_T_FOLLOW = 3.0  # s

LongitudinalPersonality = log.LongitudinalPersonality

# personality -> param holding its curve string
_CURVE_PARAMS = {
  int(LongitudinalPersonality.relaxed): "LongTFollowCurveRelaxed",
  int(LongitudinalPersonality.standard): "LongTFollowCurveStandard",
  int(LongitudinalPersonality.aggressive): "LongTFollowCurveAggressive",
}


def parse_t_follow_curve(s: str) -> tuple[np.ndarray, np.ndarray] | None:
  """Parse a "mph:seconds, ..." string into (bp_v_ms, t_vals) for np.interp, or None if invalid.

  Valid requires: >=1 pair, each second in [MIN_T_FOLLOW, MAX_T_FOLLOW], speeds >= 0 and
  strictly increasing. Speeds are converted mph -> m/s.
  """
  speeds_mph: list[float] = []
  t_vals: list[float] = []
  for pair in s.split(","):
    pair = pair.strip()
    if not pair:
      continue
    parts = pair.split(":")
    if len(parts) != 2:
      return None
    try:
      mph = float(parts[0])
      sec = float(parts[1])
    except ValueError:
      return None
    if mph < 0.0 or not (MIN_T_FOLLOW <= sec <= MAX_T_FOLLOW):
      return None
    if speeds_mph and mph <= speeds_mph[-1]:  # strictly increasing
      return None
    speeds_mph.append(mph)
    t_vals.append(sec)

  if not speeds_mph:
    return None

  return np.array(speeds_mph) * CV.MPH_TO_MS, np.array(t_vals)


def load_t_follow_curves(params) -> dict[int, tuple[np.ndarray, np.ndarray]] | None:
  """Load the per-personality T_FOLLOW curves, or None to use the stock get_T_FOLLOW path.

  Returns None when the feature is disabled or no stored curve parses. A personality whose
  stored string fails to parse is omitted, so the planner falls back to stock for it.
  """
  if not params.get_bool("LongTFollowCustomEnabled"):
    return None

  curves: dict[int, tuple[np.ndarray, np.ndarray]] = {}
  for personality, key in _CURVE_PARAMS.items():
    curve = parse_t_follow_curve(params.get(key, return_default=True))
    if curve is not None:
      curves[personality] = curve

  return curves or None
