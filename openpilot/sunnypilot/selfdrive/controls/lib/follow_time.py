"""
Single adjustable longitudinal follow time, replacing the three driving personalities.

The distance button steps it closer and the LKAS button steps it farther, both while engaged.
"""
import numpy as np

MIN_FOLLOW_TIME = 0.7  # s
MAX_FOLLOW_TIME = 2.0  # s
FOLLOW_TIME_STEP = 0.1  # s
DEFAULT_FOLLOW_TIME = 1.2  # s, matches the LongFollowTime param default

# dash distance bars: 1 bar < 1.0 s, 2 bars < 1.3 s, 3 bars < 1.7 s, else 4
_BAR_THRESHOLDS = (1.0, 1.3, 1.7)


def clip_follow_time(t: float) -> float:
  # round to the step grid so repeated +/-0.1 never drifts across a bar threshold
  return round(float(np.clip(t, MIN_FOLLOW_TIME, MAX_FOLLOW_TIME)), 1)


def step_follow_time(t: float, direction: int) -> float:
  return clip_follow_time(t + direction * FOLLOW_TIME_STEP)


def read_follow_time(params) -> float:
  try:
    return clip_follow_time(float(params.get("LongFollowTime", return_default=True)))
  except (TypeError, ValueError):
    return DEFAULT_FOLLOW_TIME


def get_follow_time(selfdrive_state_sp) -> float:
  # selfdrived owns the stepped follow time; never hand an unset 0.0 to the MPC or dash
  t = selfdrive_state_sp.followTime
  return clip_follow_time(t) if t > 0. else DEFAULT_FOLLOW_TIME


def follow_time_bars(t: float) -> int:
  return 1 + sum(round(t, 1) >= th for th in _BAR_THRESHOLDS)
