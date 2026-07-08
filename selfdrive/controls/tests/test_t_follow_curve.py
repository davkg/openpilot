import numpy as np
import pytest

from cereal import log, messaging

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.t_follow_curve import parse_t_follow_curve, MIN_T_FOLLOW, MAX_T_FOLLOW

Personality = log.LongitudinalPersonality


class TestParseTFollowCurve:
  def test_valid_multi_pair(self):
    bp_v, t_vals = parse_t_follow_curve("20:1.3, 40:1.4, 60:1.6")
    np.testing.assert_allclose(bp_v, np.array([20.0, 40.0, 60.0]) * CV.MPH_TO_MS)
    np.testing.assert_allclose(t_vals, [1.3, 1.4, 1.6])

  def test_single_pair_is_valid(self):
    bp_v, t_vals = parse_t_follow_curve("30:1.5")
    np.testing.assert_allclose(bp_v, [30.0 * CV.MPH_TO_MS])
    np.testing.assert_allclose(t_vals, [1.5])

  def test_whitespace_and_trailing_comma(self):
    assert parse_t_follow_curve("  20:1.3 ,40:1.4 , ") is not None

  def test_bounds_edges_ok(self):
    assert parse_t_follow_curve(f"20:{MIN_T_FOLLOW}, 40:{MAX_T_FOLLOW}") is not None

  @pytest.mark.parametrize("s", [
    "",                       # empty
    "   ",                    # blank
    "garbage",                # no colon
    "20",                     # missing seconds
    "20:1.3:4",               # too many parts
    "abc:1.3",                # non-numeric speed
    "20:xyz",                 # non-numeric seconds
    "20:0.7",                 # below min
    "20:3.1",                 # above max
    "-5:1.3",                 # negative speed
    "40:1.3, 20:1.4",         # not strictly increasing
    "20:1.3, 20:1.4",         # equal speeds (not strictly increasing)
  ])
  def test_invalid(self, s):
    assert parse_t_follow_curve(s) is None


class TestMpcUsesCurve:
  def _t_follow_after_update(self, mpc, v_ego, personality):
    mpc.set_cur_state(v_ego, 0.0)
    radarstate = messaging.new_message('radarState').radarState
    mpc.update(radarstate, v_cruise=v_ego, personality=personality)
    return mpc.params[0, 4]  # lead_t_follow param fed to the solver

  def test_none_matches_stock(self):
    mpc = LongitudinalMpc()
    mpc.t_follow_curves = None
    for v_ego in (5.0, 15.0, 30.0):
      assert self._t_follow_after_update(mpc, v_ego, Personality.standard) == \
        pytest.approx(get_T_FOLLOW(Personality.standard, v_ego))

  def test_curve_interpolates(self):
    mpc = LongitudinalMpc()
    bp_v, t_vals = parse_t_follow_curve("20:1.2, 40:1.6, 60:2.0")
    mpc.t_follow_curves = {int(Personality.standard): (bp_v, t_vals)}
    # at an anchor, below the range (flat hold), and between anchors
    for v_mph in (20.0, 10.0, 30.0):
      v_ego = v_mph * CV.MPH_TO_MS
      expected = float(np.interp(v_ego, bp_v, t_vals))
      assert self._t_follow_after_update(mpc, v_ego, Personality.standard) == pytest.approx(expected)

  def test_missing_personality_falls_back_to_stock(self):
    mpc = LongitudinalMpc()
    bp_v, t_vals = parse_t_follow_curve("20:1.2, 60:2.0")
    mpc.t_follow_curves = {int(Personality.standard): (bp_v, t_vals)}  # only standard configured
    v_ego = 30.0
    assert self._t_follow_after_update(mpc, v_ego, Personality.aggressive) == \
      pytest.approx(get_T_FOLLOW(Personality.aggressive, v_ego))
