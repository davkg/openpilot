import numpy as np
import pytest

from cereal import log, messaging

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.t_follow_curve import (
  parse_t_follow_curve, load_t_follow_curves, MIN_T_FOLLOW, MAX_T_FOLLOW,
)

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


def read_personality(val):
  # A capnp _DynamicEnum, exactly as the planner reads it off selfdriveState at runtime.
  # This is the type the MPC actually receives -- it hashes/compares by name, not int.
  ss = messaging.new_message('selfdriveState').selfdriveState
  ss.personality = val
  return ss.as_reader().personality


class TestMpcUsesCurve:
  def _t_follow_after_update(self, mpc, v_ego, personality):
    mpc.set_cur_state(v_ego, 0.0)
    radarstate = messaging.new_message('radarState').radarState
    mpc.update(radarstate, v_cruise=v_ego, personality=personality)
    return mpc.params[0, 4]  # lead_t_follow param fed to the solver

  def test_none_matches_stock(self):
    mpc = LongitudinalMpc()
    mpc.t_follow_curves = None
    personality = read_personality(Personality.standard)
    for v_ego in (5.0, 15.0, 30.0):
      assert self._t_follow_after_update(mpc, v_ego, personality) == \
        pytest.approx(get_T_FOLLOW(personality, v_ego))

  def test_curve_interpolates(self):
    mpc = LongitudinalMpc()
    bp_v, t_vals = parse_t_follow_curve("20:1.2, 40:1.6, 60:2.0")
    mpc.t_follow_curves = {"standard": (bp_v, t_vals)}
    personality = read_personality(Personality.standard)
    # at an anchor, below the range (flat hold), and between anchors
    for v_mph in (20.0, 10.0, 30.0):
      v_ego = v_mph * CV.MPH_TO_MS
      expected = float(np.interp(v_ego, bp_v, t_vals))
      assert self._t_follow_after_update(mpc, v_ego, personality) == pytest.approx(expected)

  def test_missing_personality_falls_back_to_stock(self):
    mpc = LongitudinalMpc()
    bp_v, t_vals = parse_t_follow_curve("20:1.2, 60:2.0")
    mpc.t_follow_curves = {"standard": (bp_v, t_vals)}  # only standard configured
    personality = read_personality(Personality.aggressive)
    v_ego = 30.0
    assert self._t_follow_after_update(mpc, v_ego, personality) == \
      pytest.approx(get_T_FOLLOW(personality, v_ego))

  def test_curve_used_with_capnp_enum_personality(self):
    # On device, personality arrives as a capnp _DynamicEnum (not an int) via
    # sm['selfdriveState'].personality, and it hashes/compares by name. The curves
    # dict is name-keyed so this looks up directly; a regression here (e.g. reverting
    # to int keys) would silently fall back to stock.
    personality = read_personality(Personality.standard)
    assert not isinstance(personality, int)

    mpc = LongitudinalMpc()
    bp_v, t_vals = parse_t_follow_curve("20:1.2, 40:1.6, 60:2.0")
    mpc.t_follow_curves = {"standard": (bp_v, t_vals)}
    v_ego = 30.0 * CV.MPH_TO_MS
    expected = float(np.interp(v_ego, bp_v, t_vals))
    assert self._t_follow_after_update(mpc, v_ego, personality) == pytest.approx(expected)
    # and it must not be the stock value it would have fallen back to
    assert expected != pytest.approx(get_T_FOLLOW(personality, v_ego))


class _FakeParams:
  def __init__(self, enabled, strings):
    self._enabled = enabled
    self._strings = strings

  def get_bool(self, key):
    return self._enabled

  def get(self, key, return_default=False):
    return self._strings.get(key, "")


class TestLoadCurves:
  def test_disabled_returns_none(self):
    params = _FakeParams(False, {"LongTFollowCurveStandard": "20:1.2, 40:1.6"})
    assert load_t_follow_curves(params) is None

  def test_keys_are_personality_names_and_hit_the_runtime_enum(self):
    # load must key by name so the MPC's plain curves.get(personality) finds the curve
    # when personality is the capnp _DynamicEnum read off selfdriveState.
    params = _FakeParams(True, {
      "LongTFollowCurveRelaxed": "20:1.6, 60:2.0",
      "LongTFollowCurveStandard": "20:1.3, 60:1.7",
      "LongTFollowCurveAggressive": "20:1.0, 60:1.4",
    })
    curves = load_t_follow_curves(params)
    assert set(curves) == {"relaxed", "standard", "aggressive"}
    for val in (Personality.relaxed, Personality.standard, Personality.aggressive):
      assert curves.get(read_personality(val)) is not None

  def test_unparseable_personality_omitted(self):
    params = _FakeParams(True, {
      "LongTFollowCurveStandard": "20:1.3, 60:1.7",
      "LongTFollowCurveRelaxed": "garbage",
    })
    curves = load_t_follow_curves(params)
    assert set(curves) == {"standard"}
