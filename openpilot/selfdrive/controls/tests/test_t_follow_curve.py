import unittest

import numpy as np

from openpilot.cereal import log, messaging

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.t_follow_curve import (
  parse_t_follow_curve, load_t_follow_curves, MIN_T_FOLLOW, MAX_T_FOLLOW, MODE_ADVANCED,
)

Personality = log.LongitudinalPersonality


class TestParseTFollowCurve(unittest.TestCase):
  def test_valid_multi_pair(self):
    curve = parse_t_follow_curve("20:1.3, 40:1.4, 60:1.6")
    assert curve is not None
    bp_v, t_vals = curve
    np.testing.assert_allclose(bp_v, np.array([20.0, 40.0, 60.0]) * CV.MPH_TO_MS)
    np.testing.assert_allclose(t_vals, [1.3, 1.4, 1.6])

  def test_single_pair_is_valid(self):
    curve = parse_t_follow_curve("30:1.5")
    assert curve is not None
    bp_v, t_vals = curve
    np.testing.assert_allclose(bp_v, [30.0 * CV.MPH_TO_MS])
    np.testing.assert_allclose(t_vals, [1.5])

  def test_whitespace_and_trailing_comma(self):
    self.assertIsNotNone(parse_t_follow_curve("  20:1.3 ,40:1.4 , "))

  def test_bounds_edges_ok(self):
    self.assertIsNotNone(parse_t_follow_curve(f"20:{MIN_T_FOLLOW}, 40:{MAX_T_FOLLOW}"))

  def test_invalid(self):
    for s in (
      "",                # empty
      "   ",             # blank
      "garbage",         # no colon
      "20",              # missing seconds
      "20:1.3:4",        # too many parts
      "abc:1.3",         # non-numeric speed
      "20:xyz",          # non-numeric seconds
      "20:0.7",          # below min
      "20:3.1",          # above max
      "-5:1.3",          # negative speed
      "40:1.3, 20:1.4",  # not strictly increasing
      "20:1.3, 20:1.4",  # equal speeds (not strictly increasing)
    ):
      with self.subTest(s=s):
        self.assertIsNone(parse_t_follow_curve(s))


def read_personality(val):
  # A capnp _DynamicEnum, exactly as the planner reads it off selfdriveState at runtime.
  # This is the type the MPC actually receives -- it hashes/compares by name, not int.
  ss = messaging.new_message('selfdriveState').selfdriveState
  ss.personality = val
  return ss.as_reader().personality


class TestMpcUsesCurve(unittest.TestCase):
  def _t_follow_after_update(self, mpc, v_ego, personality):
    # Capture the params at the moment update() hands them to the solver. Reading mpc.params
    # afterwards is unreliable: a non-zero solution_status makes run() reset(), which zeroes them.
    captured = {}
    run = mpc.run

    def spy():
      captured['params'] = mpc.params.copy()
      return run()

    mpc.run = spy
    mpc.set_cur_state(v_ego, 0.0)
    radarstate = messaging.new_message('radarState').radarState
    mpc.update(radarstate, personality=personality)
    return captured['params'][0, 4]  # lead_t_follow param fed to the solver

  def test_none_matches_stock(self):
    mpc = LongitudinalMpc()
    mpc.t_follow_curves = None
    personality = read_personality(Personality.standard)
    for v_ego in (5.0, 15.0, 30.0):
      with self.subTest(v_ego=v_ego):
        self.assertAlmostEqual(self._t_follow_after_update(mpc, v_ego, personality),
                               get_T_FOLLOW(personality), places=6)

  def test_curve_interpolates(self):
    mpc = LongitudinalMpc()
    curve = parse_t_follow_curve("20:1.2, 40:1.6, 60:2.0")
    assert curve is not None
    bp_v, t_vals = curve
    mpc.t_follow_curves = {"standard": (bp_v, t_vals)}
    personality = read_personality(Personality.standard)
    # at an anchor, below the range (flat hold), and between anchors
    for v_mph in (20.0, 10.0, 30.0):
      v_ego = v_mph * CV.MPH_TO_MS
      expected = float(np.interp(v_ego, bp_v, t_vals))
      with self.subTest(v_mph=v_mph):
        self.assertAlmostEqual(self._t_follow_after_update(mpc, v_ego, personality), expected, places=6)

  def test_missing_personality_falls_back_to_stock(self):
    mpc = LongitudinalMpc()
    curve = parse_t_follow_curve("20:1.2, 60:2.0")
    assert curve is not None
    bp_v, t_vals = curve
    mpc.t_follow_curves = {"standard": (bp_v, t_vals)}  # only standard configured
    personality = read_personality(Personality.aggressive)
    self.assertAlmostEqual(self._t_follow_after_update(mpc, 30.0, personality),
                           get_T_FOLLOW(personality), places=6)

  def test_curve_used_with_capnp_enum_personality(self):
    # On device, personality arrives as a capnp _DynamicEnum (not an int) via
    # sm['selfdriveState'].personality, and it hashes/compares by name. The curves
    # dict is name-keyed so this looks up directly; a regression here (e.g. reverting
    # to int keys) would silently fall back to stock.
    personality = read_personality(Personality.standard)
    self.assertNotIsInstance(personality, int)

    mpc = LongitudinalMpc()
    curve = parse_t_follow_curve("20:1.2, 40:1.6, 60:2.0")
    assert curve is not None
    bp_v, t_vals = curve
    mpc.t_follow_curves = {"standard": (bp_v, t_vals)}
    v_ego = 30.0 * CV.MPH_TO_MS
    expected = float(np.interp(v_ego, bp_v, t_vals))
    self.assertAlmostEqual(self._t_follow_after_update(mpc, v_ego, personality), expected, places=6)
    # and it must not be the stock value it would have fallen back to
    self.assertNotAlmostEqual(expected, get_T_FOLLOW(personality), places=6)


class _FakeParams:
  def __init__(self, enabled, strings, mode=MODE_ADVANCED):
    self._enabled = enabled
    self._strings = dict(strings)
    self._strings.setdefault("LongTFollowMode", str(mode))

  def get_bool(self, key):
    return self._enabled

  def get(self, key, return_default=False):
    return self._strings.get(key, "")


class TestLoadCurves(unittest.TestCase):
  def test_disabled_returns_none(self):
    params = _FakeParams(False, {"LongTFollowCurveStandard": "20:1.2, 40:1.6"})
    self.assertIsNone(load_t_follow_curves(params))

  def test_keys_are_personality_names_and_hit_the_runtime_enum(self):
    # load must key by name so the MPC's plain curves.get(personality) finds the curve
    # when personality is the capnp _DynamicEnum read off selfdriveState.
    params = _FakeParams(True, {
      "LongTFollowCurveRelaxed": "20:1.6, 60:2.0",
      "LongTFollowCurveStandard": "20:1.3, 60:1.7",
      "LongTFollowCurveAggressive": "20:1.0, 60:1.4",
    })
    curves = load_t_follow_curves(params)
    assert curves is not None
    self.assertEqual(set(curves), {"relaxed", "standard", "aggressive"})
    for val in (Personality.relaxed, Personality.standard, Personality.aggressive):
      self.assertIsNotNone(curves.get(read_personality(val)))

  def test_unparseable_personality_omitted(self):
    params = _FakeParams(True, {
      "LongTFollowCurveStandard": "20:1.3, 60:1.7",
      "LongTFollowCurveRelaxed": "garbage",
    })
    curves = load_t_follow_curves(params)
    assert curves is not None
    self.assertEqual(set(curves), {"standard"})

  def test_simple_mode_builds_flat_curves(self):
    params = _FakeParams(True, {
      "LongTFollowSimpleRelaxed": "1.75",
      "LongTFollowSimpleStandard": "1.45",
      "LongTFollowSimpleAggressive": "1.25",
    }, mode=0)
    curves = load_t_follow_curves(params)
    assert curves is not None
    self.assertEqual(set(curves), {"relaxed", "standard", "aggressive"})
    bp_v, t_vals = curves["standard"]
    self.assertAlmostEqual(float(np.interp(5.0, bp_v, t_vals)), 1.45)
    self.assertAlmostEqual(float(np.interp(35.0, bp_v, t_vals)), 1.45)
