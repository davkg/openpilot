import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  BlendedTransitioner,
  BLENDED_TRANSITION_JERK,
  BLENDED_TRANSITION_START_JERK,
  BLENDED_TRANSITION_HARD_A,
)


def drive(easer, steps, prev0=0.0):
  """Feed (raw_a_target, a_target_mpc, is_e2e, reset_state) tuples, threading prev_a_target the way
  the planner does (the eased output becomes next tick's baseline). Returns the list of outputs."""
  prev = prev0
  outs = []
  for raw, mpc, is_e2e, reset in steps:
    out = easer.update(raw, mpc, prev, is_e2e, reset)
    prev = out
    outs.append(out)
  return outs


def max_step():
  return BLENDED_TRANSITION_JERK * DT_MDL


class TestBlendedTransitioner:
  def test_steady_acc_passes_through(self):
    # No mode change: output tracks the raw target exactly, no easing engaged.
    e = BlendedTransitioner(DT_MDL)
    outs = drive(e, [(-0.4, -0.4, False, False)] * 10)
    assert all(o == -0.4 for o in outs)
    assert not e.transitioning

  def test_reset_passes_through(self):
    # reset_state always bypasses easing even across an is_e2e edge.
    e = BlendedTransitioner(DT_MDL)
    outs = drive(e, [(0.0, 0.0, False, True), (-1.0, 0.0, True, True)])
    assert outs[-1] == -1.0
    assert not e.transitioning

  def test_onset_eases_in(self):
    # acc(0) -> blended(-1.0): first tick is the soft START jerk, then it ramps in, bounded by the
    # max jerk every tick, monotonically approaching the target.
    e = BlendedTransitioner(DT_MDL)
    steps = [(0.0, 0.0, False, False)] + [(-1.0, 0.0, True, False)] * 40
    outs = drive(e, steps)

    assert outs[0] == 0.0                                              # steady acc
    assert outs[1] == -BLENDED_TRANSITION_START_JERK * DT_MDL          # first handoff tick is soft
    deltas = np.diff(outs[1:])
    assert np.all(deltas <= 1e-9)                                      # monotonically more braking
    assert np.all(np.abs(deltas) <= max_step() + 1e-9)                # never exceeds max jerk
    assert outs[-1] == -1.0 and not e.transitioning                   # caught up to the target

  def test_release_eases_out(self):
    # blended(-1.2, mpc only -0.3) -> acc: braking is released gradually, not snapped, bounded by
    # the max jerk, monotonically rising to the acc target.
    e = BlendedTransitioner(DT_MDL)
    steps = [(-1.2, -0.3, True, False)] * 20 + [(-0.3, -0.3, False, False)] * 40
    outs = drive(e, steps)

    release = outs[20:]
    assert release[0] > -1.2                                          # did not snap to -0.3
    deltas = np.diff(release)
    assert np.all(deltas >= -1e-9)                                    # monotonically less braking
    assert np.all(np.abs(deltas) <= max_step() + 1e-9)               # never exceeds max jerk
    assert outs[-1] == -0.3 and not e.transitioning                  # caught up to acc target

  def test_hard_onset_bypasses_easing(self):
    # An emergency-firm braking onset (below HARD_A) is applied immediately, no easing.
    e = BlendedTransitioner(DT_MDL)
    raw = BLENDED_TRANSITION_HARD_A - 0.5
    outs = drive(e, [(0.0, 0.0, False, False)] + [(raw, 0.0, True, False)] * 3)
    assert all(o == raw for o in outs[1:])
    assert not e.transitioning

  def test_mpc_braking_passes_through_during_release(self):
    # If the MPC wants firmer braking part-way through a release ease, it is applied immediately
    # (never brake less than the MPC wants).
    e = BlendedTransitioner(DT_MDL)
    steps = [(-1.2, -0.3, True, False)] * 20 + [(-0.3, -0.3, False, False)] * 3
    drive(e, steps)
    # mid-release, MPC suddenly demands hard braking
    prev = e.update(-0.3, -0.3, -0.5, False, False)  # continue the release from ~-0.5
    out = e.update(-1.8, -1.8, prev, False, False)   # MPC firm braking (below HARD_A)
    assert out == -1.8

  def test_intra_blended_increase_not_rearmed(self):
    # While already in blended (is_e2e stays True), the model increasing its braking does NOT
    # re-arm an ease -- it passes straight through once the initial onset has caught up.
    e = BlendedTransitioner(DT_MDL)
    drive(e, [(0.0, 0.0, False, False)] + [(-0.3, -0.3, True, False)] * 30)  # onset settles
    assert not e.transitioning
    out = e.update(-0.5, -0.3, -0.3, True, False)  # e2e now wants more braking, still blended
    assert out == -0.5            # passed through, not eased
    assert not e.transitioning

  def test_onset_then_release_round_trip(self):
    # A full acc -> blended -> acc cycle ends back at the acc target with the latch cleared.
    e = BlendedTransitioner(DT_MDL)
    steps = ([(0.0, 0.0, False, False)] +
             [(-1.0, 0.0, True, False)] * 40 +
             [(0.0, 0.0, False, False)] * 40)
    outs = drive(e, steps)
    assert outs[-1] == 0.0
    assert not e.transitioning
