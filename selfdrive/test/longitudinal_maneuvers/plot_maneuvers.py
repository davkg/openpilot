#!/usr/bin/env python3
"""
Tuning dashboards for longitudinal maneuvers.

Defines a curated maneuver suite, runs it against the real planner/MPC
across all three personalities, and emits one annotated PNG per
(maneuver, personality) plus a `maneuver_scores.csv` scoreboard. Each
dashboard shows ego/lead/set speed, commanded acceleration + jerk,
gap-to-lead vs desired follow gap, and time headway vs t_follow -- with
the background shaded by which MPC obstacle is binding (cruise / lead0 /
lead1). On the accel axis: coast-window shading (light decel band), the
-COMFORT_BRAKE reference line, and jerk-spike markers (>8 m/s^3).
Nothing is asserted; this is a visualization aid for tuning long_mpc.

Run: .venv/bin/python selfdrive/test/longitudinal_maneuvers/plot_maneuvers.py
"""
import argparse
import csv
import os
import re
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import FuncFormatter

from cereal import log
import openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc as _long_mpc_mod
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  COMFORT_BRAKE, get_STOP_DISTANCE, get_T_FOLLOW,
)
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant


def _solver_constants_hash():
  """Hash of the long_mpc.py constants that get baked into the C solver.

  Only values referenced inside `gen_long_ocp` (the CasADi expressions that
  become the compiled QP) require a rebuild. Everything else (cost weights
  set via cost_set, slack costs via Zl, lead_danger_factor / t_follow /
  stop_distance via params, cruise accel clipping, FCW thresholds, coast
  bias) is runtime Python and doesn't need a rebuild.

  Baked into the C solver:
    - COMFORT_BRAKE: appears in get_safe_obstacle_distance() which is called
      inside gen_long_ocp to build the desired_dist_comfort CasADi expression
      used in both cost and constraint.
    - N, MAX_T: define the horizon structure (T_IDXS array baked into Tf,
      ocp.dims.N, and integrator timesteps).
    - X_DIM, U_DIM, PARAM_DIM, COST_DIM, COST_E_DIM, CONSTR_DIM: structural
      dimensions baked into the C array sizes.
  """
  import hashlib
  parts = [
    f'COMFORT_BRAKE={_long_mpc_mod.COMFORT_BRAKE}',
    f'N={_long_mpc_mod.N}',
    f'MAX_T={_long_mpc_mod.MAX_T}',
    f'X_DIM={_long_mpc_mod.X_DIM}',
    f'U_DIM={_long_mpc_mod.U_DIM}',
    f'PARAM_DIM={_long_mpc_mod.PARAM_DIM}',
    f'COST_DIM={_long_mpc_mod.COST_DIM}',
    f'COST_E_DIM={_long_mpc_mod.COST_E_DIM}',
    f'CONSTR_DIM={_long_mpc_mod.CONSTR_DIM}',
  ]
  return hashlib.md5('|'.join(parts).encode()).hexdigest()


def _check_solver_freshness():
  """Warn and exit if the constants baked into the C solver don't match
  what long_mpc.py currently has. The check is content-based, not mtime-based:
  cache restores via scons don't update mtimes but the cached content IS
  correct. We trust a stamp file written after a successful build.

  Run after a rebuild:  scons ... && python -m \\
    openpilot.selfdrive.test.longitudinal_maneuvers.plot_maneuvers --write-stamp
  ...but more practically, this stamp is written automatically whenever a
  fresh sim run succeeds (see _write_solver_stamp at the end of main()).
  """
  long_mpc_path = _long_mpc_mod.__file__
  c_code_dir = os.path.join(os.path.dirname(long_mpc_path), 'c_generated_code')
  c_code_path = os.path.join(c_code_dir, 'acados_solver_long.c')
  if not os.path.exists(c_code_path):
    return  # never built; let the import error handle it

  stamp_path = os.path.join(c_code_dir, '.constants_hash')
  current_hash = _solver_constants_hash()
  stamped_hash = None
  if os.path.exists(stamp_path):
    try:
      with open(stamp_path) as f:
        stamped_hash = f.read().strip()
    except OSError:
      stamped_hash = None

  # If the stamp matches, we're definitely good.
  if stamped_hash == current_hash:
    return

  # No stamp yet, or stamp is stale. Fall back to the mtime hint: if the C
  # code is newer than long_mpc.py, the user almost certainly just rebuilt
  # and we'll write the stamp on this run (don't block them). If long_mpc.py
  # is newer than the C code, that's a stronger signal something needs a
  # rebuild.
  if os.path.getmtime(long_mpc_path) > os.path.getmtime(c_code_path):
    print('=' * 78, file=sys.stderr)
    print('ERROR: long_mpc.py is newer than the generated C solver AND the', file=sys.stderr)
    print('       constants stamp does not match current long_mpc.py values.', file=sys.stderr)
    print('       Constants baked into the solver (COMFORT_BRAKE,', file=sys.stderr)
    print('       desired_dist_comfort, etc.) WILL NOT reflect current edits.', file=sys.stderr)
    print('', file=sys.stderr)
    print('       Rebuild before running:', file=sys.stderr)
    print('         scons -u -j$(sysctl -n hw.ncpu)   # macOS', file=sys.stderr)
    print('         scons -u -j$(nproc)               # linux', file=sys.stderr)
    print('=' * 78, file=sys.stderr)
    sys.exit(2)
  # Stamp missing or stale but C file is newer than long_mpc.py -- proceed
  # and refresh the stamp on the way out.


def _write_solver_stamp():
  """Write the constants hash to .constants_hash after a successful run.
  Lets subsequent freshness checks pass quickly via the stamp."""
  long_mpc_path = _long_mpc_mod.__file__
  c_code_dir = os.path.join(os.path.dirname(long_mpc_path), 'c_generated_code')
  if not os.path.isdir(c_code_dir):
    return
  stamp_path = os.path.join(c_code_dir, '.constants_hash')
  try:
    with open(stamp_path, 'w') as f:
      f.write(_solver_constants_hash())
  except OSError:
    pass


_check_solver_freshness()

# ---- configuration -------------------------------------------------------
# Personalities to run. Order controls PNG sort order via the slug prefix.
PERSONALITIES = (
  log.LongitudinalPersonality.aggressive,
  log.LongitudinalPersonality.standard,
  log.LongitudinalPersonality.relaxed,
)
PERSONALITY_SLUGS = {
  int(log.LongitudinalPersonality.aggressive): 'agg',
  int(log.LongitudinalPersonality.standard):   'std',
  int(log.LongitudinalPersonality.relaxed):    'rlx',
}

# Coast band: aTarget within this range counts as "lift-off-throttle" coasting.
COAST_BAND = (-0.5, -0.05)
JERK_SPIKE_THRESHOLD = 8.0  # m/s^3 -- mark anything above this

MPH = 0.44704
SOURCE_NAMES = {0: 'cruise', 1: 'lead0', 2: 'lead1'}
SOURCE_COLORS = {0: 'tab:blue', 1: 'tab:green', 2: 'tab:orange'}


def build_maneuvers(personality):
  """Curated suite for tuning. Each maneuver targets a specific behavior;
  see the W1 section of the plan for what each one stresses."""
  p = int(personality)
  maneuvers = [
    # ---- baselines kept from the original suite ------------------------
    Maneuver('B1 below set speed with a distant lead', duration=40., initial_speed=15.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 1.], speed_lead_values=[20., 20.],
             cruise_values=[75 * MPH, 75 * MPH], personality=p),
    Maneuver('B2 approach a stopped car', duration=25., initial_speed=20.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 1.], speed_lead_values=[0., 0.],
             cruise_values=[20., 20.], personality=p),
    Maneuver('B3 following on street', duration=60., initial_speed=25 * MPH,
             lead_relevancy=True, initial_distance_lead=17.,
             breakpoints=[0., 5., 15., 25., 35., 45.],
             speed_lead_values=[25 * MPH, 25 * MPH, 15 * MPH, 25 * MPH, 15 * MPH, 25 * MPH],
             cruise_values=[40 * MPH] * 6, personality=p, mpc_lead=True),

    # ---- M1-M4: stop-n-go ladder (parking-lot to relaxed-suburban) -----
    # All four use mpc_lead=True so the lead accelerates/decelerates with
    # realistic jerk-limited transitions instead of corner-y linear interp.
    Maneuver('M1 dense crawl 0to2 mps (parking lot)', duration=20., initial_speed=0.,
             lead_relevancy=True, initial_distance_lead=6.,
             breakpoints=[0., 4., 8., 12., 16.],
             speed_lead_values=[0., 2., 0., 2., 0.],
             cruise_values=[4.] * 5, personality=p, mpc_lead=True),
    Maneuver('M2 dense city stop-n-go 0to7 mps', duration=32., initial_speed=0.,
             lead_relevancy=True, initial_distance_lead=12.,
             breakpoints=[0., 6., 11., 17., 22., 28.],
             speed_lead_values=[0., 7., 0., 7., 0., 7.],
             cruise_values=[9.] * 6, personality=p, mpc_lead=True),
    Maneuver('M3 moderate stop-n-go 0to10 mps', duration=35., initial_speed=0.,
             lead_relevancy=True, initial_distance_lead=22.,
             breakpoints=[0., 8., 15., 23., 30.],
             speed_lead_values=[0., 10., 0., 10., 0.],
             cruise_values=[12.] * 5, personality=p, mpc_lead=True),
    Maneuver('M4 relaxed stop-n-go 0to11 mps long cycle', duration=65., initial_speed=0.,
             lead_relevancy=True, initial_distance_lead=30.,
             breakpoints=[0., 15., 30., 45., 60.],
             speed_lead_values=[0., 11., 0., 11., 0.],
             cruise_values=[13.] * 5, personality=p, mpc_lead=True),

    # ---- M5 family: long-range slower-lead approach --------------------
    # Constrained to initial_distance_lead <= 100m (model vision limit).
    # The "coast-then-brake" target shape is: long light decel -> peak decel
    # near the end -> short coast as we settle to follow gap. Today's MPC
    # produces the *inverse* (peak decel early, taper to coast).
    Maneuver('M5 highway slower lead 10 mps closing', duration=25., initial_speed=32.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 25.], speed_lead_values=[22., 22.],
             cruise_values=[32., 32.], personality=p),
    Maneuver('M5b moderate closing 7 mps slower lead', duration=30., initial_speed=25.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 30.], speed_lead_values=[18., 18.],
             cruise_values=[25., 25.], personality=p),
    # M5c: the canonical "coast-then-brake" target scenario David described.
    # Closing rate 7 m/s at 100 m. Ideal profile: minimal decel for ~5 s,
    # then ramp to peak decel to settle into the follow gap without dipping
    # below v_lead.
    Maneuver('M5c canonical coast-then-brake 7mps closing', duration=30., initial_speed=22.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 30.], speed_lead_values=[15., 15.],
             cruise_values=[22., 22.], personality=p),
    # M5d: stopped-lead approach from low speed. Tests whether "free coast
    # time" exists -- the MPC currently holds speed for several seconds while
    # there's runway to start a gentle lift-off.
    Maneuver('M5d coast-to-stop from 7mps', duration=25., initial_speed=7.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 25.], speed_lead_values=[0., 0.],
             cruise_values=[7., 7.], personality=p),

    # ---- M6: sudden hard lead brake from steady follow -----------------
    Maneuver('M6 sudden hard lead brake from steady follow', duration=30., initial_speed=25.,
             lead_relevancy=True, initial_distance_lead=35.,
             breakpoints=[0., 10., 13.], speed_lead_values=[25., 25., 8.],
             cruise_values=[25., 25., 25.], personality=p),
    # M6b/M6c: moderate (real-world common) lead brake from steady follow.
    # Lead bleeds 10 m/s over 4s (~2.5 m/s^2) -- firm but comfortable. This is
    # the mid-severity regime where LDF/jerk_factor tuning shows up most.
    Maneuver('M6b moderate lead brake from 28mps steady', duration=30., initial_speed=28.,
             lead_relevancy=True, initial_distance_lead=42.,
             breakpoints=[0., 10., 14.], speed_lead_values=[28., 28., 18.],
             cruise_values=[28., 28., 28.], personality=p),
    Maneuver('M6c moderate lead brake from 18mps steady', duration=25., initial_speed=18.,
             lead_relevancy=True, initial_distance_lead=29.,
             breakpoints=[0., 10., 14.], speed_lead_values=[18., 18., 8.],
             cruise_values=[18., 18., 18.], personality=p),
    # M6d: lead brakes from 25 -> 8 m/s. mpc_lead caps lead decel at ~1.2 m/s^2
    # (CRUISE_MIN_ACCEL of the lead's MPC), so this is a sustained moderate
    # brake -- not a sudden hard one. Use M6 (scripted) for true hard brake.
    Maneuver('M6d realistic sustained lead brake from steady follow', duration=45., initial_speed=25.,
             lead_relevancy=True, initial_distance_lead=35.,
             breakpoints=[0., 10., 11.], speed_lead_values=[25., 25., 8.],
             cruise_values=[25., 25., 25.], personality=p, mpc_lead=True),
    # M6d: moderate lead brake from steady follow, lead runs its own MPC for
    # a realistic jerk-limited decel profile. Drops 10 m/s like M6b.
    Maneuver('M6e realistic lead brake from steady follow', duration=45., initial_speed=25.,
             lead_relevancy=True, initial_distance_lead=35.,
             breakpoints=[0., 10., 11.], speed_lead_values=[25., 25., 15.],
             cruise_values=[25., 25., 25.], personality=p, mpc_lead=True),

    # ---- M7/M8: cut-in (uses only_lead2 to simulate sudden appearance) -
    Maneuver('M7 distant cut-in rel 8 mps', duration=25., initial_speed=27.,
             lead_relevancy=True, initial_distance_lead=60.,
             breakpoints=[0., 25.], speed_lead_values=[19., 19.],
             cruise_values=[27., 27.], personality=p, only_lead2=True),
    Maneuver('M8 closer cut-in rel 12 mps', duration=20., initial_speed=27.,
             lead_relevancy=True, initial_distance_lead=35.,
             breakpoints=[0., 20.], speed_lead_values=[15., 15.],
             cruise_values=[27., 27.], personality=p, only_lead2=True),

    # ---- M10: highway pace at v_lead + margin (steady-state cruise) ----
    Maneuver('M10 highway pace at v_lead plus margin 60s', duration=60., initial_speed=32.,
             lead_relevancy=True, initial_distance_lead=50.,
             breakpoints=[0., 60.], speed_lead_values=[27., 27.],
             cruise_values=[32., 32.], personality=p),

    # ---- M11: lead clears from stop, re-accelerate ---------------------
    Maneuver('M11 lead clears re-accelerate', duration=20., initial_speed=0.,
             lead_relevancy=True, initial_distance_lead=4.5,
             breakpoints=[0., 3., 7., 20.],
             speed_lead_values=[0., 0., 15., 15.],
             cruise_values=[15.] * 4, personality=p, mpc_lead=True),
  ]

  # M9: stopped-lead approach with late detection (pop-in at 100m, 70mph)
  popin = Maneuver('M9 late detection 70mph ego vs 55mph lead pop-in', duration=40.,
                   initial_speed=70 * MPH, lead_relevancy=True, initial_distance_lead=400.,
                   breakpoints=[0., 4.99, 5., 30.],
                   speed_lead_values=[55 * MPH] * 4,
                   prob_lead_values=[0., 0., 1., 1.],
                   cruise_values=[70 * MPH] * 4, personality=p)
  popin.lead_swaps = [(5., 100.)]  # at t=5s lead becomes visible exactly 100 m ahead
  maneuvers.append(popin)

  # S1: lead-swap (lead leaves, distant lead instantly appears) -- swap-test baseline.
  swap = Maneuver('S1 lead leaves distant lead appears', duration=50.,
                  initial_speed=18., lead_relevancy=True, initial_distance_lead=33.,
                  breakpoints=[0., 50.], speed_lead_values=[18., 18.],
                  cruise_values=[31., 31.], personality=p)
  swap.lead_swaps = [(15., 90.)]  # at t=15s lead A is replaced by a lead 90 m ahead
  maneuvers.append(swap)

  # S1b: lead-swap (lead leaves, a closer 60 m lead instantly appears).
  swap = Maneuver('S1b lead leaves closer lead appears', duration=50.,
                  initial_speed=18., lead_relevancy=True, initial_distance_lead=33.,
                  breakpoints=[0., 50.], speed_lead_values=[18., 18.],
                  cruise_values=[31., 31.], personality=p)
  swap.lead_swaps = [(15., 60.)]  # at t=15s lead A is replaced by a lead 60 m ahead
  maneuvers.append(swap)

  return maneuvers


def simulate(m):
  """Run a maneuver through the real planner/MPC, capturing per-step state.

  A maneuver may carry an optional `lead_swaps` attribute -- a list of
  (time, gap) -- which teleports the lead to a new gap at that time. The Plant
  only tracks one continuous lead, so this is how a lead leaving the lane and
  a new, more distant one taking its place is modeled.
  """
  plant = Plant(lead_relevancy=m.lead_relevancy, speed=m.speed, distance_lead=m.distance_lead,
                enabled=m.enabled, only_lead2=m.only_lead2, only_radar=m.only_radar,
                e2e=m.e2e, personality=m.personality, force_decel=m.force_decel,
                mpc_lead=m.mpc_lead,
                initial_lead_speed=float(m.speed_lead_values[0]) if m.mpc_lead else 0.0)
  swaps = sorted(getattr(m, 'lead_swaps', []))
  d = defaultdict(list)
  while plant.current_time < m.duration:
    t = plant.current_time
    if swaps and t >= swaps[0][0]:
      plant.distance_lead = plant.distance + swaps.pop(0)[1]
    v_lead = float(np.interp(t, m.breakpoints, m.speed_lead_values))
    prob_lead = float(np.interp(t, m.breakpoints, m.prob_lead_values))
    cruise = float(np.interp(t, m.breakpoints, m.cruise_values))
    pitch = float(np.interp(t, m.breakpoints, m.pitch_values))
    prob_throttle = float(np.interp(t, m.breakpoints, m.prob_throttle_values))
    out = plant.step(v_lead, prob_lead, cruise, pitch, prob_throttle)
    visible = m.lead_relevancy and (m.only_radar or prob_lead > 0.5)
    # In mpc_lead mode, the interp value is the lead's cruise *target*; the
    # actual velocity is what plant just used in the radar message.
    v_lead_actual = plant.v_lead_prev if m.mpc_lead else v_lead
    d['t'].append(t)
    d['v_ego'].append(out['speed'])
    d['a'].append(out['acceleration'])
    d['v_lead'].append(v_lead_actual if visible else np.nan)
    d['v_lead_target'].append(v_lead if visible else np.nan)
    d['v_cruise'].append(cruise)
    d['d_rel'].append((out['distance_lead'] - out['distance']) if visible else np.nan)
    d['source'].append(int(plant.planner.mpc.source))
  return {k: np.array(v) for k, v in d.items()}


def shade_sources(ax, t, source):
  """Shade the background by which MPC obstacle is binding."""
  i = 0
  while i < len(source):
    j = i
    while j < len(source) and source[j] == source[i]:
      j += 1
    t1 = t[j] if j < len(t) else t[-1]
    ax.axvspan(t[i], t1, color=SOURCE_COLORS.get(int(source[i]), 'gray'), alpha=0.10, lw=0)
    i = j


def _coast_band_intervals(t, a, lo, hi):
  """Return list of (t_start, t_end) where a is within [lo, hi] continuously."""
  in_band = (a >= lo) & (a <= hi)
  intervals = []
  i = 0
  while i < len(in_band):
    if not in_band[i]:
      i += 1
      continue
    j = i
    while j < len(in_band) and in_band[j]:
      j += 1
    t_end = t[j - 1] if j - 1 < len(t) else t[-1]
    intervals.append((t[i], t_end))
    i = j
  return intervals


def _time_to_first_brake(t, a, threshold=-0.1):
  """Time at which aTarget first crosses below `threshold`. NaN if never."""
  below = np.where(a < threshold)[0]
  return float(t[below[0]]) if len(below) else float('nan')


def plot_maneuver(m, d, out_dir='.', label=''):
  t, v_ego, a = d['t'], d['v_ego'], d['a']
  v_lead, v_cruise, d_rel, source = d['v_lead'], d['v_cruise'], d['d_rel'], d['source']
  v_lead_target = d.get('v_lead_target')
  has_lead = bool(m.lead_relevancy)

  jerk = np.gradient(a, t)
  tf = np.array([get_T_FOLLOW(m.personality, v) for v in v_ego])
  stop = get_STOP_DISTANCE(m.personality)
  desired_gap = tf * v_lead + stop
  headway = np.where(v_ego > 1.0, d_rel / np.maximum(v_ego, 1e-3), np.nan)

  coast_intervals = _coast_band_intervals(t, a, COAST_BAND[0], COAST_BAND[1])
  total_coast = sum(e - s for s, e in coast_intervals)
  spike_idx = np.where(np.abs(jerk) > JERK_SPIKE_THRESHOLD)[0]
  t_first_brake = _time_to_first_brake(t, a)

  fig, axs = plt.subplots(4, 1, figsize=(13, 15), sharex=True)
  pers_name = {0: 'aggressive', 1: 'standard', 2: 'relaxed'}.get(int(m.personality), '?')
  fig.suptitle(f"{m.title}    |    personality: {pers_name}", fontsize=13, y=0.998)
  for ax in axs:
    shade_sources(ax, t, source)

  # --- speed ---
  axs[0].plot(t, v_ego, color='tab:blue', lw=2.5, label='ego speed')
  if has_lead:
    axs[0].plot(t, v_lead, color='tab:red', lw=1.8, ls='--', label='lead speed')
    if m.mpc_lead and v_lead_target is not None:
      axs[0].plot(t, v_lead_target, color='tab:red', lw=0.9, ls=':', alpha=0.55,
                  label='lead cruise target')
  axs[0].plot(t, v_cruise, color='dimgray', lw=1.2, ls=':', label='set speed')
  axs[0].set_ylabel('speed (m/s)')
  src_handles = [Patch(color=SOURCE_COLORS[k], alpha=0.3, label=f'MPC source: {v}')
                 for k, v in SOURCE_NAMES.items()]
  axs[0].legend(handles=axs[0].get_legend_handles_labels()[0] + src_handles,
                loc='best', ncol=2, fontsize=8)
  axs[0].grid(alpha=0.3)
  secax = axs[0].secondary_yaxis('right', functions=(lambda v: v / MPH, lambda v: v * MPH))
  secax.set_ylabel('speed (mph)')

  # --- acceleration + jerk ---
  # Coast-band shading first so it sits behind the line.
  for s, e in coast_intervals:
    axs[1].axvspan(s, e, color='tab:olive', alpha=0.12, lw=0)
  axs[1].plot(t, a, color='tab:blue', lw=2.2, label='commanded accel (aTarget)')
  axs[1].axhline(0, color='gray', lw=0.8, ls=':')
  axs[1].axhline(-COMFORT_BRAKE, color='tab:red', lw=0.9, ls='--', alpha=0.6,
                 label=f'-COMFORT_BRAKE ({-COMFORT_BRAKE:.1f})')
  axs[1].axhline(COAST_BAND[0], color='tab:olive', lw=0.6, ls=':', alpha=0.7)
  axs[1].axhline(COAST_BAND[1], color='tab:olive', lw=0.6, ls=':', alpha=0.7)
  i_pd = int(np.argmin(a))
  axs[1].plot(t[i_pd], a[i_pd], 'v', color='tab:blue', ms=10)
  axs[1].annotate(f'peak {a[i_pd]:.2f}', (t[i_pd], a[i_pd]), textcoords='offset points',
                  xytext=(8, -2), fontsize=8, color='tab:blue')
  axs[1].set_ylabel('acceleration  (m/s²  |  mph/s)')
  axs[1].yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:.1f}  |  {v / MPH:.1f}'))
  axb = axs[1].twinx()
  axb.plot(t, jerk, color='tab:purple', lw=0.9, alpha=0.55, label='jerk')
  if len(spike_idx):
    axb.plot(t[spike_idx], jerk[spike_idx], 'x', color='tab:red', ms=7,
             label=f'|jerk| > {JERK_SPIKE_THRESHOLD:.0f}')
  axb.set_ylabel('jerk (m/s³)', color='tab:purple')
  axb.tick_params(axis='y', colors='tab:purple')
  # Add the coast-band shading to the accel-axis legend so it's labeled.
  ax1_handles, ax1_labels = axs[1].get_legend_handles_labels()
  coast_patch = Patch(facecolor='tab:olive', alpha=0.25,
                      label=f'coast band [{COAST_BAND[0]:+.2f}, {COAST_BAND[1]:+.2f}]')
  axs[1].legend(handles=ax1_handles + [coast_patch], labels=ax1_labels + [coast_patch.get_label()],
                loc='upper right', fontsize=8)
  axb.legend(loc='lower right', fontsize=8)
  axs[1].grid(alpha=0.3)

  # --- gap to lead ---
  if has_lead:
    axs[2].plot(t, d_rel, color='tab:green', lw=2.5, label='gap to lead (dRel)')
    axs[2].plot(t, desired_gap, color='darkgoldenrod', lw=1.6, ls='--',
                label='desired follow gap (t_follow·v_lead + stop)')
    i_mg = int(np.nanargmin(d_rel))
    axs[2].plot(t[i_mg], d_rel[i_mg], 'o', color='tab:green', ms=8)
    axs[2].annotate(f'min {d_rel[i_mg]:.1f} m', (t[i_mg], d_rel[i_mg]),
                    textcoords='offset points', xytext=(8, 6), fontsize=8, color='tab:green')
    axs[2].set_ylim(bottom=0)
  axs[2].set_ylabel('relative distance (m)')
  axs[2].legend(loc='best', fontsize=8)
  axs[2].grid(alpha=0.3)

  # --- time headway ---
  if has_lead:
    axs[3].plot(t, headway, color='tab:cyan', lw=2.2, label='time headway (dRel / vEgo)')
    axs[3].plot(t, tf, color='darkgoldenrod', lw=1.6, ls='--', label='t_follow target')
    axs[3].set_ylim(0, 10)
  axs[3].set_ylabel('time headway (s)')
  axs[3].set_xlabel('time (s)')
  axs[3].legend(loc='best', fontsize=8)
  axs[3].grid(alpha=0.3)

  # --- metrics ---
  min_gap = float(np.nanmin(d_rel)) if has_lead else float('nan')
  final_gap = float(d_rel[-1]) if has_lead else float('nan')
  metrics = {
    'title': m.title,
    'personality': pers_name,
    'peak_accel': float(a.max()),
    'peak_decel': float(a.min()),
    'max_abs_jerk': float(np.abs(jerk).max()),
    'jerk_spikes': int(len(spike_idx)),
    'min_gap': min_gap,
    'final_gap': final_gap,
    't_first_brake': t_first_brake,
    'coast_time': float(total_coast),
    'coast_intervals': len(coast_intervals),
  }

  bits = [
    f'peak decel {metrics["peak_decel"]:+.2f}',
    f'peak accel {metrics["peak_accel"]:+.2f}',
    f'max |jerk| {metrics["max_abs_jerk"]:.1f}',
    f'jerk spikes {metrics["jerk_spikes"]}',
    f'coast {metrics["coast_time"]:.1f}s',
  ]
  if has_lead:
    bits += [f'min gap {min_gap:.1f} m', f'final gap {final_gap:.1f} m']
    if v_ego[-1] > 1.0:
      bits.append(f'final headway {final_gap / v_ego[-1]:.2f} s')
  fig.text(0.5, 0.967, '      '.join(bits), ha='center', fontsize=9,
           bbox=dict(boxstyle='round', fc='white', ec='0.7'))

  fig.tight_layout(rect=[0, 0, 1, 0.955])
  pers_slug = PERSONALITY_SLUGS.get(int(m.personality), 'pXX')
  slug = re.sub(r'[^a-z0-9]+', '_', m.title.lower()).strip('_')
  parts = ['maneuver']
  if label:
    parts.append(label)
  parts += [pers_slug, slug]
  fname = os.path.join(out_dir, '_'.join(parts) + '.png')
  fig.savefig(fname, dpi=110)
  plt.close(fig)
  return fname, bits, metrics


def main():
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0] if __doc__ else '')
  parser.add_argument('--label', default='',
                      help='Label suffix for output files (e.g. "baseline", "h1_v1").')
  parser.add_argument('--out-dir', default='.',
                      help='Output directory for PNGs and CSV (default: cwd).')
  args = parser.parse_args()

  os.makedirs(args.out_dir, exist_ok=True)
  all_metrics = []
  for personality in PERSONALITIES:
    pers_slug = PERSONALITY_SLUGS[int(personality)]
    print(f'==== personality: {pers_slug} ====')
    for m in build_maneuvers(personality):
      print(f'running: {pers_slug}  {m.title}')
      fname, bits, metrics = plot_maneuver(m, simulate(m),
                                           out_dir=args.out_dir, label=args.label)
      all_metrics.append(metrics)
      print(f'  saved {fname}  ({"  ".join(bits)})')

  # Scoreboard CSV -- one row per (maneuver, personality). Order matches the
  # plot loop, so the file is grouped by personality then title.
  csv_name = f'maneuver_scores_{args.label}.csv' if args.label else 'maneuver_scores.csv'
  csv_path = os.path.join(args.out_dir, csv_name)
  fieldnames = ['personality', 'title', 'peak_accel', 'peak_decel', 'max_abs_jerk',
                'jerk_spikes', 'min_gap', 'final_gap', 't_first_brake',
                'coast_time', 'coast_intervals']
  with open(csv_path, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    for row in all_metrics:
      w.writerow({k: row[k] for k in fieldnames})
  print(f'wrote {csv_path} with {len(all_metrics)} rows')
  _write_solver_stamp()


if __name__ == '__main__':
  main()
