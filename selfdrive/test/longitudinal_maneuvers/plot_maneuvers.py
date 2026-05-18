#!/usr/bin/env python3
"""
Tuning dashboards for longitudinal maneuvers.

Define maneuvers in build_maneuvers() (same Maneuver objects used by
test_longitudinal), run this file, and get one annotated PNG per maneuver.
Each dashboard shows ego/lead/set speed, commanded acceleration + jerk,
gap-to-lead vs the desired follow gap, and time headway vs t_follow -- with
the background shaded by which MPC obstacle is binding (cruise / lead0 /
lead1). Nothing is asserted; this is purely a visualization aid for tuning
long_mpc.

All configuration is in-code (PERSONALITY and build_maneuvers below).

Run: .venv/bin/python selfdrive/test/longitudinal_maneuvers/plot_maneuvers.py
"""
import re
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import FuncFormatter

from cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_STOP_DISTANCE
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant

# ---- configuration -------------------------------------------------------
# .aggressive / .standard / .relaxed
PERSONALITY = log.LongitudinalPersonality.aggressive

MPH = 0.44704
SOURCE_NAMES = {0: 'cruise', 1: 'lead0', 2: 'lead1'}
SOURCE_COLORS = {0: 'tab:blue', 1: 'tab:green', 2: 'tab:orange'}


def build_maneuvers():
  """The maneuvers to plot. Edit freely -- add, remove, or keep just one."""
  p = int(PERSONALITY)
  maneuvers = [
    Maneuver('approach slower lead, 75 to 50mph', duration=35., initial_speed=75 * MPH,
             lead_relevancy=True, initial_distance_lead=110.,
             breakpoints=[0., 1.], speed_lead_values=[50 * MPH, 50 * MPH],
             cruise_values=[75 * MPH, 75 * MPH], personality=p),
    Maneuver('below set speed with a distant lead', duration=40., initial_speed=15.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 1.], speed_lead_values=[20., 20.],
             cruise_values=[75 * MPH, 75 * MPH], personality=p),
    Maneuver('approach a stopped car', duration=25., initial_speed=20.,
             lead_relevancy=True, initial_distance_lead=100.,
             breakpoints=[0., 1.], speed_lead_values=[0., 0.],
             cruise_values=[20., 20.], personality=p),
    Maneuver('lead brakes to a stop while following', duration=45., initial_speed=25.,
             lead_relevancy=True, initial_distance_lead=44.,
             breakpoints=[0., 12., 28.], speed_lead_values=[25., 25., 0.],
             cruise_values=[25., 25., 25.], personality=p),
    Maneuver('stop and go', duration=50., initial_speed=10.,
             lead_relevancy=True, initial_distance_lead=22.,
             breakpoints=[0., 5., 12., 18., 24., 31., 38., 44.],
             speed_lead_values=[10., 10., 0., 0., 10., 10., 0., 0.],
             cruise_values=[14.] * 8, personality=p),
  ]
  swap = Maneuver('lead leaves, distant lead instantly appears', duration=50.,
                  initial_speed=18., lead_relevancy=True, initial_distance_lead=33.,
                  breakpoints=[0., 50.], speed_lead_values=[18., 18.],
                  cruise_values=[31., 31.], personality=p)
  swap.lead_swaps = [(15., 90.)]  # at t=15s lead A is replaced by a lead 90 m ahead
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
                e2e=m.e2e, personality=m.personality, force_decel=m.force_decel)
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
    d['t'].append(t)
    d['v_ego'].append(out['speed'])
    d['a'].append(out['acceleration'])
    d['v_lead'].append(v_lead if visible else np.nan)
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


def plot_maneuver(m, d):
  t, v_ego, a = d['t'], d['v_ego'], d['a']
  v_lead, v_cruise, d_rel, source = d['v_lead'], d['v_cruise'], d['d_rel'], d['source']
  has_lead = bool(m.lead_relevancy)

  jerk = np.gradient(a, t)
  tf = np.array([get_T_FOLLOW(m.personality, v) for v in v_ego])
  stop = get_STOP_DISTANCE(m.personality)
  desired_gap = tf * v_lead + stop
  headway = np.where(v_ego > 1.0, d_rel / np.maximum(v_ego, 1e-3), np.nan)

  fig, axs = plt.subplots(4, 1, figsize=(13, 15), sharex=True)
  pers = {0: 'aggressive', 1: 'standard', 2: 'relaxed'}.get(int(m.personality), '?')
  fig.suptitle(f"{m.title}    |    personality: {pers}", fontsize=13, y=0.998)
  for ax in axs:
    shade_sources(ax, t, source)

  # --- speed ---
  axs[0].plot(t, v_ego, color='tab:blue', lw=2.5, label='ego speed')
  if has_lead:
    axs[0].plot(t, v_lead, color='tab:red', lw=1.8, ls='--', label='lead speed')
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
  axs[1].plot(t, a, color='tab:blue', lw=2.2, label='commanded accel (aTarget)')
  axs[1].axhline(0, color='gray', lw=0.8, ls=':')
  i_pd = int(np.argmin(a))
  axs[1].plot(t[i_pd], a[i_pd], 'v', color='tab:blue', ms=10)
  axs[1].annotate(f'peak {a[i_pd]:.2f}', (t[i_pd], a[i_pd]), textcoords='offset points',
                  xytext=(8, -2), fontsize=8, color='tab:blue')
  axs[1].set_ylabel('acceleration  (m/s²  |  mph/s)')
  axs[1].yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:.1f}  |  {v / MPH:.1f}'))
  axb = axs[1].twinx()
  axb.plot(t, jerk, color='tab:purple', lw=0.9, alpha=0.55, label='jerk')
  axb.set_ylabel('jerk (m/s³)', color='tab:purple')
  axb.tick_params(axis='y', colors='tab:purple')
  axs[1].legend(loc='upper left', fontsize=8)
  axb.legend(loc='upper right', fontsize=8)
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

  # --- metrics strip ---
  bits = [f'peak decel {a.min():+.2f}', f'peak accel {a.max():+.2f}',
          f'max |jerk| {np.abs(jerk).max():.1f}']
  if has_lead:
    bits += [f'min gap {np.nanmin(d_rel):.1f} m', f'final gap {d_rel[-1]:.1f} m']
    if v_ego[-1] > 1.0:
      bits.append(f'final headway {d_rel[-1] / v_ego[-1]:.2f} s')
  fig.text(0.5, 0.967, '      '.join(bits), ha='center', fontsize=9,
           bbox=dict(boxstyle='round', fc='white', ec='0.7'))

  fig.tight_layout(rect=[0, 0, 1, 0.955])
  slug = re.sub(r'[^a-z0-9]+', '_', m.title.lower()).strip('_')
  fname = f'maneuver_{slug}.png'
  fig.savefig(fname, dpi=110)
  plt.close(fig)
  return fname, bits


def main():
  for m in build_maneuvers():
    print(f'running: {m.title}')
    fname, bits = plot_maneuver(m, simulate(m))
    print(f'  saved {fname}  ({"  ".join(bits)})')


if __name__ == '__main__':
  main()
