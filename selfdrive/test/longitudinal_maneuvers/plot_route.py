#!/usr/bin/env python3
"""
Per-route dashboard generator. Loads an extract_signals.py CSV, smooths the
human's driving for benchmark display, replays the current offline MPC against
the logged radar+ego state (open-loop), and emits one annotated dashboard PNG
covering the entire route.

PlotJuggler caps exports at 30s, so each route file is already a manageable
window -- no need to event-split. The visual language matches plot_maneuvers.py
so synthetic-vs-real comparisons read naturally. The benchmark trace is aEgo
(smoothed) -- what *you* actually did. The blue overlay is replay_aTarget --
what the current MPC tuning would do given the same radar+ego state. Tune to
reduce the RMS gap between them.

Usage:
  plot_route.py --route <route_ext.csv> [--label tag] [--out-dir dir]
                [--aego-smooth-k 11]
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import FuncFormatter

from cereal import log, messaging
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  COMFORT_BRAKE, get_STOP_DISTANCE, get_T_FOLLOW,
)

# Reuse plot_maneuvers' constants and helpers (also triggers its freshness check).
from openpilot.selfdrive.test.longitudinal_maneuvers.plot_maneuvers import (
  COAST_BAND, JERK_SPIKE_THRESHOLD, MPH, SOURCE_NAMES, SOURCE_COLORS,
  PERSONALITY_SLUGS, shade_sources,
  _coast_band_intervals,
  _write_solver_stamp,
)


NAMES = ['t', 'act_accel', 'aEgo', 'vCruise', 'vEgo', 'aTarget', 'accels0', 'hasLead',
         'source', 'speeds0', 'l1_aLeadK', 'l1_aLeadTau', 'l1_dRel', 'l1_modelProb',
         'l1_status', 'l1_vLead', 'l1_vLeadK', 'l1_vRel', 'l2_aLeadK', 'l2_aLeadTau',
         'l2_dRel', 'l2_modelProb', 'l2_status', 'l2_vLead', 'l2_vLeadK', 'expMode',
         'personality']


def _ffill(a):
  mask = np.isnan(a)
  idx = np.where(~mask, np.arange(len(a)), 0)
  np.maximum.accumulate(idx, out=idx)
  return a[idx]


def ffill(a):
  return _ffill(_ffill(a)[::-1])[::-1]


def medfilt(a, k):
  half = k // 2
  return np.array([np.median(a[max(0, i-half):i+half+1]) for i in range(len(a))])


def load_route(csv_path):
  raw = np.genfromtxt(csv_path, delimiter=',', skip_header=1)
  raw_t = raw[:, 0] - raw[0, 0]
  cols = {n: ffill(raw[:, i]) for i, n in enumerate(NAMES)}
  t = np.arange(0.0, raw_t[-1], DT_MDL)
  d = {n: np.interp(t, raw_t, cols[n]) for n in NAMES}
  d['t'] = t
  return d


def mk_lead(dRel, vLead, vLeadK, aLeadK, aLeadTau, status, modelProb, v_ego):
  lead = log.RadarState.LeadData.new_message()
  lead.dRel = float(dRel)
  lead.vLead = float(vLead)
  lead.vLeadK = float(vLeadK)
  lead.aLeadK = float(aLeadK)
  lead.aLeadTau = float(aLeadTau) if aLeadTau > 1e-3 else 1.5
  lead.vRel = float(vLead - v_ego)
  lead.status = bool(status > 0.5)
  lead.modelProb = float(modelProb)
  return lead


def build_sm(r):
  radar = messaging.new_message('radarState')
  control = messaging.new_message('controlsState')
  ss = messaging.new_message('selfdriveState')
  car_state = messaging.new_message('carState')
  lp = messaging.new_message('liveParameters')
  car_control = messaging.new_message('carControl')
  model = messaging.new_message('modelV2')
  car_state_sp = messaging.new_message('carStateSP')
  live_map_data_sp = messaging.new_message('liveMapDataSP')
  gps = messaging.new_message('gpsLocation')

  radar.radarState.leadOne = mk_lead(r['l1_dRel'], r['l1_vLead'], r['l1_vLeadK'],
                                     r['l1_aLeadK'], r['l1_aLeadTau'], r['l1_status'],
                                     r['l1_modelProb'], r['vEgo'])
  radar.radarState.leadTwo = mk_lead(r['l2_dRel'], r['l2_vLead'], r['l2_vLeadK'],
                                     r['l2_aLeadK'], r['l2_aLeadTau'], r['l2_status'],
                                     r['l2_modelProb'], r['vEgo'])
  control.controlsState.longControlState = LongCtrlState.pid
  ss.selfdriveState.experimentalMode = bool(r['expMode'] > 0.5)
  ss.selfdriveState.enabled = True
  ss.selfdriveState.personality = int(round(r['personality']))
  car_state.carState.vEgo = float(r['vEgo'])
  car_state.carState.aEgo = float(r['aEgo'])
  car_state.carState.vCruise = float(r['vCruise'])
  car_state.carState.standstill = bool(r['vEgo'] < 0.01)
  car_control.carControl.orientationNED = [0.0, 0.0, 0.0]
  model.modelV2.meta.disengagePredictions.gasPressProbs = [1.0] * 6

  return {'radarState': radar.radarState, 'carState': car_state.carState,
          'carControl': car_control.carControl, 'controlsState': control.controlsState,
          'selfdriveState': ss.selfdriveState, 'liveParameters': lp.liveParameters,
          'modelV2': model.modelV2, 'carStateSP': car_state_sp.carStateSP,
          'liveMapDataSP': live_map_data_sp.liveMapDataSP, 'gpsLocation': gps.gpsLocation}


def replay(d):
  """Open-loop replay: feed logged radar+ego into the planner cycle-by-cycle.
  Returns (aTarget_replay, mpc_source) arrays."""
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  planner = LongitudinalPlanner(CP, CP_SP, init_v=d['vEgo'][0])
  N = len(d['t'])
  aT = np.zeros(N)
  src = np.zeros(N, dtype=int)
  for i in range(N):
    r = {n: d[n][i] for n in NAMES}
    planner.update(build_sm(r))
    aT[i] = planner.output_a_target
    src[i] = int(planner.mpc.source)
  return aT, src


def plot_route(d, replay_aT, replay_src, route_name, out_dir, label):
  t = d['t']
  v_ego = d['vEgo']
  has_lead = d['l1_status'] > 0.5
  v_lead = np.where(has_lead, d['l1_vLeadK_smooth'], np.nan)
  v_cruise = d['vCruise'] * 0.27778  # kph -> m/s
  d_rel = np.where(has_lead, d['l1_dRel_smooth'], np.nan)
  aEgo = d['aEgo_smooth']
  aTarget_replay = replay_aT
  aTarget_logged = d['aTarget']
  source = replay_src
  personality = int(round(np.median(d['personality'])))

  jerk = np.gradient(aEgo, t)
  tf_arr = np.array([get_T_FOLLOW(personality, v) for v in v_ego])
  stop = get_STOP_DISTANCE(personality)
  desired_gap = tf_arr * v_lead + stop  # NaN where no lead
  headway = np.where(v_ego > 1.0, d_rel / np.maximum(v_ego, 1e-3), np.nan)

  coast_intervals = _coast_band_intervals(t, aEgo, COAST_BAND[0], COAST_BAND[1])
  spike_idx = np.where(np.abs(jerk) > JERK_SPIKE_THRESHOLD)[0]

  # MPC vs human divergence
  diff = aTarget_replay - aEgo
  rms_diff = float(np.sqrt(np.mean(diff * diff)))
  max_abs_diff = float(np.max(np.abs(diff)))

  fig, axs = plt.subplots(4, 1, figsize=(13, 15), sharex=True)
  pers_name = {0: 'aggressive', 1: 'standard', 2: 'relaxed'}.get(personality, '?')
  fig.suptitle(f"{route_name}  (t={t[0]:.1f}-{t[-1]:.1f}s)  |  personality: {pers_name}",
               fontsize=13, y=0.998)
  for ax in axs:
    shade_sources(ax, t, source)

  # --- speed ---
  axs[0].plot(t, v_ego, color='tab:blue', lw=2.5, label='ego speed (logged)')
  axs[0].plot(t, v_lead, color='tab:red', lw=1.8, ls='--', label='lead speed (smoothed)')
  axs[0].plot(t, v_cruise, color='dimgray', lw=1.2, ls=':', label='set speed')
  axs[0].set_ylabel('speed (m/s)')
  src_handles = [Patch(color=SOURCE_COLORS[k], alpha=0.3, label=f'MPC source: {v}')
                 for k, v in SOURCE_NAMES.items()]
  axs[0].legend(handles=axs[0].get_legend_handles_labels()[0] + src_handles,
                loc='best', ncol=2, fontsize=8)
  axs[0].grid(alpha=0.3)
  secax = axs[0].secondary_yaxis('right', functions=(lambda v: v / MPH, lambda v: v * MPH))
  secax.set_ylabel('speed (mph)')

  # --- accel + jerk: human (benchmark) vs MPC replay ---
  for cs, ce in coast_intervals:
    axs[1].axvspan(cs, ce, color='tab:olive', alpha=0.12, lw=0)
  axs[1].plot(t, aEgo, color='black', lw=2.8, label='aEgo (human, benchmark)')
  axs[1].plot(t, aTarget_replay, color='tab:blue', lw=1.8,
              label='aTarget (current MPC replay)')
  axs[1].plot(t, aTarget_logged, color='tab:gray', lw=0.9, ls=':', alpha=0.7,
              label='aTarget (logged in-car)')
  axs[1].axhline(0, color='gray', lw=0.8, ls=':')
  axs[1].axhline(-COMFORT_BRAKE, color='tab:red', lw=0.9, ls='--', alpha=0.6,
                 label=f'-COMFORT_BRAKE ({-COMFORT_BRAKE:.1f})')
  axs[1].axhline(COAST_BAND[0], color='tab:olive', lw=0.6, ls=':', alpha=0.7)
  axs[1].axhline(COAST_BAND[1], color='tab:olive', lw=0.6, ls=':', alpha=0.7)
  axs[1].set_ylabel('acceleration  (m/s²  |  mph/s)')
  axs[1].yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:.1f}  |  {v / MPH:.1f}'))
  axb = axs[1].twinx()
  axb.plot(t, jerk, color='tab:purple', lw=0.7, alpha=0.5, label='jerk (human, smoothed)')
  if len(spike_idx):
    axb.plot(t[spike_idx], jerk[spike_idx], 'x', color='tab:red', ms=7,
             label=f'|jerk| > {JERK_SPIKE_THRESHOLD:.0f}')
  axb.set_ylabel('jerk (m/s³)', color='tab:purple')
  axb.tick_params(axis='y', colors='tab:purple')
  coast_patch = Patch(facecolor='tab:olive', alpha=0.25,
                      label=f'coast band [{COAST_BAND[0]:+.2f}, {COAST_BAND[1]:+.2f}]')
  ax1_handles, ax1_labels = axs[1].get_legend_handles_labels()
  axs[1].legend(handles=ax1_handles + [coast_patch],
                labels=ax1_labels + [coast_patch.get_label()],
                loc='upper left', fontsize=8)
  axb.legend(loc='upper right', fontsize=8)
  axs[1].grid(alpha=0.3)

  # --- gap to lead ---
  axs[2].plot(t, d_rel, color='tab:green', lw=2.5, label='gap to lead (smoothed)')
  axs[2].plot(t, desired_gap, color='darkgoldenrod', lw=1.6, ls='--',
              label='desired follow gap (t_follow·v_lead + stop)')
  axs[2].set_ylim(bottom=0)
  axs[2].set_ylabel('relative distance (m)')
  axs[2].legend(loc='best', fontsize=8)
  axs[2].grid(alpha=0.3)

  # --- time headway ---
  axs[3].plot(t, headway, color='tab:cyan', lw=2.2, label='time headway (dRel / vEgo)')
  axs[3].plot(t, tf_arr, color='darkgoldenrod', lw=1.6, ls='--', label='t_follow target')
  axs[3].set_ylim(0, 10)
  axs[3].set_ylabel('time headway (s)')
  axs[3].set_xlabel('time (s)')
  axs[3].legend(loc='best', fontsize=8)
  axs[3].grid(alpha=0.3)

  # --- metrics strip ---
  bits = [
    f'RMS(replay-aEgo) {rms_diff:.2f}',
    f'max|replay-aEgo| {max_abs_diff:.2f}',
    f'aEgo peak {aEgo.min():+.2f}',
    f'replay peak {aTarget_replay.min():+.2f}',
  ]
  has_any_lead = bool(np.any(has_lead))
  if has_any_lead:
    bits.append(f'min gap {np.nanmin(d_rel):.1f} m')
  fig.text(0.5, 0.967, '      '.join(bits), ha='center', fontsize=9,
           bbox=dict(boxstyle='round', fc='white', ec='0.7'))

  fig.tight_layout(rect=[0, 0, 1, 0.955])
  pers_slug = PERSONALITY_SLUGS.get(personality, 'pXX')
  parts = [route_name]
  if label:
    parts.append(label)
  parts.append(pers_slug)
  fname = os.path.join(out_dir, '_'.join(parts) + '.png')
  fig.savefig(fname, dpi=110)
  plt.close(fig)

  metrics = {
    'route': route_name,
    'personality': pers_name,
    'duration': float(t[-1] - t[0]),
    'rms_replay_vs_aEgo': rms_diff,
    'max_abs_replay_vs_aEgo': max_abs_diff,
    'aEgo_peak_decel': float(aEgo.min()),
    'replay_peak_decel': float(aTarget_replay.min()),
    'aEgo_peak_accel': float(aEgo.max()),
    'replay_peak_accel': float(aTarget_replay.max()),
    'min_gap': float(np.nanmin(d_rel)) if has_any_lead else float('nan'),
    'coast_time_human': float(sum(ce - cs for cs, ce in coast_intervals)),
  }
  return fname, metrics


def main():
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[1] if __doc__ else '')
  parser.add_argument('--route', required=True,
                      help='Path to route_ext.csv (output of extract_signals.py)')
  parser.add_argument('--label', default='',
                      help='Optional tag (e.g. "rebuilt", "cb_2_0") for output filenames')
  parser.add_argument('--out-dir', default='.', help='Output directory')
  parser.add_argument('--aego-smooth-k', type=int, default=11,
                      help='Median-filter width (samples) for smoothing aEgo. '
                           'Default 11 (~0.55s at 50ms cadence).')
  parser.add_argument('--lead-smooth-k', type=int, default=25,
                      help='Median-filter width for lead vLeadK/dRel display. Default 25.')
  args = parser.parse_args()

  if not os.path.exists(args.route):
    print(f"ERROR: route file not found: {args.route}", file=sys.stderr)
    sys.exit(1)

  os.makedirs(args.out_dir, exist_ok=True)

  route_name = os.path.basename(args.route).replace('_ext.csv', '').replace('.csv', '')
  print(f"loading {args.route} ...")
  d = load_route(args.route)
  print(f"  {len(d['t'])} samples ({d['t'][-1]:.0f} s, "
        f"personality={int(round(np.median(d['personality'])))})")

  d['aEgo_smooth'] = medfilt(d['aEgo'], k=args.aego_smooth_k)
  d['l1_vLeadK_smooth'] = medfilt(d['l1_vLeadK'], k=args.lead_smooth_k)
  d['l1_dRel_smooth'] = medfilt(d['l1_dRel'], k=args.lead_smooth_k)

  print('running offline MPC replay (open-loop) ...')
  aT_replay, src_replay = replay(d)
  err_logged = aT_replay - d['aTarget']
  err_human = aT_replay - d['aEgo_smooth']
  print(f"  whole-route RMS(replay - logged_aTarget): {np.sqrt(np.mean(err_logged**2)):.3f} "
        f"m/s²  (low = current tuning matches in-car tuning)")
  print(f"  whole-route RMS(replay - aEgo_smoothed):  {np.sqrt(np.mean(err_human**2)):.3f} "
        f"m/s²  (low = MPC drives like you)")

  fname, metrics = plot_route(d, aT_replay, src_replay, route_name,
                              args.out_dir, args.label)
  print(f"  saved {fname}  rms={metrics['rms_replay_vs_aEgo']:.2f}")
  print(f"    aEgo:   peak_accel {metrics['aEgo_peak_accel']:+5.2f}  "
        f"peak_decel {metrics['aEgo_peak_decel']:+5.2f}")
  print(f"    replay: peak_accel {metrics['replay_peak_accel']:+5.2f}  "
        f"peak_decel {metrics['replay_peak_decel']:+5.2f}")

  csv_name = f'route_scores_{route_name}'
  if args.label:
    csv_name += f'_{args.label}'
  csv_name += '.csv'
  csv_path = os.path.join(args.out_dir, csv_name)
  fieldnames = ['route', 'personality', 'duration',
                'rms_replay_vs_aEgo', 'max_abs_replay_vs_aEgo',
                'aEgo_peak_decel', 'replay_peak_decel',
                'aEgo_peak_accel', 'replay_peak_accel',
                'min_gap', 'coast_time_human']
  with open(csv_path, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    w.writerow({k: metrics[k] for k in fieldnames})
  print(f"wrote {csv_path}")
  _write_solver_stamp()


if __name__ == '__main__':
  main()
