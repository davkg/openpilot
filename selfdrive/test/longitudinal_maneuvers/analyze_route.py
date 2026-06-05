#!/usr/bin/env python3
"""
Per-route analyzer: plot a real route and validate the offline MPC against it
via open-loop replay (feed logged radarState + ego state through the real
LongitudinalPlanner, compare planned aTarget to what the car logged).

Usage: analyze_route.py <route_ext.csv>
"""
import sys
import numpy as np
import matplotlib.pyplot as plt

from cereal import log, messaging
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

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


def medfilt(a, k=25):
  half = k // 2
  return np.array([np.median(a[max(0, i-half):i+half+1]) for i in range(len(a))])


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
  cot = messaging.new_message('cameraObjectTracksSP')

  # optional camera object tracks: r['cam_tracks'] = list of (objectId, dRel, yRel, valid) per slot
  ct = r.get('cam_tracks')
  if ct:
    tl = cot.cameraObjectTracksSP.init('tracks', len(ct))
    for i, (oid, dr, yr, v) in enumerate(ct):
      tl[i].slot = i
      tl[i].objectId = int(oid)
      tl[i].dRel = float(dr)
      tl[i].yRel = float(yr)
      tl[i].valid = bool(v)

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
          'liveMapDataSP': live_map_data_sp.liveMapDataSP, 'gpsLocation': gps.gpsLocation,
          'cameraObjectTracksSP': cot.cameraObjectTracksSP}


def replay(frames):
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  planner = LongitudinalPlanner(CP, CP_SP, init_v=frames[0]['vEgo'])
  aT, src = [], []
  for r in frames:
    planner.update(build_sm(r))
    aT.append(planner.output_a_target)
    src.append(int(planner.mpc.source))
  return np.array(aT), np.array(src)


def main():
  path = sys.argv[1]
  name = path.replace('_ext.csv', '').split('/')[-1]
  raw = np.genfromtxt(path, delimiter=',', skip_header=1)
  raw_t = raw[:, 0] - raw[0, 0]
  cols = {n: ffill(raw[:, i]) for i, n in enumerate(NAMES)}
  t = np.arange(0.0, raw_t[-1], DT_MDL)
  d = {n: np.interp(t, raw_t, cols[n]) for n in NAMES}
  frames = [{n: d[n][i] for n in NAMES} for i in range(len(t))]

  print(f"\n=== {name}  ({t[-1]:.0f}s, personality={int(round(d['personality'][0]))}, "
        f"expMode={int(round(d['expMode'][0]))}) ===")
  aT_replay, src_replay = replay(frames)
  logged = d['aTarget']
  err = aT_replay - logged
  print(f"  validation: RMS {np.sqrt(np.mean(err**2)):.3f}  mean|err| {np.mean(np.abs(err)):.3f}"
        f"  max|err| {np.max(np.abs(err)):.3f} m/s^2")
  print(f"  planSource match: {np.mean(src_replay == np.round(d['source']))*100:.0f}%")
  print(f"  peak decel: logged {logged.min():+.2f}  replay {aT_replay.min():+.2f} m/s^2")
  print(f"  vEgo {d['vEgo'].min():.1f}-{d['vEgo'].max():.1f} m/s, "
        f"leadOne dRel {np.nanmin(d['l1_dRel']):.0f}-{np.nanmax(d['l1_dRel']):.0f} m")
  drops = int(np.sum((d['l1_modelProb'][:-1] >= 0.5) & (d['l1_modelProb'][1:] < 0.5)))
  print(f"  leadOne modelProb dropouts: {drops}, raw vLeadK 1-sample noise std "
        f"{np.std(np.diff(d['l1_vLeadK'])):.2f} m/s")

  vLead_f = medfilt(d['l1_vLeadK'])
  dRel_f = medfilt(d['l1_dRel'])

  fig, axs = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
  fig.suptitle(f"{name}: real route vs offline MPC replay (cap disabled)")

  axs[0].plot(t, logged, label='logged aTarget (real car)', lw=3, color='black')
  axs[0].plot(t, aT_replay, label='replay aTarget (offline MPC)', lw=1.6, color='tab:blue')
  axs[0].plot(t, d['aEgo'], label='aEgo (actual)', lw=1, color='tab:gray', alpha=0.6)
  axs[0].axhline(0, ls=':', color='gray')
  axs[0].set_ylabel("accel (m/s^2)"); axs[0].legend(loc='lower right'); axs[0].grid(True)

  axs[1].plot(t, d['vEgo'], label='vEgo', lw=2.5, color='tab:blue')
  axs[1].plot(t, d['l1_vLeadK'], label='leadOne vLeadK (raw)', lw=0.6, color='tab:orange', alpha=0.4)
  axs[1].plot(t, vLead_f, label='leadOne vLeadK (filtered)', lw=2, color='tab:orange')
  axs[1].plot(t, d['vCruise'] * 0.27778, label='vCruise', ls='--', color='gray')
  axs[1].set_ylabel("speed (m/s)"); axs[1].legend(loc='upper right'); axs[1].grid(True)

  axs[2].plot(t, d['l1_dRel'], label='leadOne dRel (raw)', lw=0.6, color='tab:green', alpha=0.4)
  axs[2].plot(t, dRel_f, label='leadOne dRel (filtered)', lw=2.5, color='tab:green')
  l2 = np.where(d['l2_status'] > 0.5, d['l2_dRel'], np.nan)
  axs[2].plot(t, l2, label='leadTwo dRel (valid)', lw=1, color='tab:olive')
  axs[2].set_ylabel("relative distance (m)"); axs[2].legend(loc='upper right'); axs[2].grid(True)
  axs[2].set_ylim(0, max(130, np.nanmax(d['l1_dRel']) * 1.1))

  axs[3].plot(t, np.round(d['source']), label='planSource logged', lw=2.5, color='black', drawstyle='steps-post')
  axs[3].plot(t, src_replay, label='planSource replay', lw=1.3, color='tab:blue', drawstyle='steps-post')
  axs[3].set_yticks([0, 1, 2]); axs[3].set_yticklabels(['cruise', 'lead0', 'lead1'])
  axs[3].set_ylabel("plan source"); axs[3].legend(loc='upper right'); axs[3].grid(True)

  axs[4].plot(t, d['l1_modelProb'], label='leadOne modelProb', lw=1.5, color='tab:green')
  axs[4].plot(t, d['l1_status'], label='leadOne status', lw=1.5, color='tab:red', drawstyle='steps-post')
  axs[4].plot(t, medfilt(d['l1_aLeadK']), label='leadOne aLeadK (filtered)', lw=1.2, color='tab:brown')
  axs[4].axhline(0, ls=':', color='gray')
  axs[4].set_ylabel("lead tracking"); axs[4].legend(loc='upper right'); axs[4].grid(True)
  axs[4].set_ylim(-3, 3); axs[4].set_xlabel("time (s)")

  plt.tight_layout()
  out = f"route_{name}.png"
  plt.savefig(out, dpi=110)
  print(f"  saved {out}")


if __name__ == "__main__":
  main()
