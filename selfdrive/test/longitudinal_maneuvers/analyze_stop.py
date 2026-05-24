#!/usr/bin/env python3
"""
Zoom in on the stop+creep region. Plots vEgo, aTarget, commanded accel,
lead dRel, lead modelProb/status, planSource, and longCtrlState transitions.

Usage: analyze_stop.py <route_ext.csv> [t_start] [t_end]
"""
import sys
import numpy as np
import matplotlib.pyplot as plt

from openpilot.common.realtime import DT_MDL

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


def main():
  path = sys.argv[1]
  name = path.replace('_ext.csv', '').split('/')[-1]
  raw = np.genfromtxt(path, delimiter=',', skip_header=1)
  raw_t = raw[:, 0] - raw[0, 0]
  cols = {n: ffill(raw[:, i]) for i, n in enumerate(NAMES)}
  t = np.arange(0.0, raw_t[-1], DT_MDL)
  d = {n: np.interp(t, raw_t, cols[n]) for n in NAMES}

  t0 = float(sys.argv[2]) if len(sys.argv) > 2 else max(0.0, t[-1] - 16.0)
  t1 = float(sys.argv[3]) if len(sys.argv) > 3 else t[-1]
  m = (t >= t0) & (t <= t1)
  ts = t[m]

  print(f"\n=== {name}: zoom {t0:.1f}-{t1:.1f}s ===")

  # locate the first crossing where vEgo first reaches <0.2 m/s
  v = d['vEgo']
  stop_idx = None
  for i in range(len(v) - 1):
    if v[i] < 0.2 and not (v[max(0, i-10):i] < 0.2).any():
      stop_idx = i
      break
  if stop_idx is not None:
    print(f"  first near-stop at t={t[stop_idx]:.2f}s (vEgo {v[stop_idx]:.2f} m/s, "
          f"lead dRel {d['l1_dRel'][stop_idx]:.2f}m, "
          f"lead modelProb {d['l1_modelProb'][stop_idx]:.2f}, "
          f"lead status {d['l1_status'][stop_idx]:.0f})")
  # find the creep peak: a local max of vEgo after stop
  if stop_idx is not None:
    after = v[stop_idx:]
    creep_i = int(np.argmax(after))
    if after[creep_i] > 0.2 + v[stop_idx]:
      print(f"  creep peak at t={t[stop_idx + creep_i]:.2f}s, vEgo {after[creep_i]:.2f} m/s, "
            f"lead dRel {d['l1_dRel'][stop_idx + creep_i]:.2f}m, "
            f"lead modelProb {d['l1_modelProb'][stop_idx + creep_i]:.2f}, "
            f"lead status {d['l1_status'][stop_idx + creep_i]:.0f}")

  # also find when lead modelProb first drops below 0.5 in the stop region
  region = (t >= (t[stop_idx] - 1.0 if stop_idx else 0))
  rt = t[region]
  rprob = d['l1_modelProb'][region]
  rstat = d['l1_status'][region]
  rdrel = d['l1_dRel'][region]
  drop_idx = None
  for i in range(1, len(rprob)):
    if rprob[i-1] >= 0.5 and rprob[i] < 0.5:
      drop_idx = i; break
  if drop_idx is not None:
    print(f"  lead modelProb first drops <0.5 at t={rt[drop_idx]:.2f}s "
          f"(dRel {rdrel[drop_idx]:.2f}m, status {rstat[drop_idx]:.0f})")
  status_off = None
  for i in range(1, len(rstat)):
    if rstat[i-1] > 0.5 and rstat[i] < 0.5:
      status_off = i; break
  if status_off is not None:
    print(f"  lead status first goes 0 at t={rt[status_off]:.2f}s "
          f"(dRel {rdrel[status_off]:.2f}m, modelProb {rprob[status_off]:.2f})")

  fig, axs = plt.subplots(5, 1, figsize=(13, 13), sharex=True)
  fig.suptitle(f"{name}: stop & creep zoom ({t0:.0f}-{t1:.0f}s)")

  axs[0].plot(ts, d['vEgo'][m], lw=2.2, color='tab:blue', label='vEgo')
  axs[0].plot(ts, d['l1_vLeadK'][m], lw=1.0, color='tab:orange', alpha=0.7, label='leadOne vLeadK')
  axs[0].axhline(0, ls=':', color='gray')
  axs[0].set_ylabel("speed (m/s)"); axs[0].grid(True); axs[0].legend(loc='upper right')

  axs[1].plot(ts, d['aTarget'][m], lw=2.0, color='black', label='planner aTarget')
  axs[1].plot(ts, d['act_accel'][m], lw=1.0, color='tab:blue', alpha=0.6, label='commanded')
  axs[1].plot(ts, d['aEgo'][m], lw=1.0, color='tab:gray', alpha=0.6, label='aEgo')
  axs[1].axhline(0, ls=':', color='gray')
  axs[1].set_ylabel("accel (m/s^2)"); axs[1].grid(True); axs[1].legend(loc='lower right')

  axs[2].plot(ts, d['l1_dRel'][m], lw=2.0, color='tab:green', label='leadOne dRel')
  l2 = np.where(d['l2_status'][m] > 0.5, d['l2_dRel'][m], np.nan)
  axs[2].plot(ts, l2, lw=1.2, color='tab:olive', label='leadTwo dRel (valid)')
  axs[2].axhline(6.5, ls=':', color='tab:red', alpha=0.6, label='stop distance ~6.5m')
  axs[2].axhline(5.5, ls=':', color='tab:purple', alpha=0.6, label='stop distance aggr ~5.5m')
  axs[2].set_ylabel("dRel (m)"); axs[2].grid(True); axs[2].legend(loc='upper right')
  axs[2].set_ylim(0, min(30, np.nanmax(d['l1_dRel'][m]) * 1.2))

  axs[3].plot(ts, d['l1_modelProb'][m], lw=1.6, color='tab:green', label='leadOne modelProb')
  axs[3].plot(ts, d['l1_status'][m], lw=1.6, color='tab:red', drawstyle='steps-post', label='leadOne status')
  axs[3].plot(ts, d['l2_status'][m], lw=1.0, color='tab:orange', drawstyle='steps-post', alpha=0.7, label='leadTwo status')
  axs[3].axhline(0.5, ls=':', color='gray')
  axs[3].set_ylabel("lead tracking"); axs[3].grid(True); axs[3].legend(loc='center right')

  axs[4].plot(ts, np.round(d['source'][m]), lw=2.0, color='black', drawstyle='steps-post', label='planSource')
  axs[4].plot(ts, d['hasLead'][m], lw=1.0, color='tab:blue', drawstyle='steps-post', alpha=0.6, label='hasLead')
  axs[4].set_yticks([0, 1, 2]); axs[4].set_yticklabels(['cruise', 'lead0', 'lead1'])
  axs[4].set_ylabel("plan source"); axs[4].set_xlabel("t (s)")
  axs[4].grid(True); axs[4].legend(loc='center right')

  plt.tight_layout()
  out = f"stop_{name}.png"
  plt.savefig(out, dpi=110)
  print(f"  saved {out}")


if __name__ == "__main__":
  main()
