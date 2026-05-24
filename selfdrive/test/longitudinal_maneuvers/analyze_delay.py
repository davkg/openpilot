#!/usr/bin/env python3
"""
Actuator-delay analyzer for a real route. Loads an extracted CSV produced by
extract_signals.py and measures, on each brake event:
  - empirical lag from commanded accel -> aEgo (peak-to-peak and xcorr)
  - overshoot: |min aEgo| / |min aTarget|  (>1 = overshoots brake command)
  - tracking RMS aEgo vs aTarget delayed by best-fit lag

Then prints a per-event table and an overall median lag. Plots commanded vs
achieved accel for each brake event.

Usage: analyze_delay.py <route_ext.csv>
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


def find_brake_events(act_accel, t, threshold=-0.4, min_dur=1.0, merge_gap=1.5):
  """Find continuous spans where commanded accel <= threshold for >= min_dur seconds.
  Merge spans separated by less than merge_gap seconds."""
  below = act_accel <= threshold
  events = []
  i = 0
  n = len(below)
  while i < n:
    if below[i]:
      j = i
      while j < n and below[j]:
        j += 1
      events.append([i, j])
      i = j
    else:
      i += 1
  if not events:
    return []
  # merge close events
  merged = [events[0]]
  for s, e in events[1:]:
    if t[s] - t[merged[-1][1] - 1] < merge_gap:
      merged[-1][1] = e
    else:
      merged.append([s, e])
  # filter by duration
  return [(s, e) for s, e in merged if t[e - 1] - t[s] >= min_dur]


def best_lag_xcorr(cmd, act, t, max_lag=1.0):
  """Sliding xcorr: shift act backward in time, find lag that maximizes corr(cmd[t], act[t+lag])."""
  dt = t[1] - t[0]
  max_shift = int(max_lag / dt)
  cmd_c = cmd - cmd.mean()
  act_c = act - act.mean()
  best_lag, best_corr = 0.0, -np.inf
  for s in range(0, max_shift + 1):
    a, b = cmd_c[:len(cmd_c) - s], act_c[s:]
    if len(a) < 5:
      continue
    denom = (np.linalg.norm(a) * np.linalg.norm(b))
    if denom < 1e-9:
      continue
    c = np.dot(a, b) / denom
    if c > best_corr:
      best_corr, best_lag = c, s * dt
  return best_lag, best_corr


def main():
  path = sys.argv[1]
  name = path.replace('_ext.csv', '').split('/')[-1]
  raw = np.genfromtxt(path, delimiter=',', skip_header=1)
  raw_t = raw[:, 0] - raw[0, 0]
  cols = {n: ffill(raw[:, i]) for i, n in enumerate(NAMES)}
  t = np.arange(0.0, raw_t[-1], DT_MDL)
  d = {n: np.interp(t, raw_t, cols[n]) for n in NAMES}
  dt = DT_MDL

  print(f"\n=== {name}: {t[-1]:.0f}s, vEgo {d['vEgo'].min():.1f}-{d['vEgo'].max():.1f} m/s ===")
  print(f"  personality={int(round(d['personality'][0]))}, expMode={int(round(d['expMode'][0]))}")

  events = find_brake_events(d['act_accel'], t, threshold=-0.4, min_dur=1.0, merge_gap=1.5)
  print(f"\nFound {len(events)} brake events (cmd accel <= -0.4 m/s^2 for >= 1.0s):\n")
  print(f"  {'#':>2}  {'t0':>6}  {'dur':>5}  {'vEgo':>9}  {'min_cmd':>8}  {'min_aEgo':>9}  "
        f"{'overshoot':>9}  {'lag_xcorr':>9}  {'lag_peak':>8}")
  print(f"  {'-'*2}  {'-'*6}  {'-'*5}  {'-'*9}  {'-'*8}  {'-'*9}  {'-'*9}  {'-'*9}  {'-'*8}")

  per_event = []
  for ei, (s, e) in enumerate(events):
    # widen by 1.5s before/after for context
    pad = int(1.5 / dt)
    ss, ee = max(0, s - pad), min(len(t), e + pad)
    seg_t = t[ss:ee] - t[ss]
    cmd = d['act_accel'][ss:ee]
    act = d['aEgo'][ss:ee]
    v0, v1 = d['vEgo'][s], d['vEgo'][e - 1]
    lag_x, corr_x = best_lag_xcorr(cmd, act, seg_t, max_lag=1.0)
    # peak-based lag
    i_cmd = int(np.argmin(cmd))
    # find next aEgo trough after i_cmd, within 1.5s
    horizon = min(len(cmd), i_cmd + int(1.5 / dt))
    i_act = i_cmd + int(np.argmin(act[i_cmd:horizon])) if horizon > i_cmd else i_cmd
    lag_peak = (i_act - i_cmd) * dt
    min_cmd = float(np.min(cmd))
    min_act = float(np.min(act))
    overshoot = abs(min_act) / max(abs(min_cmd), 1e-6)
    per_event.append({
      's': s, 'e': e, 'ss': ss, 'ee': ee, 't0': t[s],
      'v0': v0, 'v1': v1, 'min_cmd': min_cmd, 'min_act': min_act,
      'overshoot': overshoot, 'lag_xcorr': lag_x, 'corr': corr_x,
      'lag_peak': lag_peak,
    })
    print(f"  {ei:>2}  {t[s]:>6.1f}  {t[e-1]-t[s]:>5.1f}  {v0:>4.1f}->{v1:<3.1f}  "
          f"{min_cmd:>+7.2f}  {min_act:>+8.2f}  {overshoot:>9.2f}  "
          f"{lag_x:>8.2f}s  {lag_peak:>7.2f}s")

  if per_event:
    lags_x = [p['lag_xcorr'] for p in per_event]
    lags_p = [p['lag_peak'] for p in per_event]
    overs = [p['overshoot'] for p in per_event]
    print(f"\n  median lag (xcorr):  {np.median(lags_x):.2f}s  (mean {np.mean(lags_x):.2f}, "
          f"range {min(lags_x):.2f}-{max(lags_x):.2f})")
    print(f"  median lag (peaks):  {np.median(lags_p):.2f}s  (mean {np.mean(lags_p):.2f}, "
          f"range {min(lags_p):.2f}-{max(lags_p):.2f})")
    print(f"  median overshoot:    {np.median(overs):.2f}x  (mean {np.mean(overs):.2f}, "
          f"range {min(overs):.2f}-{max(overs):.2f})")
    print(f"  current CP.longitudinalActuatorDelay = 0.30s")

  # full-route plot + per-event subplots
  fig = plt.figure(figsize=(14, 4 + 3 * max(1, len(per_event))))
  gs = fig.add_gridspec(1 + len(per_event), 1, height_ratios=[2] + [1] * len(per_event))

  ax0 = fig.add_subplot(gs[0])
  ax0.plot(t, d['act_accel'], label='commanded (actuators.accel)', lw=1.2, color='tab:blue')
  ax0.plot(t, d['aEgo'], label='achieved (aEgo)', lw=1.0, color='tab:gray', alpha=0.7)
  ax0.plot(t, d['aTarget'], label='planner aTarget', lw=0.8, color='tab:orange', alpha=0.6)
  for ei, p in enumerate(per_event):
    ax0.axvspan(t[p['s']], t[p['e'] - 1], color='red', alpha=0.10)
    ax0.text(t[p['s']], 1.0, f"#{ei}", color='red', fontsize=9)
  ax0.axhline(0, ls=':', color='gray')
  ax0.set_ylabel("accel (m/s^2)"); ax0.legend(loc='lower right'); ax0.grid(True)
  ax0.set_title(f"{name}: brake events (red bands) — current delay = 0.30s")

  for ei, p in enumerate(per_event):
    ax = fig.add_subplot(gs[ei + 1])
    seg_t = t[p['ss']:p['ee']] - t[p['ss']]
    cmd = d['act_accel'][p['ss']:p['ee']]
    act = d['aEgo'][p['ss']:p['ee']]
    ax.plot(seg_t, cmd, label='cmd', lw=1.6, color='tab:blue')
    ax.plot(seg_t, act, label='aEgo', lw=1.4, color='tab:gray')
    # overlay: cmd shifted by best lag (this is what the car "felt")
    shift = int(p['lag_xcorr'] / dt)
    if shift > 0:
      ax.plot(seg_t[shift:], cmd[:-shift], ls='--', lw=1.0, color='tab:blue', alpha=0.5,
              label=f'cmd shifted +{p["lag_xcorr"]:.2f}s')
    ax.axhline(0, ls=':', color='gray')
    ax.axhline(p['min_cmd'], ls=':', color='tab:blue', alpha=0.4)
    ax.axhline(p['min_act'], ls=':', color='tab:gray', alpha=0.4)
    ax.set_title(f"event #{ei}: t={p['t0']:.1f}s, vEgo {p['v0']:.1f}->{p['v1']:.1f} m/s, "
                 f"lag={p['lag_xcorr']:.2f}s, overshoot={p['overshoot']:.2f}x")
    ax.legend(loc='lower right', fontsize=8); ax.grid(True)
    ax.set_ylabel("m/s^2")

  plt.tight_layout()
  out = f"delay_{name}.png"
  plt.savefig(out, dpi=110)
  print(f"\n  saved {out}")


if __name__ == "__main__":
  main()
