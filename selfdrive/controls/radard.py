#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


class VisionLeadSpeedFilter:
  """Correct the model's over-estimated distant-lead speed. Uses HONDA_BOSCH_RADARLESS leadDistance.
  The model over-reports distant (>~70m) lead speed, causing late braking. The LKAS camera's
  leadDistance is reliable, so we derive speed from it via d(distance)/dt. The derived speed is
  blended into vLead using the model's velocity uncertainty (vStd: high trust nearby, low trust
  distant). The blend is downward-only and rate-limited. With no camera lead we pass the model's
  vLead through unchanged. We use the LKAS camera leadDistance over the model's dRel because
  leadDistance is less noisy and cleanly tracks the lead when a cut-in occurs. dRel will
  re-range onto a cut-in car, which is difficult to differentiate from an abrupt slowdown. aLeadK
  is untouched.
  """
  W_VSTD = (1.0, 1.6)        # vStd blend window: weight ramps 0->1 (off below ~60 m, where the model is confident)
  OPEN_MAX = 8.0             # m/s -- max opening rate (caps upward re-range spikes)
  CLOSE_MARGIN = 2.0         # m/s -- closing capped at -(vEgo+margin); floors vLead at ~-margin
  V_RATE = 8.0               # m/s^2 -- max rate the corrected speed eases (MPC jerk-limits the brake downstream)
  JUMP_MARGIN = 8.0          # m -- re-range margin; threshold = v_ego*(tau+dt) + this (so fast steady closing isn't a re-range)
  DROP_HOLD = 5              # frames to hold state through a brief camera-lead dropout
  CAM_TAU = (0.1, 0.15)      # s -- (leadDistance EMA, speed EMA) smoothing; light, the full-rate leadDistance is clean

  def __init__(self, dt: float):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.ready = False                    # state initialized on the first valid frame
    self.ema_drel = 0.0                   # smoothed camera leadDistance
    self.ema_vego = 0.0                   # v_ego smoothed to the closing's lag (ego-matched, unbiased)
    self.v_abs = 0.0                      # smoothed position-derived lead speed
    self.v_corr = 0.0                     # rate-limited corrected speed
    self.miss = 0                         # consecutive dropout frames

  def correct(self, lead: dict[str, Any], v_ego: float, v_std: float, lead_distance: float,
              lead_valid: bool) -> dict[str, Any]:
    # radar lead: dRel is already a good measurement, not the model's over-estimate -- pass through.
    if lead.get('radar', False):
      self.reset()
      return lead
    # Only correct with the clean camera lead. No camera lead (or no model lead) -> pass the model's
    # vLead through; hold state briefly so a flickering camera lead doesn't force a re-ramp.
    if not (lead_valid and lead['status']):
      self.miss += 1
      if self.miss > self.DROP_HOLD:
        self.reset()
      return lead
    self.miss = 0
    dt = self.dt
    v_model = lead['vLead']
    drel_tau, gap_tau = self.CAM_TAU
    if not self.ready:
      self.ready = True
      self.ema_drel = lead_distance
      self.ema_vego = v_ego
      self.v_abs = v_model
      self.v_corr = v_model

    # ABSOLUTE lead speed = ego + closing, ego folded in BEFORE smoothing (so ego accel can't bias it).
    # Snap (no closing) on a camera re-range; the threshold is vEgo-relative (~v_ego*(tau+dt)) so a
    # fast approach to a stopped/slow lead isn't mistaken for one.
    self.ema_vego += (dt / (drel_tau + dt)) * (v_ego - self.ema_vego)
    prev = self.ema_drel
    if abs(lead_distance - prev) > v_ego * (drel_tau + dt) + self.JUMP_MARGIN:
      self.ema_drel = lead_distance
      sample = self.v_abs
    else:
      self.ema_drel += (dt / (drel_tau + dt)) * (lead_distance - prev)
      lo = -(self.ema_vego + self.CLOSE_MARGIN)   # a lead can't move backward, so vLead >= ~0
      closing = min(max((self.ema_drel - prev) / dt, lo), self.OPEN_MAX)
      sample = self.ema_vego + closing
    self.v_abs += (dt / (gap_tau + dt)) * (sample - self.v_abs)

    # vStd-weighted blend (model velocity vs position-derived), downward-only, eased
    w = min(max((v_std - self.W_VSTD[0]) / (self.W_VSTD[1] - self.W_VSTD[0]), 0.0), 1.0)
    target = min(v_model, (1.0 - w) * v_model + w * self.v_abs)
    step = self.V_RATE * dt
    self.v_corr += min(max(target - self.v_corr, -step), step)
    v_new = min(self.v_corr, v_model)
    if v_new < v_model - 0.05:
      lead = dict(lead)
      lead['vLead'] = float(v_new)
      lead['vLeadK'] = float(v_new)
      lead['vRel'] = float(v_new - v_ego)
    return lead


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_prob: float, low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any],
                    lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    self.CP = CP
    self.CP_SP = CP_SP

    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    # Correct the model's over-estimated distant-lead speed on radarless cars (see class docstring).
    # Only leadOne -- it's the camera's designated in-path lead; leadTwo has no camera lead, and the
    # filter only corrects with the camera (see VisionLeadSpeedFilter).
    self.lead_one_speed_filter = VisionLeadSpeedFilter(DT_MDL)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, self.CP, self.CP_SP,
                          self.lead_prob_filters[0].x, low_speed_override=True)
      lead_two = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, self.CP, self.CP_SP,
                          self.lead_prob_filters[1].x, low_speed_override=False)
      if self.CP.radarUnavailable:  # vision-only lead speed is over-estimated at distance; correct it
        cs_sp = sm['carStateSP']
        cam_lead_valid = sm.valid['carStateSP'] and cs_sp.cameraLeadValid
        v_std0 = leads_v3[0].vStd[0] if len(leads_v3[0].vStd) else 0.0
        # only leadOne -- the camera's designated in-path lead; leadTwo is passed through
        lead_one = self.lead_one_speed_filter.correct(lead_one, self.v_ego, v_std0, cs_sp.cameraLeadDistance, cam_lead_valid)
      self.radar_state.leadOne = lead_one
      self.radar_state.leadTwo = lead_two

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  cloudlog.info("radard is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got CarParamsSP")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks', 'carStateSP'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
