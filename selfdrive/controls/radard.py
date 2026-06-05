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


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
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
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


class VisionLeadSpeedFilter:
  """Correct the model's over-estimated distant-lead speed on radarless cars.

  On a radarless car the lead's vLead comes straight from the model, which over-reports a distant
  lead's speed (biased toward ego speed) and converges only as the lead closes -- so the planner
  doesn't realize the lead is slower until it is close, and brakes late. The lead's *position*
  (dRel) is more reliable than its inferred velocity, so a position-derived speed
  (v_ego + d(dRel)/dt) recovers the true speed.

  Blend the position-derived speed into vLead weighted by the model's own velocity uncertainty
  (leadsV3 vStd): trust the model when it is confident (vStd low, lead close -- where precise
  braking matters) and fade toward the position-derived speed when the model is uncertain (vStd
  high, lead far). Downward-only (never raise the reported speed), robustified (median + clip + EMA
  on the noisy dRel derivative, which rejects re-ranging jumps) and rate-limited so it eases rather
  than jumps. aLeadK is left untouched: this corrects a speed estimate, it does not imply the lead
  is decelerating. Validated via offline replay (late_brake brakes ~3 s earlier; low-speed/gentle
  unchanged). Thresholds tuned on limited routes -- revisit as data accumulates.
  """
  W_VSTD = (0.8, 1.6)        # leadsV3 vStd window: blend weight ramps 0 -> 1 across it
  CLOSE_CLIP = (-12.0, 8.0)  # m/s -- plausible closing-rate bound (rejects dRel re-ranging jumps)
  V_RATE = 5.0               # m/s^2 -- max rate the corrected speed eases
  DREL_TAU, GAP_TAU = 0.3, 0.6  # s -- smoothing time constants

  def __init__(self, dt: float):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.ema_drel: float | None = None
    self.closings: deque[float] = deque(maxlen=11)
    self.gap_off = 0.0        # smoothed (position-derived speed - v_ego)
    self.v_corr: float | None = None

  def correct(self, lead: dict[str, Any], v_ego: float, v_std: float, freeze: bool = False) -> dict[str, Any]:
    # freeze: a lead re-range (cut-in) is in progress (see CutInDetector) -- the model's dRel is
    # jumping between vehicles, so its derivative is a fabricated closing rate that would otherwise
    # be turned into phantom braking. Pass the model's own vLead through and re-init, so the filter
    # re-engages cleanly on the settled lead once the re-range is done.
    if (not lead['status']) or lead.get('radar', False) or freeze:  # only steady vision leads
      self.reset()
      return lead
    dt = self.dt
    d_rel = lead['dRel']
    v_model = lead['vLead']
    if self.ema_drel is None:
      self.ema_drel = d_rel
      self.v_corr = v_model

    # position-derived speed: median-filtered, clipped closing rate added to v_ego
    prev = self.ema_drel
    self.ema_drel += (dt / (self.DREL_TAU + dt)) * (d_rel - prev)
    self.closings.append((self.ema_drel - prev) / dt)
    med = sorted(self.closings)[len(self.closings) // 2]
    med = min(max(med, self.CLOSE_CLIP[0]), self.CLOSE_CLIP[1])
    self.gap_off += (dt / (self.GAP_TAU + dt)) * (med - self.gap_off)
    gap_v = v_ego + self.gap_off

    # vStd-weighted blend, downward-only, eased
    w = min(max((v_std - self.W_VSTD[0]) / (self.W_VSTD[1] - self.W_VSTD[0]), 0.0), 1.0)
    target = min(v_model, (1.0 - w) * v_model + w * gap_v)
    step = self.V_RATE * dt
    self.v_corr += min(max(target - self.v_corr, -step), step)
    v_new = min(self.v_corr, v_model)
    if v_new < v_model - 0.05:
      lead = dict(lead)
      lead['vLead'] = float(v_new)
      lead['vLeadK'] = float(v_new)
      lead['vRel'] = float(v_new - v_ego)
    return lead


class CutInDetector:
  """Detect a lead re-range (cut-in) on radarless cars using the stock camera's lead track.

  The model's leadOne can smoothly re-range from a far lead onto a nearer car that cuts in. That
  looks like a hard-decelerating lead (dRel collapsing ~100->40 m in ~1 s) even though nothing
  actually decelerated -- it fabricates a closing rate that VisionLeadSpeedFilter would turn into
  phantom braking. The stock forward camera reports identity-stable object tracks
  (cameraObjectTracksSP); its lead track (slot 0) holds a steady distance while the model
  re-ranges, so a cut-in shows up as either:

    * the model's leadOne closing much faster than the camera's in-path lead actually is, or
    * the camera lead's objectId changing.

  Either flags a re-range; while flagged the speed filter is frozen. The camera only sees to
  ~100 m, so when it has no in-path lead (lead beyond range, or only adjacent-lane cars) it can't
  corroborate and never flags -- leaving the filter free to do its job on distant leads. The
  camera's lateral is curve-compensated, so the in-path test holds on bends. (slot 0 == camera
  TRACK_INDEX 1; the in-path gate guards the case where it parks on an adjacent car.)
  """
  Y_INPATH = 1.8       # m, |yRel| within this is in the ego path (vs an adjacent-lane car)
  DIV_ENGAGE = 8.0     # m/s, model closing this much faster than the camera lead -> re-range
  TAU = 0.3            # s, smoothing for dRel and the closing-rate estimates

  def __init__(self, dt: float):
    self.dt = dt
    self.hold_frames = int(round(1.0 / dt))  # keep the flag up through the ~1 s re-range
    self.reset()

  def reset(self) -> None:
    self.prev_id: int | None = None
    self.ema_cam: float | None = None
    self.ema_model: float | None = None
    self.cam_closing = 0.0
    self.model_closing = 0.0
    self.flagged = False
    self.hold = 0

  def _ema(self, prev: float | None, x: float) -> float:
    return x if prev is None else prev + (self.dt / (self.TAU + self.dt)) * (x - prev)

  def update(self, cam_tracks, model_lead: dict[str, Any]) -> bool:
    if not model_lead['status']:
      self.reset()
      return False

    # slot 0 is the camera's designated lead; only trust it as a lead when it is in-path
    cam = cam_tracks[0] if len(cam_tracks) else None
    cam_inpath = cam is not None and cam.valid and abs(cam.yRel) < self.Y_INPATH

    id_changed = False
    if cam_inpath:
      if self.prev_id is not None and cam.objectId != self.prev_id:
        id_changed = True
        self.ema_cam = float(cam.dRel)  # don't differentiate dRel across an identity change
      prev = self.ema_cam
      self.ema_cam = self._ema(self.ema_cam, float(cam.dRel))
      if prev is not None and not id_changed:
        self.cam_closing = self._ema(self.cam_closing, -(self.ema_cam - prev) / self.dt)
      self.prev_id = cam.objectId
    else:
      self.prev_id = None
      self.ema_cam = None
      self.cam_closing = 0.0

    prev_m = self.ema_model
    self.ema_model = self._ema(self.ema_model, float(model_lead['dRel']))
    if prev_m is not None:
      self.model_closing = self._ema(self.model_closing, -(self.ema_model - prev_m) / self.dt)

    divergence = self.model_closing - self.cam_closing
    trigger = cam_inpath and (id_changed or divergence > self.DIV_ENGAGE)
    if trigger:
      self.flagged = True
      self.hold = self.hold_frames
    elif self.flagged:
      if self.hold <= 0:
        self.flagged = False
      else:
        self.hold -= 1
    return self.flagged


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP, low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

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

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    # Correct the model's over-estimated distant-lead speed on radarless cars (see class docstring)
    self.lead_one_speed_filter = VisionLeadSpeedFilter(DT_MDL)
    self.lead_two_speed_filter = VisionLeadSpeedFilter(DT_MDL)
    # Freeze the speed filter during a lead re-range / cut-in so a model lead swap isn't turned
    # into phantom braking (uses the stock camera's object tracks; see CutInDetector)
    self.cut_in_detector = CutInDetector(DT_MDL)

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
      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, self.CP, self.CP_SP, low_speed_override=True)
      lead_two = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, self.CP, self.CP_SP, low_speed_override=False)
      if self.CP.radarUnavailable:  # vision-only lead speed is over-estimated at distance; correct it
        cam_tracks = sm['cameraObjectTracksSP'].tracks if sm.valid['cameraObjectTracksSP'] else []
        freeze = self.cut_in_detector.update(cam_tracks, lead_one)  # lead re-range (cut-in) in progress?
        v_std0 = leads_v3[0].vStd[0] if len(leads_v3[0].vStd) else 0.0
        v_std1 = leads_v3[1].vStd[0] if len(leads_v3[1].vStd) else 0.0
        lead_one = self.lead_one_speed_filter.correct(lead_one, self.v_ego, v_std0, freeze=freeze)
        lead_two = self.lead_two_speed_filter.correct(lead_two, self.v_ego, v_std1, freeze=freeze)
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
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks', 'cameraObjectTracksSP'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
