"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import numpy as np

import cereal.messaging as messaging
from cereal import log, custom

# OP lane-center fit for dash rendering (consumed by Honda Bosch radarless LANE_PATH). The lane center
# (mean of the two ego lane lines) shows the car's position WITHIN the lane -- off-center shows up as a
# near offset -- plus the lane curvature ahead. (modelV2.position is the trajectory from the car origin,
# so it can't show in-lane position; that's why we use the lane lines here.)
DASH_PATH_FIT_MAX = 110.0      # m, cubic fit domain -- must stay > lane_path.D_MAX (100 m) so the far points are interpolated, not extrapolated
# Hysteresis + hold + fade so the rendered lane doesn't flicker when ego-lane-line confidence chatters
# (the right line often hovers ~0.3, so a single 0.3 gate toggled ~1/s and blanked the dash).
DASH_PATH_PROB_ENGAGE = 0.40   # (re)start showing the lane only above this confidence
DASH_PATH_PROB_MAINTAIN = 0.20 # once showing, frames above this keep it "confident" (low hysteresis rail)
DASH_PATH_FADE_T = 0.5         # s: shrink reach 1->0 over this long (retract far->near) before blanking
# Lane-change rendering. During a cross the model craters laneLineProbs[1,2] to ~0 for ~2 s while the ego
# lines re-index across the line, but the lane-center geometry stays smooth -- so below MAINTAIN we keep
# rendering the LIVE fit (not a frozen one) for this long after the last confident frame, which carries the
# crossing instead of blanking it. (Replaces the old 0.6 s hold.) A genuine model dropout has no fresh fit,
# so it just holds the last poly for this long, then fades.
DASH_PATH_CROSS_HOLD_T = 2.5   # s
# Lane-cross flag (pure model geometry, so it works regardless of OP engagement / desire): set while an ego lane
# line is within CROSS_LINE_EPS of the car center at CROSS_NEAR_X -- i.e. a line is under the car, so we're
# crossing it. Stateless: the line is only this close for ~0.3-0.5 s as it passes, which already lands on a 5 Hz
# LKAS_HUD_2 frame, so no edge/latch/de-bounce is needed (a noisy re-index re-asserting it for a frame is harmless).
DASH_PATH_CROSS_NEAR_X = 4.0   # m, look-ahead at which the ego lines are measured for crossing detection
DASH_PATH_CROSS_LINE_EPS = 0.4 # m, an ego line this close to the car center = a line is under the car (crossing)
# Draw length (LKAS_HUD_2 LANE_LENGTH). The stock dash draws the lane out to roughly how far it's usable ahead,
# NOT the full extrapolated path (whose far end is least confident). Two stock behaviours, taken as the LONGER
# of the two so the lane is never shorter than the lead.
DASH_PATH_FULL_LEN_SPEED = 27.0  # m/s at which the lane reaches full draw length (~60 mph)
DASH_PATH_LEAD_FULL_DIST = 70.0  # m lead distance at which the lane reaches full length

from opendbc.car import structs
from opendbc.sunnypilot.car.honda.lane_path import LANE_LENGTH_MAX_VALUE
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()

    # dash lane render (LANE_PATH) hold/fade state
    self._dash_on = False
    self._dash_poly: list[float] = []
    self._dash_good_t = 0.0

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP']
    self.pm_services_ext = ['carControlSP']

  def initialize_lateral_control(self, lac, CI, dt):
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      if self.CP.lateralTuning.which() == 'torque':
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)  # FIXME-SP: revert when upstream fixes tuning issues with v1
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["liveDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lead_data(ld: log.RadarState.LeadData) -> dict:
    return {
      "dRel": ld.dRel,
      "yRel": ld.yRel,
      "vRel": ld.vRel,
      "aRel": ld.aRel,
      "vLead": ld.vLead,
      "dPath": ld.dPath,
      "vLat": ld.vLat,
      "vLeadK": ld.vLeadK,
      "aLeadK": ld.aLeadK,
      "fcw": ld.fcw,
      "status": ld.status,
      "aLeadTau": ld.aLeadTau,
      "modelProb": ld.modelProb,
      "radar": ld.radar,
      "radarTrackId": ld.radarTrackId,
    }

  @staticmethod
  def _dash_cross_direction(model: log.ModelDataV2, left_blinker: bool, right_blinker: bool, near_lines: tuple) -> int:
    """Side being crossed: +1 right, -1 left. Prefer OP's commanded direction (engaged explicit lane change),
    then the blinker (manual cross), then geometry -- the ego line passing under the car, whose sign gives the
    side (observed on route 000000e9 seg21: the +y ego line crossing under = a right change)."""
    d = str(model.meta.laneChangeDirection)
    if d == 'left':
      return -1
    if d == 'right':
      return 1
    if right_blinker:
      return 1
    if left_blinker:
      return -1
    crossing_y = near_lines[0] if abs(near_lines[0]) < abs(near_lines[1]) else near_lines[1]
    return 1 if crossing_y > 0 else -1

  def get_dash_path(self, model: log.ModelDataV2, model_valid: bool, left_blinker: bool, right_blinker: bool,
                    v_ego: float, lead_d: float) -> dict:
    """Fit OP's lane center (mean of the two ego lane lines) to a cubic for dash rendering. Shows the car's
    position within the lane (off-center -> near offset) plus the lane curvature ahead, and renders lane
    changes: the model re-indexes its ego lines as the car crosses, so the lane-center fit naturally swings
    into the new pair of lines (the boundary line stays under the car; the far line swaps) -- exactly how the
    stock dash animates a cross.

    Hysteresis: PROB_ENGAGE to (re)start, PROB_MAINTAIN to stay "confident". Below MAINTAIN we keep rendering
    the LIVE fit (the geometry stays smooth even as the model craters its confidence mid-cross) for
    CROSS_HOLD_T after the last confident frame, then fade out over FADE_T and blank. The rendered length
    (reach) is the longer of a speed term (FULL_LEN_SPEED) and a lead term (lead_d / LEAD_FULL_DIST), so the
    dash shortens the lane at low speed but still extends it out to a lead -- never shorter than the lead car.
    A separate, engagement-independent geometry detector pulses laneCross when an ego line passes under the
    car, for the LKAS_HUD_2 LEFT/RIGHT_LANE_CROSSED flag.
    """
    now = time.monotonic()

    fresh_prob, fresh_poly, near_lines = -1.0, None, None
    if model_valid:
      lls, probs = model.laneLines, model.laneLineProbs
      if len(lls) >= 3 and len(probs) >= 3:
        x = np.array(lls[1].x)
        yc = (np.array(lls[1].y) + np.array(lls[2].y)) / 2.0
        m = x <= DASH_PATH_FIT_MAX
        if m.sum() >= 4:
          fresh_prob = min(probs[1], probs[2])
          fresh_poly = [float(v) for v in np.polyfit(x[m], yc[m], 3)[::-1]]  # [c0, c1, c2, c3]
          near_lines = (float(np.interp(DASH_PATH_CROSS_NEAR_X, lls[1].x, lls[1].y)),
                        float(np.interp(DASH_PATH_CROSS_NEAR_X, lls[2].x, lls[2].y)))

    # confident frame refreshes both the geometry and the hold clock; an unconfident frame with geometry still
    # refreshes the geometry (so a crossing renders live) but not the clock.
    if fresh_poly is not None and fresh_prob >= (DASH_PATH_PROB_MAINTAIN if self._dash_on else DASH_PATH_PROB_ENGAGE):
      self._dash_on = True
      self._dash_good_t = now
    if fresh_poly is not None and self._dash_on:
      self._dash_poly = fresh_poly

    # lane-cross flag: set while an ego line is under the car (a line within CROSS_LINE_EPS of the car center)
    lane_cross = 0
    if near_lines is not None and self._dash_on and min(abs(near_lines[0]), abs(near_lines[1])) < DASH_PATH_CROSS_LINE_EPS:
      lane_cross = self._dash_cross_direction(model, left_blinker, right_blinker, near_lines)

    if not self._dash_on:
      return {"valid": False, "poly": [], "reach": 0.0, "laneCross": 0}

    # dropout fade: full reach while fresh, then retract far->near over FADE_T, then blank
    elapsed = now - self._dash_good_t
    if elapsed <= DASH_PATH_CROSS_HOLD_T:
      fade = 1.0
    elif elapsed <= DASH_PATH_CROSS_HOLD_T + DASH_PATH_FADE_T:
      fade = 1.0 - (elapsed - DASH_PATH_CROSS_HOLD_T) / DASH_PATH_FADE_T
    else:
      self._dash_on = False
      return {"valid": False, "poly": [], "reach": 0.0, "laneCross": 0}

    # draw length = longer of the speed term and the lead term (lane never shorter than the lead), times the
    # dropout fade. When nothing is drawn (standstill, no lead, or fully faded) blank LANE_PATH too (valid False)
    # so both messages agree -- matching the camera's standstill frame.
    speed_reach = v_ego / DASH_PATH_FULL_LEN_SPEED
    lead_reach = lead_d / DASH_PATH_LEAD_FULL_DIST   # lead_d == 0 when no lead -> no extension
    reach = fade * float(np.clip(max(speed_reach, lead_reach), 0.0, 1.0))
    if round(reach * LANE_LENGTH_MAX_VALUE) <= 0:
      return {"valid": False, "poly": [], "reach": 0.0, "laneCross": 0}

    return {"valid": True, "poly": self._dash_poly, "reach": reach, "laneCross": int(lane_cross)}

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    CC_SP.leadOne = self.get_lead_data(sm['radarState'].leadOne)
    CC_SP.leadTwo = self.get_lead_data(sm['radarState'].leadTwo)

    # MADS state
    CC_SP.mads = sm['selfdriveStateSP'].mads

    CC_SP.intelligentCruiseButtonManagement = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.speedLimit = sm['selfdriveStateSP'].speedLimit

    # OP lane center for dash rendering (Honda Bosch radarless LANE_PATH)
    cs = sm['carState']
    lead = sm['radarState'].leadOne
    lead_d = lead.dRel if lead.status else 0.0   # extend the lane out to the lead (0 = no lead)
    CC_SP.dashPath = self.get_dash_path(sm['modelV2'], sm.valid['modelV2'], cs.leftBlinker, cs.rightBlinker, cs.vEgo, lead_d)

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
