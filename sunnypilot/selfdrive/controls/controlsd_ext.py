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
DASH_PATH_FIT_MAX = 90.0       # m, fit domain (covers the dash look-ahead)
# Hysteresis + hold + fade so the rendered lane doesn't flicker when ego-lane-line confidence chatters
# (the right line often hovers ~0.3, so a single 0.3 gate toggled ~1/s and blanked the dash).
DASH_PATH_PROB_ENGAGE = 0.40   # (re)start showing the lane only above this confidence
DASH_PATH_PROB_MAINTAIN = 0.20 # once showing, frames above this keep it alive (low hysteresis rail)
DASH_PATH_HOLD_T = 0.6         # s: hold the last good path at full reach this long after the last good frame
DASH_PATH_FADE_T = 0.5         # s: then shrink reach 1->0 over this long (retract far->near) before blanking

from opendbc.car import structs
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

  def get_dash_path(self, model: log.ModelDataV2, model_valid: bool) -> dict:
    """Fit OP's lane center (mean of the two ego lane lines) to a cubic for dash rendering, with
    hysteresis + hold + fade so a chattering lane-line confidence doesn't flicker the dash. Shows the
    car's position within the lane (off-center -> near offset) plus the lane curvature ahead.

    While shown, the last good fit is held through brief dropouts; after HOLD_T with no good frame the
    rendered length (reach) shrinks 1->0 over FADE_T to retract the lane far->near, then it blanks.
    """
    now = time.monotonic()

    fresh_prob, fresh_poly = -1.0, None
    if model_valid:
      lls, probs = model.laneLines, model.laneLineProbs
      if len(lls) >= 3 and len(probs) >= 3:
        x = np.array(lls[1].x)
        yc = (np.array(lls[1].y) + np.array(lls[2].y)) / 2.0
        m = x <= DASH_PATH_FIT_MAX
        if m.sum() >= 4:
          fresh_prob = min(probs[1], probs[2])
          fresh_poly = [float(v) for v in np.polyfit(x[m], yc[m], 3)[::-1]]  # [c0, c1, c2, c3]

    # hysteresis: need PROB_ENGAGE to (re)start, only PROB_MAINTAIN to stay alive (refreshing the geometry)
    if fresh_poly is not None and fresh_prob >= (DASH_PATH_PROB_MAINTAIN if self._dash_on else DASH_PATH_PROB_ENGAGE):
      self._dash_on = True
      self._dash_poly = fresh_poly
      self._dash_good_t = now

    if not self._dash_on:
      return {"valid": False, "poly": [], "reach": 0.0}

    elapsed = now - self._dash_good_t
    if elapsed <= DASH_PATH_HOLD_T:
      reach = 1.0
    elif elapsed <= DASH_PATH_HOLD_T + DASH_PATH_FADE_T:
      reach = 1.0 - (elapsed - DASH_PATH_HOLD_T) / DASH_PATH_FADE_T  # retract far->near
    else:
      self._dash_on = False
      return {"valid": False, "poly": [], "reach": 0.0}

    return {"valid": True, "poly": self._dash_poly, "reach": float(reach)}

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    CC_SP.leadOne = self.get_lead_data(sm['radarState'].leadOne)
    CC_SP.leadTwo = self.get_lead_data(sm['radarState'].leadTwo)

    # MADS state
    CC_SP.mads = sm['selfdriveStateSP'].mads

    CC_SP.intelligentCruiseButtonManagement = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.speedLimit = sm['selfdriveStateSP'].speedLimit

    # OP lane center for dash rendering (Honda Bosch radarless LANE_PATH)
    CC_SP.dashPath = self.get_dash_path(sm['modelV2'], sm.valid['modelV2'])

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
