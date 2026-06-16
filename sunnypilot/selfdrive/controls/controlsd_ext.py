"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
from collections import deque

import numpy as np

import cereal.messaging as messaging
from cereal import log, custom

# OP lane render for the Honda Bosch radarless dash
DASH_PATH_FIT_MAX = 110.0      # m, cubic fit domain -- must stay > lane_path.D_MAX (100 m) so the far points are interpolated, not extrapolated
DASH_PATH_PROB_ON = 0.25       # prob >= this to start drawing a line
DASH_PATH_PROB_OFF = 0.10      # prob below this drops it (low hysteresis rail)
# When only one ego line is trusted, shift the center so the dash draws that line where the model sees it.
# Tied to LANE_WIDTH_MAYBE=32; calibrate on-device.
DASH_HALF_OFFSET = 1.65        # m, dash's lateral line offset from the center path
DASH_PATH_FADE_T = 0.5         # s: shrink reach 1->0 over this long (retract far->near) before blanking
# Lane-change rendering. During a cross the model craters laneLineProbs[1,2] to ~0 for ~2 s while the ego
# lines re-index across the line, but the lane-center geometry stays smooth -- so below MAINTAIN we keep
# rendering the LIVE fit (not a frozen one) for this long after the last confident frame, which carries the
# crossing instead of blanking it. (Replaces the old 0.6 s hold.) A genuine model dropout has no fresh fit,
# so it just holds the last poly for this long, then fades.
DASH_PATH_CROSS_HOLD_T = 2.5   # s
# Lane-cross pulse (LKAS_HUD_2 LEFT/RIGHT_LANE_CROSSED)
DASH_PATH_CROSS_NEAR_X = 4.0      # m, look-ahead at which the lane center is measured
DASH_PATH_CROSS_SWING = 2.0      # m, lane-center swing over the window that means a re-index
DASH_PATH_CROSS_WINDOW = 0.8     # s, look-back window for the swing
DASH_PATH_CROSS_REFRACTORY = 1.5 # s, collapse a crossing's swing into one pulse
# Limit how far to draw lane. Drawing too long can show inaccurate lanes on the far end.
DASH_PATH_FULL_LEN_SPEED = 27.0  # m/s at which the lane reaches full draw length (~60 mph)
DASH_PATH_LEAD_FULL_DIST = 70.0  # m lead distance at which the lane reaches full length
DASH_PATH_MIN_REACH = 0.15

from opendbc.car import structs
from opendbc.sunnypilot.car.honda.lane_path import LANE_LENGTH_MAX_VALUE
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


def _line_trusted(prob: float, was_on: bool) -> bool:
  """Draw a line when its existence prob clears the rail (hysteresis: ON to start drawing, OFF to keep)."""
  return prob >= (DASH_PATH_PROB_OFF if was_on else DASH_PATH_PROB_ON)


def _fit_cubic(x: np.ndarray, y: np.ndarray) -> list[float] | None:
  """Cubic coeffs [c0, c1, c2, c3] fit over x <= DASH_PATH_FIT_MAX; None if too few points in range."""
  m = x <= DASH_PATH_FIT_MAX
  if m.sum() < 4:
    return None
  return [float(v) for v in np.polyfit(x[m], y[m], 3)[::-1]]


def select_lane_render(model: log.ModelDataV2, prev_left: bool, prev_right: bool) -> tuple[list[float] | None, bool, bool]:
  """Choose the dash center cubic + which ego lines to draw, from per-side model confidence (stateless).

  Returns (center_poly, left_on, right_on); center_poly is None when neither ego line is trusted -- only detected
  lanes are shown (the caller then holds/fades the last render, then blanks). Cases:
    both ego lines trusted -> center = mean(ego lines), both lines on (shows in-lane position + curvature)
    one trusted            -> center = that line -+ DASH_HALF_OFFSET so the dash draws it where the model sees it
    neither                -> None
  """
  lls, probs = model.laneLines, model.laneLineProbs
  if len(lls) < 3 or len(probs) < 3 or len(lls[1].x) == 0:
    return None, False, False

  left = _line_trusted(probs[1], prev_left)
  right = _line_trusted(probs[2], prev_right)
  x = np.array(lls[1].x)
  yl, yr = np.array(lls[1].y), np.array(lls[2].y)
  if left and right:
    poly = _fit_cubic(x, (yl + yr) / 2.0)
  elif right:
    poly = _fit_cubic(x, yr - DASH_HALF_OFFSET)
  elif left:
    poly = _fit_cubic(x, yl + DASH_HALF_OFFSET)
  else:
    return None, False, False
  return (poly, left, right) if poly is not None else (None, False, False)


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()

    # dash lane render (LANE_PATH) hold/fade + per-side state
    self._dash_on = False
    self._dash_poly: list[float] = []
    self._dash_good_t = 0.0
    self._left_on = False
    self._right_on = False
    self._lane_hist: deque = deque()  # (t, near lane-center) over the last CROSS_WINDOW, for the lane-change swing
    self._cross_t = 0.0               # last lane-cross pulse time (refractory)

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

  def get_dash_path(self, model: log.ModelDataV2, model_valid: bool, v_ego: float, lead_d: float) -> dict:
    """Pick the dash center cubic + which ego lines to draw (select_lane_render: per-side prob+std, with a
    road-edge midpoint fallback when neither line is trusted), then apply the temporal machinery. Renders lane
    changes for free: the model re-indexes its ego lines as the car crosses, so the lane-center fit swings into
    the new pair of lines -- exactly how the stock dash animates a cross.
    """
    now = time.monotonic()
    blank = {"valid": False, "poly": [], "reach": 0.0, "laneCross": 0, "leftLine": False, "rightLine": False}

    poly, left_on, right_on, near_lines = None, False, False, None
    if model_valid:
      poly, left_on, right_on = select_lane_render(model, self._left_on, self._right_on)
      lls = model.laneLines  # near ego-line positions for the cross flag (independent of which lines we draw)
      if len(lls) >= 3 and len(lls[1].x):
        near_lines = (float(np.interp(DASH_PATH_CROSS_NEAR_X, lls[1].x, lls[1].y)),
                      float(np.interp(DASH_PATH_CROSS_NEAR_X, lls[2].x, lls[2].y)))

    # A fresh render refreshes the geometry + hold clock; otherwise hold through the crater of a lane change (the
    # model briefly craters both ego-line probs as it re-indexes across the line) and keep BOTH lines on -- like
    # the stock camera -- so a one-frame per-side prob asymmetry isn't frozen into a spurious line drop for the hold.
    if poly is not None:
      self._dash_on = True
      self._dash_good_t = now
      self._dash_poly = poly
      self._left_on, self._right_on = left_on, right_on
    elif self._dash_on:
      self._left_on = self._right_on = True

    # lane-cross: a lane change re-indexes the ego pair, swinging the near lane-center ~a lane width through the
    # car. Fire ONE pulse when that swing exceeds CROSS_SWING within CROSS_WINDOW; the swing sign is the side.
    lane_cross = 0
    if near_lines is not None and self._dash_on:
      lane_c = (near_lines[0] + near_lines[1]) / 2.0
      self._lane_hist.append((now, lane_c))
      while self._lane_hist[0][0] < now - DASH_PATH_CROSS_WINDOW:
        self._lane_hist.popleft()
      swing = lane_c - self._lane_hist[0][1]
      if abs(swing) > DASH_PATH_CROSS_SWING and now - self._cross_t > DASH_PATH_CROSS_REFRACTORY:
        lane_cross = -1 if swing < 0 else 1
        self._cross_t = now

    if not self._dash_on:
      return blank

    # dropout fade: full reach while fresh, then retract far->near over FADE_T, then blank
    elapsed = now - self._dash_good_t
    if elapsed <= DASH_PATH_CROSS_HOLD_T:
      fade = 1.0
    elif elapsed <= DASH_PATH_CROSS_HOLD_T + DASH_PATH_FADE_T:
      fade = 1.0 - (elapsed - DASH_PATH_CROSS_HOLD_T) / DASH_PATH_FADE_T
    else:
      self._dash_on = False
      return blank

    # draw length = longest of the speed term, the lead term, and a min floor
    speed_reach = v_ego / DASH_PATH_FULL_LEN_SPEED
    lead_reach = lead_d / DASH_PATH_LEAD_FULL_DIST   # lead_d == 0 when no lead -> no extension
    reach = fade * float(np.clip(max(speed_reach, lead_reach, DASH_PATH_MIN_REACH), 0.0, 1.0))
    if round(reach * LANE_LENGTH_MAX_VALUE) <= 0:
      return blank

    return {"valid": True, "poly": self._dash_poly, "reach": reach, "laneCross": int(lane_cross),
            "leftLine": self._left_on, "rightLine": self._right_on}

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
    CC_SP.dashPath = self.get_dash_path(sm['modelV2'], sm.valid['modelV2'], cs.vEgo, lead_d)

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
