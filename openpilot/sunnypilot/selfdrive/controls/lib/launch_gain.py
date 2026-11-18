"""
Launch gain: the e2e model's acceleration scaled up at low speed, per model.

Some models launch from a stop more gently than wanted. Scaling only the model's positive accel keeps its shape:
a hesitation stays small, braking and stopping are untouched, and the lead MPC and the car's accel limit still cap it.

LAUNCH_GAIN = target / model over 0-10 m/s, fading out by 16 m/s. A model not listed drives stock.
Target launch accel by speed (m/s^2), halfway between Light Speed v2 and the driver's own launches without a lead:
  0-1: 1.2  1-2: 1.9  2-4: 2.0  4-6: 1.9  6-8: 1.4  8-10: 1.25  10-12: 0.9 m/s
To add a model: per launch from a stop (engaged, no gas), the median modelV2.action.desiredAcceleration in each speed
band; gain = target / median across launches.
"""
import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.models.helpers import get_selected_bundle

LAUNCH_GAIN = {
  'LSMV2': 1.25,  # Light Speed Model v2: 1.2-1.3 in every band, 1 engaged + 13 open-loop launches (2026-11)
}
FADE_BP = [10., 16.]  # m/s
GAIN_RATE = 0.5  # 1/s, a big/small model handover eases between gains instead of stepping


class LaunchGain:
  def __init__(self, params: Params | None = None):
    params = params or Params()
    self.model_gain = {}
    for source, big in (('qcom', False), ('chestnut', True)):
      bundle = get_selected_bundle(params, source)
      self.model_gain[big] = LAUNCH_GAIN.get(bundle.internalName, 1.) if bundle is not None else 1.
    self.gain = 1.

  def update(self, sm, a_model: float) -> float:
    """Return the e2e candidate with the gain applied."""
    md = sm['modelV2']
    target = float(np.interp(sm['carState'].vEgo, FADE_BP, [self.model_gain[md.big], 1.]))
    self.gain = float(np.clip(target, self.gain - GAIN_RATE * DT_MDL, self.gain + GAIN_RATE * DT_MDL))
    if a_model <= 0. or md.action.shouldStop:
      return a_model
    return a_model * self.gain
