import unittest

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import VisionLeadSpeedFilter


def _lead(d_rel, v_lead, present=True, radar=False):
  return {'present': present, 'dRel': float(d_rel), 'vLead': float(v_lead),
          'vLeadK': float(v_lead), 'vRel': 0.0, 'radar': radar}


def _run(frames):
  """frames: list of (lead, v_ego, lead_distance, lead_valid). Returns corrected vLead list."""
  f = VisionLeadSpeedFilter(DT_MDL)
  return [f.correct(*fr)['vLead'] for fr in frames]


class TestVisionLeadSpeedFilter(unittest.TestCase):
  def test_no_camera_passes_model_through(self):
    # No camera lead (e.g. leadTwo, or leadOne before the camera locks) -> never touch vLead, even if the
    # model dRel is collapsing/noisy. This is the leadTwo cut-in over-brake fix.
    v_ego = 28.0
    frames = []
    d_model = 80.0
    for _ in range(120):
      d_model = max(d_model - 15.0 * DT_MDL, 36.0)   # model dRel collapsing (re-range); must be ignored
      frames.append((_lead(d_model, 24.0), v_ego, 0.0, False))
    out = _run(frames)
    self.assertTrue(all(o == 24.0 for o in out), f"expected clean passthrough, got min {min(out):.2f}")

  def test_radar_lead_passes_through(self):
    out = _run([(_lead(80.0, 20.0, radar=True), 25.0, 80.0, True) for _ in range(20)])
    self.assertTrue(all(o == 20.0 for o in out))

  def test_camera_corrects_stopped_lead(self):
    # Camera shows a stopped lead (gap closing at v_ego); the model over-reports its speed. The filter
    # should pull vLead well below the model's estimate.
    v_ego = 20.0
    frames = []
    dist = 200.0
    for _ in range(100):
      dist -= v_ego * DT_MDL                          # closing on a stopped lead, still beyond the blend window
      frames.append((_lead(dist, 20.0), v_ego, dist, True))
    out = _run(frames)
    self.assertLess(out[-1], 5.0, f"stopped lead should read ~0, got {out[-1]:.2f}")

  def test_camera_steady_gap_no_spurious_brake(self):
    # Lead keeping pace (gap steady), model speed correct -> no downward crash.
    v_ego = 28.0
    out = _run([(_lead(90.0, 28.0), v_ego, 90.0, True) for _ in range(120)])
    self.assertLess(abs(out[-1] - 28.0), 1.0, f"steady lead crashed to {out[-1]:.2f}")

  def test_uses_camera_not_model_drel(self):
    # Camera gap steady (lead keeping pace) but the model dRel is noisy/low -> output follows the camera,
    # not the noisy model dRel (guards against regressing to model-dRel differentiation).
    rng = np.random.default_rng(0)
    v_ego = 28.0
    out = _run([(_lead(70.0 + rng.normal(0, 8.0), 28.0), v_ego, 90.0, True) for _ in range(120)])
    self.assertGreater(out[-1], 25.0, f"noisy model dRel leaked into vLead: {out[-1]:.2f}")

  def test_near_lead_keeps_model_speed(self):
    # Inside the blend window's near edge the model is trusted, so a closing camera gap must not pull vLead
    # down -- this is what keeps nearby leads on the model's own estimate.
    v_ego = 20.0
    frames = []
    dist = 48.0
    for _ in range(40):
      dist -= v_ego * DT_MDL
      frames.append((_lead(dist, 20.0), v_ego, dist, True))  # below W_DIST[0]
    out = _run(frames)
    self.assertTrue(all(o == 20.0 for o in out), f"near lead corrected, min {min(out):.2f}")

  def test_weight_ramps_with_distance(self):
    # Same approach (lead 10 m/s slower than the model says) ending at different distances: the correction
    # grows from none at the near edge to full beyond the far edge.
    v_ego = 25.0
    def approach(end):
      f = VisionLeadSpeedFilter(DT_MDL)
      out = 0.0
      for k in range(40):
        d = end + 10.0 * DT_MDL * (39 - k)
        out = f.correct(_lead(d, 25.0), v_ego, d, True)['vLead']
      return out
    near, mid, far = approach(30.0), approach(60.0), approach(100.0)
    self.assertEqual(near, 25.0)
    self.assertLess(mid, near - 1.0)
    self.assertLess(far, mid - 1.0)
