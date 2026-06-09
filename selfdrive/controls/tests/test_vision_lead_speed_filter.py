import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import VisionLeadSpeedFilter


def _lead(d_rel, v_lead, status=True, radar=False):
  return {'status': status, 'dRel': float(d_rel), 'vLead': float(v_lead),
          'vLeadK': float(v_lead), 'vRel': 0.0, 'radar': radar}


def _run(frames):
  """frames: list of (lead, v_ego, v_std, lead_distance, lead_valid). Returns corrected vLead list."""
  f = VisionLeadSpeedFilter(DT_MDL)
  return [f.correct(*fr)['vLead'] for fr in frames]


def test_no_camera_passes_model_through():
  # No camera lead (e.g. leadTwo, or leadOne before the camera locks) -> never touch vLead, even if the
  # model dRel is collapsing/noisy. This is the leadTwo cut-in over-brake fix.
  v_ego = 28.0
  frames = []
  d_model = 80.0
  for _ in range(120):
    d_model = max(d_model - 15.0 * DT_MDL, 36.0)   # model dRel collapsing (re-range); must be ignored
    frames.append((_lead(d_model, 24.0), v_ego, 2.0, 0.0, False))
  out = _run(frames)
  assert all(o == 24.0 for o in out), f"expected clean passthrough, got min {min(out):.2f}"


def test_radar_lead_passes_through():
  out = _run([(_lead(40.0, 20.0, radar=True), 25.0, 2.0, 40.0, True) for _ in range(20)])
  assert all(o == 20.0 for o in out)


def test_camera_corrects_stopped_lead():
  # Camera shows a stopped lead (gap closing at v_ego); the model over-reports its speed. The filter
  # should pull vLead well below the model's estimate.
  v_ego = 20.0
  frames = []
  dist = 150.0
  for _ in range(120):
    dist -= v_ego * DT_MDL                          # closing on a stopped lead
    frames.append((_lead(dist, 20.0), v_ego, 2.0, dist, True))
  out = _run(frames)
  assert out[-1] < 5.0, f"stopped lead should read ~0, got {out[-1]:.2f}"


def test_camera_steady_gap_no_spurious_brake():
  # Lead keeping pace (gap steady), model speed correct -> no downward crash.
  v_ego = 28.0
  out = _run([(_lead(60.0, 28.0), v_ego, 2.0, 60.0, True) for _ in range(120)])
  assert abs(out[-1] - 28.0) < 1.0, f"steady lead crashed to {out[-1]:.2f}"


def test_uses_camera_not_model_drel():
  # Camera gap steady (lead keeping pace) but the model dRel is noisy/low -> output follows the camera,
  # not the noisy model dRel (guards against regressing to model-dRel differentiation).
  rng = np.random.default_rng(0)
  v_ego = 28.0
  out = _run([(_lead(40.0 + rng.normal(0, 8.0), 28.0), v_ego, 2.0, 60.0, True) for _ in range(120)])
  assert out[-1] > 25.0, f"noisy model dRel leaked into vLead: {out[-1]:.2f}"
