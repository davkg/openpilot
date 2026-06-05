from openpilot.selfdrive.controls.radard import CutInDetector

DT = 0.05


class MockTrack:
  def __init__(self, valid=True, objectId=1, dRel=50.0, yRel=0.0):
    self.valid = valid
    self.objectId = objectId
    self.dRel = dRel
    self.yRel = yRel


def lead(dRel, status=True):
  return {'status': status, 'dRel': dRel}


def run(det, frames):
  """frames: list of (cam_tracks, model_lead). Returns whether it flagged at any point."""
  flagged = False
  for cam_tracks, model_lead in frames:
    flagged = det.update(cam_tracks, model_lead) or flagged
  return flagged


def test_no_model_lead_never_flags():
  det = CutInDetector(DT)
  # camera shows a wild in-path object jumping, but there is no model lead -> nothing to gate
  frames = [([MockTrack(dRel=80 - i)], lead(0.0, status=False)) for i in range(40)]
  assert not run(det, frames)


def test_genuine_slowing_lead_does_not_flag():
  # model lead and camera lead (same objectId) close together at ~5 m/s -> no divergence
  det = CutInDetector(DT)
  frames = []
  d = 90.0
  for _ in range(60):
    d -= 5.0 * DT
    frames.append(([MockTrack(objectId=6, dRel=d, yRel=-0.5)], lead(d + 25)))  # camera reads a bit nearer; trend matches
  assert not run(det, frames)


def test_cut_in_flags():
  # model leadOne collapses (re-range) while the camera's in-path lead holds steady
  det = CutInDetector(DT)
  frames = []
  md = 108.0
  for _ in range(40):
    md = max(38.0, md - 2.3)  # ~46 m/s fabricated closing
    frames.append(([MockTrack(objectId=2, dRel=77.0, yRel=0.0)], lead(md)))
  assert run(det, frames)


def test_objectid_change_flags():
  # camera's in-path lead identity changes (a different car becomes the lead)
  det = CutInDetector(DT)
  frames = [([MockTrack(objectId=2, dRel=40.0, yRel=0.0)], lead(40.0)) for _ in range(20)]
  frames += [([MockTrack(objectId=9, dRel=38.0, yRel=0.0)], lead(38.0)) for _ in range(5)]
  assert run(det, frames)


def test_distant_lead_beyond_camera_does_not_flag():
  # the model has a lead the camera can't corroborate (only an adjacent-lane track) -> never flag,
  # so the speed filter stays free to correct distant leads (its whole purpose)
  det = CutInDetector(DT)
  frames = []
  md = 120.0
  for _ in range(40):
    md = max(60.0, md - 1.5)  # model lead closing
    frames.append(([MockTrack(objectId=23, dRel=45.0, yRel=-2.9)], lead(md)))  # only an adjacent car
  assert not run(det, frames)


def test_flag_releases_after_rerange():
  # once the re-range is over (model settles, matches a steady in-path camera lead), the flag clears
  det = CutInDetector(DT)
  md = 108.0
  for _ in range(30):
    md = max(38.0, md - 2.5)
    det.update([MockTrack(objectId=2, dRel=77.0, yRel=0.0)], lead(md))
  assert det.flagged
  # settled: camera lead is now the cut-in car at a steady distance matching the model
  for _ in range(60):
    det.update([MockTrack(objectId=12, dRel=38.0, yRel=0.0)], lead(38.0))
  assert not det.flagged
