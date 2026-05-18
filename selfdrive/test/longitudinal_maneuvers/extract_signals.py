#!/usr/bin/env python3
"""
Extract the longitudinal-analysis signal set from a big all-series PlotJuggler
CSV. Column layouts differ between exports, so columns are found by name.

Usage: extract_signals.py <input.csv> <output.csv>
"""
import sys
import subprocess

WANT = [
  '__time',
  '/carControl/actuators/accel',
  '/carState/aEgo',
  '/carState/vCruise',
  '/carState/vEgo',
  '/longitudinalPlan/aTarget',
  '/longitudinalPlan/accels/0',
  '/longitudinalPlan/hasLead',
  '/longitudinalPlan/longitudinalPlanSource',
  '/longitudinalPlan/speeds/0',
  '/radarState/leadOne/aLeadK',
  '/radarState/leadOne/aLeadTau',
  '/radarState/leadOne/dRel',
  '/radarState/leadOne/modelProb',
  '/radarState/leadOne/status',
  '/radarState/leadOne/vLead',
  '/radarState/leadOne/vLeadK',
  '/radarState/leadOne/vRel',
  '/radarState/leadTwo/aLeadK',
  '/radarState/leadTwo/aLeadTau',
  '/radarState/leadTwo/dRel',
  '/radarState/leadTwo/modelProb',
  '/radarState/leadTwo/status',
  '/radarState/leadTwo/vLead',
  '/radarState/leadTwo/vLeadK',
  '/selfdriveState/experimentalMode',
  '/selfdriveState/personality',
]


def main():
  inp, outp = sys.argv[1], sys.argv[2]
  with open(inp) as f:
    header = f.readline().rstrip('\n').split(',')
  missing = [n for n in WANT if n not in header]
  if missing:
    print(f"MISSING columns in {inp}:")
    for m in missing:
      print(f"  {m}")
    sys.exit(1)
  fields = ','.join(str(header.index(n) + 1) for n in WANT)  # cut is 1-based
  subprocess.run(f"cut -d, -f{fields} '{inp}' > '{outp}'", shell=True, check=True)
  print(f"{outp}: {len(WANT)} columns extracted")


if __name__ == "__main__":
  main()
