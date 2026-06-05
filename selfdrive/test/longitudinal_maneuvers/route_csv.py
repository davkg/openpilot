#!/usr/bin/env python3
"""Shared loader for PlotJuggler route CSVs.

A full PlotJuggler export is a large all-series dump (~500-700 MB, ~15k columns)
with one shared `__time` column. It is sparse -- each signal updates on its own
clock, so most cells in a row are empty -- and column order differs between
exports, so signals must be located by name.

`load_route` pulls the requested signals out by name (via `cut`, which streams the
big file cheaply), forward-fills each sparse column, and resamples onto a uniform
time grid by as-of (nearest-previous) sampling -- correct for both continuous
signals (vEgo, dRel) and discrete ones (status, plan source). Returns a DataFrame
indexed by seconds-from-start.

Example:
    from openpilot.selfdrive.test.longitudinal_maneuvers.route_csv import load_route
    df = load_route("route.csv",
                    ["/carState/vEgo", "/radarState/leadOne/dRel"],
                    rename={"/carState/vEgo": "vEgo", "/radarState/leadOne/dRel": "dRel"})
    df["vEgo"]  # series indexed by t (s)
"""
import os
import subprocess
import tempfile

import numpy as np
import pandas as pd

from openpilot.common.realtime import DT_MDL


def load_route(csv_path: str, signals: list[str], dt: float = DT_MDL,
               rename: dict[str, str] | None = None) -> pd.DataFrame:
  """Extract `signals` from a sparse PlotJuggler CSV, ffill, resample to `dt`.

  signals: full PlotJuggler column names (e.g. "/carState/vEgo"). "__time" is
           always included automatically.
  rename:  optional {full_name: short_name} applied to the returned columns.
  Returns a DataFrame indexed by seconds-from-start (index name "t").
  """
  with open(csv_path) as f:
    header = f.readline().rstrip('\n').split(',')
  if '__time' not in header:
    raise ValueError(f"{csv_path}: no __time column")
  want = ['__time'] + [s for s in signals if s != '__time']
  missing = [s for s in want if s not in header]
  if missing:
    raise ValueError(f"{csv_path}: missing columns:\n  " + "\n  ".join(missing))

  # cut streams the big file and keeps only the columns we need (1-based indices).
  # argv list (no shell) so an arbitrary csv_path can't inject shell metacharacters.
  idx = ','.join(str(header.index(s) + 1) for s in want)
  fd, tmp = tempfile.mkstemp(suffix='.csv')
  os.close(fd)
  try:
    with open(tmp, 'w') as out_f:
      subprocess.run(['cut', '-d,', f'-f{idx}', csv_path], stdout=out_f, check=True)
    df = pd.read_csv(tmp)
  finally:
    os.unlink(tmp)

  df = df.set_index('__time').sort_index()
  df = df[~df.index.duplicated(keep='last')]
  df = df.ffill()  # fill sparse cells with last known value

  # as-of resample onto a uniform grid (each column is now a step function)
  grid = np.arange(df.index[0], df.index[-1], dt)
  out = df.reindex(grid, method='ffill')
  out.index = out.index - out.index[0]
  out.index.name = 't'
  if rename:
    out = out.rename(columns=rename)
  return out
