"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params


class ModelStateBase:
  # set by modeld each frame; jetlink's joining model reads it to hand a large model back
  frame_drop_ratio: float = 0.

  def __init__(self):
    self.lat_delay = Params().get("LagdValueCache", return_default=True)
