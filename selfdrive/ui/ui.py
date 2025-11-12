#!/usr/bin/env python3
import os
import pyray as rl

from openpilot.system.hardware import TICI
from openpilot.common.realtime import config_realtime_process, set_core_affinity
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.ui_state import ui_state


def main():
  cores = {5, }
  config_realtime_process(0, 51)

  gui_app.init_window("UI")
  main_layout = MainLayout()
  scaled_width = gui_app.width * 0.75
  scaled_height = gui_app.height * 0.75
  y_offset = gui_app.height - scaled_height
  main_layout.set_rect(rl.Rectangle(0, y_offset, scaled_width, scaled_height))
  for should_render in gui_app.render():
    ui_state.update()
    if should_render:
      main_layout.render()

      # reaffine after power save offlines our core
      if TICI and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(list(cores))
        except OSError:
          pass


if __name__ == "__main__":
  main()
