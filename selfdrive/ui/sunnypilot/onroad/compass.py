import math
import pyray as rl

from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


BG_COLOR = rl.Color(0, 0, 0, 166)
WHITE = rl.Color(255, 255, 255, 255)
INDICATOR_COLOR = rl.Color(220, 0, 0, 150)
ARROW_BASE_HALF_WIDTH = 14
ARROW_TIP_RADIUS_FRAC = 1.0
ARROW_BASE_RADIUS_FRAC = 0.60

CENTER_FONT_SIZE = 76
INVALID_ALPHA = 0.5
COMPASS_GAP_ABOVE_EXP_BUTTON = 20


def _bearing_to_cardinal(deg: float) -> str:
  if (337.5 <= deg <= 360) or (0 <= deg <= 22.5):
    return "N"
  elif 22.5 < deg < 67.5:
    return "NE"
  elif 67.5 <= deg <= 112.5:
    return "E"
  elif 112.5 < deg < 157.5:
    return "SE"
  elif 157.5 <= deg <= 202.5:
    return "S"
  elif 202.5 < deg < 247.5:
    return "SW"
  elif 247.5 <= deg <= 292.5:
    return "W"
  else:
    return "NW"


def _rotate(x: float, y: float, angle_deg: float, cx: float, cy: float) -> tuple[float, float]:
  a = math.radians(angle_deg)
  s, c = math.sin(a), math.cos(a)
  dx, dy = x - cx, y - cy
  return (cx + dx * c - dy * s, cy + dx * s + dy * c)


def _with_alpha(color: rl.Color, factor: float) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, int(color.a * factor))


class CompassRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._last_valid_bearing: float | None = None
    self._is_valid: bool = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)

  def _get_gps_data(self):
    sm = ui_state.sm
    if sm.valid['gpsLocationExternal']:
      return sm['gpsLocationExternal']
    elif sm.valid['gpsLocation']:
      return sm['gpsLocation']
    return None

  def _update_state(self) -> None:
    gps = self._get_gps_data()
    if gps is not None and gps.bearingAccuracyDeg != 180.0:
      self._last_valid_bearing = gps.bearingDeg
      self._is_valid = True
    else:
      self._is_valid = False

  def _render(self, rect: rl.Rectangle) -> None:
    # Anchored above the exp button (bottom-left), same x and size.
    button_size = UI_CONFIG.button_size
    border_size = UI_CONFIG.border_size
    radius = button_size / 2
    cx = rect.x + border_size * 2 + radius
    cy = rect.y + rect.height - border_size * 2 - button_size - COMPASS_GAP_ABOVE_EXP_BUTTON - radius

    alpha = 1.0 if self._is_valid else INVALID_ALPHA

    rl.draw_circle(int(cx), int(cy), radius, _with_alpha(BG_COLOR, alpha))

    if self._last_valid_bearing is not None:
      self._draw_indicator(cx, cy, radius, self._last_valid_bearing, alpha)
      self._draw_center_text(cx, cy, _bearing_to_cardinal(self._last_valid_bearing), alpha)

  def _draw_indicator(self, cx: float, cy: float, radius: float, bearing_deg: float, alpha: float) -> None:
    # Needle points to North. Car heading is bearing_deg CW from North,
    # so on screen (which is car-fixed) North sits at -bearing_deg.
    angle = -bearing_deg
    tip_dist = radius * ARROW_TIP_RADIUS_FRAC
    base_dist = radius * ARROW_BASE_RADIUS_FRAC

    tip = _rotate(cx, cy - tip_dist, angle, cx, cy)
    base_left = _rotate(cx - ARROW_BASE_HALF_WIDTH, cy - base_dist, angle, cx, cy)
    base_right = _rotate(cx + ARROW_BASE_HALF_WIDTH, cy - base_dist, angle, cx, cy)

    color = _with_alpha(INDICATOR_COLOR, alpha)
    rl.draw_triangle(rl.Vector2(*tip), rl.Vector2(*base_left), rl.Vector2(*base_right), color)

  def _draw_center_text(self, cx: float, cy: float, text: str, alpha: float) -> None:
    size = measure_text_cached(self._font_bold, text, CENTER_FONT_SIZE)
    pos = rl.Vector2(cx - size.x / 2, cy - size.y / 2)
    rl.draw_text_ex(self._font_bold, text, pos, CENTER_FONT_SIZE, 0, _with_alpha(WHITE, alpha))
