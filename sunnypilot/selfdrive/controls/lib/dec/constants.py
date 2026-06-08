class WMACConstants:
  # Lead detection parameters
  LEAD_WINDOW_SIZE = 6  # Stable detection window
  LEAD_PROB = 0.45  # Balanced threshold for lead detection

  # Slow down detection parameters
  SLOW_DOWN_WINDOW_SIZE = 5  # Responsive but stable
  SLOW_DOWN_PROB = 0.3  # Balanced threshold for slow down scenarios
  SLOW_DOWN_RELEASE_RATIO = 0.6  # hysteresis: release has_slow_down at this fraction of the engage threshold

  # Optimized slow down distance curve - smooth and progressive
  SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55., 60.]
  SLOW_DOWN_DIST = [32., 46., 64., 86., 108., 130., 145., 165.]

  # Slowness detection parameters
  SLOWNESS_WINDOW_SIZE = 10  # Stable slowness detection
  SLOWNESS_PROB = 0.55  # Clear threshold for slowness
  SLOWNESS_CRUISE_OFFSET = 1.025  # Conservative cruise speed offset

  # Close-lead gate: leads within this distance are handled by ACC (steady following / stop-n-go)
  LEAD_CLOSE_BP = [0., 30., 60.]     # km/h
  LEAD_CLOSE_DIST = [70., 70., 90.]  # m

  # e2e-decel trigger: engage blended when the e2e model's desiredAcceleration shows it wants to brake
  # earlier than the trajectory-endpoint shortfall sees (distant-lead / red-light anticipation).
  E2E_DECEL_ENGAGE = -0.3   # m/s^2 -- e2e desiredAcceleration at/below this -> blended
  E2E_DECEL_RELEASE = -0.1  # m/s^2 -- release once e2e rises above this (e2e basically done decelerating)
  # Above this speed, e2e-decel anticipation engages blended even behind a close lead
  E2E_DECEL_OVERRIDE_MIN_SPEED = 40.0  # km/h (24.9 mph, 11.1 m/s)
  E2E_DECEL_OVERRIDE_MARGIN = 0.1      # m/s^2
