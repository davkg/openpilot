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

  # Upcoming-turn anticipation (same 97th-pct predicted-lat-accel signal SCC-V uses for its ENTERING cue).
  # Engages blended early so the e2e model's curve easing starts before the reactive e2e-decel trigger.
  TURN_LAT_ACC_ENGAGE = 1.5   # m/s^2 predicted lat accel ahead -> engage blended for curve easing
  TURN_LAT_ACC_RELEASE = 1.2  # m/s^2 release once the predicted curve clears (hysteresis)
  TURN_LOOKAHEAD_T = 5.0      # s -- only scan the model path within this horizon (full path spans ~10s)

  # e2e-decel trigger
  E2E_DECEL_ENGAGE = -0.3   # m/s^2 -- e2e desiredAcceleration at/below this -> blended
  E2E_DECEL_RELEASE = -0.1  # m/s^2 -- release once e2e rises above this (e2e basically done decelerating)
  # Above this speed, e2e-decel anticipation engages blended even behind a close lead
  E2E_DECEL_OVERRIDE_MIN_SPEED = 40.0  # km/h (24.9 mph, 11.1 m/s)
  E2E_DECEL_OVERRIDE_MARGIN = 0.1      # m/s^2
