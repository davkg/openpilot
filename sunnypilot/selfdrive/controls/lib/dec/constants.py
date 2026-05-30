class WMACConstants:
  # Lead detection parameters
  LEAD_WINDOW_SIZE = 6  # Stable detection window
  LEAD_PROB = 0.45  # Balanced threshold for lead detection

  # Slow down detection parameters
  SLOW_DOWN_WINDOW_SIZE = 5  # Responsive but stable
  SLOW_DOWN_PROB = 0.3  # Balanced threshold for slow down scenarios

  # Optimized slow down distance curve - smooth and progressive
  SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55., 60.]
  SLOW_DOWN_DIST = [32., 46., 64., 86., 108., 130., 145., 165.]

  # Slowness detection parameters
  SLOWNESS_WINDOW_SIZE = 10  # Stable slowness detection
  SLOWNESS_PROB = 0.55  # Clear threshold for slowness
  SLOWNESS_CRUISE_OFFSET = 1.025  # Conservative cruise speed offset

  # Close-lead gate: leads within this distance are handled by ACC (steady following / stop-n-go)
  LEAD_CLOSE_BP = [0., 30., 60.]      # km/h
  LEAD_CLOSE_DIST = [70., 70., 90.]  # m
