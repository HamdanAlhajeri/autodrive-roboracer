"""Convert AutoDRIVE LaserScan to AVLite's canonical sensor-frame cloud."""

import numpy as np


def scan_hit_mask(msg):
    """Mark beams that should be included in obstacle checks.

    Positive infinity and readings at maximum range mean no measured wall. Corrupt beams
    remain included because scan_cloud() places them close to the car.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    return ~(np.isposinf(ranges) | (np.isfinite(ranges) & (ranges >= msg.range_max)))


def scan_cloud(msg):
    """Convert LaserScan ranges and angles into AVLite's N-by-4 sensor-frame point array.

    The first two columns are x/y in metres; the remaining columns stay zero. Clear beams
    are placed at maximum range, while corrupt readings become nearby points. Use
    scan_hit_mask() to distinguish these cases during planned obstacle checks.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    # Positive infinity means no return out to range_max. Unknown/corrupt beams
    # become close obstacles, never holes through which gap selection can drive.
    ranges = np.where(np.isposinf(ranges), msg.range_max, ranges)
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    ranges = np.where(valid, ranges, max(msg.range_min, 0.01))
    cloud = np.zeros((len(ranges), 4), dtype=np.float32)
    cloud[:, 0] = ranges * np.cos(angles)
    cloud[:, 1] = ranges * np.sin(angles)
    return cloud
