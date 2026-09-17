"""Convert AutoDRIVE LaserScan to AVLite's canonical sensor-frame cloud."""

import numpy as np


def scan_hit_mask(msg):
    """Exclude clear beams, retaining corrupt beams as the close obstacles in scan_cloud."""
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    return ~(np.isposinf(ranges) | (np.isfinite(ranges) & (ranges >= msg.range_max)))


def scan_cloud(msg):
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
