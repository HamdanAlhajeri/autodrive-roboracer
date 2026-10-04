"""Planar poses and a small time-indexed transform buffer (no ROS).

A pose is an (x, y, yaw) tuple in metres and radians. compose(a, b) applies b in the
frame described by a, matching the parent->child convention of ROS transforms.
"""

from bisect import bisect_left
import math


def wrap_angle(angle):
    """Return an equivalent angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw from a quaternion, rejecting nonfinite or badly unnormalized values."""
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("nonfinite orientation")
    norm = math.sqrt(sum(v * v for v in values))
    if not 0.9 < norm < 1.1:
        raise ValueError("invalid orientation quaternion")
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def compose(a, b):
    """Return pose b expressed in a's parent frame (a then b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2]))


def inverse(a):
    """Return the inverse transform of pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap_angle(-a[2]))


def relative(a, b):
    """Return pose b expressed in the frame of pose a (inverse(a) then b)."""
    return compose(inverse(a), b)


def interpolate(a, b, fraction):
    """Linearly interpolate two poses, taking the shorter way around for yaw."""
    return (a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction,
            wrap_angle(a[2] + wrap_angle(b[2] - a[2]) * fraction))


class TransformBuffer:
    """Store parent->child transforms by source stamp and answer chained lookups.

    Static transforms apply at every time. Dynamic transforms are interpolated between
    samples and extrapolated by at most max_extrapolation_s, so a lookup can never silently
    use an arbitrarily old pose.
    """

    def __init__(self, max_extrapolation_s=0.1):
        self.max_extrapolation_s = max_extrapolation_s
        self.static = {}
        self.dynamic = {}

    def add(self, parent, child, stamp_s, pose, static=False):
        """Record one transform; out-of-order dynamic samples are inserted in time order."""
        if not all(math.isfinite(v) for v in (*pose, stamp_s)):
            raise ValueError("nonfinite transform")
        if static:
            self.static[child] = (parent, tuple(pose))
            return
        series = self.dynamic.setdefault(child, (parent, [], []))
        if series[0] != parent:
            raise ValueError(f"frame {child!r} has parents {series[0]!r} and {parent!r}")
        times, poses = series[1], series[2]
        index = bisect_left(times, stamp_s)
        if index < len(times) and times[index] == stamp_s:
            poses[index] = tuple(pose)
        else:
            times.insert(index, stamp_s)
            poses.insert(index, tuple(pose))

    def parent_of(self, child):
        """Return the parent frame of child, or None if it is a root or unknown."""
        if child in self.static:
            return self.static[child][0]
        if child in self.dynamic:
            return self.dynamic[child][0]
        return None

    def _edge(self, child, stamp_s):
        """Return the parent->child pose at stamp_s, or None if it is unavailable."""
        if child in self.static:
            return self.static[child][1]
        _, times, poses = self.dynamic[child]
        if not times:
            return None
        index = bisect_left(times, stamp_s)
        if index < len(times) and times[index] == stamp_s:
            return poses[index]
        if index == 0:
            return poses[0] if times[0] - stamp_s <= self.max_extrapolation_s else None
        if index == len(times):
            return poses[-1] if stamp_s - times[-1] <= self.max_extrapolation_s else None
        t0, t1 = times[index - 1], times[index]
        return interpolate(poses[index - 1], poses[index], (stamp_s - t0) / (t1 - t0))

    def lookup(self, target, source, stamp_s):
        """Return the pose of frame source in frame target at stamp_s, or None.

        Walk from source towards the root until target is reached. Return None when the
        chain is broken, a transform is too old, or target is not an ancestor of source.
        """
        pose = (0.0, 0.0, 0.0)
        frame = source
        visited = set()
        while frame != target:
            if frame in visited:
                return None
            visited.add(frame)
            parent = self.parent_of(frame)
            if parent is None:
                return None
            edge = self._edge(frame, stamp_s)
            if edge is None:
                return None
            pose = compose(edge, pose)
            frame = parent
        return pose
