"""Partial world-frame outline from actual LiDAR returns and simulator odometry.

This is a visualization aid, not SLAM or an occupancy map. Header stamps are
assigned by the bridge on reception; motion within a scan is not compensated.
"""

from collections import Counter, deque
import math

from avlite_autodrive.ros_utils import odom_state


class TrackMap:
    """Accumulate bounded, deduplicated hits without importing ROS packages."""

    RESOLUTION_M = 0.03
    SYNC_SKEW_NS = 20_000_000
    FRESHNESS_S = 0.1
    SCAN_INTERVAL_S = 0.1
    LIDAR_OFFSET_M = (0.2733, 0.0, 0.096)

    def __init__(self, *, max_cells=100_000, buffer_size=32):
        if max_cells < 1 or buffer_size < 1:
            raise ValueError("map and synchronization buffer limits must be positive")
        self.max_cells = max_cells
        self.buffer_size = buffer_size
        self._cells = {}
        self._odometry = deque()
        self._scans = deque()
        self._last_pose = None
        self._latest_received_s = -math.inf
        self._last_used_received_s = -math.inf
        self._last_used_stamp_ns = None
        self._last_odom_stamp_ns = None
        self._last_scan_stamp_ns = None
        self._stamp_floor_ns = 0
        self._highest_stamp_ns = 0
        self._scans_received = 0
        self._scans_used = 0
        self._odometry_received = 0
        self._reset_count = 0
        self._hits_received = 0
        self._truncated = False
        self._rejections = Counter()
        self._max_used_skew_s = 0.0

    @staticmethod
    def _stamp(msg):
        try:
            stamp = msg.header.stamp
            if not (isinstance(stamp.sec, int) and isinstance(stamp.nanosec, int)):
                return None
            if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                return None
            value = stamp.sec * 1_000_000_000 + stamp.nanosec
            return value if value > 0 else None
        except AttributeError:
            return None

    def _advance_time(self, received_s, kind):
        if not math.isfinite(received_s):
            self._rejections[kind + "_invalid_receive_time"] += 1
            return False
        self._latest_received_s = max(self._latest_received_s, received_s)
        cutoff = self._latest_received_s - self.FRESHNESS_S
        while self._scans and self._scans[0][1] < cutoff:
            self._scans.popleft()
            self._rejections["scan_unsynchronized"] += 1
        while self._odometry and self._odometry[0][1] < cutoff:
            self._odometry.popleft()
        if received_s < cutoff:
            self._rejections[kind + "_stale"] += 1
            return False
        return True

    def _valid_stamp(self, msg, kind):
        stamp = self._stamp(msg)
        if stamp is None:
            self._rejections[kind + "_invalid_stamp"] += 1
            return None
        previous = (self._last_odom_stamp_ns if kind == "odom"
                    else self._last_scan_stamp_ns)
        if stamp <= self._stamp_floor_ns or (previous is not None and stamp <= previous):
            self._rejections[kind + "_nonmonotonic_stamp"] += 1
            return None
        return stamp

    def add_odometry(self, msg, received_s):
        """Offer world -> rear-axle odometry with local monotonic receive time."""
        self._odometry_received += 1
        if not self._advance_time(received_s, "odom"):
            return
        if (getattr(getattr(msg, "header", None), "frame_id", None) != "world"
                or getattr(msg, "child_frame_id", None) != "roboracer_1"):
            self._rejections["odom_frame_mismatch"] += 1
            return
        stamp = self._valid_stamp(msg, "odom")
        if stamp is None:
            return
        try:
            x, y, yaw, _ = odom_state(msg)
        except (AttributeError, TypeError, ValueError):
            self._rejections["odom_invalid_pose"] += 1
            return
        if self._last_pose and math.hypot(x - self._last_pose[0], y - self._last_pose[1]) > 1:
            # Invalidate pre-jump pairing, including scans that arrived ahead
            # of this odometry. Existing hits are already in the world frame.
            self._clear_sync(stamp - 1)
        self._last_pose = (x, y)
        self._last_odom_stamp_ns = stamp
        self._highest_stamp_ns = max(self._highest_stamp_ns, stamp)
        self._odometry.append((stamp, received_s, (x, y, yaw)))
        while len(self._odometry) > self.buffer_size:
            self._odometry.popleft()
        self._match()

    def add_scan(self, msg, received_s):
        """Offer a LiDAR scan; no-return values never become plotted walls."""
        self._scans_received += 1
        if not self._advance_time(received_s, "scan"):
            return
        if getattr(getattr(msg, "header", None), "frame_id", None) != "lidar":
            self._rejections["scan_frame_mismatch"] += 1
            return
        stamp = self._valid_stamp(msg, "scan")
        if stamp is None:
            return
        try:
            params = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
            if (not all(math.isfinite(v) for v in params)
                    or msg.angle_increment <= 0 or not 0 <= msg.range_min < msg.range_max):
                raise ValueError("invalid scan geometry")
            hits = [(msg.angle_min + i * msg.angle_increment, float(distance))
                    for i, distance in enumerate(msg.ranges)
                    if math.isfinite(distance) and msg.range_min <= distance < msg.range_max]
            if not all(math.isfinite(angle) for angle, _ in hits):
                raise ValueError("invalid ray angles")
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._rejections["scan_invalid_geometry"] += 1
            return
        self._last_scan_stamp_ns = stamp
        self._highest_stamp_ns = max(self._highest_stamp_ns, stamp)
        self._scans.append((stamp, received_s, hits))
        while len(self._scans) > self.buffer_size:
            self._scans.popleft()
            self._rejections["scan_buffer_full"] += 1
        self._match()

    def _match(self):
        while self._scans and self._odometry:
            stamp, received_s, hits = self._scans[0]
            # Wait for an odometry timestamp to reach the scan before choosing
            # its nearest pose. Otherwise a scan preceding jump odometry could
            # be projected using the previous, pre-reset pose.
            if self._odometry[-1][0] < stamp:
                return
            self._scans.popleft()
            odom_stamp, odom_received_s, pose = min(
                self._odometry, key=lambda item: abs(item[0] - stamp)
            )
            skew_ns = abs(odom_stamp - stamp)
            if (skew_ns > self.SYNC_SKEW_NS
                    or abs(odom_received_s - received_s) > self.FRESHNESS_S
                    or self._latest_received_s - received_s > self.FRESHNESS_S
                    or self._latest_received_s - odom_received_s > self.FRESHNESS_S):
                self._rejections["scan_unsynchronized"] += 1
                continue
            if (self._latest_received_s - self._last_used_received_s
                    < self.SCAN_INTERVAL_S - 1e-9
                    or (self._last_used_stamp_ns is not None
                        and stamp - self._last_used_stamp_ns < 100_000_000)):
                self._rejections["scan_rate_limited"] += 1
                continue
            self._last_used_received_s = self._latest_received_s
            self._last_used_stamp_ns = stamp
            self._scans_used += 1
            self._max_used_skew_s = max(self._max_used_skew_s, skew_ns / 1e9)
            x, y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            for angle, distance in hits:
                local_x = self.LIDAR_OFFSET_M[0] + distance * math.cos(angle)
                local_y = self.LIDAR_OFFSET_M[1] + distance * math.sin(angle)
                world_x = x + c * local_x - s * local_y
                world_y = y + s * local_x + c * local_y
                if not (math.isfinite(world_x) and math.isfinite(world_y)):
                    continue
                self._hits_received += 1
                cell = (math.floor(world_x / self.RESOLUTION_M),
                        math.floor(world_y / self.RESOLUTION_M))
                if cell not in self._cells:
                    if len(self._cells) < self.max_cells:
                        self._cells[cell] = [world_x, world_y]
                    else:
                        self._truncated = True

    def _clear_sync(self, stamp_floor_ns):
        self._rejections["scan_reset_discarded"] += len(self._scans)
        self._scans.clear()
        self._odometry.clear()
        self._last_pose = None
        self._last_odom_stamp_ns = None
        self._last_scan_stamp_ns = None
        self._stamp_floor_ns = max(self._stamp_floor_ns, stamp_floor_ns)
        self._reset_count += 1

    def reset(self):
        """Clear pairing history after a reset; preserve observed world hits."""
        self._clear_sync(self._highest_stamp_ns)

    def snapshot(self):
        """Return JSON-safe points and enough provenance to label the outline."""
        return {
            "version": 1,
            "frame_id": "world",
            "units": "m",
            "source": "lidar_hits_with_simulator_ground_truth",
            "resolution_m": self.RESOLUTION_M,
            "points": [list(point) for point in self._cells.values()],
            "scans_received": self._scans_received,
            "scans_used": self._scans_used,
            "scans_pending": len(self._scans),
            "odometry_received": self._odometry_received,
            "hit_count": self._hits_received,
            "point_count": len(self._cells),
            "max_points": self.max_cells,
            "truncated": self._truncated,
            "reset_count": self._reset_count,
            "rejections": dict(self._rejections),
            "synchronization": {
                "max_source_skew_s": self.SYNC_SKEW_NS / 1e9,
                "max_receive_age_s": self.FRESHNESS_S,
                "max_used_source_skew_s": self._max_used_skew_s,
                "buffer_size": self.buffer_size,
                "stamp_origin": "bridge_receive_time",
            },
            "max_scan_rate_hz": 1 / self.SCAN_INTERVAL_S,
            "lidar_mount_translation_m": list(self.LIDAR_OFFSET_M),
            "lidar_mount_yaw_rad": 0.0,
            "limitations": [
                "Partial outline of observed surfaces; may include obstacles.",
                "Uses simulator ground-truth odometry, not independent localization.",
                "Approximate receive-time alignment; no motion compensation within a scan.",
            ],
        }
