"""Load and check the Jetson hardware profile (no ROS).

The profile holds measured vehicle geometry, sensor mounts, steering conversion, motor
limits and localization acceptance thresholds. A null value means "not measured yet".
Recording needs only interfaces and frames; autonomous motion needs every measurement.
Simulator values must never be copied into this file as hardware facts.
"""

import copy
import hashlib
import json
import math
from pathlib import Path

import yaml

# Settings that recording and replay need before anything has been commissioned.
RECORDING_REQUIRED = (
    "frames.map", "frames.odom", "frames.base_link", "frames.laser",
    "topics.scan", "topics.odom",
    "transform_owners.odom->base_link", "transform_owners.map->odom",
)

# Measurements that must exist before the supervisor may command motion.
AUTONOMY_REQUIRED = (
    "mounts.laser.x_m", "mounts.laser.y_m", "mounts.laser.yaw_rad",
    "vehicle.wheelbase_m", "vehicle.width_m", "vehicle.length_m",
    "vehicle.rear_axle_to_front_m",
    "steering.max_angle_rad", "steering.positive_left",
    "motor.max_command_speed_mps",
    "measurements.command_latency_s", "measurements.braking_deceleration_mps2",
    "measurements.max_acceleration_mps2", "measurements.stop_test_speed_mps",
    "measurements.stopping_distance_m",
    "limits.mapping_speed_mps", "limits.racing_speed_mps",
    "localization.max_position_stddev_m", "localization.max_yaw_stddev_rad",
    "localization.min_scan_alignment", "localization.max_motion_mismatch_m",
    "localization.max_motion_mismatch_rad",
)

POSITIVE = {
    "vehicle.wheelbase_m", "vehicle.width_m", "vehicle.length_m",
    "vehicle.rear_axle_to_front_m", "steering.max_angle_rad",
    "motor.max_command_speed_mps", "measurements.command_latency_s",
    "measurements.braking_deceleration_mps2", "measurements.max_acceleration_mps2",
    "measurements.stop_test_speed_mps", "measurements.stopping_distance_m",
    "limits.mapping_speed_mps", "limits.racing_speed_mps",
    "timing.command_timeout_s", "timing.authorization_timeout_s",
    "timing.sensor_timeout_s", "timing.pose_timeout_s",
    "localization.max_position_stddev_m", "localization.max_yaw_stddev_rad",
    "localization.scan_alignment_tolerance_m", "localization.max_motion_mismatch_m",
    "localization.max_motion_mismatch_rad", "localization.stable_duration_s",
}


def _get(data, dotted):
    """Read a dotted path such as 'mounts.laser.x_m', returning None when absent."""
    value = data
    for key in dotted.split("."):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _number(value):
    """Return whether value is a finite real number (booleans excluded)."""
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


class HardwareProfile:
    """Validated, read-only view of a hardware profile mapping."""

    def __init__(self, data, source=None):
        """Check structure, units and limit ordering; raise ValueError on the first problem.

        Unmeasured values (null) are allowed here. Use missing_for_autonomy() or
        require_commissioned() before enabling any motion.
        """
        if not isinstance(data, dict) or data.get("version") != 1:
            raise ValueError("Hardware profile must be a mapping with version: 1")
        self.data = copy.deepcopy(data)
        self.source = str(source) if source is not None else None
        missing = [key for key in RECORDING_REQUIRED if not _get(self.data, key)]
        if missing:
            raise ValueError("Hardware profile lacks recording settings: " + ", ".join(missing))
        for key in POSITIVE:
            value = _get(self.data, key)
            if value is not None and (not _number(value) or value <= 0):
                raise ValueError(f"{key} must be a positive finite number or null")
        for key in ("mounts.laser.x_m", "mounts.laser.y_m", "mounts.laser.yaw_rad",
                    "steering.center_offset_rad", "motor.stop_command_speed_mps"):
            value = _get(self.data, key)
            if value is not None and not _number(value):
                raise ValueError(f"{key} must be a finite number or null")
        if _get(self.data, "steering.positive_left") not in (None, True, False):
            raise ValueError("steering.positive_left must be true, false or null")
        if _get(self.data, "motor.stop_command_speed_mps") not in (None, 0, 0.0):
            raise ValueError("motor.stop_command_speed_mps must be 0 for the Ackermann output")
        alignment = _get(self.data, "localization.min_scan_alignment")
        if alignment is not None and not 0 < alignment <= 1:
            raise ValueError("localization.min_scan_alignment must be in (0, 1]")
        mapping = _get(self.data, "limits.mapping_speed_mps")
        racing = _get(self.data, "limits.racing_speed_mps")
        ceiling = _get(self.data, "motor.max_command_speed_mps")
        if mapping is not None and racing is not None and mapping > racing:
            raise ValueError("limits.mapping_speed_mps cannot exceed limits.racing_speed_mps")
        for name, value in (("mapping", mapping), ("racing", racing)):
            if value is not None and ceiling is not None and value > ceiling:
                raise ValueError(f"limits.{name}_speed_mps exceeds motor.max_command_speed_mps")
        test_speed = _get(self.data, "measurements.stop_test_speed_mps")
        if racing is not None and test_speed is not None and racing > test_speed:
            raise ValueError("limits.racing_speed_mps exceeds the measured stop-test speed; "
                             "measure stopping at the higher speed first")
        steer = _get(self.data, "steering.max_angle_rad")
        if steer is not None and steer >= math.pi / 2:
            raise ValueError("steering.max_angle_rad must be below pi/2")
        geometry = [self.get(f"vehicle.{key}") for key in
                    ("wheelbase_m", "rear_axle_to_front_m", "length_m")]
        if all(v is not None for v in geometry) and not geometry[0] <= geometry[1] <= geometry[2]:
            raise ValueError("vehicle geometry requires wheelbase <= rear_axle_to_front <= length")
        owners = self.data.get("transform_owners")
        if not isinstance(owners, dict) or not all(
                isinstance(k, str) and "->" in k and isinstance(v, str) and v
                for k, v in owners.items()):
            raise ValueError("transform_owners must map 'parent->child' to one publisher name")

    @classmethod
    def load(cls, filename):
        """Read a YAML profile from disk."""
        path = Path(filename)
        with path.open(encoding="utf-8") as stream:
            return cls(yaml.safe_load(stream), path)

    def get(self, dotted, default=None):
        """Return a profile value by dotted path."""
        value = _get(self.data, dotted)
        return default if value is None else value

    @property
    def sha256(self):
        """Hash the canonical profile so recordings and maps name the settings they used."""
        return hashlib.sha256(json.dumps(self.data, sort_keys=True).encode()).hexdigest()

    def frame(self, name):
        return self.data["frames"][name]

    def topic(self, name):
        """Return a configured topic name, or None when the interface is absent."""
        return (self.data.get("topics") or {}).get(name)

    def missing_for_autonomy(self):
        """List measurements that are still null; autonomy is refused until this is empty."""
        return [key for key in AUTONOMY_REQUIRED if _get(self.data, key) is None]

    def require_commissioned(self):
        """Raise ValueError naming every missing measurement."""
        missing = self.missing_for_autonomy()
        if missing:
            raise ValueError("Hardware profile is not commissioned; measure: " + ", ".join(missing))

    @property
    def laser_mount(self):
        """Return (x, y, yaw) of the LiDAR in base_link, or None when not measured."""
        mount = [self.get(f"mounts.laser.{key}") for key in ("x_m", "y_m", "yaw_rad")]
        return None if any(v is None for v in mount) else tuple(float(v) for v in mount)

    @property
    def vehicle_radius_m(self):
        """Radius enclosing body corners around the rear-to-front axle segment."""
        front = self.get("vehicle.rear_axle_to_front_m")
        overhang = max(front - self.get("vehicle.wheelbase_m"),
                       self.get("vehicle.length_m") - front)
        return math.hypot(self.get("vehicle.width_m") / 2, overhang)

    def stopping_distance(self, speed_mps):
        """Predict stopping distance from measured latency and deceleration, in metres.

        d = v * latency + v^2 / (2 * deceleration). Valid only up to the measured stop-test
        speed; commissioning must confirm the prediction against the measured distance.
        """
        latency = self.get("measurements.command_latency_s")
        braking = self.get("measurements.braking_deceleration_mps2")
        return speed_mps * latency + speed_mps ** 2 / (2 * braking)

    def summary(self):
        """Return a JSON-ready description for recording metadata and status reports."""
        return {"name": self.data.get("name"), "source": self.source, "sha256": self.sha256,
                "missing_for_autonomy": self.missing_for_autonomy()}
