"""Allowlisted numeric diagnostics; incoming telemetry cannot overwrite vehicle state."""

import json
import math


CONTROLLER_FIELDS = frozenset({
    "target_velocity_mps", "lookahead_m", "clearance_m", "curvature_1pm",
    "curvature_speed_limit_mps", "clearance_speed_limit_mps",
    "steering_saturated", "acceleration_saturated", "controller_loop_dt_s",
    "controller_step_time_s", "controller_overrun", "controller_lidar_age_s",
    "controller_odom_age_s",
    "gap_preview_requested_m", "gap_preview_selected_m", "gap_preview_fallback",
    "target_bearing_raw_rad", "target_bearing_rad",
    "planned_mode", "plan_valid", "planned_speed_mps", "path_progress_m",
    "path_deviation_m", "obstacle_speed_limit_mps", "speed_limit_reason",
})
ACTUATOR_FIELDS = frozenset({
    "actuator_target_speed_mps", "throttle_saturated", "actuator_steering_saturated",
    "braking_requested", "actuator_command_age_s", "actuator_odom_age_s",
    "actuator_lidar_age_s", "actuator_loop_dt_s",
})


def diagnostic_values(text, allowed):
    """Return a complete snapshot, clearing absent/nonfinite fields to null."""
    try:
        values = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(values, dict):
        return None
    return {
        key: value if isinstance(value, (int, float)) and math.isfinite(value) else None
        for key in allowed for value in [values.get(key)]
    }
