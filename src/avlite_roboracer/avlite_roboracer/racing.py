"""Bind the AVLite racing profile to measured hardware limits (no ROS, no AVLite import).

The racing YAML supplies planner and Pure Pursuit tuning. Geometry and limits come from
the hardware profile: they are filled in or checked here so a simulator value can never
become a hardware default.
"""

import copy


def hardware_race_config(config, profile, map_path):
    """Return a resolved AVLite config for the finalized hardware map.

    Wheelbase, vehicle radius, the map frame and steering limits are taken from the
    profile. Speed, acceleration and braking may be lower than measured, never higher.
    Raise ValueError naming any violation.
    """
    profile.require_commissioned()
    config = copy.deepcopy(config)
    planning = config.setdefault("planning", {})
    control = config["c30_control"]
    wheelbase = profile.get("vehicle.wheelbase_m")
    steer = profile.get("steering.max_angle_rad")
    ceiling = profile.get("limits.racing_speed_mps")
    planning.update(map_path=str(map_path), frame_id=profile.frame("map"),
                    wheelbase_m=wheelbase, vehicle_radius_m=profile.vehicle_radius_m)
    control["c32_ego_distance_front_axle"] = wheelbase
    for key in ("c32_ego_max_steering", "c32_ego_min_steering"):
        if abs(control[key]) > steer:
            raise ValueError(f"c30_control.{key} exceeds measured steering.max_angle_rad")
    speed = planning.get("max_velocity_mps", control["c32_ego_max_velocity"])
    if speed > ceiling:
        raise ValueError(f"Racing speed {speed} exceeds limits.racing_speed_mps {ceiling}")
    planning["max_velocity_mps"] = control["c32_ego_max_velocity"] = speed
    control["c35_cruise_velocity"] = min(control.get("c35_cruise_velocity", speed), speed)
    checks = (
        ("acceleration_mps2", "measurements.max_acceleration_mps2"),
        ("braking_deceleration_mps2", "measurements.braking_deceleration_mps2"),
    )
    for key, measured in checks:
        if planning.get(key) is None:
            raise ValueError(f"planning.{key} is required")
        if planning[key] > profile.get(measured):
            raise ValueError(f"planning.{key} exceeds measured {measured}")
    if control["c32_ego_max_acceleration"] > profile.get("measurements.max_acceleration_mps2"):
        raise ValueError("c32_ego_max_acceleration exceeds the measured acceleration")
    if -control["c32_ego_min_acceleration"] > profile.get("measurements.braking_deceleration_mps2"):
        raise ValueError("c32_ego_min_acceleration exceeds the measured braking deceleration")
    reaction = planning.get("reaction_time_s")
    if reaction is None or reaction < profile.get("measurements.command_latency_s"):
        raise ValueError("planning.reaction_time_s is below the measured command latency")
    return config


def mapping_control_config(config, profile):
    """Control settings for the slow Follow the Gap mapping lap."""
    profile.require_commissioned()
    control = copy.deepcopy(config["c30_control"])
    speed = profile.get("limits.mapping_speed_mps")
    control.update(c32_ego_distance_front_axle=profile.get("vehicle.wheelbase_m"),
                   c32_ego_max_velocity=speed, c35_cruise_velocity=speed)
    steer = profile.get("steering.max_angle_rad")
    control["c32_ego_max_steering"] = min(control["c32_ego_max_steering"], steer)
    control["c32_ego_min_steering"] = max(control["c32_ego_min_steering"], -steer)
    control["c32_ego_min_acceleration"] = max(
        control["c32_ego_min_acceleration"], -profile.get("measurements.braking_deceleration_mps2"))
    control["c32_ego_max_acceleration"] = min(
        control["c32_ego_max_acceleration"], profile.get("measurements.max_acceleration_mps2"))
    return control


def mapping_racing_config(config, profile):
    """Bound FTG's stopping model by measured braking and end-to-end command delay."""
    control = mapping_control_config(config, profile)
    racing = copy.deepcopy(config.get("racing", {}))
    braking = -control["c32_ego_min_acceleration"]
    racing["braking_deceleration_mps2"] = min(
        racing.get("braking_deceleration_mps2", braking), braking)
    latency = profile.get("measurements.command_latency_s")
    racing["reaction_time_s"] = max(racing.get("reaction_time_s", latency), latency)
    return racing
