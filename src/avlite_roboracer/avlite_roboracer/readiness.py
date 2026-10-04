"""Readiness gates for autonomous motion (no ROS). Each returns failure reasons."""


def _fresh(age, timeout):
    return age is not None and 0 <= age <= timeout


def actuator_failures(actuator, now, timeout_s):
    """Check the actuator process's latest status snapshot.

    actuator is (receive_time, status_dict) or None. The actuator must be alive, report no
    fault and have no manual override engaged.
    """
    if actuator is None or not _fresh(now - actuator[0], timeout_s):
        return ["actuator process unavailable"]
    status = actuator[1]
    failures = []
    if status.get("fault"):
        failures.append(f"actuator fault: {status['fault']}")
    if status.get("manual_override"):
        failures.append("manual override engaged")
    return failures


def sensor_failures(scan_age, odom_age, timeout_s):
    failures = []
    if not _fresh(scan_age, timeout_s):
        failures.append("LiDAR scans stale or missing")
    if not _fresh(odom_age, timeout_s):
        failures.append("odometry stale or missing")
    return failures


def mapping_failures(profile, scan_age, odom_age, actuator, slam_running, now):
    """Preconditions for map-start: commissioned profile, fresh sensors, live actuator."""
    failures = []
    missing = profile.missing_for_autonomy()
    if missing:
        failures.append("hardware profile not commissioned: " + ", ".join(missing))
    failures += sensor_failures(scan_age, odom_age, profile.get("timing.sensor_timeout_s"))
    failures += actuator_failures(actuator, now, 2 * profile.get("timing.command_timeout_s"))
    if slam_running:
        failures.append("a SLAM process is already running; stop it first")
    return failures


def race_failures(profile, scan_age, odom_age, actuator, localization_failures,
                  plan_valid, localization_processes, now):
    """Preconditions for race-start: everything above plus stable localization and a plan."""
    failures = []
    missing = profile.missing_for_autonomy()
    if missing:
        failures.append("hardware profile not commissioned: " + ", ".join(missing))
    failures += sensor_failures(scan_age, odom_age, profile.get("timing.sensor_timeout_s"))
    failures += actuator_failures(actuator, now, 2 * profile.get("timing.command_timeout_s"))
    if localization_processes != 1:
        failures.append(f"expected exactly one localization process publishing map->odom, "
                        f"found {localization_processes}")
    failures += list(localization_failures)
    if not plan_valid:
        failures.append("no validated racing plan")
    return failures
