"""Readiness gates for autonomous motion (no ROS). Each returns failure reasons."""


def _fresh(age, timeout):
    return age is not None and 0 <= age <= timeout


def _joy_detail(status):
    """Say which of the two it was, when the actuator reported the joystick details."""
    if "joy_age_s" not in status:
        return ""
    age, pressed = status.get("joy_age_s"), status.get("joy_button_pressed")
    last_release = status.get("since_last_release_s")
    parts = ["no /joy message seen yet" if age is None else f"last /joy message {age:.2f} s ago"]
    if pressed is not None:
        parts.append("button " + ("pressed" if pressed else "NOT pressed"))
    parts.append(f"{status.get('joy_messages', 0)} messages, "
                 f"{status.get('joy_releases', 0)} with the button not pressed")
    if last_release is not None:
        parts.append(f"last release {last_release:.2f} s ago")
    held = status.get("deadman_held_for_s")
    if held is not None:
        parts.append(f"held for {held:.2f} s")
    return " (" + "; ".join(parts) + ")"


def actuator_failures(actuator, now, timeout_s, require_deadman=False):
    """Check the actuator process's latest status snapshot.

    actuator is (receive_time, status_dict) or None. The actuator must be alive, report no
    fault and have no manual override engaged. While the car is meant to move
    (require_deadman), the operator must also be holding the dead-man button.
    """
    if actuator is None or not _fresh(now - actuator[0], timeout_s):
        return ["actuator process unavailable"]
    status = actuator[1]
    failures = []
    if status.get("fault"):
        failures.append(f"actuator fault: {status['fault']}")
    if status.get("manual_override"):
        failures.append("manual override engaged")
    if require_deadman and not status.get("deadman_held"):
        if status.get("stop_latched"):
            failures.append("stop button pressed")
        else:
            failures.append("dead-man button released or joystick silent" + _joy_detail(status))
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
        failures.append(f"expected exactly one localization source (AVLite ICP or SLAM Toolbox), "
                        f"found {localization_processes}")
    failures += list(localization_failures)
    if not plan_valid:
        failures.append("no validated racing plan")
    return failures
