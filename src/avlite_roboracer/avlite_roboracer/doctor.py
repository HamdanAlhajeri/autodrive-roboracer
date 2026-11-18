"""Hardware profile doctor: what is still unmeasured, and how to measure each item (no ROS).

Run on the laptop or the Jetson:
    python -m avlite_roboracer.doctor [--profile config/roboracer/hardware.yaml] [--json]

Exit status 0 means every autonomy measurement is present; 1 means something is missing.
It reads the profile only. It sends nothing and starts nothing.
"""

import argparse
import json
import sys
from pathlib import Path

from .profile import AUTONOMY_REQUIRED, HardwareProfile

DEFAULT_PROFILE = Path(__file__).resolve().parents[3] / "config" / "roboracer" / "hardware.yaml"

# (step, how to get it) per measurement. Steps follow docs/jetson-quickstart.md.
HOW = {
    "mounts.laser.x_m": (3, "Metres forward from base_link (rear-axle centre) to the LiDAR centre. "
                            "Must equal the static transform in the F1TENTH bringup "
                            "(`ros2 run tf2_ros tf2_echo base_link laser`)."),
    "mounts.laser.y_m": (3, "Metres left of base_link to the LiDAR centre (0 if centred)."),
    "mounts.laser.yaw_rad": (3, "LiDAR rotation about the vertical axis; 0 when it faces forward."),
    "vehicle.wheelbase_m": (3, "Front axle to rear axle centre-to-centre, metres."),
    "vehicle.width_m": (3, "Widest point of the car including wheels, metres."),
    "vehicle.length_m": (3, "Bumper to bumper, metres."),
    "vehicle.rear_axle_to_front_m": (3, "Rear axle to the front bumper, metres."),
    "steering.max_angle_rad": (3, "Largest steering angle the servo reaches without binding, "
                                  "radians (wheels off the ground, command it up slowly)."),
    "steering.positive_left": (3, "true if a positive steering_angle turns the car left. "
                                  "Test with the wheels off the ground."),
    "motor.max_command_speed_mps": (3, "Hard clamp on speed sent to the VESC. Start at 1.0."),
    "measurements.command_latency_s": (3, "From the commission step: command published to wheels "
                                          "responding."),
    "measurements.braking_deceleration_mps2": (3, "From the commission step, on the real floor."),
    "measurements.max_acceleration_mps2": (3, "From the commission step."),
    "measurements.stop_test_speed_mps": (3, "Highest speed at which the stop test was run."),
    "measurements.stopping_distance_m": (3, "Measured distance to stop from that speed."),
    "limits.mapping_speed_mps": (3, "Slow mapping lap, about 0.5-1.0 m/s to begin with."),
    "limits.racing_speed_mps": (3, "At most the stop-test speed. Start at 1.0."),
    "localization.max_position_stddev_m": (4, "From the localize step's report (sensor and pose "
                                              "spread on a second lap)."),
    "localization.max_yaw_stddev_rad": (4, "Same report."),
    "localization.min_scan_alignment": (4, "Same report; 0.7 is a sensible start (share of scan "
                                           "hits within 10 cm of the map)."),
    "localization.max_motion_mismatch_m": (4, "Same report: map-frame motion against odometry "
                                              "per pose update."),
    "localization.max_motion_mismatch_rad": (4, "Same report."),
    "deadman.button": (3, "Run `ros2 topic echo /joy`, hold the button you want, read its "
                          "buttons[] index."),
}


def inspect(path):
    """Return a JSON-ready report: loaded, error, present and missing measurements."""
    report = {"profile": str(path), "loaded": False, "error": None, "present": {}, "missing": []}
    try:
        profile = HardwareProfile.load(path)
    except (OSError, ValueError) as exc:
        report["error"] = str(exc)
        return report
    report["loaded"] = True
    for key in AUTONOMY_REQUIRED:
        value = profile.get(key) if key != "steering.positive_left" else (
            profile.data.get("steering", {}).get("positive_left"))
        if value is None:
            step, how = HOW.get(key, (3, ""))
            report["missing"].append({"key": key, "step": step, "how": how})
        else:
            report["present"][key] = value
    report["commissioned"] = not report["missing"]
    return report


def render(report):
    lines = [f"Hardware profile: {report['profile']}"]
    if not report["loaded"]:
        return "\n".join(lines + [f"CANNOT LOAD: {report['error']}"])
    total = len(AUTONOMY_REQUIRED)
    lines.append(f"{len(report['present'])} of {total} autonomy measurements present.")
    if report["commissioned"]:
        lines.append("Commissioned: autonomous motion is allowed by the profile.")
        lines.append("Still confirm with stationary tests (quickstart step 6) before moving.")
        return "\n".join(lines)
    lines.append("NOT commissioned: the supervisor refuses autonomous motion until these are set.")
    for step in sorted({m["step"] for m in report["missing"]}):
        lines.append(f"\nQuickstart step {step}:")
        for item in report["missing"]:
            if item["step"] == step:
                lines.append(f"  - {item['key']}: {item['how']}")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    report = inspect(Path(args.profile))
    print(json.dumps(report, indent=2) if args.json else render(report))
    return 0 if report.get("commissioned") else 1


if __name__ == "__main__":
    sys.exit(main())
