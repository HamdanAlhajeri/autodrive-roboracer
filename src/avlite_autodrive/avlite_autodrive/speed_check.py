"""Offline speed feasibility for the actual configured track; never starts ROS."""

import argparse
import math

from .actuation import Limits
from .configuration import load_config
from .race_planning import prepare_plan


def main():
    """Validate the configured map and print the limits behind its planned speed range.

    Check that planner and actuator share a ceiling, compare throttle feedforward with its
    cap, and estimate acceleration/stopping distances. The target argument is only a
    comparison value; this offline command sends no driving commands and changes no
    settings.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="/config/avlite.yaml")
    parser.add_argument("--actuator-config", default="/config/actuator.yaml")
    parser.add_argument("--target-mps", type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.target_mps) or args.target_mps <= 0:
        parser.error("--target-mps must be finite and positive")
    config = load_config(args.config)
    if config.get("driving_mode", "follow_the_gap") != "planned":
        parser.error("Select planned mode to inspect a mapped speed profile")
    actuator = Limits(**load_config(args.actuator_config)[
        "avlite_actuator_adapter"]["ros__parameters"])
    prepared = prepare_plan(config, args.config)
    settings = prepared.settings
    if actuator.max_speed != settings.max_velocity_mps:
        parser.error("Planner and actuator speed ceilings differ")
    low, high = min(prepared.global_plan.velocity), max(prepared.global_plan.velocity)
    target = args.target_mps
    print(f"Shared ceiling: {actuator.max_speed:.3f} m/s; "
          f"effective planner ceiling: {settings.speed_limit:.3f} m/s")
    print(f"Validated map profile: {low:.3f} to {high:.3f} m/s; "
          f"lap length: {prepared.path.length:.2f} m")
    print(f"Throttle cap: {actuator.max_throttle:.3f}; feedforward at shared ceiling: "
          f"{actuator.feedforward * actuator.max_speed:.3f} (model demand, not calibration)")
    if actuator.feedforward * actuator.max_speed > actuator.max_throttle:
        print("Throttle cap clips feedforward at the requested ceiling.")
    if high <= target:
        print(f"This plan never exceeds {target:.3f} m/s. Raising throttle alone cannot "
              "change its corner/acceleration/braking targets.")
    else:
        print(f"This plan permits targets above {target:.3f} m/s. "
              "Actual speed and clean laps still require simulator testing.")
    stopping = target * settings.reaction_time_s + target**2 / (
        2 * settings.braking_deceleration_mps2)
    print(f"At {target:.3f} m/s with these limits: acceleration from rest needs "
          f"{target**2 / (2 * settings.acceleration_mps2):.2f} m; "
          f"reaction plus stopping needs {stopping:.2f} m.")
    print("Offline check only; no driving commands were sent.")


if __name__ == "__main__":
    main()
