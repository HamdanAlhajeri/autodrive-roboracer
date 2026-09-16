"""Wall-clock paced launcher for AVLite's genuine synchronous execution stack."""

import argparse
import logging
import signal
import time

from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c30_control.c39_settings import ControlSettings
from avlite.c40_execution.c49_settings import ExecutionSettings
from avlite.c40_execution.c44_sync_executer import SyncExecuter
from .plugin import AutoDRIVEFollowTheGap, AutoDRIVEWorldBridge
from .configuration import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/config/avlite.yaml")
    args = parser.parse_args()
    config = load_config(args.config)
    # Apply the explicit small-car profile BEFORE constructing the controller.
    for key, value in config["c30_control"].items():
        if not hasattr(ControlSettings, key):
            raise ValueError("Unknown AVLite control setting: " + key)
        setattr(ControlSettings, key, value)
    ExecutionSettings.c41_world_stack_capabilities = ["LOCALIZATION"]
    ExecutionSettings.c41_world_capabilities = ["LIDAR_2D"]
    logging.basicConfig(level=logging.INFO)
    logging.info(
        "Driving settings: cruise=%.3f m/s, max_velocity=%.3f m/s",
        ControlSettings.c35_cruise_velocity,
        ControlSettings.c32_ego_max_velocity,
    )
    world = AutoDRIVEWorldBridge()
    controller = AutoDRIVEFollowTheGap(racing=config.get("racing"))
    stack = SyncExecuter(
        perception_model=PerceptionModel(ego_vehicle=EgoState(x=0, y=0)),
        controller=controller,
        world=world,
        control_dt=0.05,
    )
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    generation, was_ready = world.reset_generation, False
    logging.info("AVLite SyncExecuter + AutoDRIVEFollowTheGap; waiting for live sensors")
    previous_start = time.monotonic()
    try:
        while not stopped:
            start = time.monotonic()
            loop_dt, previous_start = start - previous_start, start
            ready = world.ready
            if ready != was_ready or generation != world.reset_generation:
                controller.reset()
                generation = world.reset_generation
                logging.info(
                    "Sensors %s", "ready" if ready else "unavailable: withholding commands"
                )
            was_ready = ready
            if ready:
                stack.step(
                    control_dt=0.05,
                    sim_dt=0.05,
                    call_perceive=False,
                    call_replan=False,
                    call_localize=False,
                    pace_control=False,
                )
                work_time = time.monotonic() - start
                world.publish_diagnostics({
                    **controller.diagnostics,
                    "controller_loop_dt_s": loop_dt,
                    "controller_step_time_s": work_time,
                    "controller_overrun": work_time > 0.05 or loop_dt > 0.075,
                })
            time.sleep(max(0, 0.05 - (time.monotonic() - start)))
    finally:
        stack.stop()
        world.close()


if __name__ == "__main__":
    main()
