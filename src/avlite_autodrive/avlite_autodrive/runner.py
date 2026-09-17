"""Wall-clock paced launcher for AVLite's genuine synchronous execution stack."""

import argparse
import logging
import signal
import time

from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c30_control.c39_settings import ControlSettings
from avlite.c40_execution.c49_settings import ExecutionSettings
from avlite.c40_execution.c44_sync_executer import SyncExecuter
from avlite.c20_planning.c26_local_path_planners import ReferencePathPlanner
from avlite.c50_common.c51_capabilities import StackCapability
from .plugin import AutoDRIVEFollowTheGap, AutoDRIVEWorldBridge
from .plugin.planned_controller import AutoDRIVEPlannedController
from .configuration import load_config
from .race_planning import prepare_plan


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
    mode = config.get("driving_mode", "follow_the_gap")
    if mode not in ("follow_the_gap", "planned"):
        raise ValueError("driving_mode must be follow_the_gap or planned")
    # Preparation happens before the ROS bridge can send any command.
    prepared = prepare_plan(config, args.config) if mode == "planned" else None
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    local_planner = None
    controller = (AutoDRIVEPlannedController(prepared) if prepared else
                  AutoDRIVEFollowTheGap(racing=config.get("racing")))
    world = AutoDRIVEWorldBridge()
    if prepared:
        world.map = prepared.planner.map
        world.reference_point = world.map.reference_point
        world.stack_capabilities |= frozenset({StackCapability.MAP_RACE_TRACK})
        ExecutionSettings.c41_world_stack_capabilities = ["LOCALIZATION", "MAP_RACE_TRACK"]
        local_planner = ReferencePathPlanner(prepared.global_plan, pm)
        world.publish_plan(prepared.artifact)
    stack = SyncExecuter(
        perception_model=pm,
        global_planner=prepared.planner if prepared else None,
        local_planner=local_planner,
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
    logging.info("AVLite SyncExecuter + %s; waiting for live sensors", type(controller).__name__)
    previous_start = time.monotonic()
    try:
        while not stopped:
            start = time.monotonic()
            loop_dt, previous_start = start - previous_start, start
            ready = world.ready
            if ready != was_ready or generation != world.reset_generation:
                controller.reset()
                if local_planner:
                    local_planner.reset()
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
                    call_replan=prepared is not None,
                    replan_dt=0.05,
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
