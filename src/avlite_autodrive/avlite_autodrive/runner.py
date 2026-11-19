"""Wall-clock paced launcher for AVLite's genuine synchronous execution stack."""

import argparse
import logging
import math
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
from .plugin.pose_view import PoseComparisonView
from .pose_view import geometry_from_map, pose_view_config
from .configuration import load_config
from .race_planning import prepare_plan
from .slam_localization import localization_config
from .avlite_localization import (
    AutoDRIVELidarLocalization, after_interruption, avlite_localization_config)
from .response_test import ResponseExperiment, measurement_config
from .plugin.response_controller import AutoDRIVEResponseController


def main():
    """Build the configured AVLite pipeline and run it at a nominal 20 Hz.

    Resolve shared settings before creating controllers, and validate the global plan before
    opening the command bridge. Reset tracking after sensor interruptions, publish timing
    diagnostics, and close ROS resources when stopping. By default simulator ground truth
    supplies localization; with localization.source: slam the pose comes from SLAM Toolbox
    and wheel odometry instead. This loop does not advance a local vehicle model.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/config/avlite.yaml")
    parser.add_argument("--response-speed-mps", "--response-speed", type=float,
                        help="Run bounded straight coast attempts at 1.0-2.5 m/s")
    parser.add_argument("--response-trials", type=int, default=3)
    args = parser.parse_args()
    config = load_config(args.config)
    if args.response_speed_mps is not None:
        ResponseExperiment(args.response_speed_mps, args.response_trials)  # validate before ROS
        config = measurement_config(config, args.response_speed_mps)
        config["response_test"] = {"speed_mps": args.response_speed_mps,
                                   "trials": args.response_trials}
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
    localization = localization_config(config, args.config)  # validate before ROS
    stack_localization = avlite_localization_config(config)
    view_config = pose_view_config(config)  # validate before ROS
    mode = config.get("driving_mode", "follow_the_gap")
    if mode not in ("follow_the_gap", "planned"):
        raise ValueError("driving_mode must be follow_the_gap or planned")
    # Preparation happens before the ROS bridge can send any command.
    prepared = prepare_plan(config, args.config) if mode == "planned" else None
    if prepared:
        logging.info(
            "Planned speed ceiling %.3f m/s (requested %.3f; braking calibrated=%s); "
            "acceleration %.3f m/s^2, braking %.3f m/s^2",
            prepared.settings.speed_limit, prepared.settings.max_velocity_mps,
            prepared.settings.braking_calibrated, prepared.settings.acceleration_mps2,
            prepared.settings.braking_deceleration_mps2,
        )
        logging.info(
            "Validated track profile: %.3f to %.3f m/s over %.2f m; "
            "the ceiling is not the speed target at every point",
            min(prepared.global_plan.velocity), max(prepared.global_plan.velocity),
            prepared.path.length,
        )
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    local_planner = None
    controller = (AutoDRIVEPlannedController(prepared) if prepared else
                  AutoDRIVEFollowTheGap(racing=config.get("racing")))
    if args.response_speed_mps is not None:
        controller = AutoDRIVEResponseController(
            prepared, args.response_speed_mps, args.response_trials)
        prepared.artifact["response_test"] = {
            **controller.experiment.metadata(),
            "straight_steering_limit_rad": controller.coast_window.steer_limit,
            "preview_distance_m": controller.coast_window.preview_distance,
        }
    world = AutoDRIVEWorldBridge(localization=localization,
                                 stack_localization=stack_localization is not None)
    localizer = None
    if stack_localization is not None:
        # AVLite's own ICP owns the pose; ground truth is read only for diagnostics.
        reference = (map_hit_points(config, args.config)
                     if stack_localization.reference == "map_hits" else None)
        localizer = AutoDRIVELidarLocalization(pm, stack_localization, world.wheel_speed,
                                               odometry_pose=world.wheel_pose,
                                               reference_points=reference)
        logging.info("Localization: AVLite LidarLocalization (ICP) from spawn pose %s, "
                     "reference %s%s, wheel-odometry prediction on",
                     stack_localization.start_pose_world, stack_localization.reference,
                     "" if reference is None else f" ({len(reference)} map points)")
    if localization is not None:
        # The bridge withholds commands until SLAM reports a healthy, fresh pose.
        world.pose_source.start()
        logging.info("Localization: SLAM Toolbox on %s from start pose %s%s",
                     localization.pose_graph, localization.start_pose,
                     "" if localization.map_origin_world is None
                     else f", world origin {localization.map_origin_world}")
    if prepared:
        world.map = prepared.planner.map
        world.reference_point = world.map.reference_point
        world.stack_capabilities |= frozenset({StackCapability.MAP_RACE_TRACK})
        # Without LOCALIZATION here AVLite runs the localization strategy, never ground truth.
        ExecutionSettings.c41_world_stack_capabilities = (
            ["MAP_RACE_TRACK", "MAP_OCCUPANCY"] if localizer else ["LOCALIZATION", "MAP_RACE_TRACK"])
        if localizer:
            # LidarLocalization requires MAP_OCCUPANCY; it builds its own reference map from
            # the first scan, so the flag only satisfies that declared requirement.
            world.stack_capabilities |= frozenset({StackCapability.MAP_OCCUPANCY})
        local_planner = ReferencePathPlanner(prepared.global_plan, pm)
        world.publish_plan(prepared.artifact)
    view = None
    if view_config is not None:
        # A browser view of estimated vs true pose; it only reads, so failing to start is not fatal.
        geometry = (geometry_from_map(prepared.planner.map, prepared.path.points)
                    if prepared else None)
        view = PoseComparisonView(view_config, geometry, localizer)
        try:
            view.start()
        except OSError as error:
            logging.warning("Pose view not started (port %d): %s", view_config.port, error)
            view = None
    stack = SyncExecuter(
        perception_model=pm,
        global_planner=prepared.planner if prepared else None,
        local_planner=local_planner,
        controller=controller,
        world=world,
        localization=localizer,
        control_dt=0.05,
        tasks=[view] if view else None,
    )
    stopped = False

    def stop(*_):
        """Set the shutdown flag so the running loop can exit through its cleanup block."""
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
                if localizer:
                    after_interruption(localizer, generation != world.reset_generation)
                if view:
                    view.reset()
                if local_planner:
                    local_planner.reset()
                generation = world.reset_generation
                logging.info(
                    "Sensors %s", "ready" if ready else "unavailable: withholding commands"
                )
            was_ready = ready
            if ready:
                if (args.response_speed_mps is not None
                        and world.node.count_publishers("/avlite/control_command") > 1):
                    raise RuntimeError("Competing control publisher; response test aborted")
                stack.step(
                    control_dt=0.05,
                    sim_dt=0.05,
                    call_perceive=False,
                    call_replan=prepared is not None,
                    replan_dt=0.05,
                    call_localize=localizer is not None,
                    localization_dt=0.05,
                    pace_control=False,
                )
                work_time = time.monotonic() - start
                world.publish_diagnostics({
                    **controller.diagnostics,
                    **(stack_localization_diagnostics(pm, world, localizer) if localizer else {}),
                    "controller_loop_dt_s": loop_dt,
                    "controller_step_time_s": work_time,
                    "controller_overrun": work_time > 0.05 or loop_dt > 0.075,
                })
            time.sleep(max(0, 0.05 - (time.monotonic() - start)))
    finally:
        stack.stop()
        if view:
            view.close()
        world.close()


def map_hit_points(config, config_path):
    """World-frame LiDAR returns recorded in the planned map, as the ICP reference."""
    import json
    from pathlib import Path
    path = Path(config_path).resolve().parent / config["planning"]["map_path"]
    hits = json.loads(path.read_text(encoding="utf-8")).get("observed_hits")
    if not hits:
        raise ValueError(f"{path} has no observed_hits to use as a localization reference")
    return hits


def stack_localization_diagnostics(pm, world, localizer=None):
    """AVLite pose error against simulator truth (diagnostic only; never drives the car)."""
    truth, ego = world.true_pose(), pm.ego_vehicle
    if truth is None:
        return {}
    yaw = math.atan2(math.sin(ego.theta - truth[2]), math.cos(ego.theta - truth[2]))
    values = {"localization_ok": 1.0,
              "localization_error_m": math.hypot(ego.x - truth[0], ego.y - truth[1]),
              "localization_yaw_error_rad": abs(yaw),
              "localization_x_error_m": ego.x - truth[0],
              "localization_y_error_m": ego.y - truth[1],
              "estimate_x_m": ego.x, "estimate_y_m": ego.y,
              "truth_x_m": truth[0], "truth_y_m": truth[1]}
    if localizer is not None and localizer.alignment is not None:
        values["localization_alignment"] = localizer.alignment
        values["localization_hits"] = float(localizer.hits)
        values["localization_recoveries"] = float(localizer.recoveries)
        values["localization_odom_clipped"] = float(localizer.odometry_clipped)
    return values


if __name__ == "__main__":
    main()
