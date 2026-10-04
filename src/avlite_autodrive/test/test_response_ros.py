"""Exercise the response CLI, command publication and latched recording in isolation."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("RUN_ROS_TESTS") != "1",
                                reason="requires isolated ROS runtime")


def test_response_runner_records_runtime_profile_and_coast_diagnostics(tmp_path):
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32, String
    from avlite_autodrive.configuration import load_config
    from avlite_autodrive.race_planning import prepare_plan
    from avlite_autodrive.response_test import measurement_config
    from avlite_autodrive.ros_utils import PREFIX

    profile = Path("/config/avlite.yaml")
    prepared = prepare_plan(measurement_config(load_config(profile), 1.5), profile)
    rclpy.init()
    node = rclpy.create_node("response_fixture")
    odom_pub = node.create_publisher(Odometry, PREFIX + "/odom", 10)
    scan_pub = node.create_publisher(LaserScan, PREFIX + "/lidar", 10)
    lap_pub = node.create_publisher(Int32, PREFIX + "/lap_count", 10)
    collision_pub = node.create_publisher(Int32, PREFIX + "/collision_count", 10)
    diagnostics, commands = [], []
    node.create_subscription(String, "/avlite/controller_diagnostics",
                             lambda m: diagnostics.append(json.loads(m.data)), 10)
    node.create_subscription(AckermannDriveStamped, "/avlite/control_command",
                             lambda m: commands.append(m.drive), 10)
    scan = LaserScan()
    scan.header.frame_id = "lidar"
    scan.angle_min, scan.angle_increment = -math.pi, math.pi / 100
    scan.range_min, scan.range_max, scan.ranges = 0.06, 10.0, [10.0] * 200
    progress = 0.0

    def pump(seconds, advance=True):
        nonlocal progress
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            # Synthetic poses validate transport/state transitions, not vehicle dynamics.
            if advance:
                progress += 0.15
            xy = prepared.path.at(progress)
            tangent = prepared.path.at(progress + 0.01) - xy
            yaw = math.atan2(tangent[1], tangent[0])
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = "world", "roboracer_1"
            odom.header.stamp = scan.header.stamp = node.get_clock().now().to_msg()
            odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, xy)
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            odom.twist.twist.linear.x = 1.5
            odom_pub.publish(odom)
            scan_pub.publish(scan)
            lap_pub.publish(Int32(data=0))
            collision_pub.publish(Int32(data=0))
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.025)

    runner_log = (tmp_path / "response.log").open("w")
    recorder_log = (tmp_path / "recorder.log").open("w")
    runner = subprocess.Popen(
        [sys.executable, "-m", "avlite_autodrive.runner", "--config", str(profile),
         "--response-speed", "1.5", "--response-trials", "3"],
        stdout=runner_log, stderr=subprocess.STDOUT)
    recorder = None
    try:
        deadline = time.monotonic() + 25
        while (not any(d.get("response_phase") == 2 for d in diagnostics)
               and time.monotonic() < deadline):
            pump(0.1)
            assert runner.poll() is None, (tmp_path / "response.log").read_text()
        assert any(d.get("response_phase") == 2 for d in diagnostics), (
            tmp_path / "response.log").read_text()
        assert any(c.acceleration <= -0.2 for c in commands)
        recorder = subprocess.Popen(
            [sys.executable, "-m", "avlite_autodrive.record", "--seconds", "1",
             "--output", str(tmp_path / "telemetry.jsonl")],
            stdout=recorder_log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 12
        while recorder.poll() is None and time.monotonic() < deadline:
            pump(0.1, advance=False)
        assert recorder.poll() == 0, (tmp_path / "recorder.log").read_text()
        artifact = json.loads((tmp_path / "telemetry.plan.json").read_text())
        assert artifact["response_test"]["requested_trials"] == 3
        assert artifact["settings"]["max_velocity_mps"] == 1.5
        assert artifact["resolved_config"]["c30_control"]["c35_cruise_velocity"] == 1.5
        rows = [json.loads(line) for line in
                (tmp_path / "telemetry.jsonl").read_text().splitlines()]
        assert any(r.get("response_trial", 0) for r in rows)
    finally:
        for process in (recorder, runner):
            if process is not None and process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
        runner_log.close()
        recorder_log.close()
        node.destroy_node()
        rclpy.shutdown()
