"""Exercise the new planner -> executor -> controller handoff over real ROS."""

import json
import os
import subprocess
import sys
import time

import pytest
import yaml

from test_race_planning import circle_map

pytestmark = pytest.mark.skipif(os.environ.get("RUN_ROS_TESTS") != "1",
                                reason="requires isolated ROS runtime")


def test_planned_runner_and_latched_recording(tmp_path):
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32
    from avlite_autodrive.ros_utils import PREFIX

    (tmp_path / "track.json").write_text(json.dumps(circle_map()))
    config = {
        "driving_mode": "planned",
        "planning": {"map_path": "track.json"},
        "c30_control": {
            "c32_ego_distance_front_axle": 0.324, "c32_ego_max_velocity": 2.5,
            "c32_ego_max_acceleration": 1.0, "c32_ego_min_acceleration": -2.0,
            "c32_ego_max_steering": 0.5235987756, "c32_ego_min_steering": -0.5235987756,
            "c35_cruise_velocity": 2.5, "c35_lookahead_speed_gain": 0.4,
            "c35_min_lookahead": 0.6, "c35_max_lookahead": 1.8,
        },
    }
    profile = tmp_path / "avlite.yaml"
    profile.write_text(yaml.safe_dump(config))
    rclpy.init()
    node = rclpy.create_node("planned_fixture")
    odom_pub = node.create_publisher(Odometry, PREFIX + "/odom", 10)
    scan_pub = node.create_publisher(LaserScan, PREFIX + "/lidar", 10)
    lap_pub = node.create_publisher(Int32, PREFIX + "/lap_count", 10)
    collision_pub = node.create_publisher(Int32, PREFIX + "/collision_count", 10)
    outputs = []
    node.create_subscription(AckermannDriveStamped, "/avlite/control_command",
                             lambda m: outputs.append(m.drive), 10)
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = "world", "roboracer_1"
    odom.pose.pose.position.x = 3.0
    odom.pose.pose.orientation.z = odom.pose.pose.orientation.w = 2**-0.5
    scan = LaserScan()
    scan.header.frame_id = "lidar"
    scan.angle_min, scan.angle_increment = -3.14, 0.01
    scan.range_min, scan.range_max, scan.ranges = 0.06, 10.0, [10.0] * 628

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            odom.header.stamp = scan.header.stamp = stamp
            odom_pub.publish(odom)
            scan_pub.publish(scan)
            lap_pub.publish(Int32(data=0))
            collision_pub.publish(Int32(data=0))
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)

    runner_log = (tmp_path / "runner.log").open("w")
    recorder_log = (tmp_path / "recorder.log").open("w")
    runner = subprocess.Popen([sys.executable, "-m", "avlite_autodrive.runner", "--config",
                               str(profile)], stdout=runner_log, stderr=subprocess.STDOUT)
    recorder = None
    try:
        deadline = time.monotonic() + 20
        while not outputs and time.monotonic() < deadline:
            pump(0.1)
            assert runner.poll() is None, (tmp_path / "runner.log").read_text()
        assert outputs, (tmp_path / "runner.log").read_text()
        assert any(cmd.acceleration > 0 and cmd.steering_angle > 0 for cmd in outputs)
        # A late subscriber must still receive the exact map/plan/settings snapshot.
        recorder = subprocess.Popen(
            [sys.executable, "-m", "avlite_autodrive.record", "--seconds", "1",
             "--output", str(tmp_path / "telemetry.jsonl")],
            stdout=recorder_log, stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + 15
        while recorder.poll() is None and time.monotonic() < deadline:
            pump(0.1)
        assert recorder.poll() == 0, (tmp_path / "recorder.log").read_text()
        artifact = json.loads((tmp_path / "telemetry.plan.json").read_text())
        assert artifact["settings"]["max_velocity_mps"] == 2.5
        assert artifact["map"]["LeftBound"]
        assert (tmp_path / "telemetry.racemap.json").exists()
        assert (tmp_path / "telemetry.planning-config.json").exists()
        lines = (tmp_path / "telemetry.jsonl").read_text().splitlines()
        rows = [json.loads(line) for line in lines]
        assert any(row.get("planned_speed_mps", 0) > 0 for row in rows)
        # A fresh but off-track pose must produce a braking request, never stale intent.
        odom.pose.pose.position.x = 20.0
        outputs.clear()
        pump(0.6)
        assert outputs[-1].acceleration < 0
    finally:
        for proc in (recorder, runner):
            if proc is not None and proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=10)
        runner_log.close()
        recorder_log.close()
        node.destroy_node()
        rclpy.shutdown()
