"""Real ROS processes; run in the AVLite image with RUN_ROS_TESTS=1."""

import json
import os
import signal
import shutil
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_TESTS") != "1", reason="requires isolated ROS_DOMAIN_ID and ROS runtime"
)


def test_stack_moves_and_watchdog_stops_after_avlite_is_killed(tmp_path):
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32, String
    from avlite_autodrive.ros_utils import PREFIX

    rclpy.init()
    node = rclpy.create_node("avlite_integration_fixture")
    scan_pub = node.create_publisher(LaserScan, PREFIX + "/lidar", 1)
    odom_pub = node.create_publisher(Odometry, PREFIX + "/odom", 1)
    outputs = []
    controller_diagnostics, actuator_diagnostics = [], []
    node.create_subscription(
        String, "/avlite/controller_diagnostics",
        lambda message: controller_diagnostics.append(json.loads(message.data)), 10,
    )
    node.create_subscription(
        String, "/avlite/actuator_diagnostics",
        lambda message: actuator_diagnostics.append(json.loads(message.data)), 10,
    )
    node.create_subscription(
        Float32,
        PREFIX + "/throttle_command",
        lambda m: outputs.append((time.monotonic(), m.data)),
        1,
    )
    scan = LaserScan()
    scan.header.frame_id = "lidar"
    scan.angle_min, scan.angle_increment = -2.35619, 0.004363323
    scan.range_min, scan.range_max = 0.06, 10.0
    scan.ranges = [3.0] * 1080
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = "world", "roboracer_1"
    odom.pose.pose.orientation.w = 1.0

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            scan.header.stamp = odom.header.stamp = stamp
            scan_pub.publish(scan)
            odom_pub.publish(odom)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

    # Use the real profiles but override both through one isolated shared file.
    # A non-default throttle cap proves the adapter consumed the shared settings.
    for profile in ("avlite.yaml", "actuator.yaml"):
        shutil.copyfile("/config/" + profile, tmp_path / profile)
    (tmp_path / "driving.yaml").write_text("speed_mps: 0.7\nmax_throttle: 0.031\n")
    adapter = subprocess.Popen(
        ["/usr/bin/python3", "-m", "avlite_autodrive.adapter",
         "--config", str(tmp_path / "actuator.yaml")]
    )
    runner = None
    try:
        pump(2)
        assert outputs and all(v == 0 for _, v in outputs), "Startup must stay stopped"
        runner = subprocess.Popen(
            [sys.executable, "-m", "avlite_autodrive.runner",
             "--config", str(tmp_path / "avlite.yaml")]
        )
        pump(6)
        assert runner.poll() is None, "AVLite process exited"
        assert any(v > 0 for _, v in outputs), "Actual AVLite commands did not reach actuators"
        assert max(v for _, v in outputs) == pytest.approx(0.031, abs=1e-6)
        assert controller_diagnostics and actuator_diagnostics
        diagnostic = controller_diagnostics[-1]
        assert 0 < diagnostic["target_velocity_mps"] <= 0.7
        assert diagnostic["lookahead_m"] > 0
        assert diagnostic["gap_preview_requested_m"] == pytest.approx(1.5)
        assert diagnostic["gap_preview_selected_m"] == pytest.approx(1.5)
        assert diagnostic["lookahead_m"] == pytest.approx(0.6)
        assert diagnostic["gap_preview_fallback"] is False
        assert abs(diagnostic["target_bearing_raw_rad"]) < 0.02
        assert abs(diagnostic["target_bearing_rad"]) < 0.02
        assert diagnostic["clearance_m"] > 0
        assert 0 <= diagnostic["controller_lidar_age_s"] < 0.5
        assert diagnostic["controller_step_time_s"] >= 0
        assert actuator_diagnostics[-1]["actuator_target_speed_mps"] <= 0.7
        assert "throttle_saturated" in actuator_diagnostics[-1]
        runner.kill()
        runner.wait(timeout=5)
        killed = time.monotonic()
        pump(1.5)
        later = [v for t, v in outputs if t > killed + 0.7]
        assert later and all(v == 0 for v in later), "Watchdog held throttle after process death"
        assert adapter.poll() is None
    finally:
        for process in (runner, adapter):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
