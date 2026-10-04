"""ROS transport smoke test; no SLAM launch, sensors or motor driver required."""

import os
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("RUN_ROS_TESTS") != "1",
                                reason="requires ROS Humble runtime")


def test_uncommissioned_runtime_starts_idle_and_only_publishes_stop(tmp_path):
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from avlite_roboracer.profile import HardwareProfile
    from avlite_roboracer.runtime import Runtime
    from roboracer_fixtures import PROFILE_PATH, ROOT

    rclpy.init()
    node = rclpy.create_node("roboracer_test_supervisor")
    runtime = None
    received = []
    node.create_subscription(AckermannDriveStamped, "/roboracer/command",
                             lambda msg: received.append(msg.drive.speed), 10)
    try:
        runtime = Runtime(node, HardwareProfile.load(PROFILE_PATH),
                          ROOT / "config/roboracer/race.yaml",
                          ROOT / "config/roboracer/slam_toolbox.yaml",
                          tmp_path / "data", tmp_path / "sock")
        status = runtime.handle_request({"command": "status"})
        assert status["state"] == "IDLE"
        assert not runtime.handle_request({"command": "map-start",
                                           "session": status["session"]})["ok"]
        assert not runtime.handle_request({"command": "race-start",
                                           "session": status["session"]})["ok"]
        assert not runtime.handle_request(["invalid"])["ok"]
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and len(received) < 3:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(received) >= 3
        assert all(speed == 0 for speed in received)
        assert runtime.slam.process is None
    finally:
        if runtime is not None:
            runtime.close()
        node.destroy_node()
        rclpy.shutdown()
