"""Record diagnostics over real ROS transport without connecting to the simulator."""

import json
import os
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_TESTS") != "1", reason="requires isolated ROS domain"
)


@pytest.mark.parametrize("with_track", [False, True])
def test_diagnostics_reach_recording_and_export_control_graph(tmp_path, with_track):
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32, String
    from avlite_autodrive.ros_utils import PREFIX

    rclpy.init()
    node = rclpy.create_node("diagnostics_recording_fixture")
    odom = Odometry()
    odom.pose.pose.orientation.w = 1.0
    odom.header.frame_id = "world"
    odom.child_frame_id = "roboracer_1"
    scan = LaserScan()
    scan.header.frame_id = "lidar"
    scan.angle_min, scan.angle_increment = -1.57, 0.01
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.ranges = [2.0] * 315
    pairs = [
        (PREFIX + "/odom", odom),
        (PREFIX + "/lidar", scan),
        (PREFIX + "/lap_count", Int32(data=0)),
        (PREFIX + "/collision_count", Int32(data=0)),
        ("/avlite/controller_diagnostics", String(data=json.dumps({
            "target_velocity_mps": 1.2, "lookahead_m": 1.0, "clearance_m": 2.5,
            "gap_preview_requested_m": 1.5, "gap_preview_selected_m": 1.14,
            "gap_preview_fallback": True,
            "target_bearing_raw_rad": 0.35, "target_bearing_rad": 0.25,
            "lap_count": 99,  # This unrecognized key must not replace actual counter data.
        }))),
        ("/avlite/actuator_diagnostics", String(data=json.dumps({
            "actuator_target_speed_mps": 1.1, "braking_requested": True,
        }))),
    ]
    publishers = [(node.create_publisher(type(msg), topic, 10), msg) for topic, msg in pairs]
    output = tmp_path / "telemetry.jsonl"
    with (tmp_path / "recorder.log").open("w") as log:
        process = subprocess.Popen(
            [sys.executable, "-m", "avlite_autodrive.record", "--seconds", "0.7",
             "--output", str(output)] + (["--track-map"] if with_track else []),
            stdout=log, stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 10
            while process.poll() is None and time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                odom.header.stamp = scan.header.stamp = stamp
                for publisher, message in publishers:
                    publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(0.01)
            assert process.poll() == 0, (tmp_path / "recorder.log").read_text()
        finally:
            if process.poll() is None:
                process.kill()
            process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert rows[-1]["target_velocity_mps"] == 1.2
    assert rows[-1]["actuator_target_speed_mps"] == 1.1
    assert rows[-1]["braking_requested"] is True
    assert rows[-1]["lap_count"] == 0
    assert rows[-1]["controller_diagnostics_age_s"] < 0.5
    assert rows[-1]["actuator_diagnostics_age_s"] < 0.5
    assert rows[-1]["gap_preview_requested_m"] == 1.5
    assert rows[-1]["gap_preview_selected_m"] == 1.14
    assert rows[-1]["gap_preview_fallback"] is True
    assert rows[-1]["target_bearing_raw_rad"] == 0.35
    assert rows[-1]["target_bearing_rad"] == 0.25
    outline_path = output.with_suffix(".track.json")
    if with_track:
        outline = json.loads(outline_path.read_text())
        assert outline["frame_id"] == "world"
        assert outline["scans_used"] >= 1
        assert len(outline["points"]) > 30
        summary = json.loads(output.with_suffix(".summary.json").read_text())
        assert summary["track_map"]["point_count"] == len(outline["points"])
    else:
        assert not outline_path.exists()
    subprocess.run(
        [sys.executable, "-m", "avlite_autodrive.plot_recording", str(output), "--lap-report"],
        check=True, timeout=30,
    )
    for suffix in (".png", ".lap.png", ".control.png"):
        assert output.with_suffix(suffix).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
