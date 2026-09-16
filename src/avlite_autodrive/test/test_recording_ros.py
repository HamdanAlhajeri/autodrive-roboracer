"""Exercise recorder discovery and lost telemetry in an isolated ROS domain."""

import json
import os
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_TESTS") != "1", reason="requires isolated ROS_DOMAIN_ID and ROS runtime"
)


@pytest.fixture
def recorder(tmp_path):
    processes = []
    logs = []

    def launch(*options):
        output = tmp_path / "telemetry.jsonl"
        log = (tmp_path / "recorder.log").open("w")
        logs.append(log)
        process = subprocess.Popen(
            [sys.executable, "-m", "avlite_autodrive.record", "--output", str(output),
             "--seconds", "1", "--wait-for-odom", "10", *options],
            stdout=log, stderr=subprocess.STDOUT,
        )
        processes.append(process)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if "Waiting up to" in (tmp_path / "recorder.log").read_text():
                return process, output
            assert process.poll() is None, (tmp_path / "recorder.log").read_text()
            time.sleep(0.02)
        pytest.fail("Recorder did not reach the odometry wait")

    yield launch
    for process in processes:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=5)
    for log in logs:
        log.close()


@pytest.fixture
def odometry():
    import rclpy
    from nav_msgs.msg import Odometry
    from avlite_autodrive.ros_utils import PREFIX

    rclpy.init()
    node = rclpy.create_node("recorder_test_publisher")
    publisher = node.create_publisher(Odometry, PREFIX + "/odom", 10)
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = 1.2

    def pump(seconds, publish=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish:
                message.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)

    yield message, pump
    node.destroy_node()
    rclpy.shutdown()


def test_missing_odometry_fails_before_recording_timer_starts(recorder):
    process, output = recorder("--wait-for-odom", "0.4")
    assert process.wait(timeout=5) == 2
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert summary["status"] == "failed"
    assert "No valid odometry" in summary["error"]
    assert summary["elapsed_s"] == 0
    assert summary["odometry_samples"] == 0
    assert not summary["has_odometry"]
    assert summary["started_utc"] is None
    assert output.read_text() == ""


def test_delayed_valid_odometry_starts_full_recording_duration(recorder, odometry):
    message, pump = odometry
    process, output = recorder()
    # A publisher alone is insufficient: invalid sensor frames must not start the timer.
    message.pose.pose.orientation.w = 0.0
    pump(1.2)
    assert process.poll() is None
    assert output.read_text() == ""
    message.pose.pose.orientation.w = 1.0
    deadline = time.monotonic() + 8
    while process.poll() is None and time.monotonic() < deadline:
        pump(0.1)
    assert process.poll() == 0
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert summary["status"] == "completed"
    assert summary["waiting_s"] >= 1.0
    assert 1.0 <= summary["elapsed_s"] < 2.0
    assert summary["invalid_odom_samples"] > 0
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert len(rows) >= 8
    assert all(row["speed"] == pytest.approx(1.2) for row in rows)
    assert rows[-1]["elapsed_s"] >= 0.8


def test_lost_odometry_fails_early_and_preserves_partial_data(recorder, odometry):
    _, pump = odometry
    process, output = recorder("--seconds", "30", "--odom-timeout", "0.6")
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline and not output.read_text():
        pump(0.1)
    assert output.read_text(), "Recorder did not receive the initial odometry"
    assert process.wait(timeout=5) == 2
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert summary["status"] == "failed"
    assert "odometry stopped" in summary["error"]
    assert summary["elapsed_s"] < 5
    assert summary["has_odometry"]
    assert not summary["clean_lap"]
    assert any(json.loads(line)["speed"] == pytest.approx(1.2)
               for line in output.read_text().splitlines())
