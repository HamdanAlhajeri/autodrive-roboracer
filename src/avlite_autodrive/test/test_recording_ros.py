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


@pytest.fixture
def screening_telemetry():
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int32
    from avlite_autodrive.ros_utils import PREFIX

    rclpy.init()
    node = rclpy.create_node("screening_test_publisher")
    messages = {"odom": Odometry(), "lap_count": Int32(), "collision_count": Int32()}
    messages["odom"].pose.pose.orientation.w = 1.0
    publishers = {
        name: node.create_publisher(type(message), PREFIX + "/" + name, 10)
        for name, message in messages.items()
    }

    def pump(seconds, odom=True, lap_count=True, collision_count=True):
        enabled = {"odom": odom, "lap_count": lap_count, "collision_count": collision_count}
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for name, publisher in publishers.items():
                if enabled[name]:
                    publisher.publish(messages[name])
            rclpy.spin_once(node, timeout_sec=0.002)
            time.sleep(0.01)

    yield messages, pump
    node.destroy_node()
    rclpy.shutdown()


def wait_for_counter_baseline(output, process, pump):
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline and process.poll() is None:
        pump(0.05)
        rows = [json.loads(line) for line in output.read_text().splitlines()]
        if rows and all(name in rows[-1] for name in ("lap_count", "collision_count")):
            return rows[-1]
    pytest.fail("Recorder never captured both counter baselines")


def pump_until_exit(process, pump, **options):
    deadline = time.monotonic() + 8
    while process.poll() is None and time.monotonic() < deadline:
        pump(0.05, **options)
    assert process.poll() is not None, "Recorder did not finish"


def test_pre_odometry_crossing_is_only_the_starting_baseline(recorder, screening_telemetry):
    messages, pump = screening_telemetry
    process, output = recorder("--seconds", "2", "--laps", "1")
    pump(0.5, odom=False)
    messages["lap_count"].data = 1
    messages["collision_count"].data = 2
    pump(0.3, odom=False)
    assert output.read_text() == ""
    pump_until_exit(process, pump)
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert process.returncode == 0
    assert summary["initial"]["lap_count"] == 1
    assert summary["initial"]["collision_count"] == 2
    assert summary["completed_laps"] == 0
    assert summary["stop_reason"] == "time_limit"
    assert not summary["clean_run"]


def test_lost_collision_stream_cannot_pass_clean_screening(recorder, screening_telemetry):
    messages, pump = screening_telemetry
    process, output = recorder("--seconds", "10", "--laps", "1")
    wait_for_counter_baseline(output, process, pump)
    messages["lap_count"].data = 1
    pump_until_exit(process, pump, collision_count=False)
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert process.returncode == 2
    assert "collision_count" in summary["error"]
    assert summary["counter_telemetry_status"] == "stale"
    assert summary["counter_max_age_s"]["collision_count"] > 0.5
    assert not summary["clean_lap"]
    assert not summary["clean_run"]
    final_row = json.loads(output.read_text().splitlines()[-1])
    assert final_row["collision_count_age_s"] > 0.5


def test_time_limit_during_finish_grace_does_not_pass(recorder, screening_telemetry):
    messages, pump = screening_telemetry
    process, output = recorder("--seconds", "2", "--laps", "1")
    row = wait_for_counter_baseline(output, process, pump)
    while row["elapsed_s"] < 1.5 and process.poll() is None:
        pump(0.05)
        row = json.loads(output.read_text().splitlines()[-1])
    messages["lap_count"].data = 1
    pump_until_exit(process, pump)
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert process.returncode == 0
    assert summary["completed_laps"] == 1
    assert summary["stop_reason"] == "time_limit"
    assert summary["counter_telemetry_valid"]
    assert not summary["clean_run"]
    assert "finish feedback window" in summary["screening_failure_reason"]


def test_incident_is_written_into_final_jsonl_row(recorder, screening_telemetry):
    messages, pump = screening_telemetry
    process, output = recorder("--seconds", "10", "--laps", "3", "--stop-on-incident")
    wait_for_counter_baseline(output, process, pump)
    messages["collision_count"].data = 1
    pump_until_exit(process, pump)
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert process.returncode == 0
    assert summary["stop_reason"] == "incident"
    assert summary["incident_detected"]
    assert not summary["clean_run"]
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert any(row.get("collision_count") == 0 for row in rows)
    assert rows[-1]["collision_count"] == 1
    assert rows[-1]["elapsed_s"] > rows[0]["elapsed_s"]


def test_fresh_counters_and_full_finish_grace_pass_screening(recorder, screening_telemetry):
    messages, pump = screening_telemetry
    process, output = recorder("--seconds", "10", "--laps", "1")
    wait_for_counter_baseline(output, process, pump)
    messages["lap_count"].data = 1
    pump_until_exit(process, pump)
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert process.returncode == 0
    assert summary["completed_laps"] == 1
    assert summary["stop_reason"] == "lap_target"
    assert summary["clean_run"]
    assert summary["screening_failure_reason"] is None
    assert summary["elapsed_s"] - summary["first_lap_elapsed_s"] >= 1


def test_missing_counter_stream_has_startup_grace_then_fails(recorder, screening_telemetry):
    _, pump = screening_telemetry
    process, output = recorder("--seconds", "10", "--laps", "1", "--wait-for-odom", "1")
    pump_until_exit(process, pump, collision_count=False)
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert process.returncode == 2
    assert summary["elapsed_s"] >= 1
    assert summary["counter_telemetry_status"] == "missing"
    assert "baseline" in summary["error"]
    assert "collision_count" in summary["error"]
    assert not summary["clean_run"]


def test_motion_before_counter_discovery_cannot_pass(recorder, screening_telemetry):
    messages, pump = screening_telemetry
    messages["odom"].twist.twist.linear.x = 1.0
    process, output = recorder("--seconds", "10", "--laps", "1")
    deadline = time.monotonic() + 5
    while not output.read_text() and time.monotonic() < deadline:
        pump(0.05, lap_count=False, collision_count=False)
    assert output.read_text()
    wait_for_counter_baseline(output, process, pump)
    messages["lap_count"].data = 1
    pump_until_exit(process, pump)
    summary = json.loads(output.with_suffix(".summary.json").read_text())
    assert process.returncode == 0
    assert summary["completed_laps"] == 1
    assert summary["counter_telemetry_status"] == "fresh"
    assert not summary["counter_baseline_before_motion"]
    assert not summary["clean_run"]
