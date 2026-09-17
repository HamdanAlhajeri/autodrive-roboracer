"""Actuator scheduling regression; run in an isolated ROS domain."""

import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_TESTS") != "1", reason="requires isolated ROS_DOMAIN_ID and ROS runtime"
)


def test_actuator_and_watchdog_continue_across_paused_and_backward_ros_clock():
    import rclpy
    from rclpy.clock import ClockType
    from rclpy.time import Time

    from avlite_autodrive.adapter import ActuatorAdapter

    class ObservedAdapter(ActuatorAdapter):
        def __init__(self):
            self.samples = []
            super().__init__()

        def publish(self, throttle, steering):
            self.samples.append({
                "time": time.monotonic(),
                "ros_time": self.get_clock().now().nanoseconds,
                "throttle": throttle,
                "steering": steering,
                "reason": self.control.reason,
            })
            super().publish(throttle, steering)

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = None
    try:
        node = ObservedAdapter()
        # A second legacy timer would double-publish whenever ROS time advances.
        assert len(list(node.timers)) == 1, "Actuator must have exactly one update timer"
        assert node.control_timer.clock.clock_type == ClockType.STEADY_TIME
        ros_clock = node.get_clock()
        assert ros_clock.ros_time_is_active
        ros_clock.set_ros_time_override(Time(seconds=100, clock_type=ClockType.ROS_TIME))

        default_timer_calls = []
        node.create_timer(0.05, lambda: default_timer_calls.append(time.monotonic()))

        def pump_until(condition, fresh_inputs, timeout=3.0):
            deadline = time.monotonic() + timeout
            while not condition() and time.monotonic() < deadline:
                if fresh_inputs:
                    now = time.monotonic()
                    node.control.receive_command(0.1, 1.0, now)
                    node.control.receive_speed(0.0, now)
                    node.control.scan_time = now
                rclpy.spin_once(node, timeout_sec=0.02)
            assert condition(), "Actuator callbacks stopped while ROS time was frozen"

        pump_until(lambda: len(node.samples) >= 5, fresh_inputs=True)
        assert all(sample["ros_time"] == 100_000_000_000 for sample in node.samples)
        assert any(sample["throttle"] > 0 for sample in node.samples)

        before_jump = len(node.samples)
        ros_clock.set_ros_time_override(Time(seconds=98, clock_type=ClockType.ROS_TIME))
        pump_until(lambda: len(node.samples) >= before_jump + 5, fresh_inputs=True)
        after_jump = node.samples[before_jump:]
        assert all(sample["ros_time"] == 98_000_000_000 for sample in after_jump)
        assert any(sample["throttle"] > 0 for sample in after_jump)

        final_input_time = node.control.command_time
        stale_start = len(node.samples)

        def stopped_samples():
            return [sample for sample in node.samples[stale_start:]
                    if sample["reason"] == "stale or missing sensors/command"]

        pump_until(lambda: len(stopped_samples()) >= 3, fresh_inputs=False)
        stopped = stopped_samples()
        assert stopped[0]["time"] - final_input_time >= node.control.limits.timeout
        assert all(sample["throttle"] == 0 and sample["steering"] == 0 for sample in stopped)
        # Verify the stimulus: a normal ROS-clock timer never became ready.
        assert not default_timer_calls
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
