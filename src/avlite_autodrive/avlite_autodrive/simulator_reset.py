"""Reset the AutoDRIVE simulator and confirm fresh stationary start feedback."""

import time

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32

from .ros_utils import PREFIX, odom_state


def main():
    """Request a simulator reset and wait for fresh, stationary feedback with zero counters.

    Wait for the bridge subscriber, pulse the reset flag, then require stable confirmation.
    Always release the flag and shut down the temporary ROS node, even if confirmation
    fails.
    """
    rclpy.init()
    node = rclpy.create_node("autodrive_simulator_reset")
    publisher = node.create_publisher(Bool, "/autodrive/reset_command", 1)
    received, state = {}, {}

    def odom(msg):
        """Remember valid forward speed and its receive time for reset confirmation."""
        try:
            state["speed"] = odom_state(msg)[3]
            received["odom"] = time.monotonic()
        except ValueError:
            pass

    def counter(name, msg):
        """Store a lap or collision counter together with its local receive time."""
        state[name] = msg.data
        received[name] = time.monotonic()

    node.create_subscription(Odometry, PREFIX + "/odom", odom, 10)
    for name in ("lap_count", "collision_count"):
        node.create_subscription(Int32, PREFIX + "/" + name,
                                 lambda msg, n=name: counter(n, msg), 10)

    def pump(duration, reset):
        """Publish the chosen reset flag repeatedly while servicing incoming ROS messages.

        The duration is measured with a monotonic clock so wall-clock adjustments do not
        stretch the reset pulse.
        """
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publisher.publish(Bool(data=reset))
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

    try:
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                raise RuntimeError("Simulator bridge reset subscriber is unavailable")
        pump(0.4, True)
        pump(0.4, False)
        released = time.monotonic()
        stable_since = None
        while time.monotonic() - released < 15:
            pump(0.05, False)
            now = time.monotonic()
            ready = (all(received.get(k, 0) > released and now - received[k] < 0.15
                         for k in ("odom", "lap_count", "collision_count"))
                     and abs(state.get("speed", 1)) < 0.05
                     and state.get("lap_count") == state.get("collision_count") == 0)
            stable_since = (stable_since or now) if ready else None
            if stable_since is not None and now - stable_since >= 0.3:
                print("Simulator reset confirmed: stationary with fresh zero counters")
                return
        raise RuntimeError("Simulator reset was not confirmed; connect it and select Autonomous")
    finally:
        pump(0.2, False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
