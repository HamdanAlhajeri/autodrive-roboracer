"""Start the stock AutoDRIVE API with a guard for incomplete startup packets."""

import math
import threading
import time


class CommandExpiry:
    """Final simulator output gate, independent of ROS timers and the actuator loop."""

    def __init__(self, timeout=0.5, clock=time.monotonic):
        """Create a final command watchdog with a monotonic clock and a lock.

        Both throttle and steering must receive fresh updates before a simulator reply can
        contain their stored values.
        """
        self.timeout, self.clock = timeout, clock
        self.lock = threading.Lock()
        self.received = {}
        self.values = {}

    def invalidate(self):
        """Forget both commands so the next simulator reply sends zero output."""
        with self.lock:
            self.received.clear()
            self.values.clear()

    def receive(self, axis, value):
        """Store a valid normalized command and its receive time for one axis.

        Round accepted values to the simulator's command precision. An invalid value clears
        both axes and returns False.
        """
        with self.lock:
            if not math.isfinite(value) or not -1 <= value <= 1:
                self.received.clear()
                self.values.clear()
                return False
            self.received[axis] = self.clock()
            self.values[axis] = round(value, 3)
            return True

    def gate(self, payload):
        """Copy a simulator reply and replace its throttle/steering with fresh values or zeros.

        Perform the age check when sending, not just when receiving ROS messages. This also
        catches a delayed bridge callback.
        """
        with self.lock:
            now = self.clock()
            fresh = all(axis in self.received and 0 <= now - self.received[axis] <= self.timeout
                        for axis in ("throttle", "steering"))
            result = dict(payload)
            for axis in ("throttle", "steering"):
                result["V1 " + axis.title()] = str(self.values[axis] if fresh else 0.0)
            return result


def install_command_expiry(stock, clock=time.monotonic):
    """Wrap the stock bridge callbacks and final socket output with command expiry checks.

    Install these wrappers before stock.main() creates ROS subscriptions. Resets and
    connection changes clear command history; each outgoing reply checks both axes again.
    """
    guard = CommandExpiry(clock=clock)
    for axis in ("throttle", "steering"):
        name = "callback_" + axis + "_command"
        original = getattr(stock, name)

        def callback(msg, axis=axis, original=original):
            """Timestamp one incoming actuator value and pass valid data to its original
            callback.

            Default arguments capture the current axis and callback separately for each loop
            iteration.
            """
            value = float(msg.data)
            if guard.receive(axis, value):
                original(msg)

        setattr(stock, name, callback)

    reset = stock.callback_reset_command

    def reset_callback(msg):
        """Clear command history for a requested reset, then run the stock reset callback."""
        if msg.data:
            guard.invalidate()
        reset(msg)

    stock.callback_reset_command = reset_callback
    emit = stock.sio.emit

    def guarded_emit(event, data=None, *args, **kwargs):
        """Apply the watchdog to vehicle command replies while forwarding other socket events
        unchanged.
        """
        if event == "Bridge" and isinstance(data, dict) and "V1 Throttle" in data:
            data = guard.gate(data)
        return emit(event, data, *args, **kwargs)

    stock.sio.emit = guarded_emit
    connect = stock.connect

    @stock.sio.on("connect")
    def connected(sid, environ):
        """Discard commands from a previous connection before running the stock connection
        handler.
        """
        guard.invalidate()
        return connect(sid, environ)

    @stock.sio.on("disconnect")
    def disconnected(sid, *args):
        """Invalidate actuator commands when the simulator disconnects."""
        guard.invalidate()

    return guard


REQUIRED_FIELDS = frozenset(
    {
        "V1 Throttle",
        "V1 Steering",
        "V1 Encoder Angles",
        "V1 Position",
        "V1 Orientation Quaternion",
        "V1 Angular Velocity",
        "V1 Linear Acceleration",
        "V1 Linear Velocity",
        "V1 LIDAR Scan Rate",
        "V1 LIDAR Range Array",
        "V1 Front Camera Image",
        "V1 Lap Count",
        "V1 Lap Time",
        "V1 Last Lap Time",
        "V1 Best Lap Time",
        "V1 Collisions",
    }
)


def install_startup_guard(stock, command_guard=None):
    """Wrap the stock sensor handler so incomplete startup packets still receive a reply.

    Unity waits for each reply before sending another frame. Returning stopped controls
    keeps this handshake moving until a complete sensor packet arrives.
    """
    original = stock.bridge

    @stock.sio.on("Bridge")
    def guarded_bridge(sid, data):
        """Reply with zero controls to an incomplete packet; otherwise call the stock bridge.

        Also clear command history so an old throttle value cannot survive an incomplete
        sensor frame.
        """
        if not isinstance(data, dict) or not REQUIRED_FIELDS.issubset(data):
            if command_guard is not None:
                command_guard.invalidate()
            # Unity can connect before the first LiDAR/camera frame exists.
            # Its next frame waits for a reply; the stock KeyError stalls that
            # handshake indefinitely. Reply stopped without inventing sensors.
            stock.sio.emit(
                "Bridge",
                data={
                    "V1 Throttle": "0",
                    "V1 Steering": "0",
                    "V1 Reset": "False",
                },
                room=sid,
            )
            return
        return original(sid, data)

    return guarded_bridge


def main():
    """Install the startup and command-expiry wrappers, then launch the stock AutoDRIVE API."""
    from autodrive_roboracer import autodrive_bridge as stock

    guard = install_command_expiry(stock)
    install_startup_guard(stock, guard)
    stock.main()


if __name__ == "__main__":
    main()
