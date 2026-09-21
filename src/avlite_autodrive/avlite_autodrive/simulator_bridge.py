"""Start the stock AutoDRIVE API with a guard for incomplete startup packets."""

import math
import threading
import time


class CommandExpiry:
    """Final simulator output gate, independent of ROS timers and the actuator loop."""

    def __init__(self, timeout=0.5, clock=time.monotonic):
        self.timeout, self.clock = timeout, clock
        self.lock = threading.Lock()
        self.received = {}
        self.values = {}

    def invalidate(self):
        with self.lock:
            self.received.clear()
            self.values.clear()

    def receive(self, axis, value):
        with self.lock:
            if not math.isfinite(value) or not -1 <= value <= 1:
                self.received.clear()
                self.values.clear()
                return False
            self.received[axis] = self.clock()
            self.values[axis] = round(value, 3)
            return True

    def gate(self, payload):
        with self.lock:
            now = self.clock()
            fresh = all(axis in self.received and 0 <= now - self.received[axis] <= self.timeout
                        for axis in ("throttle", "steering"))
            result = dict(payload)
            for axis in ("throttle", "steering"):
                result["V1 " + axis.title()] = str(self.values[axis] if fresh else 0.0)
            return result


def install_command_expiry(stock, clock=time.monotonic):
    """Install before stock.main creates subscriptions; gate at the final socket emit.

    Both axis callbacks must be fresh. Reconnect/reset invalidates earlier commands.
    Applying expiry at emit time also covers slow camera decoding in stock.bridge.
    """
    guard = CommandExpiry(clock=clock)
    for axis in ("throttle", "steering"):
        name = "callback_" + axis + "_command"
        original = getattr(stock, name)

        def callback(msg, axis=axis, original=original):
            value = float(msg.data)
            if guard.receive(axis, value):
                original(msg)

        setattr(stock, name, callback)

    reset = stock.callback_reset_command

    def reset_callback(msg):
        if msg.data:
            guard.invalidate()
        reset(msg)

    stock.callback_reset_command = reset_callback
    emit = stock.sio.emit

    def guarded_emit(event, data=None, *args, **kwargs):
        if event == "Bridge" and isinstance(data, dict) and "V1 Throttle" in data:
            data = guard.gate(data)
        return emit(event, data, *args, **kwargs)

    stock.sio.emit = guarded_emit
    connect = stock.connect

    @stock.sio.on("connect")
    def connected(sid, environ):
        guard.invalidate()
        return connect(sid, environ)

    @stock.sio.on("disconnect")
    def disconnected(sid, *args):
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
    original = stock.bridge

    @stock.sio.on("Bridge")
    def guarded_bridge(sid, data):
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
    from autodrive_roboracer import autodrive_bridge as stock

    guard = install_command_expiry(stock)
    install_startup_guard(stock, guard)
    stock.main()


if __name__ == "__main__":
    main()
