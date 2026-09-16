"""Start the stock AutoDRIVE API with a guard for incomplete startup packets."""

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


def install_startup_guard(stock):
    original = stock.bridge

    @stock.sio.on("Bridge")
    def guarded_bridge(sid, data):
        if not isinstance(data, dict) or not REQUIRED_FIELDS.issubset(data):
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

    install_startup_guard(stock)
    stock.main()


if __name__ == "__main__":
    main()
