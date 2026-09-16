# AutoDRIVE RoboRacer — AVLite integration

Run AV-Lab's AVLite execution stack on the AutoDRIVE RoboRacer practice simulator.
The original LiDAR pure-pursuit controller is also available.

## AVLite quick start

Requires Docker Compose, an NVIDIA GPU/container runtime, and a local X11 display.

```bash
# Stop the original controller before starting AVLite.
docker compose down
xhost +si:localuser:root
docker compose -f docker-compose.avlite.yml build
docker compose -f docker-compose.avlite.yml up -d
docker compose -f docker-compose.avlite.yml logs -f avlite actuator
```

This starts the simulator in batch mode and drives automatically once live LiDAR
and odometry arrive. The speed demand is capped at **0.5 m/s** and normalized
throttle at **0.02**. These are controller limits, not guaranteed physical speed
bounds. The independent adapter commands zero if AVLite or sensor data stops.

Stop driving first, leaving the adapter and API alive to transmit zero:

```bash
docker compose -f docker-compose.avlite.yml stop avlite
sleep 1
docker compose -f docker-compose.avlite.yml down
```

See [setup and architecture](docs/avlite-setup.md), the
[saved implementation plan](docs/avlite-integration-plan.md), and
[measured validation results](docs/avlite-validation.md).

## Original controller quick start

```bash
# Allow X11 forwarding (run once per session)
xhost local:root

# Pull images (first time only)
docker pull autodriveecosystem/autodrive_roboracer_sim:2026-icra-practice
docker pull autodriveecosystem/autodrive_roboracer_api:2026-icra-practice

# Start both containers
docker compose up
```

In the simulator GUI: leave IP as `127.0.0.1` and port `4567`, click **Connection**, then click **Driving Mode → Autonomous**.

## Development workflow

```bash
# Open interactive shells
docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm devkit

# Inside the container — build and run your package
colcon build --packages-select my_team_racer
source install/setup.bash
ros2 launch my_team_racer racer.launch.py
```

Source files on your host under `src/` are mounted at `/home/autodrive_devkit/src/my_packages` — edits are visible immediately.

## Algorithm

`racer_node.py` implements a wall-following pure pursuit controller:

1. Parse the 270° LaserScan into cartesian points
2. Separate left / right wall points in the forward hemisphere
3. Compute lateral offset from the track centerline
4. Project a lookahead point `LOOKAHEAD_DIST` metres ahead along the centerline
5. Steer toward that point using pure-pursuit geometry
6. Scale throttle inversely with steering magnitude (slow in corners)

Key tuning constants at the top of [racer_node.py](src/my_team_racer/my_team_racer/racer_node.py):

| Constant | Default | Effect |
|---|---|---|
| `LOOKAHEAD_DIST` | `0.8 m` | Longer = smoother but lazier |
| `MAX_THROTTLE` | `0.4` | Throttle ceiling on straights |
| `WHEELBASE` | `0.32 m` | Vehicle length used in steering geometry |
| `THROTTLE_DECAY` | `3.0` | Throttle reduction in corners |

## ROS 2 topics

| Topic | Direction | Type |
|---|---|---|
| `/autodrive/roboracer_1/lidar` | Subscribe | `sensor_msgs/LaserScan` |
| `/autodrive/roboracer_1/throttle_command` | Publish | `std_msgs/Float32` |
| `/autodrive/roboracer_1/steering_command` | Publish | `std_msgs/Float32` |

## Competition submission

```bash
# Commit your devkit container as a new image
docker commit -m "racing algorithm" autodrive_roboracer_api <dockerhub_user>/<repo>:2026-icra-practice
docker push <dockerhub_user>/<repo>:2026-icra-practice
```

The `docker/autodrive_devkit.sh` script is the entrypoint for your submitted image — it builds and launches the racer automatically without touching `~/.bashrc`.
