# AutoDRIVE RoboRacer — Pure Pursuit

ICRA 2026 autonomous racing submission using a LiDAR-based pure pursuit algorithm.

## Quick start

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
| `MAX_THROTTLE` | `0.6` | Top speed on straights |
| `STEER_GAIN` | `1.2` | Steering aggressiveness |
| `THROTTLE_DECAY` | `2.5` | Corner braking strength |

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
