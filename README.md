# AutoDRIVE RoboRacer — AVLite integration

Run AV-Lab's AVLite execution stack on the AutoDRIVE RoboRacer practice simulator.
The original LiDAR pure-pursuit controller is also available.

## Windows quick start

With Docker Desktop running Linux containers, run in PowerShell:

```powershell
.\run-windows.ps1
```

This downloads the official ICRA 2026 Windows practice simulator on first use,
builds AVLite, and opens the simulator. The native Windows simulator uses your
GPU; the bridge, actuator, and AVLite run in Docker. If needed, select
**Connection** (`127.0.0.1:4567`) and **Autonomous** in the simulator.
Downloads and simulator logs are stored under the ignored `log/windows/` folder.

```powershell
# Stop AVLite first, then the other services and simulator.
.\run-windows.ps1 -Stop

# Inspect controller logs.
docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml logs -f avlite actuator
```

## Shared speed and throttle settings

Edit only [config/driving.yaml](config/driving.yaml). Example values:

```yaml
speed_mps: 5.0
max_throttle: 0.2
```

`speed_mps` sets AVLite's cruise/max velocity and the actuator's speed-demand
limit together. `max_throttle` is a separate normalized throttle cap, not a speed.
The two profiles reference this file through `shared_settings: driving.yaml`
and `${...}` values, which the launchers resolve on startup.
The recorded clean-lap validation used 0.5 m/s and a 0.02 throttle cap.

After saving, reload both services (the simulator stays open):

```powershell
docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml restart avlite actuator
```

On Linux, omit `-f docker-compose.windows.yml`. Editing YAML needs no image
rebuild. When upgrading an existing installation to shared settings for the first
time, run `./run-windows.ps1` on Windows, or rebuild/recreate the services with
`docker compose -f docker-compose.avlite.yml up -d --build actuator avlite` on Linux.

## Record a run and view debugging graphs (Windows)

Start the simulator and connect it, then run this in a second PowerShell window:

```powershell
.\record-windows.ps1 -Seconds 120 -Label corner-test
```

Finish starting or restarting the simulator and controllers before recording.
The timer starts only after valid odometry arrives (up to 30 seconds of waiting;
adjust with `-WaitForOdomSeconds`). Recording stops with an explanation if odometry
then disappears for 10 seconds. If the bridge is replaced during a run, rerun the
recording command to attach to its new connection.

Reproduce the issue while the command runs. When it finishes, open `telemetry.png`
in the new dated folder under `log/recordings/`. It graphs measured speed, throttle
command/feedback, steering in radians, requested acceleration, the vehicle path,
and lap/collision/reset counters. Capture at about 10 Hz is intended for debugging
these signals; this is not a full camera/LiDAR replay recording.

Each folder also contains `telemetry.csv` for Excel, the original `telemetry.jsonl`,
a summary, copies of the configuration files, and controller logs. Samples have
both elapsed seconds and UTC timestamps. Stale readings appear as gaps in the
time-series plots. Config copies reflect the files on disk: restart the controllers
after config edits before recording so the snapshots match the running settings.

Recording only observes the run. Finishing a recording does not stop the car.
Keep the whole run folder when reporting a bug. These folders are ignored by Git.

The agreed racing and mapping milestones are saved in the
[September 21 plan](docs/plan-2026-09-21.md).

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
and odometry arrive. Speed demand and normalized throttle are capped by
`config/driving.yaml`. These are controller limits, not guaranteed physical speed
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
