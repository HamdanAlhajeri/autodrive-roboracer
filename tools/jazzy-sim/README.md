# Jazzy test harness (no car needed)

Tests the Jetson stack on the car's software (Ubuntu 24.04, ROS 2 Jazzy, Python 3.12)
in Docker, with a simulated F1TENTH car in place of the hardware.

| File | Purpose |
| --- | --- |
| `Dockerfile` | `ros:jazzy-ros-base` plus a `fasty` user with sudo, like the Jetson |
| `fake_car.py` | Simulated car: UST-10LX-like `/scan`, drifting `/odom`, F1TENTH frames. `--mode scripted` drives itself (a manual lap); `--mode drive` obeys `/drive` |
| `pipeline.sh` | Records two laps, runs the sensor report, maps lap 1 and localizes lap 2 through SLAM Toolbox, with ground-truth error |
| `closed_loop.sh` | Launches the supervisor and actuator, then `map-start` → READY → `race-start` → `stop` |

Both scripts use the unit tests' commissioned profile, never real measurements.

## Run (from the repository root, Git Bash on Windows)

```bash
docker build -t roboracer-jazzy-test tools/jazzy-sim
git -c core.autocrlf=false archive --format=tar --prefix=autodrive-roboracer/ HEAD \
  | docker run -i --name rr-jazzy roboracer-jazzy-test bash -c \
    'mkdir -p ~/Desktop && tar -x -C ~/Desktop && cd ~/Desktop/autodrive-roboracer && bash tools/jetson-setup.sh'
docker commit rr-jazzy roboracer-jazzy-installed && docker rm rr-jazzy
MSYS_NO_PATHCONV=1 docker run --rm -v "$(cygpath -wa tools/jazzy-sim):/work" \
  -v "$(cygpath -wa src):/repo-src:ro" roboracer-jazzy-installed bash /work/pipeline.sh
```

Use `closed_loop.sh` in place of `pipeline.sh` for the autonomous flow. Results land in
`tools/jazzy-sim/out/` and `tools/jazzy-sim/closed/` (git-ignored).

## Results so far (10 November 2026)

- `jetson-setup.sh` completes on Jazzy (x86_64). The arm64 run was interrupted.
- Full test suite on Jazzy with `RUN_ROS_TESTS=1`: 470 passed.
- Recording and export work; SLAM Toolbox on Jazzy publishes `/slam_toolbox/graph_visualization`,
  `/map` and `/pose`.
- Not yet completed: mapping/localization replay, `closed_loop.sh`, arm64 install.
- Under CPU load the simulator stalls, and the sensor report correctly flags the scan and
  odometry gaps. Close other heavy programs before running.
