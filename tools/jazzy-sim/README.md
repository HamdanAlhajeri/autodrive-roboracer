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

## Findings, 17 November 2026 (Humble image, `closed_loop.sh`)

`closed_loop.sh` now runs on Humble too (`ROS_SETUP`, `WS_SETUP`, `REPO`, `OUT`, `SLAM_OVERRIDES`
override its defaults) and saves the supervisor data and a transform-age log (`tf_age.py`).
It had never completed `map-start`. Three causes, found in order:

1. **Wall-clock steps stopped the mapping lap.** The Docker Desktop VM steps its clock back by
   0.1-0.5 s now and then; the supervisor measured pose age against the wall clock and faulted
   "mapping pose stale" on one negative reading. Pose age is now monotonic (time since a new
   transform arrived), which also protects a Jetson from NTP steps. The fault message now names
   which pose is stale and by how much.
2. **The VM stalls for 0.3-4 s** now and then, which trips the 0.25 s sensor limit. The harness
   profile uses 1.5 s for sensor and pose timeouts; the real profile is unchanged.
3. **The mapping controller deadlocks at the fake track's hairpin.** Follow the Gap stops at
   about 18.7 m of travel and commands zero speed, with every health check green. Not yet fixed:
   widen the fake track or retune `c35_bubble_radius` / `c35_min_gap_width` for it. This blocks
   the end-to-end run to READY, so the ICP stage has not run in this harness yet.

## Harness commands (18 November 2026)

| Command | Purpose |
| --- | --- |
| `bash tools/jazzy-sim/run_humble.sh [name]` | Full map, finalize, ICP, READY, race flow on the working tree (about 8 min) |
| `python tools/jazzy-sim/analyze_closed.py tools/jazzy-sim/<name> [--mapping] [--png f.png]` | Score the supervisor's pose trace against the fake car's true path |
| `bash tools/jazzy-sim/unit.sh` | Unit tests for both packages in the pinned image (about 1 min) |

Environment for `run_humble.sh`: `CAR_ARGS` (fake-car stress, e.g. `--odom-scale 1.1 --yaw-bias 0.02 --scan-noise 0.03
--dropout 0.25`), `SLAM_OVERRIDES` (JSON patch to slam_toolbox.yaml), `RACE_OVERRIDES` (JSON merged into race.yaml),
`RACE_SECONDS`, `MAP_WAIT_S`. Do not edit files under `tools/jazzy-sim/` or `src/` while a run is going.
