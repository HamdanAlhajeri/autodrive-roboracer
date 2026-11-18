# Heads-up for the next agent

Read this first. It replaces learning the whole repo. Last updated 18 Nov 2026 (overnight session).

## What this project is

A RoboRacer/F1TENTH-style race stack. Two halves share one planner:

1. **Simulator half** (`src/avlite_autodrive`): AutoDRIVE simulator on Windows, AVLite planner
   (GlobalRacePlanner + Pure Pursuit), driven through ROS 2 Humble in Docker. Speed tuning is done.
2. **Real-car half** (`src/avlite_roboracer`): supervisor + actuator for a Jetson Orin car
   (Hokuyo LiDAR, VESC/F1TENTH). It maps one lap, builds a race map, localizes against it, then races.
   **Never run on the real car yet.**

## State of play

| Area | State |
| --- | --- |
| Speed ceiling in the sim | Solved (four hidden causes, see `README.md`). Best settings are in `config/avlite.yaml`. |
| Localization in the sim | AVLite ICP finishes 3 clean laps (~10 cm typical error). SLAM Toolbox localization slides ~1 m and stays wrong; cause unknown. |
| Car runtime (supervisor) | Works end to end on a simulated F1TENTH car: map, finalize, ICP localization, READY, race. |
| Real car | Not tested. Needs measurements, F1TENTH drivers for the Jetson's ROS, and test laps. |

Branch is `sim-slam-localization`. **Nothing from the overnight session is committed.** The owner said
"don't commit yet" and wants no Claude attribution in commits or PRs (see the memory note). Ask before committing.

## Do not redo these (already solved)

- Pose age is **monotonic** (not wall clock): Docker/Jetson clock steps used to fault "mapping pose stale".
- `lap_route` stops at the first return to the start: a 3-lap mapping run used to make boundaries cross.
- Boundary smoothing escalates (`smoothing_sigmas`) until the rings are simple.
- A SLAM node we just stopped stays in the ROS graph for seconds; `stop_slam()` remembers it so READY
  does not fail with "found 2 localization sources".
- ICP is guarded: a step that fits the map worse than the odometry prediction is undone; a lost pose
  triggers a grid recovery search (`IcpPose`, settings in `config/roboracer/race.yaml`).

## Where things are

| Need | Look at |
| --- | --- |
| ICP pose for the car | `src/avlite_roboracer/avlite_roboracer/icp_pose.py` (wraps `avlite_autodrive/avlite_localization.py`) |
| Supervisor, finalize, status | `avlite_roboracer/runtime.py` (CRLF file) |
| Map builder, boundaries | `avlite_roboracer/map_builder.py` |
| Hardware profile (all nulls = not commissioned) | `config/roboracer/hardware.yaml`, `profile.py` |
| What is missing on the car | `bash tools/car-day.sh` or `python -m avlite_roboracer.doctor` |
| Car setup docs | `docs/jetson-quickstart.md`, `docs/jetson.md` |
| Overnight report | `local-reports/` (git-ignored) and `docs/validation/sim-localization-20261117/` |

## How to test (no car, no simulator window needed)

All from Git Bash at the repo root. Docker Desktop must be running. Image: `autodrive-roboracer-avlite:local`.

```bash
bash tools/jazzy-sim/unit.sh                      # both packages' unit tests, ~1 min
RACE_SECONDS=150 bash tools/jazzy-sim/run_humble.sh my-run   # full map -> ICP -> race flow, ~8 min
python tools/jazzy-sim/analyze_closed.py tools/jazzy-sim/my-run --png tools/jazzy-sim/my-run/icp.png
CAR_ARGS="--odom-scale 1.1 --dropout 0.25" bash tools/jazzy-sim/run_humble.sh stress   # stress the sensors
```

**Do not edit anything under `tools/jazzy-sim/` or `src/` while a run is going**: bash reads the script
as it runs, and an edit corrupted one run already. The run copies the source at its start.

The visible simulator needs a human to click Connection and Autonomous: `tools/sim-localization/TODO.md`
has the older procedure (its branch name is stale; the branch is `sim-slam-localization`).

## Gotchas learned the hard way

- Many files are CRLF. Edit with a CRLF-aware script, not a heredoc with backslashes.
- Git Bash: use `MSYS_NO_PATHCONV=1` for docker paths. PowerShell 5.1 treats docker stderr as an error when redirected.
- The Docker VM stalls for 0.3-4 s and its clock steps. The harness uses looser timeouts; the real profile keeps tight ones.
- `python3` is not on the laptop's PATH; use `python`.
- Simulator tuning values (gains, cornering limits) must **not** be copied to the real car.

## Open questions, in value order

1. Why does SLAM Toolbox localization slide in the simulator? (ICP is the car's default, so this is not blocking.)
2. Real-car commissioning: measurements, stop tests, the F1TENTH Jazzy drivers.
3. Odometry speed scale on the real VESC (the stress runs show how much error ICP tolerates).
4. Rewrite README commit-hash links: they are stale after the history rewrite.

## The real-car day, short version

1. On the Jetson: `bash tools/jetson-setup.sh`, then `bash tools/car-day.sh` and fix every `[FAIL]`.
2. Start the F1TENTH bringup, run `car-day.sh` again: it checks `/scan`, `/odom`, `/drive`, `/joy`.
3. Fill `config/roboracer/hardware.yaml` using the list `car-day.sh` prints (each item says how to measure it).
4. Follow `docs/jetson-quickstart.md` steps 2-8. Nothing moves autonomously until the profile is complete.
5. First autonomous speed cap is 1.0 m/s. Raise only on measured evidence.

## Overnight findings (18 Nov 2026), details in `docs/validation/overnight-20261118/README.md`

- ICP while racing, 150 s: about 0.14 m typical error, 0.35 m worst, no drift. With odometry 10% off plus a yaw bias
  and a noisy LiDAR it is about 0.35 m typical, 0.85 m worst, still no loss of track.
- SLAM Toolbox as localizer fails in the harness too (never READY, alignment 0.55). Do not spend more time there.
- SLAM mapping pose slides up to 2.8 m on a long straight at the default `distance_variance_penalty: 0.5`.
  Lower values fix that with accurate odometry but break with bad odometry (0.05 gave 17 m error).
  Default kept. After the real odometry error is measured, retest 0.2 on the car.
- A beam missing in one scan only now counts as half a disagreement in the lap check (LiDAR dropout used to
  block lap confirmation). The "found 2 localization sources" READY failure is fixed (`stop_slam`).
- Run data is in `tools/jazzy-sim/` (git-ignored): `final-icp`, `stress-*`, `mid-*`, `tune-*`.
  Pose traces: `<run>/data/sessions/*/icp_trace.csv` (racing) and `slam_trace.csv` (mapping).
- Report with charts: `local-reports/Overnight-Localization-and-Mapping-Report.docx` (git-ignored).
