# Jetson agent report, 3 November 2026

Followed `docs/plans/jetson-agent-brief.md` on the car (Jetson Orin NX, Ubuntu 24.04,
ROS 2 Jazzy, Python 3.12). No motion commands were sent and no driver was started.
`config/roboracer/hardware.yaml` was not touched.

## 1. Driver packages

| Package | Status |
| --- | --- |
| `urg_node`, `urg_c`, `urg_node_msgs` | Installed (apt, `ros-jazzy-*`) |
| `joy`, `teleop_twist_joy` | Installed (apt) |
| `slam_toolbox`, `ackermann_msgs` | Installed by `tools/jetson-setup.sh` |
| `urg_node2` | **Missing.** Not in the Jazzy apt index. Needs a source build. |
| `vesc_driver` (and `vesc_ackermann`) | **Missing.** Not in the Jazzy apt index. Needs a source build (f1tenth/vesc, `ros2` branch). |
| `ackermann_mux` | **Missing.** Not in the Jazzy apt index. Needs a source build. `ros-jazzy-twist-mux` is available but handles Twist, not Ackermann. |

F1TENTH workspace: **none found.** The only colcon workspace under `~` (depth 3) is
`~/Desktop/SDP/install`. It holds `detection_and_avoidance` and is a separate
project that uses an Arduino RC bridge, not the F1TENTH stack.

Suggested driver plan (to run later, with your OK): create `~/f1tenth_ws`, clone
`f1tenth/vesc` (ros2 branch), `Hokuyo-aist/urg_node2` and `f1tenth/ackermann_mux`,
then `rosdep install` and `colcon build`. Check that each builds on Jazzy, because
upstream F1TENTH targets Humble. **Do not start `vesc_driver`** until the wheels are
off the ground.

## 2. Network and devices

| Interface | State | Address |
| --- | --- | --- |
| `wlP1p1s0` (Wi-Fi) | UP | 10.10.180.198/20 |
| `enP8p1s0` (wired Ethernet) | **DOWN** | none |
| `l4tbr0` (USB device-mode bridge) | DOWN | 192.168.55.1/24 |
| `usb0`, `usb1` | DOWN | none |
| `can0` | DOWN | none |
| `docker0` | DOWN | 172.17.0.1/16 |

- **LiDAR: not reachable.** The wired port has no link and no `192.168.0.x` address.
  Model still unknown.
- **VESC: not detected right now.** There is no `/dev/ttyACM*` and no STMicro device in
  `lsusb`. It was seen earlier, so it is probably unplugged or powered off.
- Logitech F710 gamepad: detected.

## 3. Setup script

`bash tools/jetson-setup.sh` **succeeded** (exit 0). It already supports Jazzy (commit
`1e225b6`), so no script change was needed.

- The venv is at `/opt/avlite-venv`. The pinned AVLite revision and
  `docker/avlite-constraints.txt` installed on Python 3.12. `evdev` built from source.
- `avlite_autodrive` and `avlite_roboracer` built in `~/roboracer_ws`.
- Warning: `urg_node2 vesc_driver ackermann_mux` were not found (expected, see above).
- Note: the venv installs numpy 2.2.6 and matplotlib 3.10.9, which shadow the
  system numpy 1.26.4 and matplotlib 3.6.3. Pytest warns "Unable to import Axes3D"
  because of the duplicate matplotlib. Jazzy's apt Python packages (for example
  `cv_bridge`) are built against numpy 1.x. Watch for ABI errors if a node imports
  them inside the venv.

## 4. Tests

The brief's command
(`/opt/avlite-venv/bin/python -m pytest src/avlite_roboracer/test src/avlite_autodrive/test -q`):
**449 passed, 21 skipped, 0 failed.** All 21 skips are the opt-in ROS tests
(`RUN_ROS_TESTS=1`).

Extra run with `RUN_ROS_TESTS=1`, isolated (`ROS_DOMAIN_ID=87`, localhost-only
discovery, no drivers running, VESC unplugged): **465 passed, 5 failed.**
`test_runtime_ros.py` (the roboracer supervisor smoke test) **passes on Jazzy**.

The 5 failures:

- `test_ros_integration.py::test_stack_moves_and_watchdog_stops_after_avlite_is_killed`
  and `test_response_ros.py::test_response_runner_records_runtime_profile_and_coast_diagnostics`:
  `FileNotFoundError: /config/avlite.yaml`. These tests expect the Docker AVLite
  image layout, so this is not a Jazzy problem.
- `test_recording_ros.py`: 3 failures (`delayed_valid_odometry_starts_full_recording_duration`,
  `missing_counter_stream_has_startup_grace_then_fails`,
  `motion_before_counter_discovery_cannot_pass`). They fail with
  `Recorder never captured both counter baselines`, `KeyError: 'invalid_odom_samples'`
  and `assert 0.0 >= 1`. These may be Jazzy discovery or timing differences. Not
  fixed; this needs a closer look.

## 5. Files changed

- `src/avlite_roboracer/test/test_runtime_ros.py`: the skip reason said "requires ROS
  Humble runtime", but the gate is `RUN_ROS_TESTS=1` and the test passes on Jazzy.
  Reworded to "requires ROS 2 runtime (Humble or Jazzy)".
- `tools/inspect-jetson.sh`: executable bit set (this change was already in the
  working tree; committed as is).
- This report (force-added, because `/log/` is gitignored).

Outside the repo, the setup script installed apt packages, created
`/opt/avlite-venv` and created `~/roboracer_ws`.

## 6. Open questions

1. **Which LiDAR is on the car?** (Still unknown.) If it is a Hokuyo UST, connect it
   to `enP8p1s0` and give that port a static `192.168.0.x/24` address.
2. **Emergency stop:** what is the physical or remote e-stop? Is the F710 the only
   stop path?
3. **Driver install:** OK to source-build the F1TENTH drivers (`vesc`, `urg_node2`,
   `ackermann_mux`) in `~/f1tenth_ws` for Jazzy?
4. Should the 3 `test_recording_ros.py` failures under Jazzy be investigated now?
