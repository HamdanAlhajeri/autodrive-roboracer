# Jetson car day, 23 November 2026

First day the software ran on the real car (Jetson Orin NX, ROS 2 Jazzy, VESC, Hokuyo UST, F710 gamepad).
This file collects what was measured and what happened. Raw recordings are not committed (they are large and
git-ignored under `log/`); small outputs, maps, logs and the final profile are in the subfolders here.

## Result in one paragraph

The profile is commissioned (23 of 23). The car drove, steered and stopped under joystick control, the motor
direction and steering centre were fixed, ICP localization works against a saved map in replay, and the car
**drove itself for the first time under the supervisor** (83 s, about 20 m of SLAM path, no faults). The
autonomous mapping lap was **not completed**: the run was stopped when the dead-man button (B) was released, before
the lap was confirmed. No race map was built, so the localized autonomous race was **not run**.

## Final state of the car (end of day)

| Item | Value |
| --- | --- |
| Repo on the Jetson | `main` at `2e4a924` (clean pull; local edit is only the five measured values in `config/roboracer/hardware.yaml`) |
| Hardware profile | 23 of 23, commissioned (`data/hardware-profile-final-23-of-23.yaml`) |
| Dead-man | button 1 (B), `deadman.stop_on_press: false` (hold to run) |
| `vesc.yaml` (tracked in `config/f1tenth/vesc.yaml`) | `speed_to_erpm_gain: -4614.0`, `steering_angle_to_servo_offset: 0.420`, `vesc_to_odom wheelbase: 0.314` |
| Speed limits | motor clamp 1.0 m/s, mapping 0.6 m/s, racing 1.0 m/s |
| Steering limit | `steering.max_angle_rad` 0.22 (derived from the servo clip, **not** a protractor measurement) |

Profile values written during the day (all through `profile_edit`): laser mount 0.27 / 0 / 0 (read from the
bringup transform, unconfirmed by tape), wheelbase 0.314, width 0.295, length 0.553, rear axle to front 0.44
(owner estimates), the five `localization.*` starting limits (simulator values, not measurements), and the five
stop-test values below.

## Hardware fixes found on the car

1. **Motor direction was reversed.** A forward command drove the car backward and odometry agreed with the command,
   so it read "forward" while the car went back. Fixed by negating `speed_to_erpm_gain` (command and odometry flip
   together). Verified: car moves forward, odometry `vx` positive in 1625 of 1625 moving samples.
2. **Steering centre was off by about 5 degrees.** The servo value for straight ahead is 0.420 (found with
   `servo-calibration/servo_center.py`), the stock offset was 0.5304.
3. **Odometry wheelbase** in `vesc_to_odom_node` was the stock 0.25 m; set to the measured 0.314 m.
4. **USB hub dropouts.** The VESC and the gamepad receiver share one hub (ports 1-2.3 and 1-2.4). The VESC dropped off
   the bus on 19 Nov (several times) and on 23 Nov at 18:32:59 and 18:51:39 (`data/usb-events-23-nov.txt`). Not
   fixed: move one of them to a separate Jetson USB port. A flat or loose supply is also possible; unconfirmed.
5. The wireless gamepad goes to sleep (`/joy` silent); a button press wakes it.

## Stop tests (commissioning)

| | First attempt | Final (applied) |
| --- | --- | --- |
| Recording | `record-20261123-174215` | `record-20261123-175543` |
| command_latency_s | 4.37 (stick chatter outlier, **not applied**) | **0.230** |
| braking_deceleration_mps2 | 1.40 | **1.823** |
| stopping_distance_m | 0.27 | **0.262** |
| stop_test_speed_mps | 1.13 | **1.183** |
| max_acceleration_mps2 | 3.89 | **1.552** |
| clean events | 14 | 4 stops, 2 starts from rest |

The first result was rejected because the driver pushed the stick again before the car had stopped, which set the
worst case. The laptop session changed the tool to skip interrupted stops (commit `f496852`) and the same recording
gave the consistent set above. Caveat: only 4 clean stops and 2 clean starts back these numbers. Clean stops from
0.75 to 1.18 m/s: 0.43 to 0.56 s, 0.17 to 0.26 m, 1.8 to 2.1 m/s2. The car **brakes** when the command goes to 0 (it
does not coast). Data: `data/stops-final-applied.json`, `data/stops-first-test-latency-4.37s-not-applied.json`.

## Maps (manual drives with live SLAM Toolbox; no race map exists)

| File | What it is |
| --- | --- |
| `maps/track2-1123-1717.{pgm,yaml}` + `.png` | Closed ring, about 8.9 x 5.9 m, resolution 0.05, clean single-line walls. Best map. |
| `maps/track1-first-drive-open-top-17-14.png` | Same track, first drive, ring closed but leaks through an open area at the top. |
| `maps/replay-10-nov-partial.png` | Replay map from 10 Nov: scattered walls, no closed loop. |

A university hallway drive on 19 Nov (open sides and glass) gave doubled walls and is not a usable map. The ring is
only about 1.3 m wide.

## Localization checks (replay, no motors)

`python3 -m avlite_roboracer.icp_replay` against `maps/track2-1123-1717.yaml` (commit `93eb853`):

| Recording | alignment mean / p5 / min | lost recoveries | pose jumps over 0.5 m | odom/ICP net turn |
| --- | --- | --- | --- | --- |
| `record-20261123-171554` (built the map) | 0.983 / 0.937 / 0.756 | 0 | 0 | 0.693 |
| `record-20261123-171252` (other drive) | 0.903 / 0.830 / 0.347 | 1 (3 tries) | 0 | 0.683 |

The start-pose search was unambiguous in both runs. **Odometry yaw reads about 31% short** on both drives (and about
8% short in distance against SLAM). `vesc_to_odom_node` takes yaw from the commanded servo with the stock steering
gain (-1.2135), which has not been calibrated for this car. Suspected, unverified: the steering gain is also off,
so planner steering angles may not match the real wheel angles. Steering calibration with a protractor was dropped
by the owner today and remains open. Data: `data/icp-replay-*.json`.

## Autonomous mapping: what happened

All runs were `tools/car-run.sh --map-only --yes` (mapping at 0.6 m/s, Follow the Gap). Logs in `runs/`.

| Run (folder time) | Outcome | Cause / finding |
| --- | --- | --- |
| 180426 | Stopped at preflight, nothing sent | `--start-stack` runs its preflight before starting the bringup (fixed upstream in `3b4c7de`). Used the manual route: bringup, `jetson-run.sh launch`, then `car-run.sh --map-only`. |
| 180556, 181249 | Stopped in 1 to 4 s, stale LiDAR | **Root cause: the Hokuyo stamps scans about 1 ms ahead of the Jetson clock and the supervisor rejected any future stamp: 96% of scans dropped.** Fixed in `bb7dcee` (50 ms tolerance, 0.25 s limit unchanged). |
| 181007 | Dead-man not held in 60 s | Gamepad asleep, B not pressed in time. |
| 182229 | Dead-man released | Diagnostics added in `a1a6698` (button state, `/joy` age). |
| 182735 | Stale scan AND odom 0.26 s old, scans seen 134, rejected 0 | A supervisor stall; did not recur in later runs. Diagnostics added in `f6a568e` (per-phase tick timing, sampler, `timing-report.txt`). |
| 184107 | Dead-man released at first motion | SLAM first pose arrives about 3.4 s after start; the car moved 1.4 cm, then B was released. Supervisor healthy: 83 ticks, longest 28 ms, 2% CPU. |
| 184238 | Fails at once | **Bug: a second mapping attempt on the same supervisor faulted before its setup ran** (`lap_started` left over; `check_faults` ran before `start_mapping`). Fixed in `185bcb4`. Workaround used: restart the supervisor before each attempt. |
| 184454 | Dead-man not held in 60 s | Operator not ready. |
| **184739** | **Drove itself 83 s, about 20 m, stopped by B release** | No faults. Mean tick 16 ms, longest 23 ms, longest gap 136 ms. SLAM path 19.9 m, mean 0.24 m/s, end 1.64 m from start. Lap not confirmed. `data/timing-report-run-184739.txt`, `data/slam-trace-run-184739-decimated.csv`. |

SLAM pose jumps in run 184739: four steps above 0.1 m (largest 0.25 m at row 2309, t+57.8 s, near the far end of the
track at about (1.9, 7.3), heading 165 to 176 degrees, 9.7 degree yaw change). All four came with 10 to 16 degree yaw
changes during turns, consistent with the short odometry yaw above. No change was made for them.

## Not done / not verified

- No complete autonomous mapping lap, so no validated race map, no race stage, and no ICP localization **under
  autonomous driving** (only in replay).
- The stationary safety checks were not done as formal steps: dead-man release stops the car (observed in practice),
  but killing the bringup, unplugging the network, and the manual-override behaviour were not tested.
- `deadman.stop_on_press` (commit `2e4a924`, off) was not tried. If it is ever enabled, first verify with the wheels
  off the ground that a sleeping or unplugged gamepad stops the car, because the link is the only fail-safe there.
- Steering gain, rear-axle and laser-mount distances, and odometry scale were never measured with a tape or protractor.
- The five `localization.*` thresholds are simulator starting values, not measurements.

## To resume

1. Plug the VESC and the gamepad receiver into **separate** Jetson USB ports; check `/dev/ttyACM0` exists.
2. `git pull` (stash the local `hardware.yaml` edit first if needed), then restart the bringup and both supervisor
   processes (`ros2 launch f1tenth_stack bringup_launch.py`, then `bash tools/jetson-run.sh launch`).
3. `bash tools/car-run.sh --map-only --yes` with the operator holding B (or on a fresh supervisor), emergency stop in
   reach. After a confirmed lap the supervisor builds the race map; then `--race-only` with ICP localization.
4. Then the steering and odometry calibration (a tape-measured straight and a protractor on the wheels).

## Commits from the laptop session used today

`f496852` commissioning ignores interrupted stops; `f66c270` race config steering and speed limits; `93eb853`
`icp_replay`; `3b4c7de` start-up fixes and `car-run.sh` preflight order; `bb7dcee` scan-stamp tolerance; `a1a6698`
dead-man diagnostics; `f6a568e` timing diagnostics; `185bcb4` second-attempt fix; `2e4a924` optional stop-on-press.
