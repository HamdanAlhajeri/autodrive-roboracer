# Driving with AVLite's built-in race planner

The AutoDRIVE integration supports two modes in `config/avlite.yaml`:
`follow_the_gap` (default) and `planned`. The installed AVLite revision stays
pinned. Planned mode uses `GlobalRacePlanner`, `ReferencePathPlanner`, and
AVLite's Pure Pursuit steering/velocity control, with our small-car and periodic
path adapters. The map is a reference-assisted corridor, not an occupancy map
or SLAM result. Localization still uses simulator ground truth.

## First run on the practice track

The supplied `config/maps/practice.json` was generated from the clean recording
`20260917-113015-651-3-laps`. Its frame is the simulator's existing `world` frame
and its origin is unchanged. It is only applicable to that practice-track layout.

1. In `config/driving.yaml`, use `speed_mps: 2.5` and `max_throttle: 0.2` for
   commissioning. Both planner and controller read the shared speed; the actuator
   continues to read the same file.
2. In `config/avlite.yaml`, set `driving_mode: planned`. Keep
   `planning.map_path: maps/practice.json` and `braking_calibrated: false`.
3. Start/connect the native simulator with Connection and Autonomous selected.
   From the repository directory, prepare the controller without driving yet:

   ```powershell
   docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml up --no-start --no-deps avlite
   .\record-one-lap.ps1 -Laps 3 -Label planned-2p5 -NoOpen
   ```

   The recording script stops AVLite, reloads the actuator, prompts for a reset,
   records stationary telemetry, then starts driving. It stops AVLite after the
   requested laps, a collision/reset, or the time limit.

Planned mode has an additional **2.5 m/s commissioning ceiling** until braking
calibration is explicitly enabled. This is visible as limiting reason 5 in the
telemetry. A shared ceiling of 20 m/s therefore does not bypass commissioning.
The initial acceleration, lateral acceleration, and braking assumptions are
1.0, 3.0, and 1.5 m/s² respectively. They are not measured vehicle guarantees.

## Prepare a map for another recording or track

Record at least two finish crossings at a low, repeatable speed, with no collisions
or resets and the default LiDAR outline capture enabled. Three laps are convenient:

```powershell
.\record-one-lap.ps1 -Laps 3 -Label mapping -NoOpen
.\prepare-race-map.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN -Name practice-v2
```

The offline command generates `config/maps/practice-v2.json`, `.plan.json`, and
`.png`. It does not send commands to the simulator. Use a new name to preserve
existing maps. Inspect the overlay, then set `planning.map_path` to
`maps/practice-v2.json`. Restart the controller after configuration/map changes;
there is no hot reload in this runner.

The converter selects a full lap between finish crossings, orders cross-sections
using the driven path, and fits left/right boundaries at 0.1 m spacing. It smooths
receive-time noise on a 0.15 m scale. It rejects discontinuous/stale poses, missing
boundaries, observation gaps over 0.5 m, crossings, and inadequate width. It is
intended for a single small-car corridor with walls observed within 2.5 m on each
side; ambiguous junctions and opponent avoidance are outside this version.

Planner output is independently checked for steering curvature, speed,
acceleration, and braking constraints, including the closing segment. The
rear-to-front-axle capsule uses wheelbase 0.324 m, radius 0.24 m, and tracking
allowance 0.05 m. The whole capsule must remain in the map at 0.025 m validation
intervals, and the route is checked against original LiDAR hits. Optimizer
boundary offsets alone are not accepted as proof of clearance.

Planning runs before commands are enabled. A missing map or invalid plan aborts
startup; the independent actuator watchdog remains responsible for expiring
commands. During driving, missing/stale sensors, an invalid pose, or a heading
opposite to the route cause a deceleration request. Progress and PID state are
cleared after resets or sensor interruptions. Lookahead wraps around the lap.

## Speed, braking, and obstacle checks

The global profile slows before corners using lateral and longitudinal limits.
The controller previews the profile over the reaction interval, then applies a
LiDAR stopping limit along the curved route and the initial commanded steering
arc. Range-max/no-return beams are excluded from obstacle hits; corrupt beams
remain close obstacles. No-return beams do not create a wall at sensor range.
This relies on the validated, static mapped corridor; it is not a guarantee of
stopping for a newly appearing obstacle beyond visibility or a dynamic opponent.

Negative acceleration currently removes forward throttle. Measure the resulting
speed reduction rather than assuming a brake force. Analyze recorded straight
throttle-off intervals with:

```powershell
.\analyze-braking.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN
```

The report requires at least three independent straight coast intervals, each
at least 0.3 seconds, using fresh speed, throttle command/feedback, and steering
feedback. It excludes turns, powered motion, stale data, resets, and poorly fitted
intervals. The suggested braking parameter is 80% of the smallest fitted
deceleration. The tool never changes settings. Insufficient evidence leaves
calibration pending. Collect controlled coast data on a suitable clear straight;
ordinary lap recordings may not contain qualifying intervals.

After inspecting the evidence, set `planning.braking_deceleration_mps2` no higher
than the measured conservative value and `braking_calibrated: true`. Increase the
shared speed in 0.5 m/s increments, requiring three clean laps per increment.
Recheck response as speed increases. Keep the throttle cap at 0.2 unless recorded
saturation demonstrates a need to change it. A speed ceiling cannot guarantee
that track length, grip, or coasting response permits that speed.

## Recording and rollback

The running controller publishes the exact plan on a latched ROS topic. Every
recording, including one started after the controller, captures:

- `telemetry.plan.json`: path, speed profile, map hash, map, and resolved settings.
- `telemetry.racemap.json` and `telemetry.planning-config.json`: independent copies.
- `telemetry.planned.png`: planned/driven paths, speed against lap distance,
  tracking error, and the active limiting reason.

Existing CSV, JSONL, lap graphs, controller graphs, and command topics remain
compatible. New numeric diagnostics are `planned_speed_mps`, `path_progress_m`,
`path_deviation_m`, `obstacle_speed_limit_mps`, `speed_limit_reason`, `planned_mode`,
and `plan_valid`. Limiting reasons are 0 speed ceiling, 1 profile/corner/braking,
2 obstacle, 3 invalid pose, 4 missing/stale scan, 5 commissioning, 6 invalid plan.
Unknown/no observed obstacle distance is null, not an invented finite distance.

To roll back, stop AVLite, set `driving_mode: follow_the_gap`, and use the desired
shared driving settings before restarting. The original reactive controller and
its clearance limits remain available. Do not launch a second controller beside
the existing one.

Validation retains the milestone requirement: three fresh runs of ten clean laps,
with measured straight-line speed, lap variation, timing, and sensor interruptions
reported separately. Automated tests and a three-lap screen do not complete that
acceptance requirement.

The [first commissioning screen](validation/planned-commissioning.md) passed three
clean laps at a measured peak of 2.5022 m/s, with rolling laps of 12.609 and
12.617 seconds. Braking calibration and higher-speed acceptance remain pending.
