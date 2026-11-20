# IROS 2026 track: speed runs in the simulator

20 November 2026. Four recorded runs on the IROS 2026 track, simulator only, AVLite ICP localization with the
wheel-odometry speed scale at 1.075 (see `docs/validation/iros2026-localization-drift-20261120`). Each run is
240 s, recorded with `tools/sim-localization/measure_pose.py --scans 5` and screen-captured as video. Runs 3 and
4 used `--auto-reset` (a car stopped for 3 s is respawned). The raw recordings and videos are in `log/` and are
not committed.

## Settings per run

| Run | speed_mps | max_throttle | Acceleration / braking (m/s^2) | braking_calibrated |
| --- | --- | --- | --- | --- |
| 1 | 2.5 | 0.2 | 1.0 / 1.5 | false |
| 2 | 2.5 | 0.35 | 1.0 / 1.5 | false |
| 3 | 7.0 | 0.6 | 1.0 / 1.5 | true (override, not calibrated) |
| 4 | 7.0 | 0.6 | 3.0 / 3.5 (actuator maximum) | true (override, not calibrated) |

Runs 3 and 4 bypass the planner's 2.5 m/s commissioning cap. Braking was not measured on this scene, so these are
exploration runs, not validated settings.

## Results

| Run | Lap time (s) | Mean / top speed (m/s) | Collisions / respawns | "Localization lost" | Error mean / median / p95 / max (m) |
| --- | --- | --- | --- | --- | --- |
| 1 | about 112 | 1.81 / 2.49 | 0 / 0 | 0 | 0.18 / 0.10 / 0.71 / 1.21 |
| 2 | about 118 | 1.75 / 2.49 | 0 / 0 | 0 | 0.24 / 0.12 / 0.96 / 1.39 |
| 3 | about 73 | 2.82 / 4.76 | 0 / 0 | 0 | 0.15 / 0.10 / 0.46 / 0.72 |
| 4 | 69 (one clean lap) | 3.42 / 6.89 | 3 / 3 | 14 | 6.03 / 0.31 / 10.4 / 186 |

Error is estimate against simulator truth. In run 4 "collisions" is the simulator counter, which rose three times at
the start line.

## Findings

1. Raising the throttle limit from 0.2 to 0.35 (run 2) changed nothing: the car averaged about 1.8 m/s in both.
   The throttle ceiling was not the limit at 2.5 m/s; the controller's acceleration limit was active 69 to 77 % of
   the time.
2. With the 7 m/s ceiling and the original 1.0 m/s^2 acceleration (run 3) the car did 3 clean laps at about 73 s,
   top speed 4.76 m/s, and localization error was lower than at 2.5 m/s (worst 0.72 m). This is the best result
   in this set.
3. With 3.0 / 3.5 m/s^2 (run 4) the car reached 6.89 m/s but localization was lost 14 times, error exceeded 10 m
   between about 60 and 85 s, and the car was respawned three times. Only one lap was clean (69 s), so the extra
   speed bought little.
4. At this speed the localizer, not the car, limits the speed on this track. Its match thresholds and 3 m recovery
   search were set for 2.5 m/s, and wheel-odometry error grows over the straights.

## Limits

- Simulator only; braking and acceleration values are the actuator maximum, not measurements.
- One 240 s run per setting; no repeat trials, so run-to-run spread is not known.
- Run 4 error statistics are dominated by the lost-localization stretch.
- Not tried: speeds between runs 3 and 4 (for example 3 m/s^2 acceleration with a 5 m/s ceiling), a wider
  recovery search, or faster scan matching.

Settings were restored afterwards: `speed_mps: 2.5`, `max_throttle: 0.2`, `braking_calibrated: false`, 1.0 / 1.5
m/s^2.

Figures: `accel-3.0-error-timeline.png` (run 4, localization error with lost events in red),
`accel-3.0-video-frames.png` (four frames of the run 4 video).
