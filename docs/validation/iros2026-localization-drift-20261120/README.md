# IROS 2026 track: why ICP localization is lost (logging runs)

20 November 2026. Follow-up to `docs/validation/iros2026-laps-20261119`. Same stack and IROS profile
(planned controller, `reference: map_hits`, `recovery_radius_m: 3.0`, `speed_mps: 2.5`, `max_throttle: 0.2`).
Recorded with `tools/sim-localization/measure_pose.py --scans 5` through `tools/sim-localization/run_trials.sh`:
simulator truth, wheel odometry, the localizer estimate, alignment, recovery count, commands and every 5th
raw LiDAR scan, per trial of 330 s. Each trial respawns the car and restarts odometry and AVLite.
Raw recordings (about 78 MB each) stay in `log/step-trials/` and are not committed.

## Results

| Batch | Trials | Collisions | "Localization lost" events | Error mean / median / p95 / max (m) |
| --- | --- | --- | --- | --- |
| base (odometry speed scale 1.0) | 3 x 330 s (9 laps) | 0 | 5, 5, 6 | 0.33 / 0.13 / 1.4 / 2.1 to 2.6 |
| `--speed-scale 1.075` | 2 x 330 s | 0 | 0, 0 | 0.17 / 0.09 / 0.6 / 1.2 to 1.4 |

Error is the distance between the localizer estimate and simulator truth.

## Findings

1. The loss is a slow drift, not a jump. Error is about 0.13 m most of the time, then grows over 12 to 22 m of
   driving to about 2 m, where alignment falls and the recovery search runs. Events repeat at the same two places
   (x about 15, y about -10.3 and x about -19, y about 2) in every trial.
2. The error is along the direction of travel only. At each event: along-track -1.4 to -2.4 m (estimate behind
   the car), cross-track under 0.16 m. When error is under 0.5 m the cross-track error is 0.011 m.
3. The wheel odometry under-reads. Over trial 1 it measured 557 m against 631 m driven. By phase (speed ratio
   wheel/truth): braking 0.80, cruise 0.93, accelerating 1.07. While the car drives straight between parallel
   walls the scan cannot correct along-track error, so it accumulates until the next corner.
4. Raising the odometry speed scale to 1.075 removed all loss events and halved the error. Maximum error is still
   1.2 to 1.4 m (just before the lap line and at place A), so the braking under-read is not fixed.
5. Laps were slower with the scale change (about 111 to 115 s per lap against 105 to 109 s). Not explained yet.

## Limits

- Simulator only. This odometry is the simulator's wheel-encoder model; the real car's VESC odometry will have
  its own scale and slip, which must be measured on the car.
- 1.075 is a fit to this simulator, not a calibrated value. Two trials only.
- Not yet tried: lower speed, speed-dependent or braking-aware odometry, other recovery settings.

Plots: `base-error-timeline.png`, `scale1075-error-timeline.png` (red dashed = lost event, dotted = lap).
