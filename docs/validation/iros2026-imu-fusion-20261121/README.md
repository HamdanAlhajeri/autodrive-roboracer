# IROS 2026 track: IMU-fused odometry and higher speed in the simulator

21 November 2026. Follow-up to `docs/validation/iros2026-speed-runs-20261120`, which ended with localization lost 14 times
at 6.9 m/s. This note records why, the fix tried, and the speeds reached with it. Simulator only, AVLite ICP
localization, IROS 2026 track, 240 s per run, recorded with `tools/sim-localization/measure_pose.py` (now also logs the
IMU). Raw recordings and videos stay in `log/` and are not committed.

## Cause of the losses

- The error was almost entirely along the track (cross-track about 0.05 m). Between two parallel walls the LiDAR cannot
  constrain the position along the corridor, so the estimate follows wheel odometry, and ICP alignment stayed near 1.0
  while the error grew.
- The simulated wheel encoders read the true speed wrongly in a way that depends on acceleration. Wheel speed divided by
  true speed: hard braking 0.62, light braking 0.78 to 0.80, cruise 0.89 to 0.92, acceleration 1.03 to 1.04, hard
  acceleration 1.14 to 1.19. A single scale factor cannot fix that, so the error grew with every brake and acceleration.
- The simulated IMU forward acceleration is accurate: over 2 s windows its integral against true change in speed gave
  correlation 0.989 and slope 0.973 (per-sample noise about 2.8 m/s^2).

## Fix

`sim_odometry.py` gained two options, both off by default:

- `--imu-fusion`: integrate IMU acceleration for speed and pull it toward the scaled wheel speed with a gain that drops
  as acceleration grows (`gain = 3.0 / (1 + (a/0.3)^2)`), so wheels dominate at cruise and the IMU dominates in
  braking and acceleration. Falls back to the wheels when there is no IMU data or after a gap above 0.5 s.
- `--slip-gain` (tried first): linear speed correction from filtered acceleration.

`--speed-scale 1.25` is a fit to this simulator's encoder model and will not transfer to the real car; the fusion
method should. Unit tests: 17 pass in `test/test_sim_odometry.py`.

## Results (one 240 s run each)

| Run | Odometry | v ceiling / a_lat / accel / brake | Clean laps | Top speed (m/s) | Collisions / respawns | Loc. lost | Error mean / p95 (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | wheels | 7 / 3.0 / 2.0 / 2.5 | partial | 5.99 | 1 crash | 6 | not stable |
| B | wheels + slip gain | 7 / 3.0 / 3.0 / 3.5 | partial | not stable | 7 respawns | 13 | not stable |
| C | fusion | 7 / 3.0 / 2.0 / 2.5 | 4 (about 56 s) | 6.78 | 0 / 0 | 0 | 0.17 / not recorded |
| D | fusion | 10 / 3.0 / 3.0 / 3.5 | 4 (about 52 s) | 8.09 | 0 / 0 | 0 | 0.10 / 0.28 |
| E | fusion | 12 / 4.5 / 3.0 / 3.5 | 5 (42.1, 43.3, 42.8, 43.0, 41.8 s) | 9.27 | 0 / 0 | 0 | 0.16 / 0.48 |

Run E detail: mean speed 4.95 m/s, above 6 m/s for 23 % of the time, error median 0.11 m, max 1.0 m, planner profile
2.07 to 9.39 m/s. The 7 m/s wheels-only reference (previous note, run 4) lost localization 14 times.

## Limits

- Simulator only. Braking cap override (`braking_calibrated: true`), the 3.0 / 3.5 m/s^2 limits (actuator maximum) and
  the 4.5 m/s^2 corner grip are not measurements. None of these speeds is validated for the real car, which stays under
  its 1.0 m/s commissioning cap.
- One run per setting, so run-to-run spread is unknown. Run E error is higher than D, so error does grow with speed.
- The wheel scale 1.25 is simulator-specific. On the car the VESC odometry scale must be measured (tape-measured
  straight run), then fusion with the real IMU re-tuned and tested.
- Speed settings used (`config/avlite.yaml`, `config/driving.yaml`: `speed_mps 12`, `max_throttle 0.9`, `recovery_radius_m
  5.0`) are local simulator overrides and are not committed.

## Reproduce

```
ODOM_ARGS_VIDEO="--speed-scale 1.25 --imu-fusion" EXTRA_MEASURE_ARGS="--auto-reset" \
  bash video-test.sh 240
```

(or `tools/sim-localization/run_trials.sh <name> 1 240 "--speed-scale 1.25 --imu-fusion"`).
