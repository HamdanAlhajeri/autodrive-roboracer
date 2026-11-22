# Active braking in the simulator: tried and reverted

21 November 2026. Simulator only (IROS 2026 track, IMU-fused odometry, cornering 7 m/s^2 unless stated). The
actuator originally slows the car only by cutting throttle to zero. This note records the attempt to add real
braking (negative throttle). It made the car slower and the code was reverted, so none of it is in the source tree.
The work is kept here as a patch for later use, for example on the real car, where the VESC brakes differently.

## Results (240 s trials)

| Setting | Trials | Average speed | Collisions | Localization error median | Deceleration at full request |
| --- | --- | --- | --- | --- | --- |
| Coasting only (baseline) | 4 | 4.72 m/s | 9 | 0.14 m | 3.80 m/s^2 |
| Negative throttle, gain 0.15, cap 0.6 | 2 | 1.23 m/s | 34 | 1.75 m | 3.8 m/s^2 |
| Negative throttle, gain 0.05, cap 0.2 | 2 | 2.22 m/s | 19 | 0.60 m | 4.08 m/s^2 |
| Anti-lock (10 % wheel slip limit), IMU reference speed, fade to zero at 0.15 rad steering, check on every wheel-speed sample | 4 | 2.58 m/s | 27 | 0.37 m | 3.95 m/s^2 |

Deceleration is the median measured deceleration while the controller requested 3 to 3.6 m/s^2.

## Findings

- Negative throttle added at most about 0.15 m/s^2 over coasting (3.80 to 3.95). The simulator has no brake input,
  only throttle and steering, so negative throttle is reverse motor torque, and coasting already slows the car about as
  fast as the tyres allow.
- It made the wheels slip: with plain braking the wheel odometry read 38 % to 124 % of the true speed while braking.
  Anti-lock control brought that to 71 % to 104 %, but crashes still tripled.
- The new crashes were in corners at 5 to 7 m/s (near (12, 4), (14, 9), (18, 7)), not at the earlier hairpin: braking
  while steering takes the rear tyres' sideways grip.
- Planned braking values above what coasting delivers (3.5 m/s^2) made the car arrive at corners too fast, so the
  planner's braking value is best set to what the car really achieves.

## What is in the patch

`anti-lock-braking.patch` (against commit `6b7b8fc`; the slip-gain code it touches in `sim_odometry.py` has since been removed, so use `git apply --3way` or check out that commit first): actuator brake settings (`brake_gain`, `max_brake`,
`brake_min_speed`, `abs_*`, `brake_steer_limit`), anti-lock control driven by the IMU reference speed, the odometry node
publishing the scaled wheel speed on `/avlite/wheel_speed`, and adapter subscriptions. `test_braking.py.txt` holds its
14 test functions. To reuse: `git apply anti-lock-braking.patch`, rename the test file, run the tests.

## Limits

- Simulator only: it says nothing about the real car's VESC braking, which holds the motor against the wheels.
- 2 to 4 trials per setting.
- The planner-braking-4.0 variant was configured wrongly and never ran.
