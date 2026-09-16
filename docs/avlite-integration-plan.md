# AVLite integration plan and progress

## Goal

Run the real AVLite execution stack against the existing AutoDRIVE RoboRacer
practice simulator. First demonstrate low-speed control, then complete one lap
without collision, reset, or manual intervention. Keep the original controller
available, but never run two actuator controllers together. No Unity or custom
track work is included.

## Implementation

- [x] Pin AVLite to `1653f592b2eb5289a14c0d1041e50fdea855322a` and isolate its
  NumPy 2 dependencies from the AutoDRIVE camera bridge.
- [x] Implement an AVLite WorldBridge for LaserScan and Odometry, using the
  simulator's ground-truth localization and LiDAR mounting transform.
- [x] Run AVLite SyncExecuter with a RoboRacer profile and a range-based
  Follow-the-Gap extension retaining upstream steering and velocity control.
- [x] Publish SI-unit Ackermann commands through an independent actuator
  adapter. Convert acceleration to a bounded speed target and track it using
  speed feedback; normalize steering using the verified simulator limit.
- [x] Add startup gating, invalid-input rejection, 0.5-second data/command
  watchdogs, reset handling, and forward-only throttle. Zero throttle is a
  command, not a promise of instantaneous physical stopping.
- [x] Provide separate Compose services and preserve the original workflow.

## Validation

- [x] Repair the three pre-existing tests using the obsolete steering API
  (baseline: six passed, three failed).
- [x] Test transforms, scan validity, gap selection, signs, saturation,
  throttle limits, stale data, and resets using the real AVLite controller.
- [x] Build the ROS package and containers; test synthetic ROS inputs and
  confirm that killing AVLite causes the independent adapter to command zero.
- [x] Demonstrate motion and steering in the actual GPU simulator, initially
  limited to 0.5 m/s.
- [x] Tune conservatively and record a complete clean lap using lap count,
  collision count, odometry, and command telemetry.

## GitHub delivery

- [x] Document setup, architecture, unit conversions, stopping, fallback, exact
  validation commands, results, and limitations.
- [x] Extend CI with tests that do not require a GPU.
- [x] Push `feat/avlite-autodrive` and open a PR against `main`. Keep it draft
  if the clean-lap milestone is incomplete; do not merge automatically.

Detailed observed results are recorded in `docs/avlite-validation.md`. A passing
unit test or successful process launch alone does not establish a clean lap.

Delivered in [pull request #1](https://github.com/HamdanAlhajeri/autodrive-roboracer/pull/1).
The clean-lap milestone passed, so the PR is ready for review. It has not been merged.
