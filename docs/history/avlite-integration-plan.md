# Initial AVLite integration: completed

Historical milestone, validated on 16 September 2026: run AVLite in the
AutoDRIVE practice simulator and complete a clean low-speed lap.

- [x] Pin AVLite 0.6.3 at `1653f592b2eb5289a14c0d1041e50fdea855322a`.
- [x] Isolate AVLite's NumPy 2 environment from the stock ROS/API environment.
- [x] Adapt LiDAR and ground-truth odometry through an AVLite WorldBridge.
- [x] Run SyncExecuter and the Follow the Gap extension with upstream control.
- [x] Convert steering/acceleration through an independent actuator adapter.
- [x] Add startup gating, validity checks, watchdogs and reset handling.
- [x] Build/test the ROS package and containers, including controller-loss tests.
- [x] Record a clean lap at a 0.5 m/s ceiling and 0.02 throttle cap.
- [x] Document results and add automated CI coverage.

The [validation report](../validation/avlite-baseline.md) contains the measured evidence.
The original delivery is recorded in
[PR #1](https://github.com/HamdanAlhajeri/autodrive-roboracer/pull/1).

For current work, use the [execution checklist](../plans/checklist-2026-09-21.md),
[setup guide](../avlite-setup.md) and [real-car plan](../plans/real-car-plugin.md).
