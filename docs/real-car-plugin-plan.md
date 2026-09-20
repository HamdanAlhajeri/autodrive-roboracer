# AVLite plugin for the real car

Agreed direction, 20 September 2026: prepare mapping, localization and remote
operation on the Ubuntu Jetson car. **Only this plan and the inspection script
exist so far.** The hardware plugin, remote service and estimated-pose driving
are not implemented or validated.

Jetson model, JetPack/ROS versions, sensor/motor models, topic interfaces and
physical stop controls still need identification. Simulator poses and actuator
calibration must not be reused as hardware measurements.

## Package structure

Create an installable `avlite_roboracer` package alongside `avlite_autodrive`.
Keep AVLite pinned at `1653f592b2eb5289a14c0d1041e50fdea855322a` initially.
Importing the plugin registers strategies; an explicit launcher starts the stack.
Imports must not open hardware, start threads or publish commands.

| Proposed module | Responsibility |
| --- | --- |
| `plugin/__init__.py` | Import/register AVLite strategies |
| `hardware_bridge.py` | ROS sensors → AVLite SensorFrame; configurable units, topics and mounts |
| `mapping.py` | Occupancy grid and occupied-point reference; save frames/resolution |
| `localization.py` | Wrap AVLite ICP, load maps, accept/reject poses and expose health |
| `motion_prediction.py` | Predict motion using wheel odometry and IMU yaw rate |
| `planning.py`, `control.py` | Connect validated paths and AVLite planners/controllers |
| `actuator.py` | Hardware conversion and independent local command expiry |
| `recording.py` | Raw sensors, transforms, commands, estimates and timestamps |
| `runner.py` | Explicit record/map/localize/drive modes and health gating |
| `supervisor.py` | One active session; start/stop/status/logs and drive-authorization expiry |

Keep NumPy algorithms independent of ROS. Select the ROS adapter after inspecting
the car; do not assume the simulator's ROS 2 Humble environment applies. Vehicle
profiles hold interfaces/transforms. Avoid importing the simulator bridge or its
ground-truth localization bypass into the hardware process.

## Reusing AVLite localization

The pinned
[LidarLocalization source](https://github.com/AV-Lab/avlite/blob/1653f592b2eb5289a14c0d1041e50fdea855322a/avlite/c10_perception/c16_localization_algs.py)
matches scans against a fixed first-scan reference, starting from the previous
pose. It applies the sensor mount but does not provide full saved-map loading,
encoder/IMU prediction, map growth or explicit match-confidence reporting.
Too few matches can leave the previous pose unchanged without reporting failure.

Reuse its ICP core through our wrapper. For scan points $p_i$ and matched
reference points $q_i$, the fitting objective is:

$$
(R^*,t^*)=\arg\min_{R,t}\sum_i\|Rp_i+t-q_i\|^2.
$$

Here $R$ is a planar rotation and $t$ is translation.
The wrapper supplies a predicted pose and bounded local map subset, checks
correspondences/residuals, and rejects stale or unobservable updates. An unchanged
pose alone does not prove a valid match. Invalid localization inhibits driving.
Measure correspondence-search cost on Jetson and regression-test any use of
pinned private helpers before upstream upgrades.

For initial small-area mapping, manually drive slowly, predict from odometry/IMU,
match against previously accepted scans and insert only accepted observations.
Ray-trace free/occupied cells while preserving unknown space. This has no loop
closure and may drift. Never match a scan against a map containing that same scan.

For localization evaluation, freeze the saved map and use a separate recording.
Enable AVLite's localization stage and remove the simulator ground-truth bypass.
Record/map/localize modes retain manual control and publish no autonomous commands.

## Remote operation

Use a reachable laptop-to-Jetson connection: Ethernet for setup, then private
Wi-Fi if permitted. Campus client isolation may block device-to-device access;
Internet access is not required for local SSH. Any overlay network must comply
with campus policy; do not expose control ports publicly.

A key-authenticated SSH launcher will call a fixed supervisor interface:
`start record|map|localize|drive`, `stop`, `status` and `logs`. These commands are
proposed, not installed. Use fixed arguments, one supervised session and preserved
run metadata. Stop must be repeatable; reboot, failure or reconnect must not
automatically rearm driving.

Drive authorization must be renewed and expire locally on lost contact, alongside
sensor/command freshness checks. Choose expiry and stop outputs from measured
hardware behavior. SSH termination alone is not a physical stop; preserve the
independent emergency stop/manual override.

Build the native or ARM-compatible container environment after identifying the
Jetson's OS/JetPack. The desktop simulator image is not the deployment image.

## Implementation and acceptance

1. Identify hardware, topic types/rates, units, frames, mounts and stop controls;
   establish laptop connectivity.
2. Package the plugin with no import side effects; implement recording/replay.
3. Add mapping and the ICP wrapper. Test known transforms, bad initial poses,
   stale/invalid/repeated scans, insufficient matches and map reload.
4. Evaluate on separate simulator recordings using reference poses only for
   initial pose and error measurement. Report pose error, failures and compute time.
5. With motors disabled, test remote start/stop, duplicate starts, process/network
   loss, expiry and reconnect without automatic movement.
6. Make a slow manual hardware map and a separate localization recording;
   inspect drift, alignment, timing and lost matches.
7. With wheels clear, verify steering signs, bounds, watchdog and physical stop.
   Measure hardware response before low-speed autonomous commissioning.

Fast real-car laps follow these checks. Simulator braking values do not replace
physical acceleration/braking measurements.

## Collect the missing hardware facts

On the Jetson, in the shell normally used for the sourced ROS/vehicle workspace:

```bash
bash tools/inspect-jetson.sh
```

The script reads model/OS/ROS details, existing topics, USB devices, local addresses
and SSH service state. It installs nothing, changes no networking and sends no
driving commands. Provide its output plus sensor/motor models and stop controls.
