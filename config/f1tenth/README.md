# F1TENTH car configuration

`vesc.yaml` is a copy of the F1TENTH driver's config for this car:
`~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml` on the Jetson. The F1TENTH workspace is not in this
repository, so this copy is the record of what the car runs with. Keep the two the same.

## Install on the Jetson

Back up the real file, copy this one over it, then restart the bringup with the car off the ground:

```bash
cp ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml ~/vesc.yaml.bak-$(date +%Y%m%d-%H%M%S)
cp config/f1tenth/vesc.yaml ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
```

The install and build copies in the workspace are symlinks to that source file.

## What differs from the stock file

- `steering_angle_to_servo_offset: 0.420` (stock 0.5304). Measured on 19 Nov 2026 with
  `servo-calibration/servo_center.py`: the front wheels point straight at servo position 0.420. With 0.5304 they
  pointed about 5.2 degrees right at a zero command. `config/roboracer/hardware.yaml` keeps
  `steering.center_offset_rad: 0.0`; do not also add an offset there or it is applied twice.

- `vesc_to_odom_node.wheelbase: 0.314` (stock .25). The measured wheelbase is 0.314 m, and this value sets the yaw
  rate in the odometry, so it matters for SLAM and ICP.

## Throttle ramp (acceleration limit)

The car jittered when the speed jumped (full joystick push). F1TENTH's `throttle_interpolator` node sits after the
mux, so it ramps the speed for the joystick and for autonomy alike. In `vesc.yaml`:

- `max_acceleration` (m/s^2): how fast the speed may rise. Now 0.5 (stock 2.5). Raise it in small steps if the car is
  too sluggish; lower it if it still jitters.
- `max_deceleration` (m/s^2): how fast it may slow down or stop. Keep it at 2.5 so the B stop stays quick (a stop from
  0.8 m/s took about 0.37 s measured at 2.5; at 0.5 it would take about 1.6 s).

The stock node has one limit for both directions and ignores `max_deceleration`, so install the patched
`throttle_interpolator.py` (stock plus the second limit; with no `max_deceleration` it behaves as stock):

```bash
F=~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/f1tenth_stack/throttle_interpolator.py
cp $F ~/throttle_interpolator.py.bak-$(date +%Y%m%d-%H%M%S)
cp config/f1tenth/throttle_interpolator.py $F
```

Install `vesc.yaml` the same way, restart the bringup with the car off the ground, then repeat a B-press stop test and
a wheels-off full-stick test before driving. Autonomy also has its own ramp, `c32_ego_max_acceleration: 0.4` in
`config/roboracer/race.yaml`; the slower of the two wins.

## Known open items

- `speed_to_erpm_gain: -4614.0` (stock 4614.0). The motor ran opposite to the command after the VESC recalibration,
  so the gain is negated: the command and the odometry flip together. This is proposed and not yet confirmed on the
  car; it counts as fixed only when the real motion is forward and the odometry vx is positive. If the direction is
  later fixed in VESC Tool or by swapping two phase wires, set it back to 4614.0. If the pole count or wheel and
  gear settings in the VESC change, recompute the gain.
- With the offset at 0.420 and the driver limits 0.15 and 0.85, the reachable steering is about +0.222 rad left and
  -0.354 rad right, so `steering.max_angle_rad` is 0.22.

## mux.yaml (joystick priority)

`mux.yaml` is the copy of `~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/mux.yaml`. Stock gives the joystick
topic priority 100, and `joy_teleop.yaml` publishes zero-speed messages on it at 9 to 20 Hz whenever no button is
held, so every autonomous `/drive` command was dropped (measured 24 Nov 2026). Priority 5 (below `/drive` at 10) lets
autonomy through while `/drive` is fresh; the zeros still keep `/odom` alive at idle and stop the car 0.2 s after
`/drive` goes quiet. Do not remove the `default` entry in `joy_teleop.yaml`: it also stopped `/odom`. Install the same
way as `vesc.yaml`, with a backup, then restart the bringup with the car off the ground.
