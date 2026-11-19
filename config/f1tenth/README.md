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

## Known open items

- `speed_to_erpm_gain: 4614.0`. If the motor drives opposite to the command, the direction is wrong in odometry too.
  The fix (negate the gain, or reverse the motor direction in VESC Tool) is not applied. If the pole count or
  wheel and gear settings in the VESC change, recompute the gain.
- `vesc_to_odom_node.wheelbase: .25` (stock). The measured wheelbase is 0.314 m, and this value sets the yaw rate
  in the odometry. It is not changed here; decide before mapping.
- With the offset at 0.420 and the driver limits 0.15 and 0.85, the reachable steering is about +0.222 rad left and
  -0.354 rad right, so `steering.max_angle_rad` is 0.22.
