# First day on the car: checklist for the person at the car

One page to follow at the car. It puts the steps from `HANDOVER.md` and `JETSON_STEPS.md` in the
order you will do them, with what good looks like and when to stop. The car has never driven
this software. Simulator speeds, gains and limits do not apply to it: start at 1.0 m/s or less.

## Rules

- A person stands at the car with the emergency stop within reach, and a person holds the joystick
  **dead-man button** during every motion step. The car moves only while it is held.
- Clear floor space, wheels on the ground only for the stop tests and the run.
- Do not raise any speed above 1.0 m/s today. The profile rejects racing speeds above the
  speed the stop tests were done at.
- Write profile values only with `python3 -m avlite_roboracer.profile_edit`, never by hand.
- Stop and write down the output of anything that fails. Do not retry blindly.

## Bring

- Tape measure, a flat floor with 5 m or more of clear space, a second person, the car's battery charged.
- Laptop on the same network as the Jetson (SSH, ideally inside `tmux`).

## Part 1: With the wheels off the ground (no driving)

| # | Do | Good looks like | If not |
| --- | --- | --- | --- |
| 1 | `git pull`, then `bash tools/car-day.sh` | Latest commit shown; it lists what is missing | Fix each `[FAIL]` first |
| 2 | Start the bringup: `ros2 launch f1tenth_stack bringup_launch.py` (own tmux window) | `/scan`, `/odom`, `/drive`, `/joy` appear | Check the Hokuyo cable (`ping 192.168.0.10`) and the VESC port |
| 3 | `bash tools/jetson-run.sh inspect` | Names match `config/roboracer/hardware.yaml` | Ask before changing the profile |
| 4 | **Motor direction.** Command a small forward speed with the joystick (wheels off the ground) | Wheels turn **forward**; `ros2 topic echo /odom` shows positive `twist.linear.x` | The VESC gain was negated on 19 Nov and never confirmed: if the wheels turn backwards, or `/odom` speed is negative while the wheels go forward, stop and tell Claude |
| 5 | **Steering.** Command a small positive angle | Wheels point **left** (profile says `positive_left: true`) and look centred at zero | Fix `positive_left` or the servo centre (0.420) |
| 6 | **Speed 0.** Release the stick while the wheels spin | The wheels stop (brake), not coast | Note it; stopping distances will be longer |
| 7 | Hold the dead-man button, then let go; kill the bringup; unplug the network | Each one stops the car | Stop here: do not continue to Part 2 |

## Part 1b: steering angle and the first offline localization check (no driving)

Added 23 Nov after the first hand-driven map: SLAM Toolbox's motion was 8 % longer than the wheel odometry
and turned 30 % more than the odometry yaw. The odometry takes yaw from the commanded servo through
`steering_angle_to_servo_gain` (stock -1.2135), so a wrong gain would also mis-steer the car.

1. **Steering calibration** (car stationary, wheels on the floor): run `servo-calibration/servo_center.py`
   (VESC servo only, no drive motor), step the servo to about 7 positions inside `servo_min` and `servo_max`
   (for example 0.15, 0.25, 0.35, 0.42, 0.50, 0.60, 0.70, 0.85), and read the left and right front wheel
   angle at each with a protractor or a photo from above. Fit `servo = gain x angle + offset` and compare it
   with the live `vesc.yaml`.
2. **Full mapping lap by hand** with `bash tools/jetson-run.sh record`: drive one complete lap slowly, start
   moving at once and go slightly past the start so the route closes within 1 m.
3. **Replay the map**: `bash tools/jetson-run.sh map`, then `python3 -m avlite_roboracer.recorder export
   <replay-dir> --profile config/roboracer/hardware.yaml`.
4. **Offline race map** (rejections name the place): `python3 -m avlite_roboracer.map_builder <replay-dir>
   --profile config/avlite.yaml --name track-real --output-dir /tmp/map-test`.
5. **ICP against the saved map**, motors off: `python3 -m avlite_roboracer.icp_replay
   log/jetson/<run>/records.jsonl --map <replay-dir>/map.yaml --profile config/roboracer/hardware.yaml
   --auto-start --output /tmp/icp-replay`. It prints how well the scans fit (alignment), recoveries, and the
   odometry-to-ICP ratios for distance and turning. A close-to-1 start-search runner-up means the start was
   ambiguous: pass `--start X Y YAW` instead.

## Part 2: Stop tests (wheels on the floor, clear space)

These give the last 5 profile values: command latency, braking deceleration, maximum acceleration,
stop-test speed, stopping distance. The profile has 18 of 23 values; these are the rest.

1. In one window: `bash tools/jetson-run.sh record`.
2. Drive with the joystick: **5 to 8 quick pushes from rest to about 0.5 m/s, letting go completely each
   time**, with a clear stop between. Then the same to about **1.0 m/s**.
3. Stop the recorder with Ctrl+C.
4. `bash tools/jetson-run.sh commission` writes `stops.json` and prints the worst-case values.
5. Apply them: `python3 -m avlite_roboracer.profile_edit commissioning log/jetson/<run>/stops.json`.
   A value it could not measure stays empty: do more stops of that kind.
6. `python3 -m avlite_roboracer.doctor --profile config/roboracer/hardware.yaml` must say
   `23 of 23` and `commissioned`.

Also write down by hand: the stopping distance you see from 1.0 m/s, so the measured value can be checked.

## Part 3: First autonomous run (only if Parts 1 and 2 passed)

```
bash tools/car-run.sh --start-stack --laps 3 --race-seconds 60
```

Run `--map-only` first if you want to watch the mapping alone. It asks you to type GO at each stage.
One person holds the dead-man button; one has the emergency stop.

| Stage | What happens | Stop if |
| --- | --- | --- |
| Preflight | Checks install, topics, profile. The car does not move | Any failure (nothing was sent to the car) |
| Mapping | Slow laps (0.6 m/s) with Follow the Gap, SLAM Toolbox builds the map, stops by itself | The car hits or scrapes a wall, or does not finish the lap |
| Finalizing | Builds the race map and racing line, checks localization. No motion | It stops with reasons: read them |
| Racing | 60 s at 1.0 m/s on the racing line with ICP localization, then stops | `car is outside the validated corridor` or `invalid pose`: stop and report |

## What to bring back

- Which steps passed, and the output of any that failed.
- `log/car-run/<timestamp>/` and the session folder.
- The stop-test numbers, your hand-measured stopping distance, and the odometry sign check.
- Anything that looked odd, even if it passed.

## Not yet checked on the car

- That the negated VESC motor gain gives the right direction and a positive `/odom` speed.
- That speed 0 brakes instead of coasting.
- Follow the Gap with the real LiDAR mount, and the SLAM Toolbox names on the installed version.
- That `/drive` does not reach the VESC unless the dead-man button is held (depends on the joystick mux).
- Wheel-odometry error: measure it later on a tape-measured straight (drive a known distance, compare `/odom`),
  because the simulator's odometry correction (scale 1.25) does not apply to this car.

Details and the stop-reason table: `HANDOVER.md` sections 5 to 8.
