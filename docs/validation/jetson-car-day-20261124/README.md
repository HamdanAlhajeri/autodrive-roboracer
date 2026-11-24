# Jetson car day 2, 24 November 2026

Second day on the real car. Day 1 is in `../jetson-car-day-20261123/`. Large recordings stay on the Jetson under `log/`
(git-ignored); the small outputs, maps, probe traces, logs and configs are here.

## Result in one paragraph

The car now **drives itself on the floor**. The reason it never moved under autonomy was a joystick/multiplexer
priority problem, now fixed. The stop paths were measured with the wheels off the ground. The first autonomous
floor mapping run (mapping speed 0.8 m/s) covered about 19 m in 92 s, stalled twice and was stopped by a B press
before one lap was finished. The planner's map builder **rejected the new track twice** (a 0.33 to 0.35 m wall
step), so there is no race map and no planned race yet.

## State at the end of the day

| Item | Value |
| --- | --- |
| Repo on the Jetson | `main` at `18d076f` (local edits: the profile only) |
| Profile | 23 of 23, commissioned; `limits.mapping_speed_mps` raised 0.6 to **0.8**; `deadman.stop_on_press: true` (no button held; B stops the car) |
| Speed caps | motor clamp 1.0, racing 1.0, race config velocities 1.0, `c32_ego_max_acceleration` 0.4 (commit `18d076f`) |
| F1TENTH `mux.yaml` (machine-local, not in the repo yet) | joystick priority **100 to 5**; originals and the change are in `config/` here |
| `joy_teleop.yaml` | original (the `default` zero stream is kept; see below) |
| `vesc.yaml` | unchanged from day 1 (`config/f1tenth/vesc.yaml` in the repo) |

## Why the car never moved autonomously (fixed)

`ackermann_mux` gives the joystick topic (`/teleop`, priority 100, timeout 0.2 s) priority over `/drive` (10).
`joy_teleop.yaml` has a `default` entry that publishes zero-speed messages on `/teleop` at 9 to 20 Hz whenever no
button is held, so the joystick topic was always fresh and every autonomous command was dropped. Measured on the
command chain while a session ran: `/drive` carried 0.6 m/s, `/ackermann_cmd` (mux output) carried only 0.0.

Tried first: disabling the `default` entry. It stopped the block, **but it also stopped `/odom`**: `vesc_to_odom`
needs a servo command (`use_servo_cmd_to_calc_angular_velocity: true`), and the zero stream was what kept the
mux, `ackermann_to_vesc` and servo-command chain alive at idle. With it gone the supervisor's own preflight failed
(`no message on /odom`, `transform odom->base_link is missing`). Restored (`config/joy_teleop.yaml-*`).

Applied instead: joystick priority 100 to 5 in `mux.yaml`, so autonomy wins while `/drive` is fresh and the zeros
still keep odometry alive and act as the fallback when `/drive` goes quiet (`config/mux.yaml-*`).

## Stop checks, wheels off the ground (car on blocks)

Probe: a ROS node timestamping VESC erpm (4614 erpm per m/s, forward is negative), `/ackermann_cmd`, `/drive`,
`/teleop`, `/joy` (`data/probe-*.csv`).

| # | Check | Result |
| --- | --- | --- |
| 1 | Idle: `/odom` alive, mux output zeros | Pass |
| 2 | Mapping session drives the wheels through the mux | Pass (command ramps 0.06 to 0.6 m/s in about 1.5 s with the 0.4 m/s2 limit) |
| 3 | B press stops the car | Pass: command cut at +0.000 s, wheels below 50% at +0.23 s, at rest +0.37 s; supervisor reason "stop button pressed" |
| 4 | Gamepad silent | Pass **in software**: `joy_node` paused (`kill -STOP`) 4.2 s. Command cut at +0.24 s after the last `/joy` message (the 0.25 s timeout), wheels at rest +0.78 s. The real USB receiver was **not** pulled. |
| 5 | Supervisor killed while commanding (SIGKILL) | Pass: `/drive` cut about 0.24 s after the kill (actuator expiry); the actuator kept running |
| 6 | LB manual drive | **Dropped on the owner's instruction** (no manual driving) |

Bare wheels hunt (about -1000 to +300 erpm at a constant 0.6 command, overshoot to about 0.88 m/s); the laptop
session says bare-wheel readings are not valid for judging jitter. The owner said it "jittered a bit at the beginning
and starts moving normally".

## First autonomous floor run (20:07:46 to 20:09:18, session a318aa22-1, mapping speed 0.8)

`tools/car-run.sh --map-only --yes`; stop-on-press; emergency stop in hand. Data:
`data/probe-floor-lap-0.8mps-autonomous-decimated.csv`, `data/slam-trace-floor-lap-a318aa22-1-decimated.csv`,
`data/timing-report-floor-lap-a318aa22-1.txt`, `runs/car-run-20261124-200742.log`.

- SLAM path 19.0 m over about 92 s; start (1.60, 0.32), end (0.91, 3.41). The lap was **not completed**.
- Commanded speed was 0.8 m/s from +2 s, but the wheels barely turned (0.01 to 0.2 m/s) for about 26 s. **Unexplained.**
  Then 0.8 m/s for about 8 s, a stop (command 0) for about 12 s near SLAM (-1.24, 0.37), a second run, and a second
  stall of about 22 s near (0.92, 3.39) with command 0.07 to 0.15 m/s. Peak wheel speed 1.26 m/s (overshoot).
- The owner pressed B at about +92 s: "stop button pressed". Supervisor timing was healthy (1796 ticks, mean 14 ms,
  longest 71 ms, longest gap 119 ms).

## Map and planner

- New track: a closed ring, about 5.5 x 5.1 m (`maps/newtrack2-*`), clean walls; a second hand drive gave a ring with a
  doubled left outer wall (`maps/newtrack4-*`).
- `map_builder` (`--profile config/roboracer/hardware.yaml`, vehicle radius 0.194 m) **rejected both drives**:
  `Ambiguous left boundary at s=6.8 m, map x=-0.79 y=0.91 (wall distance changes by 0.35 m, limit 0.3 m)` and
  `... s=6.1 m, map x=0.19 y=0.92 (0.33 m)`. Both points land next to the same feature, the lower-left corner of
  the inner island, where its outline steps by about 0.3 m (`maps/map-builder-rejection-points-on-both-maps.png`).
  The rule is meant to catch side openings and the laptop session will not loosen it for a planner that drives within
  about 0.1 m of a wall. The owner could not see a step on the real track, so it was not filled.
- No `racemap.json` exists, so `map-load` (commit `34df07c`) cannot run and no race was planned.
- `save_map` returned result 255 in 3 of 5 tries (live and in `recorder replay`); finalize now retries it (`6b4b636`),
  the replay command does not.

## Other findings

- The stale-sensor stop (scan and odometry both about 0.27 s old with healthy ticks) happened in 2 of 7 sessions since
  the scan-stamp fix, both the first session after a supervisor start; the 0.25 s limit stays (it fails safe).
- The gamepad sleeps; a button press wakes it. The VESC and the gamepad receiver still share one USB hub (the VESC
  dropped off the bus again at 18:32:59 and 18:51:39 on 23 Nov; 24 Nov events in `data/usb-events-24-nov.txt`).
- `deadman.stop_on_press` is on. Its only protection against a dead gamepad is `/joy` silence (measured above in
  software only).
- Speed: the owner asked for 3 m/s. Declined: standing cap 1.0 m/s, stop tests only to 1.18 m/s, about 3.2 m needed
  to stop from 3 m/s on a track about 5 m across and 1.5 m wide, steering gain unverified, odometry yaw reads 31%
  short. Agreed staging with the laptop session: raise the mapping speed one step per clean lap (0.6, 0.8, 1.0); above
  1.0 needs a clean planned lap at 1.0, stop tests at the higher speed, a protractor check of the steering gain and an
  explicit request from the owner.

## Not done

- No completed autonomous lap, no validated race map, no planned or localized race on the new track.
- The real receiver pull; the actuator-only kill (5b).
- The island step was not filled and re-mapped; no one confirmed it exists.
- Steering gain / odometry calibration with a protractor and tape; the five `localization.*` thresholds are still
  simulator starting values.
- The 26 s of commanded motion with no wheel motion at the start of the floor run is unexplained.

## To resume

1. Plug the VESC and gamepad receiver into separate Jetson USB ports.
2. `git pull` (stash the local `hardware.yaml` edits first), restart the bringup and the supervisor
   (`ros2 launch f1tenth_stack bringup_launch.py`, `bash tools/jetson-run.sh launch`); copy the tracked `mux.yaml`
   once the laptop session adds it.
3. Decide the island step (fill it, or the laptop session reviews the 0.3 m rule), then a clean autonomous mapping lap
   and `map_builder`; after that a `map-load` dry run to READY with the car standing still, then the race.
4. Raise the mapping speed one step per clean lap.

## Laptop-session commits pulled today

See `data/laptop-commits-pulled-24-nov.txt`.
