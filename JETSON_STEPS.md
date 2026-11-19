# JETSON_STEPS.md: step-by-step for the agent on the Jetson

You are the agent running on the car's Jetson Orin NX. Follow the steps in order. Each step says what to run, what
good looks like, and when to stop and ask the person at the car. Background and details are in `HANDOVER.md`
(sections are named below); the repository map is in `AGENT.md`.

## Rules (do not break them)

- Never move the car yourself. Motion happens only inside `tools/car-run.sh` (and `tools/jetson-run.sh launch`) with a
  person at the car holding the dead-man button, with the emergency stop within reach.
- Do not use `--yes` unless that person has told you in chat that they are at the car and ready.
- Write hardware values only with `python3 -m avlite_roboracer.profile_edit`. Never edit `hardware.yaml` by hand.
- Never guess a measurement. Ask the person for anything marked **PERSON**.
- Start at 1.0 m/s or less. Do not raise speeds, and do not pass `--allow-fast`, without measured evidence and the
  person's approval.
- Do not commit or push. No AI attribution anywhere.
- Report outcomes faithfully: if a step fails, show the output. If you skipped or could not check something, say so.
- Run long things inside `tmux` (the SSH link may drop). Use one window for the F1TENTH bringup, one for commands.
- Run commands from the repo root (`~/autodrive-roboracer`) with the workspace sourced
  (`source /opt/ros/<distro>/setup.bash; source ~/roboracer_ws/install/setup.bash`).

After each step, run `bash tools/car-day.sh` (read-only) and read what it says is next.

## Part A. Install (no motion)

**A1. Code.** `git clone <repo URL> ~/autodrive-roboracer`, or `git pull` if it exists.
Good: `git log -1` shows the latest commit.

**A2. Runtime.** `bash tools/jetson-setup.sh`, then add the two `source` lines it prints to `~/.bashrc` and open a new
shell. Good: `/opt/avlite-venv/bin/python -c "import avlite"` works, and `ros2 pkg executables avlite_roboracer` lists
the supervisor and the actuator.

**A3. F1TENTH drivers.** Build them with the commands in `docs/jetson.md`, section "F1TENTH car". Jazzy has no apt
package. This was verified once on 10 Nov and not re-run since. If a command fails, stop and report the output.

**A4. Network.** The Hokuyo uses the wired port at `192.168.0.15/24` (sensor at `192.168.0.10`). Do not change that
port. Check with `ip addr` and `ping 192.168.0.10`. The VESC port is set in `f1tenth_stack/config/vesc.yaml`.

## Part B. Bringup and interface check (no motion)

**B1. Start the bringup** in its own tmux window: `ros2 launch f1tenth_stack bringup_launch.py`. Leave it running.

**B2. Check names.** `bash tools/jetson-run.sh inspect`. The topics and frames must match
`config/roboracer/hardware.yaml`: `/scan`, `/odom`, `/drive`, `/teleop`, frames `odom`, `base_link`, `laser`.
If a name differs, report it and ask the person before changing the profile.

**B3. Status.** `bash tools/car-day.sh`. Expect it to complain about the profile; that is the next part.

## Part C. Hardware profile (the 23 values that unlock motion)

Check progress at any time: `python3 -m avlite_roboracer.doctor --profile config/roboracer/hardware.yaml`.
The goal is `23 of 23` and `commissioned`.

**C1. Values you can get yourself**

```bash
python3 -m avlite_roboracer.profile_edit laser-from-tf            # needs the bringup running
python3 -m avlite_roboracer.profile_edit deadman-from-joy         # PERSON holds the dead-man button when asked
python3 -m avlite_roboracer.profile_edit defaults-localization    # simulator starting thresholds, not measurements
```

Tell the person that the localization thresholds are starting values that passed in the simulator only.

**C2. Values only the person can give (PERSON).** Ask, then write with `profile_edit set KEY=VALUE`:

| Key | What to ask for |
| --- | --- |
| `vehicle.wheelbase_m` | front axle to rear axle centre-to-centre, in metres |
| `vehicle.width_m` | widest point including wheels |
| `vehicle.length_m` | bumper to bumper |
| `vehicle.rear_axle_to_front_m` | rear axle to front bumper (wheelbase <= this <= length) |
| `steering.max_angle_rad` | wheels off the ground, steer to the largest angle without binding |
| `steering.positive_left` | wheels off the ground, command a small positive angle: `true` if the wheels point left |

Set these safe starting limits yourself, and tell the person you did:

```bash
python3 -m avlite_roboracer.profile_edit set motor.max_command_speed_mps=1.0 limits.mapping_speed_mps=0.6 limits.racing_speed_mps=1.0
```

A rejected value changes nothing. Read the error and fix the value; do not work around the editor.

**C3. Stop tests (PERSON drives with the joystick, wheels on the floor, clear space).**

1. Run `bash tools/jetson-run.sh record` in a window.
2. Ask the person to do 5 to 8 quick pushes from rest to about 0.5 m/s, then let go completely, then the same to about
   1.0 m/s, with a clear stop between each.
3. Stop the recorder with Ctrl+C. Run `bash tools/jetson-run.sh commission`. It writes `stops.json` and prints the
   worst-case latency, deceleration, stopping distance and acceleration.
4. Apply: `python3 -m avlite_roboracer.profile_edit commissioning log/jetson/<run>/stops.json`.
   Any value it could not measure stays `null`. Ask for more stops until nothing is missing.

`limits.racing_speed_mps` must not exceed `measurements.stop_test_speed_mps`, so the 1.0 m/s stops come first.

**C4. Confirm.** `doctor` must say `23 of 23` and `commissioned`. If not, list what is still null and why.

## Part D. Stationary tests (PERSON does and watches; nothing may move)

With the bringup running, start `bash tools/jetson-run.sh launch` in another window and have the person check, wheels
off the ground or the car held:

1. Steering direction matches `positive_left`.
2. Actuator bounds hold.
3. A manual joystick command overrides autonomy.
4. Killing a process stops the car.
5. Losing the connection stops the car.
6. A speed command of 0 brakes rather than coasts.

Record the result of each test in your report. Do not continue to Part E if any test fails.

## Part E. The run (PERSON at the car)

Before starting, get the person's confirmation in chat that: the car is in clear space, one person holds the dead-man
button, one person has the emergency stop, and you are inside tmux.

```bash
bash tools/car-run.sh --start-stack --laps 3 --race-seconds 60
```

Type GO at the prompts only when the person says to. The stages:

1. **Preflight** (`car-day.sh`). A failure here means nothing was sent to the car.
2. **Mapping:** slow laps with Follow the Gap while SLAM Toolbox builds the map. It stops when the lap is confirmed.
3. **Finalizing:** builds the race map and racing line, and checks localization. The car does not move.
4. **Racing** for 60 s with ICP localization, then a stop.

Exit codes: 0 done, 1 failed or stopped, 2 setup problem (nothing sent), 130 interrupted. Logs are in
`log/car-run/<timestamp>/` (`run.log`, `bringup.log`, `supervisor.log`, `preflight.log`).

Useful options: `--map-only` (map and validate, do not race), `--race-only` (race an existing READY map), `--remap`.
Run `--map-only` first if the person prefers to see the mapping alone.

**If it stops or fails:** read `run.log` and the stop-reason table in `HANDOVER.md` section 7. Do not retry blindly.
An "invalid pose" stop (`speed_limit_reason` 3) cannot restart on its own; stop, report, and ask.

## Part F. Report back

Send the person (see `HANDOVER.md` section 8 for the full list):

- Which steps passed, which failed, with the command output for failures.
- The measured values written to the profile (and which ones you set as starting values).
- The run's exit code, how long each stage took, the stop reason if any, and the localization numbers from `status`.
- The path of `log/car-run/<timestamp>/` and the session folder, so they can be copied off the Jetson.
- Anything you could not verify.

## Most likely first problems (unverified on hardware)

- Interface names that do not match the profile (B2).
- The `--start-stack` path in `car-run.sh` (the real-process start has only been tested against a fake).
- SLAM Toolbox node names on the installed version.
- VESC speed 0 braking versus coasting (D6).
- Follow the Gap with the real LiDAR mount, and the `/drive` mux behaviour.
- All localization thresholds.

If any of these goes wrong, stop and report; do not improvise a fix on the car.
