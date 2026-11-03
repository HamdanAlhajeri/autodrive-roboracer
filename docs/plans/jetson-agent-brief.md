# Brief for an agent working on the Jetson car

Give this file to a Claude agent running **on the Jetson** (in
`~/Desktop/autodrive-roboracer`). It covers the setup and checks that can run before any
hardware is commissioned. Nothing in it moves the car.

## Safety rules (never break these)

The car has a motor controller (VESC at `/dev/ttyACM0`) and steering. You cannot see it,
so assume the wheels may be on the ground and a person may be near it.

- **Never send a motion command.** Do not publish to `/drive`, `/teleop`,
  `/ackermann_cmd`, `/commands/*` or any `AckermannDriveStamped` topic. Do not write to
  `/dev/ttyACM0`. Do not run `vesc_driver`, `ackermann_to_vesc`, `roboracer_actuator`,
  `roboracer_supervisor`, `roboracer.launch.py` or `tools/jetson-run.sh launch`.
- **Never fill in measured values in `config/roboracer/hardware.yaml`.** Fields that are
  `null` must stay `null` until a person measures them. Never copy simulator values
  into it. You may correct topic and frame **names** when the running system shows the
  real ones.
- **Ask the user before** `sudo` commands, installing packages, changing network
  settings, or editing files outside this repository.
- **Do not commit or push** unless the user asks.
- Stop and ask the user if anything is unclear or a step needs the car to move.

## What is already known (3 November 2026)

| Item | Status |
| --- | --- |
| Computer | Jetson Orin NX, Ubuntu 24.04, L4T R39, Python 3.12 |
| ROS | ROS 2 **Jazzy** at `/opt/ros/jazzy`. Not sourced by default. |
| Repo | `~/Desktop/autodrive-roboracer` (docs often say `~/autodrive-roboracer`) |
| User | `fasty`; hostname is `localhost`, so `jetson.local` does not work |
| VESC | Detected: STMicro ChibiOS Virtual COM Port, `/dev/ttyACM0` |
| Joystick | Logitech F710 detected |
| LiDAR | **Not detected.** No Hokuyo on USB, no `192.168.0.x` address. Model unknown. |
| F1TENTH drivers | Unknown whether installed for Jazzy (upstream F1TENTH targets Humble) |
| Laptop SSH | Not working yet (uni Wi-Fi blocks it; USB-C link fails on the laptop side) |
| `hardware.yaml` | All measurements `null`; the stack refuses motion until filled |

The code and docs were written for ROS 2 Humble and Python 3.10. Read
[docs/jetson.md](../jetson.md) for the full design and checklist.

## Tasks, in order

### 1. Make sure the setup script supports Jazzy

Run `git log --oneline -5` and check `tools/jetson-setup.sh`. If it still hard-codes
`humble` (for example `ros-humble-slam-toolbox` or a Python 3.10 check), the Jazzy fix
has not been pulled. Ask the user whether to `git pull`. If the fix is still missing
after pulling, make the script detect the distro: use `$ROS_DISTRO` or the single
install under `/opt/ros`, expect Python 3.10 for Humble and 3.12 for Jazzy, install
`ros-$DISTRO-slam-toolbox` and `ros-$DISTRO-ackermann-msgs`, and add `python3-dev` and
`build-essential` so pip can build `evdev`.

### 2. Find out what is installed (read-only)

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep -E 'urg|vesc|ackermann|f1tenth|slam|joy'
ip -br addr
find ~ -maxdepth 3 -name install -type d
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Report which driver packages exist, where any F1TENTH workspace is, and every network
interface. A Hokuyo UST LiDAR needs a wired interface with an address on
`192.168.0.x` (the sensor defaults to `192.168.0.10`). Ask the user which LiDAR model
is on the car; do not guess.

### 3. Install the stack (needs the user's OK for sudo)

```bash
bash tools/jetson-setup.sh
```

Watch for whether the pinned AVLite revision and `docker/avlite-constraints.txt`
install on Python 3.12. If a pin fails, report the exact error and propose the
smallest change. Do not loosen pins silently.

### 4. Run the software tests (no hardware)

```bash
/opt/avlite-venv/bin/python -m pytest src/avlite_roboracer/test src/avlite_autodrive/test -q
```

Report failures with their output. Fix failures caused by Python 3.12 or Jazzy
differences; report anything else before changing it.

### 5. Plan the drivers (do not start them)

From steps 2 and the user's answer about the LiDAR, write down which driver packages are
missing for Jazzy and how to get them (apt package or source build). Do not launch any
driver, because the VESC driver can drive the motor. Starting drivers is the user's job,
with the wheels off the ground.

## What to report back

A short summary with:

1. Driver packages found and missing, and the F1TENTH workspace path, if any.
2. Network interfaces, and whether the LiDAR is reachable.
3. Setup script result: success, or the exact error.
4. Test results: pass/fail counts and any failure output.
5. Every file you changed, and why.
6. Open questions for the user (LiDAR model, emergency stop, driver install).

Save the same summary to `log/jetson/agent-report-<date>.md` so it can be read from
another machine.
