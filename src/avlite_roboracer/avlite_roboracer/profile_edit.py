"""Fill in the hardware profile from the command line, so an agent or operator never hand-edits YAML.

    python -m avlite_roboracer.profile_edit set vehicle.wheelbase_m=0.33 steering.positive_left=true
    python -m avlite_roboracer.profile_edit commissioning stops.json
    python -m avlite_roboracer.profile_edit laser-from-tf          # needs the F1TENTH bringup running
    python -m avlite_roboracer.profile_edit deadman-from-joy       # operator holds the button when asked
    python -m avlite_roboracer.profile_edit defaults-localization  # starting acceptance thresholds

Every write is checked first: the key must already exist in the profile, the value must have
the right type, and the whole profile must still pass ``HardwareProfile`` validation (limit
ordering, positive numbers, geometry order). A rejected value changes nothing. The line is
edited in place, so comments in the file are kept, and the previous file is saved as
``hardware.yaml.bak``. Speeds above 1.5 m/s need ``--allow-fast``: the first autonomous tests
should stay at or below 1.0 m/s.
"""

import argparse
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import yaml

from .profile import AUTONOMY_REQUIRED, HardwareProfile

DEFAULT_PROFILE = Path(__file__).resolve().parents[3] / "config" / "roboracer" / "hardware.yaml"

# Keys beyond the autonomy measurements that may be set from here.
EXTRA_KEYS = (
    "steering.center_offset_rad", "topics.imu", "frames.imu", "deadman.timeout_s", "deadman.stop_on_press",
    "localization.scan_alignment_tolerance_m", "localization.stable_duration_s",
    "timing.command_timeout_s", "timing.authorization_timeout_s",
    "timing.sensor_timeout_s", "timing.pose_timeout_s",
)
EDITABLE = frozenset(AUTONOMY_REQUIRED) | frozenset(EXTRA_KEYS)
SPEED_KEYS = ("limits.mapping_speed_mps", "limits.racing_speed_mps", "motor.max_command_speed_mps",
              "measurements.stop_test_speed_mps")
FAST_MPS = 1.5

# Acceptance thresholds that passed in the simulator. They are policy limits, not hardware
# facts: start here, then tighten them from the numbers in `status` and the localization report.
LOCALIZATION_STARTING_POINT = {
    "localization.max_position_stddev_m": 0.15,
    "localization.max_yaw_stddev_rad": 0.08,
    "localization.min_scan_alignment": 0.7,
    "localization.max_motion_mismatch_m": 0.15,
    "localization.max_motion_mismatch_rad": 0.10,
}


def parse_value(text):
    """Turn a command-line value into null, a bool, an int or a float (anything else is refused)."""
    lowered = text.strip().lower()
    if lowered in ("null", "none", "~"):
        return None
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        return int(lowered)
    except ValueError:
        pass
    try:
        value = float(lowered)
    except ValueError:
        raise ValueError(f"cannot read {text!r} as a number, true/false or null") from None
    if not math.isfinite(value):
        raise ValueError(f"{text!r} is not a finite number")
    return value


def format_value(value):
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    return repr(round(float(value), 9))


_VALUE = r"(?P<value>[^\s#,}]+)"


def _replace_in_text(lines, dotted, new_text):
    """Replace the value of ``dotted`` in place; return the old text. Raises KeyError if absent."""
    parts = dotted.split(".")
    start, parent_indent = 0, -1
    for depth, part in enumerate(parts):
        last = depth == len(parts) - 1
        found = None
        for i in range(start, len(lines)):
            raw = lines[i]
            stripped = raw.strip()
            if not stripped or stripped.startswith("#"):
                continue
            indent = len(raw) - len(raw.lstrip())
            if indent <= parent_indent and depth > 0:
                break  # left the parent block
            if depth == 0 and indent != 0:
                continue
            if depth > 0 and indent <= parent_indent:
                break
            if re.match(rf"{re.escape(part)}\s*:", stripped):
                found = i
                break
        if found is None:
            raise KeyError(dotted)
        if last:
            match = re.match(rf"(\s*{re.escape(part)}\s*:\s*){_VALUE}(.*)$", lines[found])
            if not match:
                raise KeyError(dotted)
            old = match.group("value")
            lines[found] = f"{match.group(1)}{new_text}{match.group(3)}"
            return old
        nxt = lines[found]
        if "{" in nxt:  # flow mapping on the same line, e.g.  laser: {x_m: null, y_m: null}
            leaf = parts[depth + 1]
            match = re.search(rf"({re.escape(leaf)}\s*:\s*){_VALUE}", nxt)
            if not match or depth + 2 != len(parts):
                raise KeyError(dotted)
            old = match.group("value")
            lines[found] = nxt[:match.start()] + f"{match.group(1)}{new_text}" + nxt[match.end():]
            return old
        start, parent_indent = found + 1, len(nxt) - len(nxt.lstrip())
    raise KeyError(dotted)


def set_values(path, updates, allow_fast=False, dry_run=False):
    """Write ``{dotted_key: value}`` into the profile. Return ``[(key, old, new)]``.

    Raises ValueError (nothing written) for an unknown key, a bad value, a speed above 1.5 m/s
    without allow_fast, or a result that fails HardwareProfile validation.
    """
    path = Path(path)
    text = path.read_bytes().decode("utf-8")  # raw bytes: read_text would hide Windows line endings
    newline = "\r\n" if "\r\n" in text else "\n"
    lines = text.replace("\r\n", "\n").split("\n")
    changes = []
    for key, value in updates.items():
        if key not in EDITABLE:
            raise ValueError(f"{key} is not a setting this tool writes; editable keys: "
                             + ", ".join(sorted(EDITABLE)))
        if key in SPEED_KEYS and isinstance(value, (int, float)) and not isinstance(value, bool) \
                and value > FAST_MPS and not allow_fast:
            raise ValueError(f"{key}={value} is above {FAST_MPS} m/s; the first autonomous tests "
                             "stay at 1.0 m/s or less. Pass --allow-fast only with measured evidence.")
        try:
            old = _replace_in_text(lines, key, format_value(value))
        except KeyError:
            raise ValueError(f"{key} was not found in {path.name}") from None
        changes.append((key, old, format_value(value)))
    new_text = newline.join(lines)
    try:
        HardwareProfile(yaml.safe_load(new_text), source=path)
    except (ValueError, yaml.YAMLError) as exc:
        raise ValueError(f"rejected, the profile would be invalid: {exc}") from None
    if not dry_run:
        backup = path.with_name(path.name + ".bak")
        backup.write_bytes(text.encode("utf-8"))
        path.write_bytes(new_text.encode("utf-8"))
    return changes


def commissioning_updates(result_path):
    """The profile values a commissioning analysis supports (keys with a measured, non-null value)."""
    data = json.loads(Path(result_path).read_text(encoding="utf-8"))
    suggested = data.get("suggested_profile", data)
    return {key: value for key, value in suggested.items()
            if key in EDITABLE and isinstance(value, (int, float)) and not isinstance(value, bool)}


def parse_tf_echo(text):
    """(x, y, yaw) from the output of ``ros2 run tf2_ros tf2_echo base_link laser``."""
    translation = re.search(r"Translation:\s*\[([^\]]+)\]", text)
    rpy = re.search(r"RPY \(radian\)\s*\[([^\]]+)\]", text)
    if not translation or not rpy:
        raise ValueError("no transform in the tf2_echo output (is the F1TENTH bringup running, "
                         "and is the frame base_link -> laser published?)")
    x, y, _ = (float(v) for v in translation.group(1).split(","))
    yaw = float(rpy.group(1).split(",")[2])
    return x, y, yaw


def pressed_button(before, after):
    """The single button index that is 0 before and 1 after; raises unless exactly one qualifies."""
    changed = [i for i, (a, b) in enumerate(zip(before, after)) if not a and b]
    if len(changed) != 1:
        raise ValueError(f"expected exactly one button to be pressed, saw {changed or 'none'}")
    return changed[0]


def _run(command, timeout):
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False)
        return done.stdout
    except subprocess.TimeoutExpired as exc:
        return (exc.stdout or b"").decode() if isinstance(exc.stdout, bytes) else (exc.stdout or "")
    except FileNotFoundError:
        raise ValueError(f"{command[0]} not found: source the ROS 2 setup first") from None


def _joy_buttons():
    text = _run(["ros2", "topic", "echo", "--once", "/joy"], 6)
    document = yaml.safe_load(text.split("---")[0]) if text.strip() else None
    if not isinstance(document, dict) or "buttons" not in document:
        raise ValueError("no message on /joy (is joy_node running and the joystick on?)")
    return [int(b) for b in document["buttons"]]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--allow-fast", action="store_true", help="accept speeds above 1.5 m/s")
    parser.add_argument("--dry-run", action="store_true", help="check and show, write nothing")
    sub = parser.add_subparsers(dest="command", required=True)
    one = sub.add_parser("set", help="KEY=VALUE ...")
    one.add_argument("pairs", nargs="+")
    com = sub.add_parser("commissioning", help="apply the output of avlite_roboracer.commissioning")
    com.add_argument("result", type=Path)
    sub.add_parser("laser-from-tf", help="read base_link -> laser from the running bringup")
    sub.add_parser("deadman-from-joy", help="find the dead-man button index (operator holds it)")
    sub.add_parser("defaults-localization", help="set the five localization thresholds to the "
                                                 "simulator-proven starting point")
    args = parser.parse_args(argv)
    try:
        if args.command == "set":
            updates = {}
            for pair in args.pairs:
                key, sep, text = pair.partition("=")
                if not sep:
                    raise ValueError(f"{pair!r}: write KEY=VALUE")
                updates[key.strip()] = parse_value(text)
        elif args.command == "commissioning":
            updates = commissioning_updates(args.result)
            if not updates:
                raise ValueError("the commissioning result has no measured values to apply")
        elif args.command == "laser-from-tf":
            x, y, yaw = parse_tf_echo(_run(["timeout", "6", "ros2", "run", "tf2_ros", "tf2_echo",
                                            "base_link", "laser"], 10))
            updates = {"mounts.laser.x_m": round(x, 4), "mounts.laser.y_m": round(y, 4),
                       "mounts.laser.yaw_rad": round(yaw, 4)}
        elif args.command == "deadman-from-joy":
            before = _joy_buttons()
            input("Hold the dead-man button you want to use, then press Enter here... ")
            updates = {"deadman.button": pressed_button(before, _joy_buttons())}
        else:
            updates = dict(LOCALIZATION_STARTING_POINT)
        changes = set_values(args.profile, updates, args.allow_fast, args.dry_run)
    except (ValueError, OSError) as exc:
        print(f"Not changed: {exc}", file=sys.stderr)
        return 1
    for key, old, new in changes:
        print(f"{'would set' if args.dry_run else 'set'} {key}: {old} -> {new}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
