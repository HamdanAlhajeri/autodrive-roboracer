"""Resolve shared driving values before passing numeric settings to AVLite or ROS."""

import math
from pathlib import Path

import yaml


def _read_mapping(path):
    """Read a UTF-8 YAML file and require a dictionary at its top level.

    Raise ValueError for lists or scalar values, since profiles are addressed by setting
    names.
    """
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def load_config(filename):
    """Load a profile and replace ${name} values with shared driving settings.

    Resolve shared_settings relative to the profile file, check speed/throttle values, and
    walk nested dictionaries and lists. Return numeric settings without changing either
    file; each call reads the latest saved values.
    """
    path = Path(filename)
    profile = _read_mapping(path)
    shared_file = profile.pop("shared_settings", None)
    shared = {}
    if shared_file is not None:
        if not isinstance(shared_file, str) or not shared_file.strip():
            raise ValueError(f"{path}: shared_settings must be a file path")
        shared_path = path.parent / shared_file
        shared = _read_mapping(shared_path)
        if set(shared) != {"speed_mps", "max_throttle"}:
            raise ValueError(f"{shared_path}: expected speed_mps and max_throttle")
        for key, value in shared.items():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{shared_path}: {key} must be a finite positive number")
            shared[key] = float(value)
        if shared["max_throttle"] > 1:
            raise ValueError(f"{shared_path}: max_throttle must be at most 1")

    def resolve(value):
        """Walk one configuration value, replacing shared placeholders wherever they occur.

        Keep ordinary values unchanged and reject unknown placeholder names instead of
        silently using a default.
        """
        if isinstance(value, dict):
            return {key: resolve(item) for key, item in value.items()}
        if isinstance(value, list):
            return [resolve(item) for item in value]
        if isinstance(value, str) and value.startswith("${"):
            key = value[2:-1]
            if not value.endswith("}") or key not in shared:
                raise ValueError(f"{path}: unknown shared reference {value!r}")
            return shared[key]
        return value

    return resolve(profile)
