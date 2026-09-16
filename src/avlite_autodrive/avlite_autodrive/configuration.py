"""Resolve shared driving values before passing numeric settings to AVLite or ROS."""

import math
from pathlib import Path

import yaml


def _read_mapping(path):
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def load_config(filename):
    """Load a profile, resolving ${key} from its relative shared_settings file.

    Plain numeric profiles remain supported. Values are read afresh on startup;
    the shared file is never cached or written back into either profile.
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
