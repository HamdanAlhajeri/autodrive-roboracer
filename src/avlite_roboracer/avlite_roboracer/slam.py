"""Start, stop and configure SLAM Toolbox processes (no rclpy import).

At most one SLAM process runs at a time, so exactly one node publishes map -> odom.
Switching from mapping to localization stops the mapping process before starting the
localization process with the saved pose graph and the car's current pose.
"""

import os
from pathlib import Path
import signal
import subprocess

import yaml

LAUNCH = {"mapping": "online_async_launch.py", "localization": "localization_launch.py"}


def require_save_success(response, service):
    """SLAM save services report failure in result, even when transport succeeds."""
    if response.result != 0:
        raise RuntimeError(f"{service} failed with result {response.result}")
    return response


def effective_parameters(base_file, profile, mode, output, map_file=None, start_pose=None,
                         use_sim_time=False):
    """Write SLAM Toolbox parameters with frames, topic and mode taken from the profile.

    map_file is the serialized pose graph without extension; start_pose is (x, y, yaw) in
    the map frame. Return the written path.
    """
    if mode not in LAUNCH:
        raise ValueError(f"unknown SLAM mode {mode!r}")
    data = yaml.safe_load(Path(base_file).read_text(encoding="utf-8"))
    params = data["slam_toolbox"]["ros__parameters"]
    if params.get("enable_interactive_mode"):
        raise ValueError("enable_interactive_mode must be false to publish pose-graph vertices")
    params.update(odom_frame=profile.frame("odom"), map_frame=profile.frame("map"),
                  base_frame=profile.frame("base_link"), scan_topic=profile.topic("scan"),
                  mode=mode, use_sim_time=use_sim_time)
    if mode == "localization":
        if map_file is None or start_pose is None:
            raise ValueError("localization needs the saved pose graph and a start pose")
        params.update(map_file_name=str(map_file),
                      map_start_pose=[float(v) for v in start_pose])
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return output


class SlamProcess:
    """Own one `ros2 launch slam_toolbox ...` process group."""

    def __init__(self, log_dir):
        self.log_dir = Path(log_dir)
        self.process = None
        self.mode = None
        self._log = None

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    @property
    def died(self):
        """True if a process was started and exited without stop() being called."""
        return self.process is not None and self.process.poll() is not None

    def start(self, mode, params_file, use_sim_time=False, env=None):
        if self.running:
            raise RuntimeError(f"SLAM already running in {self.mode} mode")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._log = (self.log_dir / f"slam_{mode}.log").open("ab")
        command = ["ros2", "launch", "slam_toolbox", LAUNCH[mode],
                   f"slam_params_file:={params_file}",
                   f"use_sim_time:={'true' if use_sim_time else 'false'}"]
        self.process = subprocess.Popen(command, stdout=self._log, stderr=subprocess.STDOUT,
                                        start_new_session=True, env=env)
        self.mode = mode

    def stop(self, timeout_s=10.0):
        """Interrupt the whole launch process group, escalating if it does not exit."""
        if self.process is not None and self.process.poll() is None:
            for sig, wait in ((signal.SIGINT, timeout_s), (signal.SIGTERM, 3.0),
                              (signal.SIGKILL, 3.0)):
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    self.process.wait(wait)
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.process = None
        self.mode = None
        if self._log is not None:
            self._log.close()
            self._log = None
