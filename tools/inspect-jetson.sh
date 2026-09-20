#!/usr/bin/env bash
# Read-only preparation report. Does not install software, change networking,
# start a driving stack, or publish vehicle commands. No sudo required.
set -u

heading() { printf '\n%s\n' "$1"; }
bounded() {
    if command -v timeout >/dev/null 2>&1; then
        timeout 12s "$@" 2>&1 || printf 'Unavailable or timed out: %s\n' "$1"
    else
        printf 'Skipping %s: timeout is unavailable.\n' "$1"
    fi
}

printf 'RoboRacer Jetson inspection — no vehicle commands sent\n'
heading 'Hardware and OS'
if [[ -r /proc/device-tree/model ]]; then
    tr -d '\000' < /proc/device-tree/model; printf '\n'
elif [[ -r /sys/firmware/devicetree/base/model ]]; then
    tr -d '\000' < /sys/firmware/devicetree/base/model; printf '\n'
else
    printf 'Jetson device-tree model unavailable on this machine.\n'
fi
uname -m
if [[ -r /etc/os-release ]]; then
    while IFS= read -r line; do
        case "$line" in PRETTY_NAME=*|VERSION_ID=*) printf '%s\n' "$line" ;; esac
    done < /etc/os-release
fi
if [[ -r /etc/nv_tegra_release ]]; then cat /etc/nv_tegra_release; fi
if command -v dpkg-query >/dev/null 2>&1; then
    dpkg-query -W -f='${Package} ${Version}\n' nvidia-jetpack nvidia-l4t-core 2>/dev/null || true
fi
if command -v python3 >/dev/null 2>&1; then python3 --version; fi

heading 'ROS environment'
printf 'ROS_VERSION=%s\nROS_DISTRO=%s\n' "${ROS_VERSION:-not sourced}" "${ROS_DISTRO:-not sourced}"
for setup in /opt/ros/*/setup.bash; do
    [[ -f "$setup" ]] && printf 'Available ROS setup: %s\n' "$setup"
done
if command -v ros2 >/dev/null 2>&1; then
    heading 'ROS 2 topics and message types (existing graph only)'
    bounded ros2 topic list -t --no-daemon
    heading 'ROS 2 nodes (existing graph only)'
    bounded ros2 node list --no-daemon
elif command -v rostopic >/dev/null 2>&1; then
    heading 'ROS 1 topics (existing graph only)'
    bounded rostopic list
else
    printf 'ROS commands unavailable in this shell. Source the installed ROS and vehicle workspace, then rerun.\n'
fi

heading 'USB devices'
if command -v lsusb >/dev/null 2>&1; then lsusb; else printf 'lsusb unavailable.\n'; fi

heading 'Local addresses and SSH availability'
if command -v hostname >/dev/null 2>&1; then hostname -I 2>/dev/null || true; fi
if command -v systemctl >/dev/null 2>&1; then
    systemctl is-active ssh 2>/dev/null || true
    systemctl is-active sshd 2>/dev/null || true
fi
if command -v ssh >/dev/null 2>&1; then ssh -V 2>&1; fi
printf '\nAn SSH client does not prove an SSH server is running or reachable from your laptop.\n'
printf 'This report does not prove sensor mounting, steering signs, braking response, or emergency-stop operation.\n'
printf 'Also provide LiDAR/IMU/motor-controller models and the physical stop/manual-override arrangement.\n'
