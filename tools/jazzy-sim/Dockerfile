# Mimics the Jetson: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, user fasty with sudo.
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends sudo git python3-pytest \
    && rm -rf /var/lib/apt/lists/* \
    && useradd -m -s /bin/bash fasty && echo 'fasty ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/fasty
USER fasty
WORKDIR /home/fasty
