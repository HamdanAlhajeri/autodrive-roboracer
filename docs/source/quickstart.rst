Quick Start Guide
=================

Get the simulator and your racer node running in under 10 minutes.

.. contents:: On this page
   :local:
   :depth: 2

Prerequisites
-------------

* Ubuntu 22.04 or 24.04 (desktop install — a display is required)
* NVIDIA GPU with drivers installed
* Git

Step 1 — Install Docker
-----------------------

.. code-block:: bash

   curl -fsSL https://get.docker.com | sh
   sudo usermod -aG docker $USER
   newgrp docker          # apply group without logging out
   docker ps              # should print an empty table, not an error

Step 2 — Install NVIDIA Container Toolkit
-----------------------------------------

.. code-block:: bash

   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
     | sudo gpg --dearmor \
         -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

   curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
     | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
     | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

   sudo apt update && sudo apt install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker

Verify GPU access inside a container:

.. code-block:: bash

   docker run --rm --gpus all nvidia/cuda:12.0-base-ubuntu22.04 nvidia-smi

Step 3 — Clone the repository
------------------------------

.. code-block:: bash

   git clone https://github.com/<your-user>/autodrive-roboracer.git
   cd autodrive-roboracer

Step 4 — Pull the Docker images
--------------------------------

.. code-block:: bash

   docker pull autodriveecosystem/autodrive_roboracer_sim:2026-icra-practice
   docker pull autodriveecosystem/autodrive_roboracer_api:2026-icra-practice

Each image is ~3–4 GB. Download only needed once.

Step 5 — Allow display access
------------------------------

Run this once per login session before starting any container:

.. code-block:: bash

   xhost local:root

Step 6 — Start both containers
--------------------------------

.. code-block:: bash

   docker compose up

This launches the Unity simulator and the ROS 2 devkit simultaneously.
The devkit will automatically build ``my_team_racer`` and start the racer node.

Step 7 — Connect in the simulator GUI
--------------------------------------

When the simulator window opens:

1. Leave the IP field as ``127.0.0.1`` and the port as ``4567``.
2. Click **Connection** — both windows should show *"Connected!"*.
3. Click **Driving Mode** to switch from *Manual* to **Autonomous**.

Your car is now lapping autonomously driven by ``racer_node.py``.

.. tip::

   Watch live node output with ``docker logs -f autodrive_roboracer_api``.

Step 8 — Stop everything
--------------------------

.. code-block:: bash

   docker compose down

   # or kill individual containers
   docker kill autodrive_roboracer_sim autodrive_roboracer_api

Troubleshooting
---------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Problem
     - Fix
   * - ``permission denied`` on ``docker ps``
     - Run ``newgrp docker`` or log out and back in after ``sudo usermod -aG docker $USER``
   * - Simulator window does not open
     - Run ``xhost local:root`` before starting the container
   * - *"Connected!"* never appears
     - Confirm both containers use ``--network=host`` (checked in ``docker-compose.yml``)
   * - ``nvidia-smi`` not found in container
     - NVIDIA drivers missing — run ``sudo ubuntu-drivers autoinstall`` and reboot
   * - Container name already in use
     - Run ``docker rm autodrive_roboracer_sim`` or ``docker rm autodrive_roboracer_api``
