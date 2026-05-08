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

This starts three containers:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Container
     - What it does
   * - ``autodrive_roboracer_sim``
     - Unity simulator — opens the racetrack window
   * - ``autodrive_roboracer_api``
     - ROS 2 devkit — builds your package then launches the racer node
   * - ``autodrive_docs``
     - Sphinx docs server at ``http://localhost:8000``

.. note::

   The devkit container takes **15–30 seconds** to appear in ``docker ps`` because
   it compiles the ROS 2 package before starting the nodes.
   Check progress with:

   .. code-block:: bash

      docker logs -f autodrive_roboracer_api

   Wait until you see:

   .. code-block:: text

      [devkit] Building my_team_racer...
      Finished <<< my_team_racer
      [devkit] Launching autodrive bringup...
      [devkit] Launching racer node...
      [racer_node]: RacerNode started — pure pursuit via LiDAR

Step 7 — Connect in the simulator GUI
--------------------------------------

Once the simulator window is open and the devkit log shows *"RacerNode started"*:

1. Leave the IP field as ``127.0.0.1`` and the port as ``4567``.
2. Click **Connection** — both windows should show *"Connected!"*.
3. Click **Driving Mode** to switch from *Manual* to **Autonomous**.

Your car is now lapping autonomously driven by ``racer_node.py``.

.. tip::

   Stream live steering/throttle commands:

   .. code-block:: bash

      docker exec -it autodrive_roboracer_api bash -c \
        "source /opt/ros/humble/setup.bash && \
         source /home/autodrive_devkit/install/setup.bash && \
         ros2 topic echo /autodrive/roboracer_1/steering_command"

Step 8 — Stop everything
--------------------------

.. code-block:: bash

   docker compose down

Step 9 — Restarting after a reboot
------------------------------------

X11 access is reset on every login. Always run this first:

.. code-block:: bash

   xhost local:root
   docker compose up

If the devkit container exited and you want to restart only it without stopping
the simulator:

.. code-block:: bash

   docker compose up devkit

Troubleshooting
---------------

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Problem
     - Fix
   * - ``permission denied`` on ``docker ps``
     - Run ``newgrp docker`` or log out and back in after ``sudo usermod -aG docker $USER``
   * - Devkit not visible in ``docker ps`` after ``docker compose up``
     - It exited immediately. Check why: ``docker logs autodrive_roboracer_api``. Most likely cause: forgot ``xhost local:root`` before starting, or stale container — run ``docker compose down`` then ``docker compose up`` again
   * - Devkit shows ``ros2: command not found`` in logs
     - You are running an old version of ``docker-compose.yml``. Pull the latest from git — the entrypoint now sources ``/opt/ros/humble/setup.bash`` before launching
   * - Simulator window does not open
     - Run ``xhost local:root`` before starting the containers
   * - *"Connected!"* never appears in simulator
     - Confirm both containers use ``network_mode: host`` (already set in ``docker-compose.yml``). Check ``docker logs autodrive_roboracer_api`` for bridge errors
   * - ``nvidia-smi`` not found in container
     - NVIDIA drivers not installed — run ``sudo ubuntu-drivers autoinstall`` and reboot
   * - Container name already in use
     - Run ``docker compose down`` to clear all containers, then ``docker compose up``
