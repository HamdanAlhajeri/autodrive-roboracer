Detailed Tutorial
=================

This guide walks through the complete development workflow: understanding the
architecture, modifying the racing algorithm, testing inside the container, and
preparing a competition submission.

.. contents:: On this page
   :local:
   :depth: 3

1. Project layout
-----------------

.. code-block:: text

   autodrive-roboracer/
   ├── docker-compose.yml          # production: both containers auto-start
   ├── docker-compose.dev.yml      # dev override: interactive shells + test runner
   ├── run_tests.sh                # one-shot test runner inside devkit container
   ├── docker/
   │   └── autodrive_devkit.sh    # competition submission entrypoint
   ├── docs/                       # this documentation (Sphinx)
   └── src/
       └── my_team_racer/          # YOUR ROS 2 package — edit only this
           ├── my_team_racer/
           │   └── racer_node.py  # racing algorithm
           ├── launch/
           │   └── racer.launch.py
           ├── test/
           │   └── test_racer_node.py
           ├── package.xml
           ├── setup.py
           └── setup.cfg

.. warning::

   Do **not** modify the ``autodrive_roboracer`` package that ships inside the
   devkit image. Your work lives exclusively inside ``src/my_team_racer/``.

2. How the system communicates
-------------------------------

The simulator and devkit containers share ``--network=host``, so they see each
other at ``127.0.0.1``. The ``autodrive_bridge`` ROS node (provided in the devkit
image) opens a TCP socket to port ``4567`` and translates the binary stream into
standard ROS 2 messages.

.. code-block:: text

   Simulator (Unity)
       │  binary sensor frames over TCP :4567
       ▼
   autodrive_bridge node
       │  publishes sensor topics
       ▼
   YOUR racer_node
       │  publishes command topics
       ▼
   autodrive_bridge node
       │  serialises commands back to TCP
       ▼
   Simulator — applies throttle + steering to the physics model

3. ROS 2 topics reference
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 45 15 40

   * - Topic
     - Direction
     - Message type
   * - ``/autodrive/roboracer_1/lidar``
     - Input
     - ``sensor_msgs/msg/LaserScan``
   * - ``/autodrive/roboracer_1/front_camera``
     - Input
     - ``sensor_msgs/msg/Image``
   * - ``/autodrive/roboracer_1/imu``
     - Input
     - ``sensor_msgs/msg/Imu``
   * - ``/autodrive/roboracer_1/left_encoder``
     - Input
     - ``sensor_msgs/msg/JointState``
   * - ``/autodrive/roboracer_1/right_encoder``
     - Input
     - ``sensor_msgs/msg/JointState``
   * - ``/autodrive/roboracer_1/throttle_command``
     - **Output**
     - ``std_msgs/msg/Float32`` [-1, 1]
   * - ``/autodrive/roboracer_1/steering_command``
     - **Output**
     - ``std_msgs/msg/Float32`` [-1, 1]
   * - ``/autodrive/roboracer_1/ips`` *(restricted)*
     - Debug only
     - ``geometry_msgs/msg/Point``
   * - ``/autodrive/roboracer_1/odom`` *(restricted)*
     - Debug only
     - ``nav_msgs/msg/Odometry``

.. note::

   Restricted topics (``ips``, ``odom``) are available for offline training and
   debugging but are **disabled during official race runs**.

4. Understanding racer_node.py
-------------------------------

The algorithm lives in :file:`src/my_team_racer/my_team_racer/racer_node.py`.

4.1 Tunable constants
~~~~~~~~~~~~~~~~~~~~~

At the top of the file:

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Constant
     - Default
     - Effect
   * - ``LOOKAHEAD_DIST``
     - ``0.8 m``
     - Distance ahead on the centerline to steer toward. Longer = smoother but reacts later to curves.
   * - ``MAX_THROTTLE``
     - ``0.6``
     - Top speed on straights (range 0–1).
   * - ``MIN_THROTTLE``
     - ``0.15``
     - Floor speed so the car never stalls in tight corners.
   * - ``STEER_GAIN``
     - ``1.2``
     - Proportional gain on the heading error. Raise to react faster; lower to reduce oscillation.
   * - ``THROTTLE_DECAY``
     - ``2.5``
     - How sharply throttle drops with steering angle. Higher = more braking in corners.
   * - ``WALL_CLIP_DIST``
     - ``4.0 m``
     - LiDAR readings beyond this are clipped (ignore far-away obstacles).

4.2 Algorithm walkthrough
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step 1 — Pre-process the scan**

``LaserScan.ranges`` may contain ``inf`` or ``NaN`` where the beam hit nothing.
These are replaced with ``WALL_CLIP_DIST`` and then clipped to ``[0, WALL_CLIP_DIST]``.

.. code-block:: python

   ranges = np.where(np.isfinite(ranges), ranges, WALL_CLIP_DIST)
   ranges = np.clip(ranges, 0.0, WALL_CLIP_DIST)

**Step 2 — Convert to cartesian**

Each beam angle is ``angle_min + i * angle_increment`` (radians).
The car's x-axis points forward, y-axis points left.

.. code-block:: python

   xs = ranges * np.cos(angles)
   ys = ranges * np.sin(angles)

**Step 3 — Estimate centerline error**

Only forward-facing beams (``xs > 0``) are used.
Left-wall distance = mean ``y`` of left beams; right-wall distance = mean ``-y`` of right beams.

.. code-block:: python

   center_error = (left_dist - right_dist) / (left_dist + right_dist)

A positive error means the car is closer to the right wall and should steer left.

**Step 4 — Pure-pursuit geometry**

A lookahead point is placed ``LOOKAHEAD_DIST`` ahead and offset laterally by
``center_error * LOOKAHEAD_DIST``. The required heading change is:

.. code-block:: python

   heading_err = math.atan2(ly, lx)          # ly = lateral offset, lx = lookahead distance
   steering = clip(STEER_GAIN * heading_err, -1, 1)

**Step 5 — Speed scheduling**

Throttle decays exponentially with the absolute steering command:

.. code-block:: python

   throttle = MAX_THROTTLE * exp(-THROTTLE_DECAY * abs(steering))
   throttle = clip(throttle, MIN_THROTTLE, MAX_THROTTLE)

5. Development workflow
------------------------

5.1 Edit → build → test loop
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open an interactive devkit shell:

.. code-block:: bash

   docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm devkit

Inside the container, your source files are mounted at
``/home/autodrive_devkit/src/my_packages``.

.. code-block:: bash

   # Inside the container
   source /opt/ros/humble/setup.bash
   cd /home/autodrive_devkit
   colcon build --packages-select my_team_racer --symlink-install
   source install/setup.bash

   # Run unit tests (no simulator needed)
   pytest src/my_packages/my_team_racer/test/ -v

   # Or launch with a live simulator
   ros2 launch autodrive_roboracer bringup_graphics.launch.py &
   ros2 launch my_team_racer racer.launch.py

Changes you save on the **host** are immediately visible inside the container
because ``src/`` is bind-mounted.

5.2 Running the full test suite from the host
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   ./run_tests.sh

This is equivalent to:

.. code-block:: bash

   docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm test

No display or GPU is required for the test run.

5.3 Introspecting live topics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

With both containers running:

.. code-block:: bash

   # Open a second shell into the devkit container
   docker exec -it autodrive_roboracer_api bash

   # Inside
   source /opt/ros/humble/setup.bash
   source /home/autodrive_devkit/install/setup.bash
   ros2 topic echo /autodrive/roboracer_1/steering_command
   ros2 topic echo /autodrive/roboracer_1/throttle_command
   ros2 topic hz  /autodrive/roboracer_1/lidar

5.4 Attaching VS Code to the running container
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Install the **Dev Containers** extension.
2. Press ``Ctrl+Shift+P`` → **Dev Containers: Attach to Running Container**.
3. Select ``autodrive_roboracer_api``.

You now have IntelliSense and the integrated terminal inside the container.

6. Writing your own algorithm
------------------------------

You can replace the pure-pursuit controller with any approach.
The contract is simple:

1. Create a ``rclpy.node.Node`` subclass.
2. Subscribe to at least ``/autodrive/roboracer_1/lidar``.
3. Publish ``Float32`` values in ``[-1, 1]`` to ``throttle_command`` and
   ``steering_command``.

Do **not** subscribe to restricted topics (``ips``, ``odom``) for control logic —
these are disabled during competition runs.

Common algorithm choices:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Algorithm
     - Notes
   * - Pure pursuit (current)
     - Simple, robust. Tune ``LOOKAHEAD_DIST`` and ``STEER_GAIN``.
   * - Gap-follow / disparity extender
     - Drives toward the largest gap in the LiDAR scan. Very fast.
   * - MPC
     - Model-predictive control using odometry + track map. High performance, complex to tune.
   * - Imitation learning
     - Record expert laps (use ``odom`` + ``ips`` offline), train a network, deploy for inference.

7. Persisting work between sessions
-------------------------------------

The ``src/`` directory on your host is always mounted into the container, so your
code persists naturally. If you also need to save container state (installed apt
packages, trained model weights, etc.):

.. code-block:: bash

   # Commit the running container to a new local image
   docker commit autodrive_roboracer_api my_racer_checkpoint:v1

   # Restore later
   docker run ... my_racer_checkpoint:v1 -c '...'

8. Competition submission
--------------------------

The submission is a DockerHub image built from your modified devkit container.

.. code-block:: bash

   # 1. Make sure autodrive_devkit.sh is your entrypoint script
   #    It must build + launch all nodes automatically.
   #    See docker/autodrive_devkit.sh for the template.

   # 2. Commit the running devkit container
   docker commit -m "racing algorithm v1" \
       autodrive_roboracer_api \
       <dockerhub_username>/<image_name>:2026-icra-practice

   # 3. Push to DockerHub
   docker login
   docker push <dockerhub_username>/<image_name>:2026-icra-practice

.. warning::

   The entrypoint for your submitted image **must not** use ``~/.bashrc`` to start
   nodes. Organizers open extra ``bash`` sessions into your container for inspection
   — any ``~/.bashrc`` autostart would trigger unexpectedly.
   Use ``docker/autodrive_devkit.sh`` as the explicit ``ENTRYPOINT`` instead.

8.1 Baking the entrypoint into the image
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a ``Dockerfile`` (in the repo root) that extends the devkit image:

.. code-block:: dockerfile

   FROM autodriveecosystem/autodrive_roboracer_api:2026-icra-practice

   COPY src/ /home/autodrive_devkit/src/my_packages/
   COPY docker/autodrive_devkit.sh /home/autodrive_devkit/autodrive_devkit.sh
   RUN chmod +x /home/autodrive_devkit/autodrive_devkit.sh

   ENTRYPOINT ["/home/autodrive_devkit/autodrive_devkit.sh"]

Build and push:

.. code-block:: bash

   docker build -t <dockerhub_username>/<image_name>:2026-icra-practice .
   docker push <dockerhub_username>/<image_name>:2026-icra-practice
