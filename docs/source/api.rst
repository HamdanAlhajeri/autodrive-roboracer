API Reference
=============

.. contents:: On this page
   :local:
   :depth: 2

racer_node module
-----------------

.. code-block:: text

   src/my_team_racer/my_team_racer/racer_node.py

RacerNode
~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Member
     - Description
   * - ``__init__()``
     - Creates subscriptions and publishers, logs startup message.
   * - ``_lidar_cb(msg)``
     - Called on every LaserScan. Pre-processes ranges, calls steering/throttle helpers, publishes.
   * - ``_pure_pursuit_steer(ranges, angle_min, angle_inc)``
     - Returns a steering value in ``[-1, 1]``. See :ref:`algorithm-walkthrough`.
   * - ``_speed_from_steer(steering)``
     - Returns a throttle value in ``[MIN_THROTTLE, MAX_THROTTLE]``.

.. _algorithm-walkthrough:

Tunable constants
~~~~~~~~~~~~~~~~~

Defined at module level in ``racer_node.py``.
Change these values and rebuild the package — no other file needs editing.

.. code-block:: python

   LOOKAHEAD_DIST  = 0.8    # metres
   MAX_THROTTLE    = 0.6    # [0, 1]
   MIN_THROTTLE    = 0.15
   STEER_GAIN      = 1.2
   THROTTLE_DECAY  = 2.5
   WALL_CLIP_DIST  = 4.0    # metres

launch files
------------

``racer.launch.py``
~~~~~~~~~~~~~~~~~~~

Starts a single ``racer_node`` with output echoed to the terminal.

.. code-block:: bash

   ros2 launch my_team_racer racer.launch.py

test suite
----------

``test/test_racer_node.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Run without a live ROS context — ROS 2 modules are stubbed out.

.. code-block:: bash

   # Inside the devkit container
   pytest src/my_packages/my_team_racer/test/ -v

   # Or from the host
   ./run_tests.sh

.. list-table::
   :header-rows: 1
   :widths: 55 45

   * - Test
     - What it checks
   * - ``test_straight_track_zero_steer``
     - Symmetric LiDAR scan → steering ≈ 0
   * - ``test_straight_track_max_throttle``
     - Symmetric scan → throttle = ``MAX_THROTTLE``
   * - ``test_all_inf_scan``
     - Open-space scan (all ``inf``) does not crash; output is bounded
   * - ``test_steering_clamped``
     - Steering always in ``[-1, 1]`` for any scan
   * - ``test_throttle_bounds``
     - Throttle always in ``[MIN_THROTTLE, MAX_THROTTLE]``
   * - ``test_throttle_lower_in_corner``
     - Close wall → steering → throttle drops below straight value
   * - ``test_symmetric_gives_zero``
     - Pure-pursuit math: symmetric input → zero error
   * - ``test_right_wall_closer_steers_left``
     - Close right wall → positive (left) steering
   * - ``test_left_wall_closer_steers_right``
     - Close left wall → negative (right) steering
