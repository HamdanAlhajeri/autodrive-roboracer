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
   :widths: 45 55

   * - Member
     - Description
   * - ``__init__()``
     - Creates subscriptions and publishers, logs startup message including active algorithm.
   * - ``_lidar_cb(msg)``
     - Called on every LaserScan. Pre-processes ranges, dispatches to the active steering
       function based on ``ALGORITHM``, then publishes throttle and steering.
   * - ``_pure_pursuit_steer(ranges, angle_min, angle_inc)``
     - Wall-following centerline algorithm. Returns steering in ``[-1, 1]``.
       See :ref:`pure-pursuit-math`.
   * - ``_gap_follow_steer(ranges, angles, angle_inc)``
     - Follow-the-Gap algorithm. Returns steering in ``[-1, 1]``.
       See :ref:`gap-follow-math`.
   * - ``_speed_from_steer(steering)``
     - Shared throttle scheduler. Returns throttle in ``[MIN_THROTTLE, MAX_THROTTLE]``.

Algorithm selector
~~~~~~~~~~~~~~~~~~

Set ``ALGORITHM`` at the top of ``racer_node.py`` to switch between implementations:

.. code-block:: python

   ALGORITHM = "gap_follow"    # recommended — handles hairpins
   ALGORITHM = "pure_pursuit"  # original wall-following baseline

Tunable constants
~~~~~~~~~~~~~~~~~

Defined at module level in ``racer_node.py``.
Change these values and rebuild the package — no other file needs editing.

.. list-table::
   :header-rows: 1
   :widths: 28 12 12 48

   * - Constant
     - Default
     - Used by
     - Effect
   * - ``LOOKAHEAD_DIST``
     - ``1.0``
     - pure_pursuit
     - Metres ahead on the estimated centreline to steer toward.
       Longer = smoother but reacts later to curves.
   * - ``MAX_THROTTLE``
     - ``0.15``
     - both
     - Top speed on straights (range 0–1).
   * - ``MIN_THROTTLE``
     - ``0.07``
     - both
     - Floor speed so the car never stalls in tight corners.
   * - ``STEER_GAIN``
     - ``1.4``
     - both
     - Proportional gain on the heading error or gap angle.
       Raise to react faster; lower to reduce oscillation.
   * - ``THROTTLE_DECAY``
     - ``4.5``
     - both
     - Exponential decay rate on throttle vs. steering magnitude.
       Higher = harder braking in corners.
   * - ``WALL_CLIP_DIST``
     - ``4.0``
     - both
     - LiDAR readings beyond this distance are clipped (metres).
   * - ``MASK_THRESH``
     - ``0.4``
     - pure_pursuit
     - Emergency brake threshold: stop if a wall is closer than this
       directly ahead (metres).
   * - ``CAR_HALF_WIDTH``
     - ``0.2``
     - gap_follow
     - Half the car width in metres. Sets the angular size of the
       safety bubble blanked around the closest obstacle.

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
