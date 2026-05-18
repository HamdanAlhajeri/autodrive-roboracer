AutoDRIVE RoboRacer Documentation
==================================

**AutoDRIVE RoboRacer** is a simulated head-to-head racing platform for the ICRA 2026
autonomous racing competition. Two Docker containers — a Unity-based simulator and a
ROS 2 Humble devkit — communicate over a local socket so your algorithm drives a
virtual F1Tenth-style car around a racetrack.

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   quickstart

.. toctree::
   :maxdepth: 3
   :caption: Detailed Guides

   tutorial

.. toctree::
   :maxdepth: 2
   :caption: Reference

   api

----

Architecture at a glance
-------------------------

.. code-block:: text

   ┌─────────────────────────────┐       TCP 127.0.0.1:4567
   │  autodrive_roboracer_sim    │ ◄────────────────────────►
   │  (Unity simulator)          │
   │  • physics & sensor sim     │
   │  • streams LiDAR / camera   │
   └─────────────────────────────┘

   ┌─────────────────────────────┐
   │  autodrive_roboracer_api    │   ROS 2 Humble
   │  (devkit container)         │
   │  • autodrive_bridge node    │   bridges socket ↔ ROS topics
   │  • YOUR racer_node          │   pure pursuit / follow-the-gap
   │  • rviz2                    │   visualisation
   └─────────────────────────────┘
