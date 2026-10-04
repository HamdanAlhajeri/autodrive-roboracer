"""Start the independent actuator and the supervisor on the Jetson.

    ros2 launch avlite_roboracer roboracer.launch.py config_dir:=/path/to/config/roboracer

Nothing moves until a laptop sends map-start or race-start; a restart never resumes.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config_dir")
    profile = PathJoinSubstitution([config, "hardware.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config_dir", default_value=os.path.expanduser(
            "~/autodrive-roboracer/config/roboracer")),
        Node(package="avlite_roboracer", executable="roboracer_actuator",
             name="roboracer_actuator", output="screen", arguments=["--profile", profile]),
        Node(package="avlite_roboracer", executable="roboracer_supervisor",
             name="roboracer_supervisor", output="screen",
             arguments=["--profile", profile,
                        "--race-config", PathJoinSubstitution([config, "race.yaml"]),
                        "--slam-params", PathJoinSubstitution([config, "slam_toolbox.yaml"])]),
    ])
