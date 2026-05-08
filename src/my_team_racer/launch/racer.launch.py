from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_team_racer",
            executable="racer_node",
            name="racer_node",
            output="screen",
            emulate_tty=True,
        )
    ])
