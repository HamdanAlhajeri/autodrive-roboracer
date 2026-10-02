from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return a ROS launch description for the standalone racer node with console logging."""
    return LaunchDescription([
        Node(
            package="my_team_racer",
            executable="racer_node",
            name="racer_node",
            output="screen",
            emulate_tty=True,
        )
    ])
