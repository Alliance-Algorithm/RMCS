from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="tlarc",
            executable="tlarc",
            respawn=True,
            respawn_delay=5.0,
        )
    )

    return ld
