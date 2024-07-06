from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="rmcs_executor",
            executable="rmcs_executor",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("rmcs_bringup"),
                        "config",
                        LaunchConfiguration("config"),
                    ]
                )
            ],
            respawn=True,
            respawn_delay=5.0,
        )
    )

    return ld
