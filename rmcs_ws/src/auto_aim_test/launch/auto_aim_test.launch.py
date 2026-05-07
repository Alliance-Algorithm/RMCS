from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="auto_aim_test",
                executable="auto_aim_test",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("auto_aim_test"), "config", "auto_aim_test.yaml"]
                    )
                ],
                output="screen",
            )
        ]
    )
