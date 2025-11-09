from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch = LaunchDescription()

    node = Node(
        package="rmcs_auto_aim_v2",
        executable="rmcs_auto_aim_v2_runtime",
        parameters= [[FindPackageShare("rmcs_auto_aim_v2"), "/config.yaml"]],
        output="screen",
        respawn=True,
        respawn_delay=1.0,
    )
    launch.add_action(node)

    return launch
