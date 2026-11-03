from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self, context: "LaunchContext"
    ) -> Optional[List["LaunchDescriptionEntity"]]:
        entities = []

        robot_config = LaunchConfiguration("robot").perform(context)
        if robot_config.startswith("auto."):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        config_path = os.path.join(
            FindPackageShare("rmcs_bringup").perform(context),
            "config",
            robot_name + ".yaml",
        )

        entities.append(
            LogInfo(
                msg=f"Starting RMCS on robot '{robot_config}'{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml"
            )
        )

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=[config_path],
                respawn=True,
                respawn_delay=1.0,
                output="screen",
            )
        )

        foxglove = LaunchConfiguration("foxglove").perform(context)
        if foxglove == "true" or (
            foxglove == "auto"
            and robot_name in ("gimbal_controller_test", "gimbal_controller_identification", "joint2_lift_test", "small_yaw_identification", "small_yaw_breakaway_identification")
        ):
            entities.append(
                Node(
                    package="foxglove_bridge",
                    executable="foxglove_bridge",
                    name="foxglove_bridge",
                    parameters=[config_path],
                    respawn=True,
                    respawn_delay=1.0,
                    output="screen",
                )
            )
        demo_launch_path = os.path.join(
            FindPackageShare("arm_moveit_config").perform(context),
            "launch",
            "demo.launch.py"
        )
        
        entities.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch_path),
                launch_arguments={
                    "use_rviz": LaunchConfiguration("use_rviz")
                }.items(),
            )
        )
       

        return entities


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "foxglove", default_value="auto", choices=["auto", "true", "false"],
                description="Start Foxglove bridge; auto enables it for gimbal and joint2 test profiles.",
            ),
            MyLaunchDescriptionEntity(),
        ]
    )

    return ld
