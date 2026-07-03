from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

        entities.append(
            LogInfo(msg=f"Starting RMCS on robot -> {robot_name}.yaml"))

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=[
                    os.path.join(
                        FindPackageShare("rmcs_bringup").perform(context),
                        "config",
                        robot_name + ".yaml",
                    ),
                ],
                respawn=True,
                respawn_delay=1.0,
                output="log",
            )
        )

        if robot_name == "flight":
            entities.append(
                Node(
                    package="odin_ros_driver",
                    executable="tmux-launch.sh",
                    output="screen",
                    emulate_tty=True,
                )
            )

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription([MyLaunchDescriptionEntity()])

    return ld
