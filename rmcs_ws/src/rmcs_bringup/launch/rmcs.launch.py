from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self, context: "LaunchContext"
    ) -> Optional[List["LaunchDescriptionEntity"]]:
        entities = []

        robot_config = LaunchConfiguration("robot").perform(context)
        auto_aim_config = LaunchConfiguration("enable_auto_aim").perform(context).strip().lower()
        enable_auto_aim = auto_aim_config in {"1", "true", "yes", "on"}

        if robot_config.startswith("auto."):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        entities.append(
            LogInfo(
                msg=f"Starting RMCS on robot '{robot_config}'{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml"
            )
        )

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
                output="log",  # stdout and stderr are logged to launch log file and stderr to the screen.
            )
        )

        if enable_auto_aim:
            entities.append(LogInfo(msg="Auto Aim: ENABLED (launching rmcs_auto_aim_v2)."))
            entities.append(
                Node(
                    package="rmcs_auto_aim_v2",
                    executable="rmcs_auto_aim_v2_runtime",
                    respawn=True,
                    respawn_delay=1.0,
                    output="log",
                )
            )
        else:
            entities.append(LogInfo(msg="Auto Aim: DISABLED (rmcs_auto_aim_v2 will not be launched)."))

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_auto_aim",
                default_value="true",
                description="Whether to launch rmcs_auto_aim_v2 auto-aim runtime node.",
            ),
            MyLaunchDescriptionEntity(),
        ]
    )

    return ld
