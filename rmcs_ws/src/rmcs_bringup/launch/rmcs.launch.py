from typing import List, Optional
import os
import pathlib

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Vision runtime is co-launched iff the robot yaml loads this component.
_VISION_COMPONENT_TAG = "rmcs::AutoAimComponent"


def _robot_needs_vision(config_path: str) -> bool:
    try:
        return _VISION_COMPONENT_TAG in pathlib.Path(config_path).read_text()
    except OSError:
        return False


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
                output="log",  # stdout and stderr are logged to launch log file and stderr to the screen.
            )
        )

        if _robot_needs_vision(config_path):
            entities.append(
                LogInfo(
                    msg=f"Co-launching rmcs_auto_aim_v2 runtime for '{robot_name}'"
                )
            )
            entities.append(
                Node(
                    package="rmcs_auto_aim_v2",
                    executable="rmcs_auto_aim_v2_runtime",
                    respawn=True,
                    respawn_delay=1.0,
                    output="screen",
                )
            )

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription([MyLaunchDescriptionEntity()])

    return ld
