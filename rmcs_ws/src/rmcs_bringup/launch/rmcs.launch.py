from typing import List, Optional
import os
import yaml

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import ExecuteProcess, LogInfo
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

        # Conditionally launch optional processes declared in the config yaml.
        config_path = os.path.join(
            FindPackageShare("rmcs_bringup").perform(context),
            "config",
            robot_name + ".yaml",
        )
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        xrce_cfg = config.get("xrce_dds_agent")
        if xrce_cfg is not None:
            device = xrce_cfg.get("device", "/dev/ttyS0")
            baudrate = str(xrce_cfg.get("baudrate", 921600))
            entities.append(
                LogInfo(msg=f"Starting MicroXRCEAgent on {device} @ {baudrate} baud")
            )
            entities.append(
                ExecuteProcess(
                    cmd=[
                        "sudo",
                        "MicroXRCEAgent",
                        "serial",
                        "--dev",
                        device,
                        "-b",
                        baudrate,
                    ],
                    output="log",
                    respawn=True,
                    respawn_delay=1.0,
                )
            )

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription([MyLaunchDescriptionEntity()])

    return ld
