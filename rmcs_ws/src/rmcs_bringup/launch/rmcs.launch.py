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
                output="screen",
                emulate_tty=True,
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

        odin_cfg = config.get("odin_ros_driver", {}).get("ros__parameters")
        if odin_cfg is not None and odin_cfg.get("enabled", True):
            odin_share = FindPackageShare("odin_ros_driver").perform(context)
            default_odin_config_file = os.path.join(
                odin_share, "config", "control_command.yaml"
            )
            odin_config_file = odin_cfg.get("config_file") or default_odin_config_file
            if not os.path.isfile(odin_config_file):
                entities.append(
                    LogInfo(
                        msg=(
                            f"Odin config '{odin_config_file}' was not found; "
                            f"falling back to '{default_odin_config_file}'"
                        )
                    )
                )
                odin_config_file = default_odin_config_file
            node_name = odin_cfg.get("node_name", "host_sdk_sample")
            respawn = odin_cfg.get("respawn", True)
            respawn_delay = float(odin_cfg.get("respawn_delay", 1.0))
            entities.append(
                LogInfo(
                    msg=(
                        f"Starting odin_ros_driver node '{node_name}' "
                        f"with config '{odin_config_file}'"
                    )
                )
            )
            entities.append(
                Node(
                    package="odin_ros_driver",
                    executable="host_sdk_sample",
                    name=node_name,
                    parameters=[{"config_file": odin_config_file}],
                    respawn=respawn,
                    respawn_delay=respawn_delay,
                    output="screen",
                    emulate_tty=True,
                )
            )

        xrce_cfg = config.get("xrce_dds_agent", {}).get("ros__parameters")
        if xrce_cfg is not None:
            device = xrce_cfg.get("device", "/dev/ttyS0")
            baudrate = str(xrce_cfg.get("baudrate", 921600))
            entities.append(
                LogInfo(msg=f"Starting MicroXRCEAgent on {device} @ {baudrate} baud")
            )
            entities.append(
                ExecuteProcess(
                    cmd=[
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
