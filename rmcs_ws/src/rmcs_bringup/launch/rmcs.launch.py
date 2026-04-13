from typing import List, Optional
import os
import yaml

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

        mavros_cfg = config.get("mavros", {}).get("ros__parameters")
        if mavros_cfg is not None and mavros_cfg.get("enabled", True):
            mavros_share = FindPackageShare("mavros").perform(context)
            pluginlists_yaml = os.path.join(
                mavros_share, "launch", "px4_pluginlists.yaml"
            )
            px4_config_yaml = os.path.join(mavros_share, "launch", "px4_config.yaml")
            fcu_url = mavros_cfg.get(
                "fcu_url",
                f"serial://{mavros_cfg.get('device', '/dev/ttyACM0')}:{mavros_cfg.get('baudrate', 921600)}",
            )
            gcs_url = mavros_cfg.get("gcs_url", "")
            target_system_id = int(mavros_cfg.get("target_system_id", 1))
            target_component_id = int(mavros_cfg.get("target_component_id", 1))
            fcu_protocol = mavros_cfg.get("fcu_protocol", "v2.0")
            respawn = mavros_cfg.get("respawn", True)
            respawn_delay = float(mavros_cfg.get("respawn_delay", 1.0))
            entities.append(
                LogInfo(
                    msg=(
                        f"Starting ROS 2 MAVROS on '{fcu_url}' "
                        f"(target {target_system_id}:{target_component_id})"
                    )
                )
            )
            entities.append(
                Node(
                    package="mavros",
                    executable="mavros_node",
                    namespace="mavros",
                    parameters=[
                        pluginlists_yaml,
                        px4_config_yaml,
                        {
                            "fcu_url": fcu_url,
                            "gcs_url": gcs_url,
                            "tgt_system": target_system_id,
                            "tgt_component": target_component_id,
                            "fcu_protocol": fcu_protocol,
                        },
                    ],
                    respawn=respawn,
                    respawn_delay=respawn_delay,
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
