from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

        executor_parameters = [
            os.path.join(
                FindPackageShare("rmcs_bringup").perform(context),
                "config",
                robot_name + ".yaml",
            )
        ]
        if robot_name in ("visual_mining_test", "lunar_rover_engineer", "climber_engineer"):
            from moveit_configs_utils import MoveItConfigsBuilder

            moveit_config = (
                MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
                .robot_description(file_path="config/arm_description.urdf.xacro")
                .robot_description_semantic(file_path="config/arm_description.srdf")
                .robot_description_kinematics(file_path="config/kinematics.yaml")
                .planning_pipelines(
                    pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
                    default_planning_pipeline="ompl",
                )
                .joint_limits(file_path="config/joint_limits.yaml")
                .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
                .to_moveit_configs()
            )
            executor_parameters.append(moveit_config.to_dict())

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=executor_parameters,
                respawn=True,
                respawn_delay=1.0,
                output="screen",
                arguments=["--ros-args", "--log-level", "rmcs_executor:=warn"],
            )
        )
        # The adjustable heavy chassis has no arm or MoveIt runtime.
        if robot_name == "adjustable_heavy":
            entities.append(
                Node(
                    package="foxglove_bridge",
                    executable="foxglove_bridge",
                    condition=IfCondition(LaunchConfiguration("use_foxglove")),
                    parameters=[
                        {
                            "port": 8765,
                            "address": "0.0.0.0",
                            "capabilities": ["connectionGraph"],
                            "sysinfo": False,
                            "topic_whitelist": [
                                "^/chassis/(steering|wheel|lift)/[^/]+/(angle|velocity|torque)$"
                            ],
                        }
                    ],
                    output="screen",
                )
            )
            return entities

        demo_launch_path = os.path.join(
            FindPackageShare("arm_moveit_config").perform(context),
            "launch",
            "demo.launch.py",
        )

        demo_arguments = {"use_rviz": LaunchConfiguration("use_rviz")}
        if robot_name == "visual_mining_test":
            demo_arguments.update(
                {
                    "rviz_scene_topic": "/arm_mtc_preview/planning_scene",
                    # Scene playback includes gripper delays and attach/detach boundaries.
                    "rviz_trajectory_topic": "/arm_mtc_preview/disabled_trajectory",
                }
            )
        entities.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch_path),
                launch_arguments=demo_arguments.items(),
            )
        )
        return entities


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            MyLaunchDescriptionEntity(),
        ]
    )

    return ld
