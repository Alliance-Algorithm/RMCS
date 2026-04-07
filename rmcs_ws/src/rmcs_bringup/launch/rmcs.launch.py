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
from moveit_configs_utils import MoveItConfigsBuilder

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

        moveit_config = (
            MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
            .robot_description(file_path="config/arm_description.urdf.xacro")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
                default_planning_pipeline="ompl",
            )
            .joint_limits(file_path="config/joint_limits.yaml")
            .robot_description_semantic(file_path="config/arm_description.srdf")
            .planning_scene_monitor(
                publish_robot_description=True, publish_robot_description_semantic=True
            )
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
            .to_moveit_configs()
        )

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=[
                    moveit_config.to_dict(),
                    os.path.join(
                        FindPackageShare("rmcs_bringup").perform(context),
                        "config",
                        robot_name + ".yaml",
                    ),
                ],
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
            MyLaunchDescriptionEntity(),
        ]
    )

    return ld
