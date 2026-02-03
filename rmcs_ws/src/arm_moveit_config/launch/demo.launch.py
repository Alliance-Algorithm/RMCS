import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_path = get_package_share_directory("arm_moveit_config")

  
    moveit_group_node_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp"],
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

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_moveit_config"),
                    "config",
                    "arm_description.urdf.xacro",
                ]
            ),
            " ",
            "initial_positions_file:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_moveit_config"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_group_node_config.to_dict(),
             
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Rviz2
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("arm_moveit_config"),
            "config",
            LaunchConfiguration("rviz_config"),
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_group_node_config.robot_description,
            moveit_group_node_config.robot_description_semantic,
            moveit_group_node_config.planning_pipelines,
            moveit_group_node_config.robot_description_kinematics,
            moveit_group_node_config.joint_limits,
            moveit_group_node_config.pilz_cartesian_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
        ],
    )
    ros2_controllers_path = os.path.join(
         get_package_share_directory("arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
        ],
        output="both",
    )
   
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    alliance_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "alliance_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    return LaunchDescription(
        [
            # rviz_config_arg,
            # rviz_node,
            static_tf,
            robot_state_publisher_node,
            move_group_node,
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # alliance_arm_controller_spawner,
        ]
    )
