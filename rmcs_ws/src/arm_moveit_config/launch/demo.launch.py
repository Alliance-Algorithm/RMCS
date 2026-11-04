from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_group_node_config = (
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
            {"robot_description_planning.default_robot_padding": 0.00},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Rviz2
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Start RViz if true",
    )
    scene_topic_arg = DeclareLaunchArgument(
        "rviz_scene_topic",
        default_value="/monitored_planning_scene",
        description="Planning scene displayed by RViz",
    )
    trajectory_topic_arg = DeclareLaunchArgument(
        "rviz_trajectory_topic",
        default_value="/display_planned_path",
        description="Independent trajectory animation displayed by RViz",
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
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        remappings=[
            ("monitored_planning_scene", LaunchConfiguration("rviz_scene_topic")),
            ("display_planned_path", LaunchConfiguration("rviz_trajectory_topic")),
        ],
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

    return LaunchDescription(
        [
            rviz_config_arg,
            use_rviz_arg,
            scene_topic_arg,
            trajectory_topic_arg,
            rviz_node,
            static_tf,
            robot_state_publisher_node,
            move_group_node,
        ]
    )
