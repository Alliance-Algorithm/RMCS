import launch
import launch_ros

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessExit, OnProcessStart
# from launch.substitutions import (
#     Command,
#     FindExecutable,
#     LaunchConfiguration,
#     PathJoinSubstitution,
# )
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = launch.LaunchDescription()

    robot_description = launch_ros.descriptions.ParameterValue(
        launch.substitutions.Command(
            [
                "xacro ",
                launch.substitutions.PathJoinSubstitution(
                    [
                        launch_ros.substitutions.FindPackageShare("rmcs_description"),
                        "urdf",
                        "medium_feed_omni_wheel.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )
    ld.add_action(
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description,
                    "publish_frequency": 1000.0,
                    "ignore_timestamp": True,
                }
            ],
        )
    )

    ld.add_action(
        launch_ros.actions.Node(package="rmcs_controller", executable="rmcs_controller")
    )

    # ld.add_action(
    #     launch_ros.actions.Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         arguments=[
    #             "-d",
    #             launch.substitutions.PathJoinSubstitution(
    #                 [
    #                     launch_ros.substitutions.FindPackageShare("rmcs_description"),
    #                     "rviz",
    #                     "display.rviz",
    #                 ]
    #             ),
    #         ],
    #     )
    # )

    return ld
