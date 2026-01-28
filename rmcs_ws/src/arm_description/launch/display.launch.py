# launch/visualize_arm.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
   
    package_name = 'arm_description'
    urdf_file = 'urdf/arm_description.urdf'
    
    # 使用FindPackageShare获取包的路径
    urdf_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        urdf_file
    ])
    
    # 声明参数
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=urdf_path,
        description='Path to URDF file'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='use_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    
   
    robot_description_content = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
        }],
        output='screen'
    )
    
    # 根据GUI参数选择使用哪个joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui' if LaunchConfiguration('use_gui') == 'true' else 'joint_state_publisher',
        executable='joint_state_publisher_gui' if LaunchConfiguration('use_gui') == 'true' else 'joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': LaunchConfiguration('use_gui'),
        }],
        output='screen'
    )
    
    # RViz2节点
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'arm_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
        model_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])