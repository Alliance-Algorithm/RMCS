#!/usr/bin/env python3
"""
简单的URDF启动文件
文件名: simple_urdf_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包目录
    pkg_path = get_package_share_directory('arm_description')
    
    # URDF文件路径
    urdf_file = 'arm_description.urdf'  # 或者 'Arm.xacro' 如果你用xacro
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file)
    
    # 检查文件是否存在
    if not os.path.exists(urdf_path):
        urdf_path = os.path.join(pkg_path, 'urdf', 'Arm.xacro')
    
    # 读取URDF内容
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    # 创建节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
            'publish_frequency': 50.0
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    rviz_config_path = os.path.join(pkg_path, 'config', 'view_urdf.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])