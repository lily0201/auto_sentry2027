#!/usr/bin/env python3
"""
完整的导航启动文件，集成ROG-Map（方案A）
基于原有的navigation_launch.py，添加ROG-Map节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('nav2_bringup')
    rog_map_dir = get_package_share_directory('rog_map')
    robot_bring_up_dir = get_package_share_directory('robot_bring_up')

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rog_map = LaunchConfiguration('use_rog_map')
    params_file = LaunchConfiguration('params_file')
    rog_map_config = LaunchConfiguration('rog_map_config')

    # 声明参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rog_map_cmd = DeclareLaunchArgument(
        'use_rog_map',
        default_value='true',
        description='Use ROG-Map for obstacle detection')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_rog_map_config_cmd = DeclareLaunchArgument(
        'rog_map_config',
        default_value=os.path.join(rog_map_dir, 'config', 'rog_map_simple.yaml'),
        description='Full path to ROG-Map configuration file')

    # ROG-Map节点（条件启动）
    rog_map_node = Node(
        condition=IfCondition(use_rog_map),
        package='rog_map',
        executable='rog_map_simple_node',
        name='rog_map_simple',
        output='screen',
        parameters=[
            rog_map_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 包含原有的navigation_launch.py
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bring_up_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    # 创建启动描述
    ld = LaunchDescription()

    # 添加声明的参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rog_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rog_map_config_cmd)

    # 添加节点
    ld.add_action(rog_map_node)
    ld.add_action(navigation_launch)

    return ld
