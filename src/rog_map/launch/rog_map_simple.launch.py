#!/usr/bin/env python3
"""
ROG-Map Simple Node Launch File for auto_sentry2025
方案A：轻量级集成
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取包路径
    rog_map_dir = get_package_share_directory('rog_map')

    # 配置文件路径
    config_file = os.path.join(rog_map_dir, 'config', 'rog_map_simple.yaml')

    # 声明启动参数
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to ROG-Map configuration file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # ROG-Map Simple节点
    rog_map_node = Node(
        package='rog_map',
        executable='rog_map_simple_node',
        name='rog_map_simple',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # 根据实际情况调整话题映射
            ('/cloud_registered_full', '/cloud_registered_full'),
            ('/aft_mapped_to_init', '/aft_mapped_to_init'),
        ]
    )

    return LaunchDescription([
        declare_config_file,
        declare_use_sim_time,
        rog_map_node,
    ])
