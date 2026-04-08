#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_msgs_bridge')
    default_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='Path to bridge YAML config'),
        DeclareLaunchArgument('enable_external_odom', default_value='false',
                              description='Enable external odometry input'),
        DeclareLaunchArgument('external_odom_topic', default_value='/odom/sample',
                              description='External odometry topic'),

        Node(
            package='px4_msgs_bridge',
            executable='bridge_node',
            name='px4_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'enable_external_odom': LaunchConfiguration('enable_external_odom'),
                    'external_odom_topic': LaunchConfiguration('external_odom_topic'),
                },
            ],
        ),
    ])
