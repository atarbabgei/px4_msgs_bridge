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
        DeclareLaunchArgument('px4_namespace', default_value='',
                              description='PX4 namespace prefix (empty for single vehicle, "drone_0" for multi)'),
        DeclareLaunchArgument('namespace', default_value='vehicle',
                              description='ROS2 output namespace for bridge topics'),
        DeclareLaunchArgument('odom_rate', default_value='50.0',
                              description='Odometry publish rate (Hz)'),
        DeclareLaunchArgument('enable_tf', default_value='true',
                              description='Broadcast TF odom -> base_link'),
        DeclareLaunchArgument('enable_external_odom', default_value='false',
                              description='Enable external odometry input for PX4 fusion'),
        DeclareLaunchArgument('external_odom_topic', default_value='/odom/sample',
                              description='External odometry topic'),
        DeclareLaunchArgument('imu_source', default_value='/fmu/out/sensor_combined',
                              description='PX4 IMU topic: /fmu/out/sensor_combined or /fmu/out/vehicle_imu'),

        Node(
            package='px4_msgs_bridge',
            executable='bridge_node',
            name='px4_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'px4_namespace': LaunchConfiguration('px4_namespace'),
                    'namespace': LaunchConfiguration('namespace'),
                    'odom_rate': LaunchConfiguration('odom_rate'),
                    'enable_tf': LaunchConfiguration('enable_tf'),
                    'enable_external_odom': LaunchConfiguration('enable_external_odom'),
                    'external_odom_topic': LaunchConfiguration('external_odom_topic'),
                    'imu_source': LaunchConfiguration('imu_source'),
                },
            ],
        ),
    ])
