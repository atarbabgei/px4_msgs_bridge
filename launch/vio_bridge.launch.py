#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_msgs_bridge',
            executable='vio_bridge_node',
            name='vio_bridge',
            output='screen',
            parameters=[],
            remappings=[
                # You can add topic remappings here if needed
                # ('/odom/sample', '/your_custom_odom_topic'),
            ]
        )
    ])
