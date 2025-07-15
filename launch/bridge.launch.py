#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_msgs_bridge',
            executable='vehicle_pose_bridge',
            name='vehicle_pose_bridge',
            output='screen',
            parameters=[],
            remappings=[
                # Add any topic remappings if needed
            ]
        )
    ])
