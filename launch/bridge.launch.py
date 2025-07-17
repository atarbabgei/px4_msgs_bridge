#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Vehicle pose bridge node - converts PX4 attitude/position to ROS pose
        Node(
            package='px4_msgs_bridge',
            executable='vehicle_pose_bridge',
            name='vehicle_pose_bridge',
            output='screen',
            parameters=[],
            remappings=[
                # Add any topic remappings if needed
            ]
        ),
        
        # VIO bridge node - converts VIO odometry to PX4 visual odometry
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
