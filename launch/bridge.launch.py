#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for path configuration
    unlimited_path_arg = DeclareLaunchArgument(
        'unlimited_path',
        default_value='false',
        description='Keep unlimited vehicle path history (true/false). Warning: may consume memory over time.'
    )
    
    max_path_size_arg = DeclareLaunchArgument(
        'max_path_size', 
        default_value='1000',
        description='Maximum number of poses to keep in path when unlimited_path=false'
    )
    
    publish_odom_arg = DeclareLaunchArgument(
        'publish_odom',
        default_value='true', 
        description='Publish vehicle odometry (nav_msgs/Odometry) with pose and twist'
    )
    
    publish_imu_arg = DeclareLaunchArgument(
        'publish_imu',
        default_value='true',
        description='Publish vehicle IMU data (sensor_msgs/Imu) from attitude and sensor_combined'
    )
    
    return LaunchDescription([
        # Launch arguments
        unlimited_path_arg,
        max_path_size_arg,
        publish_odom_arg,
        publish_imu_arg,
        
        # Vehicle pose bridge node - converts PX4 attitude/position to ROS pose
        Node(
            package='px4_msgs_bridge',
            executable='vehicle_pose_bridge',
            name='vehicle_pose_bridge',
            output='screen',
            parameters=[{
                'unlimited_path': LaunchConfiguration('unlimited_path'),
                'max_path_size': LaunchConfiguration('max_path_size'),
                'publish_odom': LaunchConfiguration('publish_odom'),
                'publish_imu': LaunchConfiguration('publish_imu'),
            }],
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
