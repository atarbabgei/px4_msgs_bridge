#!/usr/bin/env python3

"""
Propeller Guard Contact Sensor Visualization Launch File

This launch file demonstrates a complete pipeline that:
1. Converts PX4 wheel encoder data to ROS2 joint states for propeller guard rotation
2. Converts PX4 debug values to contact point data for collision detection
3. Visualizes both the rotating propeller guard and contact points in RViz

Pipeline: PX4 Simulation → Bridge Conversion → URDF Model + Contact Points → RViz Visualization
- /fmu/out/wheel_encoders → /vehicle/propeller_guard/joint_states → robot_state_publisher → TF
- /fmu/out/debug_value → /vehicle/propeller_guard/contact_point → RViz contact visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('px4_msgs_bridge')
    
    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_share, 'urdf', 'quadcopter_custom_propeller_guard.urdf.xacro'),
        description='Path to URDF file with custom propeller guard (includes rotating guard for contact sensor)'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'bridge_propeller_guard_visualization.rviz'),
        description='Path to RViz configuration file with propeller guard contact sensor visualization'
    )
    
    vehicle_namespace_arg = DeclareLaunchArgument(
        'vehicle_namespace',
        default_value='vehicle',
        description='Vehicle namespace for ROS topics'
    )
    
    # PX4 Bridge Manager Node (with joint state publishing enabled)
    bridge_node = Node(
        package='px4_msgs_bridge',
        executable='bridge_node',
        name='px4_bridge_manager',
        output='screen',
        parameters=[{
            'vehicle_namespace': LaunchConfiguration('vehicle_namespace'),
            'px4_to_ros.publish_joint_states': True,
            'px4_to_ros.publish_contact_point': True,  # Enable contact sensor
            'px4_to_ros.contact_debug_index': 0,       # Expected debug index
            'px4_to_ros.publish_pose': True,
            'px4_to_ros.publish_path': True,
            'px4_to_ros.publish_odometry': True,
            'px4_to_ros.publish_imu': True,
            'px4_to_ros.tf_publishing.enable_tf': True,
            'px4_to_ros.tf_publishing.publish_odom_tf': True,
            'px4_to_ros.tf_publishing.publish_map_tf': False,
        }]
    )
    
    # Robot State Publisher - using OpaqueFunction to properly handle namespace
    def create_robot_state_publisher(context, *args, **kwargs):
        vehicle_namespace = LaunchConfiguration('vehicle_namespace').perform(context)
        joint_states_topic = f"/{vehicle_namespace}/propeller_guard/joint_states"
        
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('urdf_file')]), 
                    value_type=str
                ),
                'use_sim_time': False,
                'publish_frequency': 100.0, 
                'frame_prefix': '',
            }],
            remappings=[
                ('joint_states', joint_states_topic)
            ]
        )]
    
    # Note: joint_state_publisher not needed - bridge publishes directly to /{vehicle_namespace}/propeller_guard/joint_states
    # The robot_state_publisher consumes the namespaced joint_states topic to update TF tree for guard rotation
    # Contact points are published to /{vehicle_namespace}/propeller_guard/contact_point for RViz visualization
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='propeller_guard_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        urdf_file_arg,
        rviz_config_arg,
        vehicle_namespace_arg,
        
        # Nodes
        bridge_node,
        OpaqueFunction(function=create_robot_state_publisher),
        rviz_node,
    ])
