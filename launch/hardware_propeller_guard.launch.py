#!/usr/bin/env python3

"""
Hardware Propeller Guard Launch File

This launch file is configured for real hardware operation:
1. Uses /fmu/out/vehicle_odometry as position source (configurable)
2. Subscribes to external /joint_states from hardware for propeller guard rotation
3. Disables simulation time (uses system clock)
4. Contact sensor disabled by default (not yet available on hardware)

Pipeline: PX4 Hardware → Bridge Conversion → URDF Model → RViz Visualization
- /fmu/out/vehicle_odometry → pose, path, odom, TF
- /joint_states (hardware) → /vehicle/propeller_guard/joint_states → robot_state_publisher → TF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('px4_msgs_bridge')
    
    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_share, 'urdf', 'quadcopter_custom_propeller_guard.urdf.xacro'),
        description='Path to URDF file with custom propeller guard'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'bridge_propeller_guard_visualization.rviz'),
        description='Path to RViz configuration file'
    )
    
    vehicle_namespace_arg = DeclareLaunchArgument(
        'vehicle_namespace',
        default_value='vehicle',
        description='Vehicle namespace for ROS topics'
    )
    
    position_source_arg = DeclareLaunchArgument(
        'position_source',
        default_value='vehicle_odometry',
        description='Position data source: "vehicle_local_position" or "vehicle_odometry"'
    )
    
    joint_state_source_arg = DeclareLaunchArgument(
        'joint_state_source',
        default_value='external',
        description='Joint state source: "wheel_encoders" or "external"'
    )
    
    external_joint_state_topic_arg = DeclareLaunchArgument(
        'external_joint_state_topic',
        default_value='/joint_states',
        description='External joint state topic (when joint_state_source is "external")'
    )
    
    external_joint_name_arg = DeclareLaunchArgument(
        'external_joint_name',
        default_value='joint_0',
        description='Joint name to look for in external topic (remapped to propeller_guard_joint for URDF)'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for visualization (true/false)'
    )
    
    # PX4 Bridge Manager Node configured for hardware
    bridge_node = Node(
        package='px4_msgs_bridge',
        executable='bridge_node',
        name='px4_bridge_manager',
        output='screen',
        parameters=[{
            'vehicle_namespace': LaunchConfiguration('vehicle_namespace'),
            'use_sim_time': False,
            'px4_to_ros.position_source': LaunchConfiguration('position_source'),
            'px4_to_ros.joint_state_source': LaunchConfiguration('joint_state_source'),
            'px4_to_ros.external_joint_state_topic': LaunchConfiguration('external_joint_state_topic'),
            'px4_to_ros.external_joint_name': LaunchConfiguration('external_joint_name'),
            'px4_to_ros.publish_joint_states': True,
            'px4_to_ros.publish_contact_point': False,  # Contact sensor not available on hardware yet
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
    
    # Conditional RViz2 launch
    def create_rviz_node(context, *args, **kwargs):
        enable_rviz = LaunchConfiguration('enable_rviz').perform(context).lower() == 'true'
        if enable_rviz:
            return [Node(
                package='rviz2',
                executable='rviz2',
                name='hardware_rviz',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')],
                parameters=[{
                    'use_sim_time': False
                }]
            )]
        return []
    
    return LaunchDescription([
        # Launch arguments
        urdf_file_arg,
        rviz_config_arg,
        vehicle_namespace_arg,
        position_source_arg,
        joint_state_source_arg,
        external_joint_state_topic_arg,
        external_joint_name_arg,
        enable_rviz_arg,
        
        # Nodes
        bridge_node,
        OpaqueFunction(function=create_robot_state_publisher),
        OpaqueFunction(function=create_rviz_node),
    ])
