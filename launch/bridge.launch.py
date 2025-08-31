#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

# === Configurable Parameters ===
# 
# This list defines all configurable parameters for the PX4 Bridge.
# 
# PARAMETER OVERRIDE BEHAVIOR:
# - Empty string ('') defaults = Use values from bridge_config.yaml (YAML-first approach)
# - Non-empty defaults = Launch-specific defaults (e.g., visualizer='false')
# - Launch arguments always override YAML config when provided
#
# TO MODIFY DEFAULTS: Edit config/bridge_config.yaml for permanent changes
# TO OVERRIDE TEMPORARILY: Use launch arguments (e.g., ros2 launch px4_msgs_bridge bridge.launch.py external_odom_topic:=/zed/odom)
#
configurable_parameters = [
    # === Core Configuration ===
    {'name': 'config_file',                           'default': '',         'description': 'Path to the bridge configuration YAML file (leave empty for default)'},
    {'name': 'visualizer',                            'default': '',         'description': 'Launch RViz2 for visualization (true/false). Empty = use bridge_config.yaml'},
    {'name': 'rviz_config',                           'default': '',         'description': 'Path to RViz configuration file (leave empty for default)'},
    
    # === Global Configuration ===
    {'name': 'vehicle_namespace',                     'default': '',         'description': 'Vehicle namespace for ROS topics (e.g., vehicle, uav1, drone_alpha). Empty = use bridge_config.yaml'},
    
    # === PX4 → ROS Converter ===
    {'name': 'publish_pose',                          'default': '',         'description': 'Enable pose publishing (true/false). Empty = use bridge_config.yaml'},
    {'name': 'publish_path',                          'default': '',         'description': 'Enable path publishing (true/false). Empty = use bridge_config.yaml'},
    {'name': 'publish_odometry',                      'default': '',         'description': 'Enable odometry publishing (true/false). Empty = use bridge_config.yaml'},
    {'name': 'publish_imu',                           'default': '',         'description': 'Enable IMU publishing (true/false). Empty = use bridge_config.yaml'},
    {'name': 'output_frame_id',                       'default': '',         'description': 'Frame ID for output messages (e.g., odom, map). Empty = use bridge_config.yaml'},
    {'name': 'child_frame_id',                        'default': '',         'description': 'Child frame ID (e.g., base_link, base_footprint). Empty = use bridge_config.yaml'},
    
    # === Path Configuration ===
    {'name': 'unlimited_path',                        'default': '',         'description': 'Enable unlimited path tracking (true/false)'},
    {'name': 'max_path_size',                         'default': '',         'description': 'Maximum path size for tracking (integer)'},
    
    # === Synchronization ===
    {'name': 'sync_threshold_us',                     'default': '',         'description': 'Max time diff between attitude/position in microseconds'},
    
    # === TF Publishing ===
    {'name': 'enable_tf',                             'default': '',         'description': 'Master TF enable/disable (true/false). Empty = use bridge_config.yaml'},
    {'name': 'publish_odom_tf',                       'default': '',         'description': 'Publish odom→base_link TF (true/false). Empty = use bridge_config.yaml'},
    {'name': 'publish_map_tf',                        'default': '',         'description': 'Publish map→odom TF (true/false). Empty = use bridge_config.yaml'},
    {'name': 'map_frame',                             'default': '',         'description': 'Map frame name (e.g., map, world). Empty = use bridge_config.yaml'},
    {'name': 'odom_frame',                            'default': '',         'description': 'Odometry frame name (e.g., odom). Empty = use bridge_config.yaml'},
    {'name': 'base_link_frame',                       'default': '',         'description': 'Base link frame name (e.g., base_link). Empty = use bridge_config.yaml'},
    {'name': 'tf_rate',                               'default': '',         'description': 'TF publishing rate in Hz for odom→base_link. Empty = use bridge_config.yaml'},
    {'name': 'map_tf_rate',                           'default': '',         'description': 'TF publishing rate in Hz for map→odom. Empty = use bridge_config.yaml'},
    
    # === ROS → PX4 External Odometry ===
    {'name': 'enable_external_odom',                  'default': '',         'description': 'Enable external odometry bridge (true/false). Empty = use bridge_config.yaml'},
    {'name': 'external_odom_topic',                   'default': '',         'description': 'External odometry topic (e.g., /zed/odom, /t265/odom/sample, /rtabmap/odom). Empty = use bridge_config.yaml'},
    {'name': 'odom_quality_threshold',                'default': '',         'description': 'Odometry quality threshold (0.0-1.0). Empty = use bridge_config.yaml'},
]

# NOTE: To modify permanent defaults, edit: config/bridge_config.yaml
# For temporary overrides, use launch arguments: ros2 launch px4_msgs_bridge bridge.launch.py param_name:=value

def launch_bridge_node(context, *args, **kwargs):
    """Create bridge node with conditional parameter overrides"""
    # Base parameters (always load YAML config first)
    parameters = [LaunchConfiguration('config_file')]
    
    # Build parameter overrides dictionary dynamically
    param_overrides = {}
    
    # Parameter mapping: launch_arg_name -> yaml_parameter_path
    param_mapping = {
        'vehicle_namespace': 'vehicle_namespace',
        'visualizer': 'visualizer',
        'publish_pose': 'px4_to_ros.publish_pose',
        'publish_path': 'px4_to_ros.publish_path', 
        'publish_odometry': 'px4_to_ros.publish_odometry',
        'publish_imu': 'px4_to_ros.publish_imu',
        'output_frame_id': 'px4_to_ros.output_frame_id',
        'child_frame_id': 'px4_to_ros.child_frame_id',
        'unlimited_path': 'px4_to_ros.path_config.unlimited_path',
        'max_path_size': 'px4_to_ros.path_config.max_path_size',
        'sync_threshold_us': 'px4_to_ros.sync_threshold_us',
        'enable_tf': 'px4_to_ros.tf_publishing.enable_tf',
        'publish_odom_tf': 'px4_to_ros.tf_publishing.publish_odom_tf',
        'publish_map_tf': 'px4_to_ros.tf_publishing.publish_map_tf',
        'map_frame': 'px4_to_ros.tf_publishing.map_frame',
        'odom_frame': 'px4_to_ros.tf_publishing.odom_frame',
        'base_link_frame': 'px4_to_ros.tf_publishing.base_link_frame',
        'tf_rate': 'px4_to_ros.tf_publishing.tf_rate',
        'map_tf_rate': 'px4_to_ros.tf_publishing.map_tf_rate',
        'enable_external_odom': 'ros_to_px4.external_odometry.enable',
        'external_odom_topic': 'ros_to_px4.external_odometry.input_topic',
        'odom_quality_threshold': 'ros_to_px4.external_odometry.quality_threshold',
    }
    
    # Boolean parameters that need conversion
    boolean_params = {
        'visualizer', 'publish_pose', 'publish_path', 'publish_odometry', 'publish_imu',
        'unlimited_path', 'enable_tf', 'publish_odom_tf', 'publish_map_tf', 
        'enable_external_odom'
    }
    
    # Numeric parameters that need conversion
    numeric_params = {
        'max_path_size': int,
        'sync_threshold_us': int,
        'tf_rate': float,
        'map_tf_rate': float,
        'odom_quality_threshold': float,
    }
    
    # Process all configurable parameters
    for param in configurable_parameters:
        param_name = param['name']
        
        # Skip special parameters handled separately
        if param_name in ['config_file', 'rviz_config']:
            continue
            
        if param_name in param_mapping:
            value = LaunchConfiguration(param_name).perform(context)
            if value and value.strip():
                yaml_path = param_mapping[param_name]
                
                # Convert value based on parameter type
                if param_name in boolean_params:
                    param_overrides[yaml_path] = value.lower() == 'true'
                elif param_name in numeric_params:
                    try:
                        param_overrides[yaml_path] = numeric_params[param_name](value)
                    except ValueError:
                        print(f"Warning: Invalid value '{value}' for parameter '{param_name}', skipping override")
                else:
                    param_overrides[yaml_path] = value
    
    # Add parameter overrides if any exist
    if param_overrides:
        parameters.append(param_overrides)
    
    return [Node(
        package='px4_msgs_bridge',
        executable='bridge_node',
        name='px4_bridge_manager',
        output='screen',
        parameters=parameters
    )]

def launch_rviz_node(context, *args, **kwargs):
    """Create RViz2 node conditionally based on visualizer parameter"""
    import yaml
    
    # Check visualizer parameter (same logic as bridge node)
    visualizer_override = LaunchConfiguration('visualizer').perform(context)
    
    # Determine if we should launch RViz
    launch_rviz = False
    
    if visualizer_override and visualizer_override.strip():
        # Launch argument override provided
        launch_rviz = visualizer_override.lower() == 'true'
    else:
        # No override provided, read from YAML config
        try:
            config_file_path = LaunchConfiguration('config_file').perform(context)
            if config_file_path and os.path.exists(config_file_path):
                with open(config_file_path, 'r') as f:
                    config = yaml.safe_load(f)
                    launch_rviz = config.get('px4_bridge_manager', {}).get('ros__parameters', {}).get('visualizer', False)
            else:
                # Fallback to false if no config file
                launch_rviz = False
        except Exception as e:
            print(f"Warning: Could not read visualizer from YAML config: {e}")
            launch_rviz = False
    
    if launch_rviz:
        return [Node(
            package='rviz2',
            executable='rviz2',
            name='px4_bridge_rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': False  # Set to True if using simulation
            }]
        )]
    else:
        return []

def generate_launch_description():
    # Package path
    pkg_share = get_package_share_directory('px4_msgs_bridge')
    
    # Default file paths
    default_config_file = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'bridge_visualization.rviz')
    
    # Generate launch arguments dynamically from configurable_parameters
    launch_arguments = []
    
    for param in configurable_parameters:
        param_name = param['name']
        param_default = param['default']
        param_description = param['description']
        
        # Handle special default values
        if param_name == 'config_file' and not param_default:
            param_default = default_config_file
        elif param_name == 'rviz_config' and not param_default:
            param_default = default_rviz_config
        
        # Create launch argument
        launch_arguments.append(
            DeclareLaunchArgument(
                param_name,
                default_value=param_default,
                description=param_description
            )
        )
    
    return LaunchDescription(
        # === Dynamically Generated Launch Arguments ===
        launch_arguments + [
        
        # === PX4 Bridge Manager Node (with conditional parameter overrides) ===
        OpaqueFunction(function=launch_bridge_node),
        
        # === RViz2 Visualization (conditional launch based on YAML/override) ===
        OpaqueFunction(function=launch_rviz_node),
    ])
