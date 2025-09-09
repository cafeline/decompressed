#!/usr/bin/env python3
"""
Demo launch file that starts both the pointcloud compressor and decompressed viewer
C++ version of compressed_viewer
All parameters are configured via YAML file
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load_launch_config(config_file):
    """Load launch configuration from YAML file"""
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            launch_params = config.get('launch_config', {}).get('ros__parameters', {})
            return {
                'enable_check_markers': launch_params.get('enable_check_markers', True),
                'enable_rviz': launch_params.get('enable_rviz', True)
            }
    except Exception as e:
        print(f"Warning: Could not load launch config from {config_file}: {e}")
        # Return defaults if loading fails
        return {
            'enable_check_markers': True,
            'enable_rviz': True
        }

def launch_setup(context, *args, **kwargs):  # pylint: disable=unused-argument
    """Setup function to conditionally add nodes based on YAML configuration"""
    config_file = LaunchConfiguration('config_file').perform(context)
    launch_config = load_launch_config(config_file)

    nodes = []

    # Try to get rviz config from compressed_viewer if it exists
    try:
        viewer_pkg_dir = get_package_share_directory('compressed_viewer')
        rviz_config = os.path.join(viewer_pkg_dir, 'rviz', 'compressed_viewer.rviz')
    except:
        # Fallback if compressed_viewer is not installed
        rviz_config = ''

    # Check markers node (conditionally launch based on YAML)
    if launch_config['enable_check_markers']:
        print("Launching check_markers node (enabled in config)")
        nodes.append(
            Node(
                package='check_markers',
                executable='check_markers',
                name='marker_checker_node',
                parameters=[config_file],
                output='screen',
                emulate_tty=True
            )
        )
    else:
        print("Skipping check_markers node (disabled in config)")

    # RViz node (conditionally launch based on YAML)
    if launch_config['enable_rviz']:
        print("Launching RViz (enabled in config)")
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if rviz_config else [],
                output='screen'
            )
        )
    else:
        print("Skipping RViz (disabled in config)")
    
    # Decompressed viewer node (C++ version) - start after 2 seconds
    nodes.append(
        TimerAction(
            period=2.0,  # 2 second delay
            actions=[
                Node(
                    package='decompressed',
                    executable='decompressed_viewer_node',
                    name='decompressed_viewer_node',
                    parameters=[config_file],
                    output='screen',
                    emulate_tty=True
                )
            ]
        )
    )

    # Pointcloud compressor node (start after 3 seconds)
    nodes.append(
        TimerAction(
            period=3.0,  # 3 second delay - start after viewer is ready
            actions=[
                Node(
                    package='pointcloud_compressor',
                    executable='pointcloud_compressor_node',
                    name='pointcloud_compressor_node',
                    parameters=[config_file],
                    output='screen',
                    emulate_tty=True
                )
            ]
        )
    )

    return nodes

def generate_launch_description():
    """Generate launch description for demo"""

    # Get package directory for decompressed
    decompressed_pkg_dir = get_package_share_directory('decompressed')
    demo_config = os.path.join(decompressed_pkg_dir, 'config', 'demo_params.yaml')

    # Declare launch arguments - only for overriding YAML values
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=demo_config,
        description='Path to configuration file (default: demo_params.yaml)'
    )

    # OpaqueFunction to handle conditional launching based on YAML config
    launch_setup_action = OpaqueFunction(
        function=launch_setup
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(config_file_arg)

    # Add the setup action that will conditionally add nodes
    ld.add_action(launch_setup_action)

    return ld