#!/usr/bin/env python3
"""
Demo launch file that starts both the pointcloud compressor and decompressed viewer
C++ version of compressed_viewer
All parameters are configured via YAML file
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for demo"""

    # Get package directory for decompressed
    decompressed_pkg_dir = get_package_share_directory('decompressed')
    demo_config = os.path.join(decompressed_pkg_dir, 'config', 'demo_params.yaml')
    
    # Try to get rviz config from compressed_viewer if it exists
    try:
        viewer_pkg_dir = get_package_share_directory('compressed_viewer')
        rviz_config = os.path.join(viewer_pkg_dir, 'rviz', 'compressed_viewer.rviz')
    except:
        # Fallback if compressed_viewer is not installed
        rviz_config = ''

    # Declare launch arguments - only for overriding YAML values
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=demo_config,
        description='Path to configuration file (default: demo_params.yaml)'
    )

    # Pointcloud compressor node (start after 3 seconds)
    compressor_node = TimerAction(
        period=3.0,  # 3 second delay - start after viewer is ready
        actions=[
            Node(
                package='pointcloud_compressor',
                executable='pointcloud_compressor_node',
                name='pointcloud_compressor_node',
                parameters=[LaunchConfiguration('config_file')],
                output='screen',
                emulate_tty=True
            )
        ]
    )

    # Decompressed viewer node (C++ version) - start after 2 seconds
    viewer_node = TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            Node(
                package='decompressed',
                executable='decompressed_viewer_node',
                name='decompressed_viewer_node',
                parameters=[LaunchConfiguration('config_file')],
                output='screen',
                emulate_tty=True
            )
        ]
    )

    # Check markers node (launch first, no delay) - always launch
    check_markers_node = Node(
        package='check_markers',
        executable='check_markers',
        name='marker_checker_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )

    # RViz node (launch first, no delay) - always launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if rviz_config else [],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(config_file_arg)

    # Add nodes (check_markers and rviz first, always launch)
    ld.add_action(check_markers_node)
    ld.add_action(rviz_node)
    ld.add_action(viewer_node)
    ld.add_action(compressor_node)

    return ld