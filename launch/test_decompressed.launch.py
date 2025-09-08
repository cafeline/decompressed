#!/usr/bin/env python3
"""
Launch file for testing decompressed viewer with check_markers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='/tmp/test_cube.pcd',
        description='Path to input PCD file'
    )
    
    
    # Get launch configurations
    input_file = LaunchConfiguration('input_file')
    
    # Check markers node (launches immediately)
    check_markers_node = Node(
        package='check_markers',
        executable='check_markers',
        name='check_markers',
        output='screen',
        parameters=[
            {'topic1': 'occupied_voxel_markers'},
            {'topic2': 'pattern_markers'},
        ]
    )
    
    # Pointcloud compressor node (start after 2 seconds)
    compressor_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='pointcloud_compressor',
                executable='pointcloud_compressor',
                name='pointcloud_compressor',
                output='screen',
                parameters=[
                    {'input_file': input_file},
                    {'voxel_size': 0.01},
                    {'publish_pattern_dictionary': True},
                    {'use_voxel_filter': True},
                    {'enable_visualization': True},
                ]
            )
        ]
    )
    
    # Decompressed viewer node (C++ version) - start after 1 second
    decompressed_viewer_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='decompressed',
                executable='decompressed_viewer_node',
                name='decompressed_viewer_node',
                output='screen',
                parameters=[
                    {'pattern_dictionary_topic': 'pattern_dictionary'},
                    {'pattern_markers_topic': 'pattern_markers'},
                    {'frame_id': 'map'},
                    {'pattern_voxel_size': 0.01},
                    {'show_pattern_visualization': True},
                ]
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(input_file_arg)
    
    # Always add check_markers and decompressed_viewer
    ld.add_action(check_markers_node)
    ld.add_action(decompressed_viewer_node)
    
    # Add compressor with delay
    ld.add_action(compressor_node)
    
    return ld