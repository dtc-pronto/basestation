#!/usr/bin/env python3
"""
ROS2 Launch file for scorecard submission node
Migrated from ROS Noetic to ROS2 Jazzy
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for scorecard submission node"""
    
    # Declare launch arguments
    threshold_path_arg = DeclareLaunchArgument(
        'threshold_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('scorecard_submitter'),
            'config',
            'threshold_config.json'
        ]),
        description='Path to threshold configuration JSON file'
    )
    
    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value='/home/dtc/data',
        description='Path to data directory for storing casualty databases'
    )
    
    start_run_arg = DeclareLaunchArgument(
        'start_run',
        default_value='false',
        description='Whether to start the run immediately'
    )
    
    robots_arg = DeclareLaunchArgument(
        'robots',
        default_value='["dione", "deimos", "phobos", "titania", "oberon"]',
        description='List of robots to monitor (JSON array format)'
    )
    
    # Create node with parameters
    scorecard_submission_node = Node(
        package='scorecard_submitter',
        executable='submission_node.py',
        name='scorecard_submission_node',
        output='screen',
        parameters=[{
            'threshold_path': LaunchConfiguration('threshold_path'),
            'data_path': LaunchConfiguration('data_path'),
            'start_run': LaunchConfiguration('start_run'),
            'robots': LaunchConfiguration('robots'),
        }],
        # Make the node required (will shutdown launch if node exits)
        on_exit=None,  # Can be set to Shutdown() for required behavior
    )
    
    return LaunchDescription([
        threshold_path_arg,
        data_path_arg,
        start_run_arg,
        robots_arg,
        scorecard_submission_node,
    ])
