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
    
    # Path to the YAML parameters file
    params_file = PathJoinSubstitution([
        FindPackageShare('scorecard_submitter'),
        'config',
        'scorecard_params.yaml'
    ])
    
    # Create node with parameters loaded from YAML file
    scorecard_submission_node = Node(
        package='scorecard_submitter',
        executable='submission_node.py',
        name='scorecard_submission_node',
        parameters=[params_file],
        on_exit=None,
    )
    
    return LaunchDescription([
        scorecard_submission_node,
    ])
