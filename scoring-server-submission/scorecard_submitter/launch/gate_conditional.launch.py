from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def generate_launch_description():

    params_file = PathJoinSubstitution([
        FindPackageShare('scorecard_submitter'),
        'config',
        'scorecard_params.yaml'
    ])

    Gate_1_condition = DeclareLaunchArgument('gate_1', default_value='false')
    Gate_2_condition = DeclareLaunchArgument('gate_2', default_value='false')
    Gate_3_condition = DeclareLaunchArgument('gate_3', default_value='false')
    Gate_4_condition = DeclareLaunchArgument('gate_4', default_value='false')
    HMT_condition = DeclareLaunchArgument('hmt', default_value='false')

    gate_1_node = Node(
            package='scorecard_submitter',
            executable='casualty_location_sub.py',
            name='gate1_node',
            condition=IfCondition(LaunchConfiguration('gate_1')),
            parameters=[params_file]
        )

    gate_2_node = Node(
            package='scorecard_submitter',
            executable='triage_report_sub.py',
            name='gate2_node',
            condition=IfCondition(LaunchConfiguration('gate_2')),
            parameters=[params_file]
        )

    gate_3_node = Node(
            package='scorecard_submitter',
            executable='assessement_report_sub.py',
            name='gate3_node',
            condition=IfCondition(LaunchConfiguration('gate_3')),
            parameters=[params_file]
        )

    gate_4_node = Node(
            package='scorecard_submitter',
            executable='vital_report_sub.py',
            name='gate4_node',
            condition=IfCondition(LaunchConfiguration('gate_4')),
            parameters=[params_file]
        )

    hmt_node = Node(
            package='scorecard_submitter',
            executable='hmt_node.py',
            name='hmt_node',
            condition=IfCondition(LaunchConfiguration('hmt')),
            parameters=[params_file]
        )

    atak_report_node = Node(
            package='scorecard_submitter',
            executable='atak_report_node.py',
            name='atak_report_node',
            condition=IfCondition(LaunchConfiguration('hmt')),
            parameters=[params_file]
        )

    system_location_node = Node(
            package='scorecard_submitter',
            executable='system_location_sub.py',
            name='system_location_submission_node',
            parameters=[params_file]
        )

    casualty_image_node = Node(
            package='scorecard_submitter',
            executable='casualty_image_node.py',
            name='casualty_image_node',
            parameters=[params_file]
        )

    return LaunchDescription([
        # Declare gate arguments
        Gate_1_condition,
        Gate_2_condition,
        Gate_3_condition,
        Gate_4_condition,
        HMT_condition,
        # Include gate nodes conditionally
        gate_1_node,
        gate_2_node,
        gate_3_node,
        gate_4_node,
        hmt_node,
        atak_report_node,
        system_location_node,
        casualty_image_node,
    ])