from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scorecard_submitter',
            executable='spoofer.py',
            name='spoofer_node',
            output='screen',
            parameters=[
                {'robots': ['deimos', 'phobos', 'titania', 'oberon', 'dione']}
            ]
        )
    ])
