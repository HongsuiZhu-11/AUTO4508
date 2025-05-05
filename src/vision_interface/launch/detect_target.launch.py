# launch/detect_target.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_interface',
            executable='detect_target_service',
            name='detect_target_node',
            output='screen'
        )
    ])
