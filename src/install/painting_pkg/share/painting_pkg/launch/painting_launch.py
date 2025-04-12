from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='painting_pkg', executable='painting_node', output='screen'),
    ])
