from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p2p_navigation',
            executable='p2p_nav_server',
            output='screen'
        ),
    ])
