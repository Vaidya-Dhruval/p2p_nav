# p2p_navigation/launch/test_p2p_navigation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p2p_navigation',
            executable='p2p_nav_server',
            name='p2p_navigation_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        Node(
            package='p2p_navigation',
            executable='p2p_nav_client',
            name='p2p_navigation_client',
            output='screen'
        )
    ])