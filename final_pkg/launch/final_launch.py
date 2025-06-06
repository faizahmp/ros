from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_pkg',
            executable='sm_main',
            name='sm_main',
            output='screen'
        )
    ])
