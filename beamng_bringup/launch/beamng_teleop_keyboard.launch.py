from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beamng_teleop_keyboard',
            executable='teleop',  
            name='beamng_teleop_keyboard',
            output='screen'
        )
    ])
