from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beamng_agent',
            executable='beamng_agent',
            name='beamng_agent',
            output='screen',
            parameters=[
                {'host': '127.0.0.1'},
                {'port': 25252},
                {'driving_mode': 'keyboard'},
                {'vehicle_id': 'car1'}
            ]
        )
    ])
