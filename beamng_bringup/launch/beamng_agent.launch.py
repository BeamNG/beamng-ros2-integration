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
                {'host': '172.21.192.1'},
                {'port': 64256},
                {'driving_mode': 'keyboard'},
                {'vehicle_id': 'ego'}
            ]
        )
    ])
