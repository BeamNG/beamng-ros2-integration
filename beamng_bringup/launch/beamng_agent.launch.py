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
                {'port': 25252},
                {'driving_mode': 'keyboard'},
                {'vehicle_id': 'ego'},
                {'vehicle_type': 'sedan'},  # Options: 'drone', 'sedan', 'truck' (default: 'sedan' for backward compat)
                {'enable_state_to_twist': False}  # Optional: enable state-to-twist conversion (requires bridge)
            ]
        )
    ])
