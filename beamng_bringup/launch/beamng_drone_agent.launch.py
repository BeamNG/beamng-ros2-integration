from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for BeamNG drone agent."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='192.168.1.133',
            description='BeamNG.tech host IP address'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='25252',
            description='BeamNG.tech port number'
        ),
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='drone',
            description='Vehicle ID of the drone in BeamNG'
        ),
        Node(
            package='beamng_agent',
            executable='beamng_agent',
            name='beamng_drone_agent',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'vehicle_type': 'drone',
                'driving_mode': 'keyboard',
            }],
            output='screen',
        ),
    ])

