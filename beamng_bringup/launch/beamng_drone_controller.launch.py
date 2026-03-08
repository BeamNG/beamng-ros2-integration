from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for BeamNG drone controller."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='127.0.0.1',
            description='BeamNG.tech host IP. Override for your setup (WSL=vEthernet IP, remote=BeamNG machine IP)'
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
            executable='drone_controller',
            name='beamng_drone_controller',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'vehicle_id': LaunchConfiguration('vehicle_id'),
            }],
            output='screen',
        ),
    ])
