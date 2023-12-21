import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    beamng_agent_pkg_prefix = get_package_share_directory('beamng_agent')

    return launch.LaunchDescription([
        Node(
            package='beamng_agent',
            executable='agent.py',  # Updated executable name
            name='agent',
            output='screen',
            parameters=[{'beamng.host': 'localhost', 'beamng.port': 64256}],  # Adjust parameters as needed
            remappings=[('/control', '/your_control_topic')]  # Replace with your actual control topic
        )
    ])
