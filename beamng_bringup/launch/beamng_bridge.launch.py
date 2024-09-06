from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition  # Import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument to choose whether to run the example client
    run_example_client = LaunchConfiguration('run_example_client', default='false')

    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'run_example_client',
            default_value='false',
            description='Run the example client node'
        ),

        # Launch the beamng_bridge node
        Node(
            package='beamng_ros2',
            executable='beamng_bridge',
            name='beamng_bridge',
            output='screen',
            parameters=[
                {'host': '172.21.192.1'}, #127.0.0.1
                {'port': 64256},
                {'launch': False},
                {'update_sec': 0.1}
            ]
        ),

        # Conditionally launch the example client based on the argument
        Node(
            package='beamng_ros2',
            executable='example_client',
            name='beamng_example_client',
            output='screen',
            condition=IfCondition(run_example_client)
        )
    ])
