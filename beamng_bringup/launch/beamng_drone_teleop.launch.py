from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch description for BeamNG drone teleop.

    Launches the cmd_vel converter so that /cmd_vel (and /cmd_vel_stamped) are
    converted to /drone_control for the BeamNG drone. The keyboard teleop
    still requires an interactive terminal and must be run separately:
        ros2 run beamng_teleop_keyboard drone_teleop
    """
    return LaunchDescription([
        Node(
            package="drone_controller",
            executable="drone_controller",
            name="cmd_vel_to_drone_control",
            output="screen",
        ),
        LogInfo(
            msg="Run 'ros2 run beamng_teleop_keyboard drone_teleop' in another terminal for keyboard control.",
        ),
    ])

