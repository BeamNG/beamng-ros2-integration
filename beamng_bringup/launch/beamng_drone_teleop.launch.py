from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
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
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="cmd_vel",
            description="Topic to subscribe for velocity commands (Twist)",
        ),
        DeclareLaunchArgument(
            "cmd_vel_stamped_topic",
            default_value="cmd_vel_stamped",
            description="Topic to subscribe for timestamped velocity commands (TwistStamped)",
        ),
        DeclareLaunchArgument(
            "drone_cmd_vel_topic",
            default_value="drone/cmd_vel",
            description="Topic to publish converted velocity commands",
        ),
        DeclareLaunchArgument(
            "drone_thrust_topic",
            default_value="drone/thrust",
            description="Topic to publish thrust commands [thrust, roll, pitch, yaw]",
        ),
        DeclareLaunchArgument(
            "drone_control_topic",
            default_value="/drone_control",
            description="Topic to publish BeamNG DroneControl messages",
        ),
        Node(
            package="drone_controller",
            executable="drone_controller",
            name="cmd_vel_to_drone_control",
            output="screen",
            parameters=[{
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "cmd_vel_stamped_topic": LaunchConfiguration("cmd_vel_stamped_topic"),
                "drone_cmd_vel_topic": LaunchConfiguration("drone_cmd_vel_topic"),
                "drone_thrust_topic": LaunchConfiguration("drone_thrust_topic"),
                "drone_control_topic": LaunchConfiguration("drone_control_topic"),
            }],
        ),
        LogInfo(
            msg="Run 'ros2 run beamng_teleop_keyboard drone_teleop' in another terminal for keyboard control.",
        ),
    ])

