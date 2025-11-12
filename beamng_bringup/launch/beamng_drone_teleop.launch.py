from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    """
    Launch description for BeamNG drone teleop.
    
    NOTE: This launch file does NOT actually launch the teleop node because
    it requires an interactive terminal (TTY) for keyboard input.
    
    To use the drone teleop, run it directly in a terminal:
        ros2 run beamng_teleop_keyboard drone_teleop
    
    Or use the alias (if added to .bashrc):
        drone_teleop
    """
    return LaunchDescription([
        LogInfo(
            msg=[
                "=" * 60,
                "Drone Teleop Launch File",
                "=" * 60,
                "",
                "The drone teleop requires an interactive terminal and cannot",
                "be launched via this launch file.",
                "",
                "Please run the teleop directly in a terminal:",
                "  ros2 run beamng_teleop_keyboard drone_teleop",
                "",
                "Make sure you have:",
                "  1. Started the drone scenario",
                "  2. Started the drone controller node",
                "  3. Then run the teleop in a separate terminal",
                "=" * 60,
            ]
        )
    ])

