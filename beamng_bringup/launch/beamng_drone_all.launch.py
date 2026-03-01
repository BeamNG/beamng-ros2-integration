"""
All-in-one launch for BeamNG drone: bridge + scenario + controller + cmd_vel converter.

Keyboard teleop (drone_teleop) requires an interactive terminal - run separately:
  ros2 run beamng_teleop_keyboard drone_teleop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            msg=[
                "=" * 60,
                "BeamNG Drone - All-in-One Launch",
                "=" * 60,
                "Bridge + scenario + drone controller + cmd_vel converter.",
                "For keyboard control, run in a separate terminal:",
                "  ros2 run beamng_teleop_keyboard drone_teleop",
                "=" * 60,
            ]
        ),
        DeclareLaunchArgument(
            "host",
            default_value="172.23.64.1",
            description="BeamNG.tech host IP (e.g. WSL2: 172.23.64.1 from ipconfig)",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="25252",
            description="BeamNG.tech port",
        ),
        DeclareLaunchArgument(
            "scenario",
            default_value="/config/scenarios/drone_scenario.json",
            description="Path to scenario JSON",
        ),
        DeclareLaunchArgument(
            "vehicle_id",
            default_value="drone",
            description="Vehicle ID in the scenario",
        ),
        DeclareLaunchArgument(
            "controller_delay",
            default_value="18.0",
            description="Seconds to wait before starting drone controller (scenario must load first)",
        ),
        # 1. Bridge
        Node(
            package="beamng_ros2",
            executable="beamng_bridge",
            name="beamng_bridge",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("host"),
                "port": PythonExpression(["int(", LaunchConfiguration("port"), ")"]),
                "launch": False,
                "update_sec": 0.1,
            }],
        ),
        # 2. Scenario (waits for bridge, then starts scenario and exits)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="beamng_ros2",
                    executable="start_scenario",
                    name="start_scenario",
                    output="screen",
                    parameters=[{
                        "scenario": LaunchConfiguration("scenario"),
                    }],
                ),
            ],
        ),
        # 3. Drone controller (start after scenario loads to avoid connection timeout)
        TimerAction(
            period=PythonExpression(["float(", LaunchConfiguration("controller_delay"), ")"]),
            actions=[
                Node(
                    package="beamng_agent",
                    executable="drone_controller",
                    name="beamng_drone_controller",
                    output="screen",
                    parameters=[{
                        "host": LaunchConfiguration("host"),
                        "port": PythonExpression(["int(", LaunchConfiguration("port"), ")"]),
                        "vehicle_id": LaunchConfiguration("vehicle_id"),
                    }],
                ),
            ],
        ),
        # 4. cmd_vel -> drone_control converter
        Node(
            package="drone_controller",
            executable="drone_controller",
            name="cmd_vel_to_drone_control",
            output="screen",
        ),
    ])
