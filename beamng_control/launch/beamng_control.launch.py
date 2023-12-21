import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    scenario = launch.actions.DeclareLaunchArgument(
        "scenario",
        default_value="west_coast_with_all_sensors",
        description="Choose the scenario",
    )

    scenario_config = launch.actions.DeclareLaunchArgument(
        "scenario_config",
        value=launch.substitutions.LaunchConfiguration("scenario"),
        description="Scenario configuration file",
    )

    port = launch.actions.DeclareLaunchArgument(
        "port",
        default_value="64256",
        description="Port for the simulator",
    )

    host = launch.actions.DeclareLaunchArgument(
        "host",
        default_value=launch.substitutions.LaunchConfiguration("env SIMULATOR_IP"),
        description="Host for the simulator",
    )

    beamng_control_node = launch_ros.actions.Node(
        package="beamng_control",
        executable="bridge",
        # executable="bridge.py",
        name="beamng_control",
        output="screen",
        # parameters=[{"host": launch.substitutions.LaunchConfiguration("host")}, {"port": launch.substitutions.LaunchConfiguration("port")}],
        # arguments=[launch.substitutions.LaunchConfiguration("scenario_config")],
    )

    open_rviz = launch.actions.DeclareLaunchArgument(
        "open_rviz",
        default_value="true",
        description="Open RViz",
    )

    rviz_node = launch.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration("open_rviz")),
        arguments=["-d", launch.substitutions.LaunchConfiguration("scenario_config")],
    )

    return launch.LaunchDescription([
        # scenario,
        # scenario_config,
        # port,
        # host,
        beamng_control_node,
        # open_rviz,
        # rviz_node,
    ])
