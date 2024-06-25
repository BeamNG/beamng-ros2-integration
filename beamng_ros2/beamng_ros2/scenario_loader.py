"""
Helps to decode BeamNG scenario from the ROS2 .json files, examples of which
are provided in the ``/config/scenarios/`` directory.
"""

from typing import Any, Callable, Dict, List, Tuple

from beamng_ros2.publishers import SensorPublisher, StatePublisher
from beamngpy import BeamNGpy, Scenario, Vehicle


def decode_sensors(sensors_spec: List[Dict[str, Any]]) -> List[SensorPublisher]:
    sensor_list: List[SensorPublisher] = [StatePublisher("state", {})]
    for s_spec in sensors_spec:
        sensor_list.append(SensorPublisher.create(s_spec.pop("name"), s_spec.pop("type"), s_spec))
    return sensor_list


def decode_scenario(
    scenario_spec: Dict[str, Any]
) -> Tuple[Scenario, List[Callable], List[Vehicle], List[Dict[str, Any]], Dict[str, bool]]:
    """
    Decodes the scenario objects from the Python dictionary provided in the ``scenario_spec`` argument.

    Args:
        scenario_spec: The scenario data.
    """

    vehicle_list: List[Vehicle] = []
    vehicle_extra_data: List[Dict[str, Any]] = []
    scenario = Scenario(scenario_spec.pop("level"), scenario_spec.pop("name"))
    for v_spec in scenario_spec["vehicles"]:
        vehicle = Vehicle(
            v_spec["name"],
            v_spec["model"],
            license=v_spec.get("license"),
            color=v_spec.get("color"),
            color2=v_spec.get("color2"),
            color3=v_spec.get("color3"),
            part_config=v_spec.get("part_config"),
            options=v_spec.get("options", {}),
        )

        ros_sensors = decode_sensors(v_spec.get("sensors", {}))
        on_scenario_start_vehicle: List[Callable[[Vehicle], None]] = []
        for sensor in ros_sensors:
            sensor.pre_scenario_start(vehicle)
        if v_spec.get("ai_mode") is not None:
            ai_mode = v_spec["ai_mode"]
            on_scenario_start_vehicle.append(lambda vehicle: vehicle.ai.set_mode(ai_mode))
        if v_spec.get("ai_speed") is not None:
            ai_speed = v_spec["ai_speed"]
            on_scenario_start_vehicle.append(lambda vehicle: vehicle.ai.set_speed(ai_speed))
        if v_spec.get("attach_couplers"):
            on_scenario_start_vehicle.append(lambda vehicle: vehicle.queue_lua_command("beamstate.toggleCouplers()"))

        vehicle_list.append(vehicle)
        vehicle_extra_data.append(dict(on_scenario_start=on_scenario_start_vehicle, ros_sensors=ros_sensors))
        if v_spec.get("cosimulation") is not None:
            vehicle_extra_data[-1]["cosimulation"] = v_spec["cosimulation"]
        scenario.add_vehicle(vehicle, pos=v_spec["pos"], rot_quat=v_spec["rot"])

    on_scenario_start: List[Callable[[BeamNGpy], None]] = []
    wp_key = "weather_presets"
    if wp_key in scenario_spec.keys():
        on_scenario_start.append(lambda beamng: beamng.env.set_weather_preset(scenario_spec[wp_key]))
    if "time_of_day" in scenario_spec.keys():
        on_scenario_start.append(lambda beamng: beamng.env.set_tod(scenario_spec["time_of_day"]))
    if "mode" in scenario_spec.keys() and scenario_spec["mode"] == "paused":
        on_scenario_start.append(lambda beamng: beamng.control.pause())
    net_viz_keys = ["network_visualization", "network_vizualization"]
    flags = dict(net_viz=False)
    for net_viz_key in net_viz_keys:
        if net_viz_key in scenario_spec and scenario_spec[net_viz_key] == "on":
            flags["net_viz"] = True
            break
    return scenario, on_scenario_start, vehicle_list, vehicle_extra_data, flags
