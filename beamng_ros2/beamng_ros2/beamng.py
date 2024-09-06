from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List

import beamng_msgs.msg as msg
import beamng_msgs.srv as srv
import rclpy
from beamng_ros2.publishers import NetworkPublisher
from beamng_ros2.scenario_loader import decode_scenario
from beamng_ros2.utils import float_to_time, load_json
from beamng_ros2.vehicle import VehicleNode
from beamngpy import BeamNGpy, Vehicle
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.timer import Timer


class BeamNGBridge(Node):
    def _setup_services(self):
        self.create_service(
            srv.GetScenarioState, "~/get_scenario_state", self.get_scenario_state
        )
        self.create_service(srv.SpawnVehicle, "~/spawn_vehicle", self.spawn_vehicle)
        self.create_service(srv.StartScenario, "~/start_scenario", self.start_scenario)
        self.create_service(
            srv.GetCurrentVehiclesInfo,
            "~/get_current_vehicles",
            self.get_current_vehicles,
        )
        self.create_service(
            srv.TeleportVehicle, "~/teleport_vehicle", self.teleport_vehicle
        )
        self.create_service(srv.ChangeSimulationState, "~/pause", self.pause)
        self.create_service(srv.ChangeSimulationState, "~/resume", self.resume)

    def _setup_sensor_definitions(self, sensor_paths: List[str | Path]):
        default_path: List[str | Path] = ["/config/sensors.json"]
        if not sensor_paths:
            sensor_paths = default_path
        sensor_defs = {}
        for path in sensor_paths:
            sensor_def = load_json(path)
            sensor_defs.update(sensor_def)
        self._sensor_defs = sensor_defs

    def __init__(self):
        super().__init__("beamng_bridge")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 64256)
        self.declare_parameter("launch", True)
        self.declare_parameter("update_sec", 1 / 30)
        self.declare_parameter("update_sec_beamng", 10.0)
        self._setup_services()

        self._beamng: BeamNGpy | None = None
        self._sensor_defs: Dict[str, Any] = {}
        self._network_publisher: NetworkPublisher | None = None
        self._publisher_timer: Timer | None = None

        self.running = False
        self._vehicles: Dict[str, VehicleNode] = {}
        self._vehicle_data: Dict[str, Any] = {}

        self.logger = self.get_logger()
        self.logger.info("Started beamng_bridge.")

        self.clock = self.get_clock()

    def destroy_node(self) -> bool:
        self.disconnect()
        return super().destroy_node()

    def disconnect(self):
        self._cleanup_scenario()
        if self._beamng:
            self._beamng.disconnect()
        self._beamng = None

    def beamng(self) -> BeamNGpy:
        if not self._beamng:
            host = self.get_parameter("host").get_parameter_value().string_value
            port = self.get_parameter("port").get_parameter_value().integer_value
            launch = self.get_parameter("launch").get_parameter_value().bool_value
            self._beamng = BeamNGpy(host, port).open(listen_ip=host, launch=launch)
        return self._beamng

    def _cancel_timers(self):
        timers_cancelled = False
        for node in self._vehicles.values():
            if node.timer:
                self.destroy_timer(node.timer)
                timers_cancelled = True
                node.timer = None
        if self._publisher_timer:
            timers_cancelled = True
            self.destroy_timer(self._publisher_timer)
            self._publisher_timer = None
        return timers_cancelled

    def set_parameters(
        self, parameter_list: List[rclpy.Parameter]
    ) -> List[SetParametersResult]:
        timers_cancelled = False
        for parameter in parameter_list:
            if self._beamng and (parameter.name == "host" or parameter.name == "port"):
                self.disconnect()

            if parameter.name == "update_sec":
                timers_cancelled = self._cancel_timers()
        result = super().set_parameters(parameter_list)
        if timers_cancelled:
            for node in self._vehicles.values():
                node.reset_timer()
            self._publisher_timer = self._create_beamng_timer()
        return result

    def get_scenario_state(
        self,
        request: srv.GetScenarioState.Request,
        response: srv.GetScenarioState.Response,
    ):
        beamng = self.beamng()

        game_state = beamng.control.get_gamestate()
        response.state.loaded = False
        response.state.running = False
        response.state.scenario_name = ""
        response.state.level_name = ""
        response.state.vehicle_ids = []

        if game_state["state"] == "scenario":
            response.state.loaded = True
            response.state.level_name = game_state["level"]
            response.state.vehicle_ids = list(beamng.vehicles.get_current(False).keys())
            if "scenario_state" in game_state.keys():
                if game_state["scenario_state"] == "running":
                    response.state.running = True
                response.state.scenario_name = beamng.scenario.get_name()
        elif game_state["state"] == "menu":
            response.state.scenario_name = game_state["state"]
        return response

    def spawn_vehicle(
        self, request: srv.SpawnVehicle.Request, response: srv.SpawnVehicle.Response
    ):
        response.success = False

        try:
            vehicle_spec = load_json(request.path_to_vehicle_config_file)
        except FileNotFoundError:
            self.logger.error(
                f'file "{request.path_to_vehicle_config_file}" '
                "does not exist, aborting subroutine"
            )
            return response
        if len(request.pos) != 3:
            self.logger.error(
                "position param does not fit " f"required format:{str(request.pos)}"
            )
            return response
        if len(request.rot_quat) != 4:
            self.logger.error(
                f"rotation param does not fit "
                "required quaternion format:{str(req.rot_quat)}"
            )
            return response

        vehicle = Vehicle(request.name, vehicle_spec["model"])
        response.success = self.beamng().vehicles.spawn(
            vehicle, request.pos, request.rot_quat
        )
        if response.success:
            self._spawn_vehicle_node(vehicle, {"ros_sensors": [], "cosimulation": None})
        return response

    def _spawn_vehicle_node(self, vehicle: Vehicle, extra_data: Dict[str, Any]):
        vehicle_node = VehicleNode(vehicle, self, **extra_data)
        self._vehicles[vehicle.vid] = vehicle_node

    def _create_beamng_timer(self):
        update_sec_beamng = (
            self.get_parameter("update_sec_beamng").get_parameter_value().double_value
        )
        return self.create_timer(update_sec_beamng, self.publisher_callback_beamng)

    def _remove_road_network(self):
        if self._network_publisher:
            self._network_publisher._delete_old_markers()

    def _cleanup_scenario(self):
        self._cancel_timers()
        for node in self._vehicles.values():
            node.destroy_node()

        self._vehicle_data = {}
        self._vehicles = {}
        self.running = False

    def start_scenario(
        self, request: srv.StartScenario.Request, response: srv.StartScenario.Response
    ):
        response.success = False
        try:
            beamng = self.beamng()
        except Exception as e:
            self.logger.error(
                f"Cannot start/connect to BeamNG. Check the `host` and `port` parameters of this node. Original error: {e}"
            )
            return response

        if self.running:
            self._cleanup_scenario()

        try:
            json_scenario = load_json(request.path_to_scenario_definition)
        except FileNotFoundError as e:
            self.logger.error(
                f"File '{request.path_to_scenario_definition}' not found. Original error: {e}"
            )
            response.success = False
            return response
        scenario, on_scenario_start, vehicle_list, vehicle_extra_data, flags = (
            decode_scenario(json_scenario)
        )
        scenario.make(beamng)

        beamng.scenario.load(scenario)
        beamng.scenario.start()
        for func in on_scenario_start:
            func(beamng)

        if flags["net_viz"]:
            self._network_publisher = NetworkPublisher()
            self._network_publisher.create_publisher(self)
        else:
            if self._network_publisher and self._network_publisher._publisher:
                self.destroy_publisher(self._network_publisher._publisher)
            self._network_publisher = None
        self._timer = self._create_beamng_timer()

        self._vehicles = {}
        for idx, vehicle in enumerate(vehicle_list):
            for func in vehicle_extra_data[idx]["on_scenario_start"]:
                func(vehicle)
            for sensor in vehicle_extra_data[idx]["ros_sensors"]:
                sensor.post_scenario_start(vehicle)
            vehicle_extra_data[idx].pop("on_scenario_start")
            self._spawn_vehicle_node(vehicle, vehicle_extra_data[idx])

        self.publisher_callback_beamng()
        self.logger.info(f'Started scenario "{scenario.name}".')
        self.running = True

        response.success = True
        return response

    def get_current_vehicles(
        self,
        request: srv.GetCurrentVehiclesInfo.Request,
        response: srv.GetCurrentVehiclesInfo.Response,
    ):
        vehicles = self.beamng().vehicles.get_current(include_config=False)

        response.vehicles = []
        for vid, vehicle in vehicles.items():
            veh_inf = msg.VehicleInfo(vehicle_id=vid, model=vehicle.model)
            response.vehicles.append(veh_inf)

        return response

    def teleport_vehicle(
        self,
        request: srv.TeleportVehicle.Request,
        response: srv.TeleportVehicle.Response,
    ):
        response.success = self.beamng().vehicles.teleport(
            request.vehicle_id, request.pos, request.rot_quat
        )
        return response

    def pause(
        self,
        request: srv.ChangeSimulationState.Request,
        response: srv.ChangeSimulationState.Response,
    ):
        try:
            self.beamng().control.pause()
            response.success = True
        except Exception as ex:
            self.logger.error(f"Unexpected error: {ex}")
            response.success = False
        finally:
            return response

    def resume(
        self,
        request: srv.ChangeSimulationState.Request,
        response: srv.ChangeSimulationState.Response,
    ):
        try:
            self.beamng().control.resume()
            response.success = True
        except Exception as ex:
            self.logger.error(f"Unexpected error: {ex}")
            response.success = False
        finally:
            return response

    def publisher_callback_beamng(self):
        if self._network_publisher:
            time = Time()
            if len(self._vehicles) > 0:
                veh = self._vehicles[next(iter(self._vehicles))]
                time_float = veh.vehicle.state.get("time", 0.0)
                time = float_to_time(veh.start_time, time_float)
            self._network_publisher.publish(self.beamng(), time)


def main(params: List[Parameter] | None = None, args=None):
    rclpy.init(args=args)

    beamng_bridge = BeamNGBridge()
    if params is not None:
        beamng_bridge.set_parameters(params)
    rclpy.spin(beamng_bridge)

    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser("beamng_bridge")
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=64256)
    parser.add_argument("--launch", action="store_false")
    parser.add_argument("--update_sec", type=float, default=0.1)

    args = parser.parse_args()
    params = [Parameter(arg, value=getattr(args, arg)) for arg in vars(args)]

    main(params)
