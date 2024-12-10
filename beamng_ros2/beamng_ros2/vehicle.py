from __future__ import annotations

from typing import List, Tuple, cast

import beamng_msgs.srv as srv
import numpy as np
from beamng_ros2.publishers import (
    AutoSensorPublisher,
    CouplingPublisher,
    SensorPublisher,
    VehicleVisualizer,
)
from beamng_ros2.utils import (
    beamng_rot_to_ros_coords,
    beamng_vec_to_ros_coords,
    float_to_duration,
    float_to_time,
    load_json,
    quat_from_dir,
    xyz_to_vec3,
    xyzw_to_quat,
)
from beamngpy.misc.colors import coerce_color
from beamngpy.types import Float3, Float4
from beamngpy.vehicle import Vehicle
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.time import Time
from rclpy.timer import Timer
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class VehicleNode(Node):
    # foxglove frame viz is not working properly if sim time > real time
    DELAY_OFFSET_SEC = 10.0

    def _create_services(self):
        self.create_service(
            srv.StartCosimulation, "~/start_cosim", self.start_cosimulation
        )
        self.create_service(
            srv.StopCosimulation, "~/stop_cosim", self.stop_cosimulation
        )

    def _create_publishers(self):
        for sensor in self.sensors:
            sensor.create_publisher(self)
        if self._visualizer:
            self._visualizer.create_publisher(self)
            self._visualizer.pre_scenario_start(self.vehicle)

    def _compute_time_offset(self):
        sim_time = float_to_duration(self.vehicle.state["time"] + self.DELAY_OFFSET_SEC)
        return self.get_clock().now() - sim_time

    def _get_vehicle_viz_args(self) -> Tuple[Float3, np.ndarray, Float4, Float4]:
        bbox = {key: np.array(value) for key, value in self.vehicle.get_bbox().items()}
        pos = self.vehicle.state["pos"]
        x = float(np.linalg.norm(bbox["front_bottom_left"] - bbox["rear_bottom_left"]))
        y = float(
            np.linalg.norm(bbox["front_bottom_left"] - bbox["front_bottom_right"])
        )
        z = float(np.linalg.norm(bbox["front_bottom_left"] - bbox["front_top_left"]))

        box_center = (bbox["rear_top_right"] + bbox["front_bottom_left"]) / 2
        offset = box_center - pos

        if self.vehicle.options.get("color", None):
            color_tuple = coerce_color(self.vehicle.options["color"])
            color = (color_tuple[0], color_tuple[1], color_tuple[2], 1.0)
        else:
            color = (0.0, 1.0, 0.0, 1.0)
        arrow_color = (1.0, 0.0, 0.0, 1.0)
        return (x, y, z), offset, color, arrow_color

    def __init__(
        self,
        vehicle: Vehicle,
        parent: Node,
        ros_sensors: List[SensorPublisher],
        cosimulation: str | None = None,
        visualize: bool = True,
    ) -> None:
        super().__init__(vehicle.vid, namespace=parent.get_namespace() + "vehicles")

        self.logger = self.get_logger()
        self.declare_parameter("static_broadcast_steps", 100)
        self.declare_parameter("update_sec", 1 / 30)
        self.next_static_broadcast = 0
        self.vehicle = vehicle
        self.node = parent
        self.destroyed = False
        if cosimulation is None:
            self.timer: Timer | None = self._create_vehicle_timer()
        else:  # cosimulation and sensor publishing not available at the same moment atm
            self.timer = None
        self.sensors: List[SensorPublisher] = ros_sensors

        self.vehicle.sensors.poll()
        vehicle_args = self._get_vehicle_viz_args()
        self._offset = vehicle_args[1]
        self._visualizer = VehicleVisualizer(*vehicle_args) if visualize else None

        self.start_time = self._compute_time_offset()
        self._create_services()
        self._create_publishers()
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)

        self._coupling_publisher: CouplingPublisher | None = None
        if cosimulation:
            self._start_cosimulation(cosimulation)

    def reset_timer(self):
        self.timer = self._create_vehicle_timer()

    def _create_vehicle_timer(self):
        update_sec = self.get_parameter("update_sec").get_parameter_value().double_value
        return self.create_timer(update_sec, self.publisher_callback)

    def destroy_node(self):
        if self.destroyed:
            return
        self.destroyed = True
        for sensor in self.sensors:
            sensor.remove()
        if self.timer:
            self.destroy_timer(self.timer)
        if self._coupling_publisher:
            self._stop_cosimulation()
            self._coupling_publisher.stop()
            self._coupling_publisher = None
        super().destroy_node()
        self.vehicle.disconnect()

    def _broadcast_sensor_pose(self, sensor: AutoSensorPublisher, time: Time):
        pos = sensor.config.get("pos", (0.0, 0.0, 0.0))
        dir = sensor.config.get("dir", (0.0, -1.0, 0.0))
        pos = beamng_vec_to_ros_coords(pos)
        dir = beamng_vec_to_ros_coords(dir)
        rotation = quat_from_dir(dir)

        tf_msg = TransformStamped(
            header=Header(stamp=time.to_msg(), frame_id=self.vehicle.vid),
            child_frame_id=sensor.custom_frame,
            transform=Transform(
                translation=xyz_to_vec3(*pos), rotation=xyzw_to_quat(*rotation)
            ),
        )
        self._tf_static_broadcaster.sendTransform(tf_msg)

    def _broadcast_vehicle_pose(self, time: Time):
        state = self.vehicle.state
        rotation = beamng_rot_to_ros_coords(state["rotation"])
        pos_centered = self._offset + state["pos"]

        tf_msg = TransformStamped(
            header=Header(stamp=time.to_msg(), frame_id="map"),
            child_frame_id=self.vehicle.vid,
            transform=Transform(
                translation=xyz_to_vec3(*pos_centered), rotation=xyzw_to_quat(*rotation)
            ),
        )
        self._tf_broadcaster.sendTransform(tf_msg)

    def publisher_callback(self):
        self.next_static_broadcast = self.next_static_broadcast - 1
        broadcast = self.next_static_broadcast < 0
        if broadcast:
            self.next_static_broadcast = (
                self.get_parameter("static_broadcast_steps")
                .get_parameter_value()
                .integer_value
            )

        try:
            self.vehicle.sensors.poll()
            time = float_to_time(self.start_time, self.vehicle.state["time"])
            self._broadcast_vehicle_pose(time)

            for sensor in self.sensors:
                if broadcast and sensor.custom_frame:
                    self._broadcast_sensor_pose(cast(AutoSensorPublisher, sensor), time)
                sensor.publish(time)
            if self._visualizer:
                self._visualizer.publish(time)
        except Exception as e:
            self.logger.error(f"Fatal error [{self.vehicle.vid}]: {str(e)}")

    def _start_cosimulation(self, path_to_cosim_definition: str):
        try:
            data = load_json(path_to_cosim_definition)
        except FileNotFoundError:
            self.logger.error(
                f'file "{path_to_cosim_definition}" '
                "does not exist, aborting subroutine"
            )
            return False

        self._coupling_publisher = CouplingPublisher(data, self, self.vehicle)
        self.vehicle._send(dict(type="StartCosimulation", **data)).ack(
            "CosimulationStarted"
        )
        return True

    def _stop_cosimulation(self):
        self.vehicle._send(dict(type="StopCosimulation")).ack("CosimulationStopped")

    def start_cosimulation(
        self,
        request: srv.StartCosimulation.Request,
        response: srv.StartCosimulation.Response,
    ):
        response.success = self._start_cosimulation(request.path_to_cosim_definition)
        return response

    def stop_cosimulation(
        self,
        request: srv.StopCosimulation.Request,
        response: srv.StopCosimulation.Response,
    ):
        self._stop_cosimulation()
        response.success = True
        return response
