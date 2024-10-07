"""
Sensor publishers are connected to BeamNG vehicles using BeamNGpy and they periodically publish
the sensor data on the ``~/beamng/<vehicle name>/<sensor name>`` topic.

The default values of sensor attributes are located in the ``/config/sensors.json`` file,
and you can use any arguments that the

Complete list of sensor types which are available in the BeamNG-ROS2 integration:

+-----------------+-------------------------------+--------------------------------------------+
| Sensor type     | BeamNG-ROS2 publisher         | BeamNGpy sensor                            |
+=================+===============================+============================================+
| ``imu``         | :class:`AdvancedIMUPublisher` | :class:`beamngpy.sensors.AdvancedIMU`      |
+-----------------+-------------------------------+--------------------------------------------+
| ``camera``      | :class:`CameraPublisher`      | :class:`beamngpy.sensors.Camera`           |
+-----------------+-------------------------------+--------------------------------------------+
| ``gps``         | :class:`GPSPublisher`         | :class:`beamngpy.sensors.GPS`              |
+-----------------+-------------------------------+--------------------------------------------+
| ``idealradar``  | :class:`IdealRadarPublisher`  | :class:`beamngpy.sensors.IdealRadar`       |
+-----------------+-------------------------------+--------------------------------------------+
| ``lidar``       | :class:`LidarPublisher`       | :class:`beamngpy.sensors.Lidar`            |
+-----------------+-------------------------------+--------------------------------------------+
| ``mesh``        | :class:`MeshPublisher`        | :class:`beamngpy.sensors.Mesh`             |
+-----------------+-------------------------------+--------------------------------------------+
| ``powertrain``  | :class:`PowertrainPublisher`  | :class:`beamngpy.sensors.PowertrainSensor` |
+-----------------+-------------------------------+--------------------------------------------+
| ``radar``       | :class:`RadarPublisher`       | :class:`beamngpy.sensors.Radar`            |
+-----------------+-------------------------------+--------------------------------------------+
| ``roadssensor`` | :class:`RoadsSensorPublisher` | :class:`beamngpy.sensors.RoadsSensor`      |
+-----------------+-------------------------------+--------------------------------------------+
| ``ultrasonic``  | :class:`UltrasonicPublisher`  | :class:`beamngpy.sensors.Ultrasonic`       |
+-----------------+-------------------------------+--------------------------------------------+
| ``damage``      | :class:`DamagePublisher`      | :class:`beamngpy.sensors.Damage`           |
+-----------------+-------------------------------+--------------------------------------------+
| ``electrics``   | :class:`ElectricsPublisher`   | :class:`beamngpy.sensors.Electrics`        |
+-----------------+-------------------------------+--------------------------------------------+
| ``gforces``     | :class:`GForcesPublisher`     | :class:`beamngpy.sensors.GForces`          |
+-----------------+-------------------------------+--------------------------------------------+
| ``state``       | :class:`StatePublisher`       | :class:`beamngpy.sensors.State`            |
+-----------------+-------------------------------+--------------------------------------------+
| ``timer``       | :class:`TimerPublisher`       | :class:`beamngpy.sensors.Timer`            |
+-----------------+-------------------------------+--------------------------------------------+

.. highlight:: javascript

Some of the sensors support shared memory (with and without explicit polling), to enable it,
set the following attributes of the sensors::

    "is_using_shared_memory": true, // enable shared memory
    "is_streaming": true, // enable streaming - updating the memory without sending a request to BeamNG

.. highlight:: python

Sensor Publishers List
======================
"""

from __future__ import annotations

from abc import abstractmethod
from typing import Any, Callable, Dict, List, Type, cast

import beamng_msgs.msg as msgs
import geometry_msgs.msg as geom_msgs
import numpy as np
import sensor_msgs.msg as sensor_msgs
from beamng_ros2.utils import load_json, xyz_to_point, xyz_to_vec3
from beamngpy import Vehicle, sensors
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from .base import VehiclePublisher

try:
    import radar_msgs.msg as radar_msgs

    RADAR_MSGS_FOUND = True
except ImportError as e:
    RADAR_MSGS_FOUND = False


class SensorPublisher(VehiclePublisher):
    """
    Base class for BeamNG sensor publishers, which have access to a connected :class:`beamngpy.Vehicle`
    instance.
    """

    @staticmethod
    def get_sensor_publisher(
        sensor_type: str,
    ) -> Callable[[str, Dict[str, Any]], "SensorPublisher"]:
        """
        Gets the constructor of the specific publisher with the provided type.
        The constructor of all sensor publishers take two arguments: name and configuration.

        Args:
            sensor_type: The sensor type (see the top for their comprehensive list).
        """
        sensor_type = sensor_type.lower()
        if sensor_type in SENSOR_MAPPING:
            return SENSOR_MAPPING[sensor_type]
        raise NotImplementedError(f"Sensor type '{sensor_type}' not implemented!")

    @staticmethod
    def get_sensor_config(sensor_type: str, sensor_config_name: str) -> Dict[str, Any]:
        """
        Gets the default configuration of a sensor by its type. Configuration is a dictionary
        where keys are strings representing the attributes of the sensor.

        Args:
            sensor_type: The sensor type (see the top for their comprehensive list).
            sensor_config_name: Some sensors include custom configurations setting multiple parameters at once.
                                If empty, the default configuration will be used.

        Returns:
            The configuration of the sensor as a dictionary.
        """
        sensor_config_name = (
            sensor_config_name.lower() if sensor_config_name else "default"
        )
        sensor_configs: Dict[str, Dict[str, Any]] = SENSOR_DEFS.get(
            sensor_type.lower(), {}
        )
        sensor_config = sensor_configs.get(sensor_config_name, {})
        return sensor_config

    @staticmethod
    def create(name: str, type: str, parameters: Dict[str, Any]) -> "SensorPublisher":
        """
        Creates an instance of a specific sensor publishers based on the name, type and
        extra parameters which will be passed to the underlying BeamNGpy sensor.

        Args:
            name: Name of the sensor. Has to be unique for a vehicle.
            type: Type of the sensor in the format of either "<TYPE>" or "<TYPE>.<CONFIGURATION>".
            parameters: Extra parameters of the sensor, all fields included in this dictionary will
                        override the parameters set up by the sensor type and configuration.
        """
        config = {}
        sensor_proto_name = type.split(".", maxsplit=1)
        if len(sensor_proto_name) == 1:
            sensor_proto_name.append("")
        sensor_type, sensor_config_name = sensor_proto_name
        sensor_publisher = SensorPublisher.get_sensor_publisher(sensor_type)
        config.update(
            SensorPublisher.get_sensor_config(sensor_type, sensor_config_name)
        )
        config.update(parameters)
        name = name

        return sensor_publisher(name, config)

    @abstractmethod
    def poll(self) -> Dict[str, Any]:
        """
        Gets the raw data from the simulator. Needs to be implemented by derived classes.
        """
        pass

    @abstractmethod
    def get_data(self, time: Time) -> Any:
        """
        Gets the data in the format publishable to the topic.

        Args:
            time: The ROS2 time to be used in the header of the message.
        """
        pass

    @abstractmethod
    def remove(self):
        pass

    def create_publisher(self, node: Node):
        self._node = node
        self._publisher = node.create_publisher(
            self.msg_type(), f"~/sensors/{self.name}", 1
        )

    def publish(self, time: Time):
        data = self.get_data(time)
        if data is not None:
            self._publisher.publish(data)
        else:
            self._node.get_logger().error(
                f"[{self.name}] Did not receive any data from the sensor."
            )


class AutoSensorPublisher(SensorPublisher):
    """
    A base class for "automated" sensors, which require to be created after a vehicle
    is loaded and connected to the scenario.

    Args:
        name: Name of the sensor.
        prototype: Constructor of the corresponding BeamNGpy sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, prototype: Callable, config: Dict[str, Any]) -> None:
        super().__init__()
        self.name = name
        self.proto = prototype
        self.config = config
        self.sensor = None

    def post_scenario_start(self, vehicle: Vehicle):
        super().post_scenario_start(vehicle)
        self.sensor = self.proto(
            name=self.name, bng=vehicle.bng, vehicle=vehicle, **self.config
        )
        if "pos" in self.config:
            self.custom_frame = self.vehicle.vid + "_" + self.name

    def poll(self) -> Dict[str, Any]:
        return self.sensor.poll()

    def remove(self):
        return self.sensor.remove()


class ClassicalSensorPublisher(SensorPublisher):
    """
    A base class for "classical" sensors, which are usually created before loading the scenario.
    Polling of all classical sensors is done in a centralized way using the :func:`beamngpy.Vehicle.sensors.poll`
    function and therefore is not included in this base class.

    Args:
        name: Name of the sensor.
        prototype: Constructor of the corresponding BeamNGpy sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, prototype: Callable, config: Dict[str, Any]) -> None:
        super().__init__()
        self.name = name
        self.proto = prototype
        self.config = config

    def pre_scenario_start(self, vehicle: Vehicle):
        super().pre_scenario_start(vehicle)
        vehicle.sensors.attach(self.name, self.proto(**self.config))

    def poll(self) -> Dict[str, Any]:
        return self.vehicle.sensors[self.name].data

    def remove(self):
        return


class AdvancedIMUPublisher(AutoSensorPublisher):
    """
    IMU sensor publisher. Returns standard :external+sensor_msgs:doc:`interfaces/msg/Imu` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.AdvancedIMU, config)

    def msg_type(self) -> Type:
        return sensor_msgs.Imu

    def get_data(self, time: Time) -> sensor_msgs.Imu:
        data = self.poll()
        if 0.0 in data:
            data = data[0.0]

        msg = sensor_msgs.Imu(
            header=self._make_header(time),
            orientation=geom_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=[
                -1.0,
            ]
            * 9,
            angular_velocity=xyz_to_vec3(*data["angVelSmooth"]),
            angular_velocity_covariance=[
                -1.0,
            ]
            * 9,
            linear_acceleration=xyz_to_vec3(*data["accSmooth"]),
            linear_acceleration_covariance=[
                -1.0,
            ]
            * 9,
        )
        return msg


class CameraPublisher(AutoSensorPublisher):
    """
    Camera data publisher. Publishes one or multiple :external+sensor_msgs:doc:`interfaces/msg/Image` messages
    depending on the configuration:

    * ``~/sensors/<CAM_NAME>/colour``: color data if ``is_render_colours=True``
    * ``~/sensors/<CAM_NAME>/annotation``: class semantic annotations if ``is_render_annotations=True``
    * ``~/sensors/<CAM_NAME>/instance``: object semantic annotations if ``is_render_instance=True``
    * ``~/sensors/<CAM_NAME>/depth`` depth image if ``is_render_depth=True``

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Camera, config)
        self._publishers: Dict[str, Publisher] = {}

    def msg_type(self) -> Type:
        return sensor_msgs.Image

    def _get_img_from_rgba(self, data: bytes, time: Time) -> sensor_msgs.Image:
        cam: sensors.Camera = self.sensor
        decoded = np.frombuffer(data, dtype=np.uint8).reshape(
            cam.resolution[1], cam.resolution[0], 4
        )
        decoded = decoded[..., :3]  # rgba -> rgb
        return sensor_msgs.Image(
            header=self._make_header(time),
            height=cam.resolution[1],
            width=cam.resolution[0],
            encoding="rgb8",
            is_bigendian=0,
            step=cam.resolution[0] * 3,
            data=decoded.tobytes(),
        )

    def _get_img_from_depth(self, data: bytes, time: Time) -> sensor_msgs.Image:
        cam: sensors.Camera = self.sensor
        depth = np.frombuffer(data, dtype=np.float32)
        if cam.postprocess_depth:
            depth = cam.depth_buffer_processing(depth)
        else:
            depth = np.clip(depth * 255.0, 0.0, 255.0)
        depth = np.array(depth, dtype=np.uint8)

        return sensor_msgs.Image(
            header=self._make_header(time),
            height=cam.resolution[1],
            width=cam.resolution[0],
            encoding="8UC1",
            is_bigendian=0,
            step=cam.resolution[0],
            data=depth.tobytes(),
        )

    def create_publisher(self, node: Node) -> None:
        self._node = node
        cam: sensors.Camera = self.sensor
        if cam.is_render_colours:
            self._publishers["colour"] = node.create_publisher(
                self.msg_type(), f"~/sensors/{self.name}/colour", 1
            )
        if cam.is_render_annotations:
            self._publishers["annotation"] = node.create_publisher(
                self.msg_type(), f"~/sensors/{self.name}/annotation", 1
            )
        if cam.is_render_instance:
            self._publishers["instance"] = node.create_publisher(
                self.msg_type(), f"~/sensors/{self.name}/instance", 1
            )
        if cam.is_render_depth:
            self._publishers["depth"] = node.create_publisher(
                self.msg_type(), f"~/sensors/{self.name}/depth", 1
            )

    def get_data(self, time: Time) -> None:
        """
        Not implemented, the camera sensor uses custom logic to poll the data.
        """
        raise NotImplementedError()

    def publish(self, time: Time) -> None:
        self.sensor = cast(sensors.Camera, self.sensor)
        if self.sensor.is_streaming:
            data = self.sensor.stream_raw()
        else:
            data = self.sensor.poll_raw()
        for key in ["colour", "annotation", "instance"]:
            if key in self._publishers and data[key]:
                self._publishers[key].publish(self._get_img_from_rgba(data[key], time))
        if "depth" in self._publishers and data["depth"]:
            self._publishers["depth"].publish(
                self._get_img_from_depth(data["depth"], time)
            )


class GPSPublisher(AutoSensorPublisher):
    """
    GPS sensor publisher publishing :external+sensor_msgs:doc:`interfaces/msg/NavSatFix` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.GPS, config)

    def msg_type(self) -> Type:
        return sensor_msgs.NavSatFix

    def get_data(self, time: Time) -> sensor_msgs.NavSatFix:
        data = self.poll()
        if 0.0 in data:
            data = data[0.0]

        msg = sensor_msgs.NavSatFix(
            header=self._make_header(time),
            status=sensor_msgs.NavSatStatus(
                status=sensor_msgs.NavSatStatus.SERVICE_GPS
            ),
            latitude=data["lat"],
            longitude=data["lon"],
            position_covariance_type=sensor_msgs.NavSatFix.COVARIANCE_TYPE_UNKNOWN,
        )
        return msg


class IdealRadarPublisher(AutoSensorPublisher):
    """
    GPS sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/IdealRadarSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.IdealRadar, config)

    def msg_type(self) -> Type:
        return msgs.IdealRadarSensor

    @staticmethod
    def _vehicle_to_msg(veh: Dict[str, Any]) -> msgs.IdealRadarSensorVehicle:
        return msgs.IdealRadarSensorVehicle(
            vehicle_id=int(veh["vehicleID"]),
            dist_to_player_vehicle_sq=veh["distToPlayerVehicleSq"],
            width=veh["width"],
            length=veh["length"],
            acc=xyz_to_vec3(**veh["acc"]),
            vel_bb=xyz_to_vec3(**veh["velBB"]),
            rel_acc_x=veh["relAccX"],
            rel_acc_y=veh["relAccY"],
            rel_dist_x=veh["relDistX"],
            rel_dist_y=veh["relDistY"],
            rel_vel_x=veh["relVelX"],
            rel_vel_y=veh["relVelY"],
        )

    def get_data(self, time: Time) -> msgs.IdealRadarSensor:
        data = self.poll()
        if 0.0 in data:  # bulk data
            data = data[0.0]
        msg = msgs.IdealRadarSensor(
            header=self._make_header(time),
            closest_vehicles1=self._vehicle_to_msg(data["closestVehicles1"]),
            closest_vehicles2=self._vehicle_to_msg(data["closestVehicles2"]),
            closest_vehicles3=self._vehicle_to_msg(data["closestVehicles3"]),
            closest_vehicles4=self._vehicle_to_msg(data["closestVehicles4"]),
        )
        return msg


class LidarPublisher(AutoSensorPublisher):
    """
    Lidar sensor publisher publishing :external+sensor_msgs:doc:`interfaces/msg/PointCloud2` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Lidar, config)
        self.tf_buffer = Buffer()
        self.tf_listener = None

    def msg_type(self) -> Type:
        return sensor_msgs.PointCloud2

    def create_publisher(self, node: Node):
        super().create_publisher(node)
        self.tf_listener = TransformListener(
            self.tf_buffer, self._node, spin_thread=True
        )

    def get_data(self, time: Time) -> sensor_msgs.PointCloud2:
        self.sensor = cast(sensors.Lidar, self.sensor)
        data = self.sensor.poll()
        if isinstance(data["colours"], list):
            data["colours"] = np.array(data["colours"]).reshape(-1, 4)
        points = cast(np.ndarray, data["pointCloud"])
        colours = cast(np.ndarray, data["colours"])[:, [2, 1, 0, 3]]  # RGBA -> BGRA
        lidar_data = np.column_stack(
            (points, colours.flatten().view("float32"))
        ).tobytes()

        fields = [
            sensor_msgs.PointField(
                name="x", offset=0, datatype=sensor_msgs.PointField.FLOAT32, count=1
            ),
            sensor_msgs.PointField(
                name="y", offset=4, datatype=sensor_msgs.PointField.FLOAT32, count=1
            ),
            sensor_msgs.PointField(
                name="z", offset=8, datatype=sensor_msgs.PointField.FLOAT32, count=1
            ),
            sensor_msgs.PointField(
                name="rgba", offset=12, datatype=sensor_msgs.PointField.UINT32, count=1
            ),
        ]

        width = points.shape[0]
        itemsize = points.itemsize
        return sensor_msgs.PointCloud2(
            header=self._make_header(time, "map"),
            height=1,
            width=width,
            fields=fields,
            is_bigendian=False,
            point_step=(itemsize * 4),
            row_step=(itemsize * 4 * width),
            data=lidar_data,
            is_dense=False,
        )


class MeshPublisher(AutoSensorPublisher):
    """
    Mesh sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/MeshSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Mesh, config)

    def msg_type(self) -> Type:
        return msgs.MeshSensor

    @staticmethod
    def _beam_to_msg(beam: Dict[str, Any]) -> msgs.MeshSensorBeam:
        return msgs.MeshSensorBeam(stress=beam["stress"])

    @staticmethod
    def _node_to_msg(node: Dict[str, Any]) -> msgs.MeshSensorNode:
        return msgs.MeshSensorNode(
            part_origin=node.get("partOrigin", ""),
            mass=node["mass"],
            pos=xyz_to_point(**node["pos"]),
            vel=xyz_to_vec3(**node["vel"]),
            force=xyz_to_vec3(**node["force"]),
        )

    def get_data(self, time: Time) -> msgs.MeshSensor:
        data = self.poll()
        msg = msgs.MeshSensor(
            header=self._make_header(time, self.vehicle.vid),
            beams=[
                self._beam_to_msg(data["beams"][float(i)])
                for i in range(len(data["beams"]))
            ],
            nodes=[
                self._node_to_msg(data["nodes"][float(i)])
                for i in range(len(data["nodes"]))
            ],
        )
        return msg


class PowertrainSensorPublisher(AutoSensorPublisher):
    """
    Powertrain sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/PowertrainSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.PowertrainSensor, config)

    def msg_type(self) -> Type:
        return msgs.PowertrainSensor

    @staticmethod
    def _device_to_msg(
        name: str, device: Dict[str, Any]
    ) -> msgs.PowertrainSensorDevice:
        return msgs.PowertrainSensorDevice(
            name=name,
            input_av=device["inputAV"],
            gear_ratio=device.get("gearRatio", float("nan")),
            is_broken=device.get("isBroken", False),
            mode=device.get("mode", ""),
            parent_name=device.get("parentName", ""),
            parent_output_index=int(device.get("parentOutputIndex", -1)),
            output_torque_1=device.get("outputTorque1", float("nan")),
            output_av_1=device.get("outputAV1", float("nan")),
            output_torque_2=device.get("outputTorque2", float("nan")),
            output_av_2=device.get("outputAV2", float("nan")),
        )

    def get_data(self, time: Time) -> msgs.PowertrainSensor:
        data = self.poll()
        data.pop("time")
        msg = msgs.PowertrainSensor(
            header=self._make_header(time, self.vehicle.vid),
            devices=[
                self._device_to_msg(name, device) for name, device in data.items()
            ],
        )
        return msg


class RadarPublisher(AutoSensorPublisher):
    """
    Radar sensor publisher publishing radar_msgs :radar_msgs:`RadarReturn` messages
    if the library is found, custom :external+beamng_msgs:doc:`interfaces/msg/RadarReturn` messages if not.

    You can force the publisher to always send custom BeamNG message type by setting the
    ``use_beamng_msg_type=True`` argument in the sensor .json definition.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        self.use_beamng_msg_type = not RADAR_MSGS_FOUND or config.get(
            "use_beamng_msg_type", False
        )
        super().__init__(name, sensors.Radar, config)

    def msg_type(self) -> Type:
        if self.use_beamng_msg_type:
            return msgs.RadarScan
        return radar_msgs.RadarScan

    @staticmethod
    def _return_to_msg(ret: List[float]) -> radar_msgs.RadarReturn:
        return radar_msgs.RadarReturn(
            range=float(ret[0]),
            azimut=float(ret[2]),
            elevation=float(ret[3]),
            doppler_velocity=float(ret[1]),
            amplitude=float(ret[5]),
        )

    @staticmethod
    def _return_to_beamng_msg(ret: List[float]) -> msgs.RadarReturn:
        return msgs.RadarReturn(
            range=float(ret[0]),
            doppler_velocity=float(ret[1]),
            azimuth=float(ret[2]),
            elevation=float(ret[3]),
            radar_cross_section=float(ret[4]),
            signal_to_noise_ratio=float(ret[5]),
            facing_factor=float(ret[6]),
        )

    def get_data(self, time: Time) -> msgs.RadarReturn:
        data = self.poll()
        if self.use_beamng_msg_type:
            returns = [self._return_to_beamng_msg(ret) for ret in data]
        else:
            returns = [self._return_to_msg(ret) for ret in data]
        msg = self.msg_type()(
            header=self._make_header(time),
            returns=returns,
        )
        return msg


class RoadsSensorPublisher(AutoSensorPublisher):
    """
    Roads sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/RoadsSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.RoadsSensor, config)

    def msg_type(self) -> Type:
        return msgs.RoadsSensor

    @staticmethod
    def _make_cubic_polynomial(
        a: float, b: float, c: float, d: float
    ) -> msgs.CubicPolynomial:
        return msgs.CubicPolynomial(a=a, b=b, c=c, d=d)

    def get_data(self, time: Time) -> msgs.RoadsSensor | None:
        data = self.poll()
        if len(data) == 0:
            return None
        msg = msgs.RoadsSensor(
            header=self._make_header(time, self.vehicle.vid),
            dist2_cl=data["dist2CL"],
            dist2_left=data["dist2Left"],
            dist2_right=data["dist2Right"],
            half_width=data["halfWidth"],
            road_radius=data["roadRadius"],
            heading_angle=data["headingAngle"],
            p0_on_cl=xyz_to_point(data["xP0onCL"], data["yP0onCL"], data["zP0onCL"]),
            p1_on_cl=xyz_to_point(data["xP1onCL"], data["yP1onCL"], data["zP1onCL"]),
            p2_on_cl=xyz_to_point(data["xP2onCL"], data["yP2onCL"], data["zP2onCL"]),
            p3_on_cl=xyz_to_point(data["xP3onCL"], data["yP3onCL"], data["zP3onCL"]),
            u_cl=self._make_cubic_polynomial(
                data["uAofCL"], data["uBofCL"], data["uCofCL"], data["uDofCL"]
            ),
            v_cl=self._make_cubic_polynomial(
                data["vAofCL"], data["vBofCL"], data["vCofCL"], data["vDofCL"]
            ),
            u_left_re=self._make_cubic_polynomial(
                data["uAofLeftRE"],
                data["uBofLeftRE"],
                data["uCofLeftRE"],
                data["uDofLeftRE"],
            ),
            v_left_re=self._make_cubic_polynomial(
                data["vAofLeftRE"],
                data["vBofLeftRE"],
                data["vCofLeftRE"],
                data["vDofLeftRE"],
            ),
            u_right_re=self._make_cubic_polynomial(
                data["uAofRightRE"],
                data["uBofRightRE"],
                data["uCofRightRE"],
                data["uDofRightRE"],
            ),
            v_right_re=self._make_cubic_polynomial(
                data["vAofRightRE"],
                data["vBofRightRE"],
                data["vCofRightRE"],
                data["vDofRightRE"],
            ),
            start_cl=xyz_to_point(data["xStartCL"], data["yStartCL"], data["zStartCL"]),
            start_l=xyz_to_point(data["xStartL"], data["yStartL"], data["zStartL"]),
            start_r=xyz_to_point(data["xStartR"], data["yStartR"], data["zStartR"]),
            drivability=data["drivability"],
            speed_limit=data["speedLimit"],
            flag1way=data["flag1way"],
        )
        return msg


class UltrasonicPublisher(AutoSensorPublisher):
    """
    Ultrasonic sensor publisher publishing :external+sensor_msgs:doc:`interfaces/msg/Range` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Ultrasonic, config)

    def msg_type(self) -> Type:
        return sensor_msgs.Range

    def get_data(self, time: Time) -> sensor_msgs.Range:
        data = self.poll()

        message = sensor_msgs.Range(
            header=self._make_header(time),
            radiation_type=sensor_msgs.Range.ULTRASOUND,
            field_of_view=0.1,
            min_range=self.config["range_min_cutoff"],
            max_range=self.config["range_direct_max_cutoff"],
            range=data["distance"],
        )
        return message


class DamagePublisher(ClassicalSensorPublisher):
    """
    Damage sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/DamageSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Damage, config)

    def msg_type(self) -> Type:
        return msgs.DamageSensor

    def get_data(self, time: Time) -> msgs.DamageSensor:
        data = self.poll()
        msg = msgs.DamageSensor(header=self._make_header(time, self.vehicle.vid))
        msg.deformgroup_id = []
        msg.part_id = []
        msg.name = []

        for k, v in data["deform_group_damage"].items():
            msg.deformgroup_id.append(k)
            msg.inv_max_events.append(v["invMaxEvents"])
            msg.damage.append(v["damage"])
            msg.event_count.append(v["eventCount"])
            msg.max_events.append(v["maxEvents"])
        if data["part_damage"]:
            for k, v in data["part_damage"].items():
                msg.part_id.append(k)
                msg.name.append(v["name"])
                msg.damage.append(v["damage"])
        return msg


class ElectricsPublisher(ClassicalSensorPublisher):
    """
    Electrics sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/ElectricsSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Electrics, config)

    def msg_type(self):
        return msgs.ElectricsSensor

    @staticmethod
    def _wheel_thermals_to_msg(data: Dict[str, float]) -> msgs.WheelThermals:
        return msgs.WheelThermals(
            brake_core_temperature=data.get("brakeCoreTemperature", float("nan")),
            brake_surface_temperature=data.get("brakeCoreTemperature", float("nan")),
            brake_thermal_efficiency=data.get("brakeCoreTemperature", float("nan")),
        )

    def get_data(self, time: Time) -> msgs.ElectricsSensor:
        data = self.poll()
        msg = msgs.ElectricsSensor(header=self._make_header(time, self.vehicle.vid))
        msg.abs = data.get("abs", float("nan"))
        msg.abs_active = data.get("abs_active", float("nan"))
        msg.accxsmooth = data.get("accXSmooth", float("nan"))
        msg.accysmooth = data.get("accYSmooth", float("nan"))
        msg.acczsmooth = data.get("accZSmooth", float("nan"))
        msg.airflowspeed = data.get("airflowspeed", float("nan"))
        msg.airspeed = data.get("airspeed", float("nan"))
        msg.altitude = data.get("altitude", float("nan"))
        msg.avg_wheel_av = data.get("avg_wheel_av", float("nan"))
        msg.boost = data.get("boost", float("nan"))
        msg.boostmax = data.get("boostMax", float("nan"))
        msg.brake = data.get("brake", float("nan"))
        msg.brake_input = data.get("brake_input", float("nan"))
        msg.brake_lights = data.get("brake_lights", float("nan"))
        msg.brakelight_signal_l = data.get("brakelight_signal_L", float("nan"))
        msg.brakelight_signal_r = data.get("brakelight_signal_R", float("nan"))
        msg.check_engine = bool(data.get("check_engine", False))
        msg.clutch = data.get("clutch", float("nan"))
        msg.clutch_input = data.get("clutch_input", float("nan"))
        msg.clutch_ratio = data.get("clutch_ratio", float("nan"))
        msg.doorflcoupler_notattached = data.get(
            "doorFLCoupler_notAttached", float("nan")
        )
        msg.doorfrcoupler_notattached = data.get(
            "doorFRCoupler_notAttached", float("nan")
        )
        msg.doorrlcoupler_notattached = data.get(
            "doorRLCoupler_notAttached", float("nan")
        )
        msg.doorrrcoupler_notattached = data.get(
            "doorRRCoupler_notAttached", float("nan")
        )
        msg.driveshaft = data.get("driveshaft", float("nan"))
        msg.dsewarningpulse = data.get("dseWarningPulse", float("nan"))
        msg.enginerunning = data.get("engineRunning", float("nan"))
        msg.engine_load = data.get("engine_load", float("nan"))
        msg.engine_throttle = data.get("engine_throttle", float("nan"))
        msg.esc = data.get("esc", float("nan"))
        msg.esc_active = bool(data.get("esc_active", False))
        msg.exhaust_flow = data.get("exhaust_flow", float("nan"))
        msg.fog_lights = data.get("fog_lights", float("nan"))
        msg.freezestate = bool(data.get("freezeState", False))
        msg.fuel = data.get("fuel", float("nan"))
        msg.fuel_capacity = data.get("fuel_capacity", float("nan"))
        msg.fuel_volume = data.get("fuel_volume", float("nan"))
        msg.gear = str(data.get("gear", None))
        msg.gearmodeindex = data.get("gearModeIndex", float("nan"))
        msg.gear_a = data.get("gear_a", float("nan"))
        msg.gear_index = data.get("gear_index", float("nan"))
        msg.gear_m = data.get("gear_m", float("nan"))
        msg.gearboxmode = str(data.get("gearboxMode", None))
        msg.hasabs = float(data.get("hasABS", float("nan")))
        msg.hasesc = float(data.get("hasESC", float("nan")))
        msg.hastcs = float(data.get("hasTCS", float("nan")))
        msg.hazard = data.get("hazard", float("nan"))
        msg.hazard_signal = bool(data.get("hazard_signal", False))
        msg.headlights = data.get("headlights", float("nan"))
        msg.highbeam = data.get("highbeam", float("nan"))
        msg.highbeam_wigwag_l = data.get("highbeam_wigwag_L", float("nan"))
        msg.highbeam_wigwag_r = data.get("highbeam_wigwag_R", float("nan"))
        msg.hoodcatchcoupler_notattached = data.get(
            "hoodCatchCoupler_notAttached", float("nan")
        )
        msg.hoodlatchcoupler_notattached = data.get(
            "hoodLatchCoupler_notAttached", float("nan")
        )
        msg.horn = int(data.get("horn", 0))
        msg.idlerpm = data.get("idlerpm", float("nan"))
        msg.ignition = bool(data.get("ignition", False))
        msg.ignitionlevel = data.get("ignitionLevel", float("nan"))
        msg.isabsbrakeactive = data.get("isABSBrakeActive", float("nan"))
        msg.istcbrakeactive = data.get("isTCBrakeActive", float("nan"))
        msg.isycbrakeactive = data.get("isYCBrakeActive", float("nan"))
        msg.is_shifting = bool(data.get("is_shifting", False))
        msg.left_signal = bool(data.get("left_signal", False))
        msg.lightbar = data.get("lightbar", float("nan"))
        msg.lights = data.get("lights", float("nan"))
        msg.lockupclutchratio = data.get("lockupClutchRatio", float("nan"))
        msg.lowbeam = data.get("lowbeam", float("nan"))
        msg.lowfuel = bool(data.get("lowfuel", False))
        msg.lowhighbeam = data.get("lowhighbeam", float("nan"))
        msg.lowhighbeam_signal_l = data.get("lowhighbeam_signal_L", float("nan"))
        msg.lowhighbeam_signal_r = data.get("lowhighbeam_signal_R", float("nan"))
        msg.lowpressure = data.get("lowpressure", float("nan"))
        msg.maxgearindex = data.get("maxGearIndex", float("nan"))
        msg.maxrpm = data.get("maxrpm", float("nan"))
        msg.mingearindex = data.get("minGearIndex", float("nan"))
        msg.nop = data.get("nop", float("nan"))
        msg.odometer = data.get("odometer", float("nan"))
        msg.oil = data.get("oil", float("nan"))
        msg.oil_temperature = data.get("oil_temperature", float("nan"))
        msg.parking = data.get("parking", float("nan"))
        msg.parkingbrake = data.get("parkingbrake", float("nan"))
        msg.parkingbrake_input = data.get("parkingbrake_input", float("nan"))
        msg.parkingbrakelight = data.get("parkingbrakelight", float("nan"))
        msg.radiator_fan_spin = data.get("radiator_fan_spin", float("nan"))
        msg.reverse = data.get("reverse", float("nan"))
        msg.reverse_wigwag_l = data.get("reverse_wigwag_L", float("nan"))
        msg.reverse_wigwag_r = data.get("reverse_wigwag_R", float("nan"))
        msg.right_signal = bool(data.get("right_signal", False))
        msg.rpm = data.get("rpm", float("nan"))
        msg.rpm_tacho = data.get("rpm_tacho", float("nan"))
        msg.rpmspin = data.get("rpmspin", float("nan"))
        msg.running = bool(data.get("running", False))
        msg.signal_l = data.get("signal_l", float("nan"))
        msg.signal_r = data.get("signal_r", float("nan"))
        msg.smoothshiftlogicav = data.get("smoothShiftLogicAV", float("nan"))
        msg.steering = data.get("steering", float("nan"))
        msg.steeringunassisted = data.get("steeringUnassisted", float("nan"))
        msg.steering_input = data.get("steering_input", float("nan"))
        msg.tailgatecoupler_notattached = data.get(
            "tailgateCoupler_notAttached", float("nan")
        )
        msg.tcs = data.get("tcs", float("nan"))
        msg.tcs_active = bool(data.get("tcs_active", False))
        msg.throttle = data.get("throttle", float("nan"))
        msg.throttle_input = data.get("throttle_input", float("nan"))
        msg.trip = data.get("trip", float("nan"))
        msg.turboboost = data.get("turboBoost", float("nan"))
        msg.turboboostmax = data.get("turboBoostMax", float("nan"))
        msg.turborpm = data.get("turboRPM", float("nan"))
        msg.turborpmratio = data.get("turboRpmRatio", float("nan"))
        msg.turbospin = data.get("turboSpin", float("nan"))
        msg.turnsignal = data.get("turnsignal", float("nan"))
        msg.two_step = bool(data.get("two_step", False))
        msg.virtualairspeed = data.get("virtualAirspeed", float("nan"))
        msg.water_temperature = data.get("water_temperature", float("nan"))
        wheel_thermals = data.get("wheelThermals", {})
        if len(wheel_thermals) == 0:
            wheel_thermals = {}
        msg.wheelthermalsfl = self._wheel_thermals_to_msg(wheel_thermals.get("FL", {}))
        msg.wheelthermalsfr = self._wheel_thermals_to_msg(wheel_thermals.get("FR", {}))
        msg.wheelthermalsrl = self._wheel_thermals_to_msg(wheel_thermals.get("RL", {}))
        msg.wheelthermalsrr = self._wheel_thermals_to_msg(wheel_thermals.get("RR", {}))
        msg.wheelspeed = data.get("wheelspeed", float("nan"))
        return msg


class GForcesPublisher(ClassicalSensorPublisher):
    """
    GForces sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/GForceSensor` messages.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.GForces, config)

    def msg_type(self):
        return msgs.GForceSensor

    def get_data(self, time: Time) -> msgs.GForceSensor:
        data = self.poll()
        return msgs.GForceSensor(
            header=self._make_header(time, self.vehicle.vid),
            g=xyz_to_vec3(data["gx"], data["gy"], data["gz"]),
            g2=xyz_to_vec3(data["gx2"], data["gy2"], data["gz2"]),
        )


class StatePublisher(ClassicalSensorPublisher):
    """
    State publisher publishing :external+beamng_msgs:doc:`interfaces/msg/StateSensor` messages.

    Keep in mind this publisher is loaded by default for every vehicle and therefore you do not
    need to specify it in the configuration file of the scenario.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.State, config)

    def pre_scenario_start(self, vehicle: Vehicle):
        self.vehicle = vehicle  # we want to override this function because the state sensor is already attached

    def msg_type(self) -> Type:
        return msgs.StateSensor

    def get_data(self, time: Time) -> msgs.StateSensor:
        data = self.poll()
        msg = msgs.StateSensor(
            header=self._make_header(time, self.vehicle.vid),
            position=xyz_to_point(*data["pos"]),
            velocity=xyz_to_vec3(*data["vel"]),
            front=xyz_to_vec3(*data["front"]),
            up=xyz_to_vec3(*data["up"]),
            dir=xyz_to_vec3(*data["dir"]),
        )
        return msg


class TimerPublisher(ClassicalSensorPublisher):
    """
    Time sensor publisher publishing :external+beamng_msgs:doc:`interfaces/msg/TimeSensor` messages.

    The timer returns time since the start of scenario, which is not the same time that is used as the
    header of the published messages. You can use the :external+beamng_msgs:doc:`interfaces/msg/StateSensor`
    messages to get the ROS2 time.

    Args:
        name: Name of the sensor.
        config: Arguments of the sensor passed to the BeamNGpy class.
    """

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Timer, config)

    def msg_type(self):
        return msgs.TimeSensor

    def get_data(self, time: Time) -> msgs.TimeSensor:
        data = self.poll()
        msg = msgs.TimeSensor()
        msg.beamng_simulation_time.sec = int(data["time"])
        msg.beamng_simulation_time.nanosec = int(
            (data["time"] - msg.beamng_simulation_time.sec) * 1e6
        )
        return msg


SENSOR_DEFS = load_json("/config/sensors.json")
SENSOR_MAPPING = {
    # auto
    "imu": AdvancedIMUPublisher,
    "camera": CameraPublisher,
    "gps": GPSPublisher,
    "idealradar": IdealRadarPublisher,
    "lidar": LidarPublisher,
    "mesh": MeshPublisher,
    "powertrain": PowertrainSensorPublisher,
    "radar": RadarPublisher,
    "roadssensor": RoadsSensorPublisher,
    "ultrasonic": UltrasonicPublisher,
    # classical
    "damage": DamagePublisher,
    "electrics": ElectricsPublisher,
    "gforces": GForcesPublisher,
    "state": StatePublisher,
    "timer": TimerPublisher,
}
