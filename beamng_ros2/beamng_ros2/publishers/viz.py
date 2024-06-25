from typing import Type

import numpy as np
from beamng_ros2.utils import (
    beamng_rot_to_ros_coords,
    rgba_to_color,
    xyz_to_point,
    xyz_to_vec3,
    xyzw_to_quat,
)
from beamngpy.types import Float3, Float4
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

from .base import VehiclePublisher


class VehicleVisualizer(VehiclePublisher):
    """
    Publishes information about the BeamNG vehicle position and rotation as a marker in the form
    of :external+viz_msgs:doc:`interfaces/msg/MarkerArray` messages.

    Args:
        scale: Scale of the vehicle in meters in the ROS2 coordinate system.
        offset: Offset between the position that BeamNG provided (the main node of the vehicle)
                and the position we visualize (the geometric center).
        color: The color of the vehicle.
        arrow_color: The color of the arrow that points in the direction of the vehicle.
    """

    def __init__(
        self, scale: Float3, offset: np.ndarray, color: Float4, arrow_color: Float4
    ) -> None:
        super().__init__()
        self.name = "marker"
        self.scale = scale
        self.offset = offset
        self.color = rgba_to_color(*color)
        self.arrow_color = rgba_to_color(*arrow_color)

    def get_data(self, time: Time) -> MarkerArray:
        state = self.vehicle.state
        ns = self.vehicle.vid + "_pos"
        pos_centered = self.offset + state["pos"]
        rotation = beamng_rot_to_ros_coords(state["rotation"])
        vehicle = Marker(
            header=self._make_header(time, frame_id="map"),
            ns=ns,
            id=0,
            type=Marker.CUBE,
            action=Marker.ADD,
            pose=Pose(
                position=xyz_to_point(*pos_centered),
                orientation=xyzw_to_quat(*rotation),
            ),
            scale=xyz_to_vec3(*self.scale),
            color=self.color,
            lifetime=Duration(sec=5),
        )
        arrow = Marker(
            header=self._make_header(time),
            ns=ns,
            id=1,
            type=Marker.ARROW,
            action=Marker.ADD,
            pose=Pose(
                position=xyz_to_point(self.scale[0] / 2, 0.0, 0.0),
                orientation=xyzw_to_quat(0.0, 0.0, 0.0, 1.0),
            ),
            scale=xyz_to_vec3(1.0, 1.0, 1.0),
            color=self.arrow_color,
            lifetime=Duration(sec=5),
        )
        return MarkerArray(markers=[vehicle, arrow])

    def msg_type(self) -> Type:
        return MarkerArray

    def publish(self, time: Time):
        self._publisher.publish(self.get_data(time))
