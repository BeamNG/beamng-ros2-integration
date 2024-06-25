from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Type

import std_msgs.msg as std_msgs
from beamngpy import Vehicle
from rclpy.node import Node
from rclpy.time import Time


class BeamNGPublisher(ABC):
    """
    Base class for all BeamNG-ROS2 publishers.
    """

    def __init__(self) -> None:
        super().__init__()
        self.name = ""
        self._node = None

    @abstractmethod
    def msg_type(self) -> Type:
        """
        Returns the message type that this publisher publishes.
        """
        pass

    def get_name(self) -> str:
        """
        Returns the name of the publisher, which is also the name of the ROS2 topic.
        """
        return self.name

    def _make_header(self, time: Time, frame_id: str | None = None) -> std_msgs.Header:
        """
        Creates the :class:`std_msgs.Header` header being used in the published messages.

        Args:
            time: The :class:`rclpy.time.Time` object.
            frame_id: The frame id to be used while publishing. If None, defaults to the map frame (``map``).
        """
        if frame_id is None:
            frame_id = "map"
        return std_msgs.Header(stamp=time.to_msg(), frame_id=frame_id)

    def create_publisher(self, node: Node) -> None:
        """
        Creates an inner instance of the :class:`rclpy.publisher.Publisher` for the node
        with the message type that is defined by the :func:`msg_type` function.

        Args:
            node: The node for which the publisher will be created.
        """
        self._node = node
        self._publisher = node.create_publisher(self.msg_type(), f"~/{self.name}", 1)


class VehiclePublisher(BeamNGPublisher):
    """
    Base class for BeamNG vehicle publishers, which have access to a connected :class:`beamngpy.Vehicle`
    instance.
    """

    def __init__(self) -> None:
        super().__init__()
        self.vehicle = None
        self.custom_frame = None

    def _make_header(self, time: Time, frame_id: str | None = None) -> std_msgs.Header:
        if frame_id is None:
            frame_id = self.vehicle.vid
        return std_msgs.Header(stamp=time.to_msg(), frame_id=frame_id)

    def pre_scenario_start(self, vehicle: Vehicle) -> None:
        """
        A hook function that is called for the connected vehicle before the scenario is loaded and started.

        Args:
            vehicle: The vehicle instance **not connected** to the simulator at the moment of calling this function.
        """
        self.vehicle = vehicle

    def post_scenario_start(self, vehicle: Vehicle) -> None:
        """
        A hook function that is called for the connected vehicle after the scenario is loaded and started.

        Args:
            vehicle: The vehicle instance **connected** to the simulator at the moment of calling this function.
        """
        return

    @abstractmethod
    def publish(self, time: Time):
        """
        Function that publishes the corresponding data using the inner instance of the ROS2 publisher. Needs
        to be implemented by derived classes.
        """
        pass
