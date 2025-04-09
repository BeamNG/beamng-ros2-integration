from typing import List, Type

from beamng_ros2.utils import xyz_to_point, xyz_to_vec3, xyzw_to_quat
from beamngpy import BeamNGpy
from geometry_msgs.msg import Pose
from rclpy.time import Duration, Time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .base import BeamNGPublisher


class NetworkPublisher(BeamNGPublisher):
    """
    A publisher which periodically publishes data about the road network of the currently running scenario.
    """

    def __init__(self) -> None:
        super().__init__()
        self.name = "road_network"
        self._road_network_left = None
        self._road_network_middle = None
        self._road_network_right = None

    def get_data(self) -> List[MarkerArray]:
        """
        Parses the road network data to be aviailable in the format that the
        :func:`publish` function expects.
        """

        markers = []
        if self._road_network_left:
            markers.append(self._road_network_left)
        if self._road_network_middle:
            markers.append(self._road_network_middle)
        if self._road_network_right:
            markers.append(self._road_network_right)
        if not markers:
            markers.append(MarkerArray())
        return markers

    def msg_type(self) -> Type:
        return MarkerArray

    def _delete_old_markers(self):
        markers = [
            Marker(id=0, ns=f"{type}_{self.name}", action=Marker.DELETEALL)
            for type in ["left", "middle", "right"]
        ]
        array = MarkerArray(markers=markers)
        self._publisher.publish(array)

    def _create_road_network(self, type: str, color: ColorRGBA, time, network_def):
        return MarkerArray(
            markers=[
                Marker(
                    header=self._make_header(time, "map"),
                    ns=f"{type}_{self.name}",
                    id=m_id,
                    type=Marker.LINE_STRIP,
                    action=Marker.ADD,
                    pose=Pose(
                        position=xyz_to_point(0.0, 0.0, 0.0),
                        orientation=xyzw_to_quat(0.0, 0.0, 0.0, 1.0),
                    ),
                    scale=xyz_to_vec3(1.0, 0.5, 0.0),
                    color=color,
                    lifetime=Duration(seconds=10).to_msg(),
                    points=[xyz_to_point(*r_point[type]) for r_point in road["edges"]],
                )
                for (m_id, (r_id, road)) in enumerate(network_def.items())
            ]
        )

    def set_up_road_network_viz(self, beamng: BeamNGpy, time: Time):
        """
        Pauses the simulation, gets the road data from BeamNG, parses it into ROS2 markers
        and adds a request to delete the previously created road markers.

        Args:
            beamng: A connected BeamNGpy instance.
            time: Time to be used in the header of the marker messages.
        """

        beamng.control.pause()
        network_def = beamng.scenario.get_road_network()

        # Check if roads is a dictionary and not empty
        if isinstance(network_def, dict) and network_def:
            self._road_network_left = self._create_road_network(
                "left", ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.8), time, network_def
            )
            self._road_network_middle = self._create_road_network(
                "middle", ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.8), time, network_def
            )
            self._road_network_right = self._create_road_network(
                "right", ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.8), time, network_def
            )

        else:
            self._road_network_middle = MarkerArray()
        self._delete_old_markers()
        beamng.control.resume()

    def publish(self, beamng: BeamNGpy, time: Time) -> None:
        """
        Publishes the road network data.

        Args:
            beamng: A connected instance of BeamNGpy.
            time: The time to include in the message header.
        """

        if self._road_network_middle is None:
            self.set_up_road_network_viz(beamng, time)

        data = self.get_data()
        for x in data:
            self._publisher.publish(x)
