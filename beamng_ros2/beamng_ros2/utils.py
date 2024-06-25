from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

import numpy as np
from ament_index_python.packages import get_package_share_directory
from beamngpy.types import Float3, Quat
from geometry_msgs.msg import Point, Quaternion, Vector3
from rclpy.time import Duration, Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA

try:
    import rapidjson as json
except ImportError:
    # rapidjson not found, use normal json library
    import json


def float_to_duration(time: float) -> Duration:
    sec = int(time)
    nanosec = int((time - sec) * 1e9)
    return Duration(seconds=sec, nanoseconds=nanosec)


def float_to_time(start_time: Time, time: float) -> Time:
    return start_time + float_to_duration(time)


def dict_to_list(data: Dict) -> List:
    return [data[float(i)] for i in range(len(data))]


def xyzw_to_quat(x: float, y: float, z: float, w: float) -> Quaternion:
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def xyz_to_vec3(x: float, y: float, z: float) -> Vector3:
    return Vector3(x=float(x), y=float(y), z=float(z))


def xyz_to_point(x: float, y: float, z: float) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def xyz_to_list(data: Dict) -> List:
    return [data["x"], data["y"], data["z"]]


def rgba_to_color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


def beamng_rot_to_ros_coords(quat: Quat) -> Quat:
    """
    Transforms a quaternion representing rotation in the BeamNG vehicle space (x=left, y=backward, z=up)
    into ROS coordinate space (x=forward, y=left, z=up).
    """
    x = 0.7071067811865476  # sqrt(2)/2
    return (
        x * (quat[1] - quat[0]),
        -x * (quat[0] + quat[1]),
        -x * (quat[2] + quat[3]),
        x * (quat[3] - quat[2]),
    )


def beamng_vec_to_ros_coords(vec: Float3) -> Float3:
    """
    Transforms a vector in the BeamNG vehicle space (x=left, y=backward, z=up)
    into ROS coordinate space (x=forward, y=left, z=up).
    """
    return (-vec[1], vec[0], vec[2])


@staticmethod
def quat_from_dir(vec) -> np.ndarray:
    b = vec / np.linalg.norm(vec)
    a = np.array((1.0, 0.0, 0.0))  # forward vector of a vehicle in ROS space
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    if abs(s - 0.0) < 1e-5:  # identity rotation
        return np.array((0.0, 0.0, 0.0, 0.1))
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))
    return Rotation.from_matrix(rotation_matrix).as_quat(False)


def get_path(filename: str | Path) -> Path:
    pkg_path = Path(get_package_share_directory("beamng_ros2"))
    file_path = Path(filename).resolve()
    relative_fp = pkg_path / Path(filename).relative_to("/")
    if not file_path.is_file() and relative_fp.is_file():
        return relative_fp
    return file_path


def load_json(filename: str | Path) -> Dict[str, Any]:
    file_path = get_path(filename)
    with file_path.open("r") as fh:
        content = json.load(fh)
    return content
