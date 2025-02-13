#pragma once
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.h"

namespace bng_ros2 {

geometry_msgs::msg::Vector3 beamng_pos_to_ros_coords(double pos[3]) {
    geometry_msgs::msg::Vector3 translation;
    translation.x = -pos[1];
    translation.y = pos[0];
    translation.z = pos[2];
    return translation;
}

geometry_msgs::msg::Quaternion beamng_rot_to_ros_coords(double rot[4]) {
    double x = 0.7071067811865476; // sqrt(2)/2

    geometry_msgs::msg::Quaternion quat;
    quat.x = x * (rot[1] - rot[0]);
    quat.y = -x * (rot[0] + rot[1]);
    quat.z = -x * (rot[2] + rot[3]);
    quat.w =  x * (rot[3] - rot[2]);
    return quat;
}

}