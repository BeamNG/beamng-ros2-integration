#pragma once
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/color_rgba.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bng_ros2 {

inline geometry_msgs::msg::Quaternion quat_forward() {
  geometry_msgs::msg::Quaternion quat;
  quat.w = 1.0;
  return quat;
}

inline tf2::Vector3 to_vec3(double vec[3]) {
  return tf2::Vector3(vec[0], vec[1], vec[2]);
}

inline tf2::Quaternion to_quat(double rot[4]) {
  return tf2::Quaternion(rot[0], rot[1], rot[2], rot[3]);
}

inline std_msgs::msg::ColorRGBA to_rgba(double color[4]) {
  std_msgs::msg::ColorRGBA rgba;
  rgba.r = color[0];
  rgba.g = color[1];
  rgba.b = color[2];
  rgba.a = color[3];
  return rgba;
}

}  // namespace bng_ros2
