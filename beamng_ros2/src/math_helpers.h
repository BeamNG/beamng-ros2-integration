#pragma once
#include <std_msgs/msg/color_rgba.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bng_ros2 {

inline tf2::Vector3 to_vec3(double vec[3]) {
  return tf2::Vector3(vec[0], vec[1], vec[2]);
}

inline tf2::Quaternion to_quat(double rot[4]) {
  return tf2::Quaternion(rot[0], rot[1], rot[2], rot[3]);
}

inline std_msgs::msg::ColorRGBA to_rgba(double color[4]) {
  std_msgs::msg::ColorRGBA rgba;
  rgba.r = static_cast<float>(color[0]);
  rgba.g = static_cast<float>(color[1]);
  rgba.b = static_cast<float>(color[2]);
  rgba.a = static_cast<float>(color[3]);
  return rgba;
}

}  // namespace bng_ros2
