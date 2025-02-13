#pragma once
#include "definitions.h"
#include "rclcpp/rclcpp.hpp"

namespace bng_ros2 {
class BeamNGNode : public rclcpp::Node {
 public:
  BeamNGNode();
};
}  // namespace bng_ros2

extern "C" {
  BNG_ROS2_API char bngros2_init_beamng_node();
  BNG_ROS2_API void bngros2_destroy_beamng_node();
  BNG_ROS2_API char bngros2_spin_beamng_node();
}