#include "beamng_node.h"
#include "sensor_registry.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

namespace bng_ros2 {
BeamNGNode::BeamNGNode() : Node("beamng_bridge") {}
}  // namespace bng_ros2

static std::shared_ptr<bng_ros2::BeamNGNode> node = nullptr;

BNG_ROS2_API char bngros2_init_beamng_node() {
  if (node) return false;
  try {
    node = std::make_shared<bng_ros2::BeamNGNode>();
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
    return false;
  }
  return true;
}

BNG_ROS2_API void bngros2_destroy_beamng_node() {
  try {
    node.reset();
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API char bngros2_spin_beamng_node() {
  if (!node) return false;
  
  try {
    rclcpp::spin_some(node);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
    return false;
  }
  return true;
}

BNG_ROS2_API void bngros2_receive_config(const char* json) {
  if (!json) {
    std::cerr << "[ROS2 ERROR] bngros2_receive_config: null config.\n";
    return;
  }
  bng_ros2::SensorRegistry::instance().load(json);
}
