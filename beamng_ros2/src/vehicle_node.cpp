#include "vehicle_node.h"
#include "definitions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "math_helpers.h"
#include "rclcpp/rclcpp.hpp"

namespace bng_ros2 {
VehicleNode::VehicleNode(const std::string& name)
    : Node(name, "beamng_bridge"), name(name) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

void VehicleNode::broadcast_tf(double pos[3], double rot[4]) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = name.c_str();

  t.transform.translation = beamng_pos_to_ros_coords(pos);
  t.transform.rotation = beamng_rot_to_ros_coords(rot);

  tf_broadcaster_->sendTransform(t);
}
}  // namespace bng_ros2

static std::shared_ptr<bng_ros2::VehicleNode> node = nullptr;

BNG_ROS2_API char bngros2_init_vehicle_node(const char* name) {
  if (node) {
    return false;
  }
  try {
    node = std::make_shared<bng_ros2::VehicleNode>(name);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
    return false;
  }
  return true;
}

BNG_ROS2_API void bngros2_destroy_vehicle_node() {
  try {
    node.reset();
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API char bngros2_spin_vehicle_node() {
  if (!node) {
    return false;
  }
  try {
    rclcpp::spin_some(node);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
    return false;
  }
  return true;
}

BNG_ROS2_API void bngros2_broadcast_tf(double pos[3], double rot[4]) {
  if (!node) {
    return;
  }
  try {
    node->broadcast_tf(pos, rot);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}
