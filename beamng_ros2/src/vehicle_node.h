#pragma once
#include "definitions.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"


namespace bng_ros2 {
class VehicleNode : public rclcpp::Node {
 public:
  VehicleNode(const std::string &name);
  void broadcast_tf(double pos[3], double rot[4]);

 private:
  std::string name;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace bng_ros2

extern "C" {
  BNG_ROS2_API char bngros2_init_vehicle_node(const char *name);
  BNG_ROS2_API void bngros2_destroy_vehicle_node();
  BNG_ROS2_API char bngros2_spin_vehicle_node();
}

extern "C" {
  BNG_ROS2_API void bngros2_broadcast_tf(double pos[3], double rot[4]);
}