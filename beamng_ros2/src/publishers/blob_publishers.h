#pragma once

#include "vehicle_node.h"

namespace bng_ros2 {

BlobPublishFn make_damage(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);
BlobPublishFn make_imu(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);
BlobPublishFn make_ideal_radar(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);
BlobPublishFn make_roads(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);
BlobPublishFn make_powertrain(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);
BlobPublishFn make_mesh(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);

BlobPublishFn build_blob_publisher(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);

}  // namespace bng_ros2
