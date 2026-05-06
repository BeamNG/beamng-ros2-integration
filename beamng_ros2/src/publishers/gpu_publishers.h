#pragma once

#include "vehicle_node.h"

namespace bng_ros2 {

BlobPublishFn make_image(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor,
                         uint32_t width, uint32_t height, const std::string& encoding);
BlobPublishFn make_pointcloud2(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);
BlobPublishFn make_range(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor);

void build_gpu_streams(rclcpp::Node& node, GpuSensor& gpu, const SensorEntry& sensor);

}  // namespace bng_ros2
