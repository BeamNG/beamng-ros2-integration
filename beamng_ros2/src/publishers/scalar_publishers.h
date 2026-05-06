#pragma once

#include "vehicle_node.h"

namespace bng_ros2 {

ScalarPublisherBuild build_scalar_publisher(rclcpp::Node& node, const std::string& topic,
                                            const SensorEntry& sensor,
                                            uint32_t scalar_base, uint32_t string_base);

}  // namespace bng_ros2
