#pragma once
#include <functional>
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include "sensor_registry.h"

namespace bng_ros2 {

// Numeric and string value stores, updated each physics step and read at publish time.
// Each sensor gets its own maps, which are created once and used by reference in PublishFn.
using ValueMap  = std::unordered_map<std::string, double>;
using StringMap = std::unordered_map<std::string, std::string>;

// Zero-argument callable returned by a RegisterFn. Reads the current values and publishes one typed ROS2 message.
using PublishFn = std::function<void()>;

// Called once per sensor at VehicleNode construction time. Creates the ROS2 publisher, resolves field offsets from introspection, and returns a PublishFn.
using RegisterFn = std::function<PublishFn(rclcpp::Node&,
                                            const std::string& topic,
                                            const ValueMap&,
                                            const StringMap&,
                                            const SensorConfig&)>;

class SensorFactoryRegistry {
  public:
    static SensorFactoryRegistry& instance();

    void register_factory(const std::string& msg_file, RegisterFn fn);
    const RegisterFn* find(const std::string& msg_file) const;

  private:
    SensorFactoryRegistry() = default;
    std::unordered_map<std::string, RegisterFn> factories_;
};

// Populate SensorFactoryRegistry with all supported message types.
// Called once at DLL load time from init.cpp.
void register_all_sensor_factories();

}  // namespace bng_ros2
