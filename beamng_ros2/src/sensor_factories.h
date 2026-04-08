#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "sensor_registry.h"

namespace bng_ros2 {

// Flat value stores - one instance per VehicleNode, allocated once at init.
// Indexed by integer position, so publish-time access is a direct array read with no string hashing.
using ScalarStore = std::vector<double>;
using StringStore = std::vector<std::string>;

// Scalar sensor publish function.
// Called by the publish thread with the stable read-slot of the SPSC buffer.
// stamp: one get_clock()->now() per batch (all messages in this drain share it).
using PublishFn = std::function<void(const ScalarStore&, const StringStore&, const rclcpp::Time& stamp)>;

// Blob sensor publish function.
// Called by the publish thread with a pointer into the stable read-slot and the number of valid bytes.
using BlobPublishFn = std::function<void(const uint8_t* data, uint32_t len, uint64_t timestamp_ns)>;

// RegisterFn creates a ROS2 publisher and returns a ScalarRegisterResult containing the PublishFn
// and the exact number of scalar/string slots consumed from ScalarStore/StringStore.
//
// BlobRegisterFn creates a ROS2 publisher and returns a BlobPublishFn.
struct ScalarRegisterResult {
  PublishFn  publish_fn;
  uint32_t   scalar_slots_used{0};
  uint32_t   string_slots_used{0};
};

using RegisterFn = std::function<ScalarRegisterResult(
    rclcpp::Node&,
    const std::string& topic,
    const SensorConfig&,
    uint32_t scalar_base_idx,
    uint32_t string_base_idx)>;

using BlobRegisterFn = std::function<BlobPublishFn(rclcpp::Node&, const std::string& topic, const SensorConfig&)>;

class SensorFactoryRegistry {
  public:
    static SensorFactoryRegistry& instance();

    void register_factory     (const std::string& msg_file, RegisterFn     fn);
    void register_blob_factory(const std::string& msg_file, BlobRegisterFn fn);

    const RegisterFn*     find     (const std::string& msg_file) const;
    const BlobRegisterFn* find_blob(const std::string& msg_file) const;

  private:
    SensorFactoryRegistry() = default;
    std::unordered_map<std::string, RegisterFn> factories_;
    std::unordered_map<std::string, BlobRegisterFn> blob_factories_;
};

// Populate SensorFactoryRegistry with all supported message types.
// Called once at DLL load time from init.cpp.
void register_all_sensor_factories();

// sensor_msgs/Image publisher factory; VehicleNode calls this per camera stream (not via blob registry).
BlobPublishFn make_camera_sensor_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config,
    const std::string& encoding,
    uint32_t width,
    uint32_t height);

}  // namespace bng_ros2
