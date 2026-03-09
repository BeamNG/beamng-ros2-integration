#pragma once
#include <cstdint>
#include "definitions.h"
#include "rclcpp/rclcpp.hpp"

// C-compatible struct shared between the Lua FFI (ffi.cdef) and vehicle_node.cpp.
// Defined outside any C++ namespace to avoid name-mangling issues.
struct BngBlobDesc {
  uint32_t       sensor_idx;   // slot index returned by bngros2_register_blob_sensors
  const uint8_t* data;         // pointer into the engine buffer (valid for the duration of the call)
  uint32_t       byte_len;
  uint64_t       timestamp_ns;
};

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
  BNG_ROS2_API void bngros2_receive_config(const char* json);
}
