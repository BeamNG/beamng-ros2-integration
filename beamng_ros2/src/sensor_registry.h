#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

namespace bng_ros2 {

struct SensorFieldConfig {
  std::string name;
  // Dot-separated path into the Lua global table (e.g. "electrics.values.rpm").
  // Used only for Scalar sensors.
  std::string path;
};

enum class DataMode {
  ScalarFrame,  // Scalar: flat double[] pushed each physics step
  Blob,         // Blob: raw binary buffer pushed each GFX step
};

struct BlobLayout {
  std::string format;          // "xyz_float32", "rgba_uint8", "depth_float32", "rgb_uint8"
  uint32_t    width{0};        // image width in pixels (images only)
  uint32_t    height{0};       // image height in pixels (images only)
  uint32_t    point_stride{0}; // bytes per point (point clouds only)
  uint32_t    max_bytes{0};    // pre-computed at load time; 0 = unknown/unbounded
};

struct SensorConfig {
  std::string id;
  std::string topic;
  std::string msg_file;
  std::string frame_id;  // ROS frame_id for the message header; empty = use node name
  DataMode    data_mode{DataMode::ScalarFrame};
  BlobLayout  blob_layout;
  std::vector<SensorFieldConfig> fields;
  // The "type" and "create" JSON keys are consumed exclusively by GE Lua for sensor creation and are intentionally not parsed here.
};

// Singleton populated once at startup via bngros2_receive_config.
class SensorRegistry {
  public:
    static SensorRegistry& instance();

    // Parse the JSON config string and populate the sensor list.
    // Expected format: { "sensors": [ { "id", "topic", "msgFile", "dataMode", "blobLayout": {...}, "fields": [...] } ] }
    bool load(const std::string& json);

    const std::vector<SensorConfig>& sensors() const { return sensors_; }
    const SensorConfig* find(const std::string& sensor_id) const;
    bool is_loaded() const { return loaded_; }

  private:
    SensorRegistry() = default;

    std::vector<SensorConfig> sensors_;
    std::unordered_map<std::string, size_t> index_;
    bool loaded_{false};
};

}  // namespace bng_ros2
