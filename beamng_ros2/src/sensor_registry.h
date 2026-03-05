#pragma once
#include <string>
#include <vector>
#include <unordered_map>

namespace bng_ros2 {

struct SensorFieldConfig {
  std::string name;
  // Dot-separated path into the Lua global table (e.g. "electrics.values.rpm")
  std::string path;
};

struct SensorConfig {
  std::string id;
  std::string topic;
  std::string msg_file;
  std::vector<SensorFieldConfig> fields;
};

// Singleton populated once at startup via bngros2_receive_config.
class SensorRegistry {
  public:
    static SensorRegistry& instance();

    // Parse the JSON config string and populate the sensor list.
    // Expected format: { "sensors": [ { "id", "topic", "msgFile", "fields": [...] } ] }
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
