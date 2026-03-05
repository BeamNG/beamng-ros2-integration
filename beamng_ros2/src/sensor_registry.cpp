#include "sensor_registry.h"
#include "json.hpp"
#include <iostream>

namespace bng_ros2 {

SensorRegistry& SensorRegistry::instance() {
  static SensorRegistry inst;
  return inst;
}

bool SensorRegistry::load(const std::string& json_str) {
  try {
    auto root = nlohmann::json::parse(json_str);

    if (!root.contains("sensors") || !root["sensors"].is_array()) {
      std::cerr << "[ROS2 ERROR] SensorRegistry: JSON missing 'sensors' array.\n";
      return false;
    }

    sensors_.clear();
    index_.clear();

    for (const auto& s : root["sensors"]) {
      SensorConfig cfg;
      cfg.id       = s.at("id").get<std::string>();
      cfg.topic    = s.at("topic").get<std::string>();
      cfg.msg_file = s.at("msgFile").get<std::string>();

      if (s.contains("fields") && s["fields"].is_array()) {
        for (const auto& f : s["fields"]) {
          cfg.fields.push_back({
            f.at("name").get<std::string>(),
            f.at("path").get<std::string>(),
          });
        }
      }

      index_[cfg.id] = sensors_.size();
      sensors_.push_back(std::move(cfg));
    }

    loaded_ = true;
    std::cout << "[ROS2 INFO] SensorRegistry: loaded " << sensors_.size() << " sensor(s).\n";
    return true;

  } catch (const nlohmann::json::exception& e) {
    std::cerr << "[ROS2 ERROR] SensorRegistry::load: " << e.what() << "\n";
    return false;
  } catch (const std::exception& e) {
    std::cerr << "[ROS2 ERROR] SensorRegistry::load: " << e.what() << "\n";
    return false;
  }
}

const SensorConfig* SensorRegistry::find(const std::string& sensor_id) const {
  auto it = index_.find(sensor_id);
  if (it == index_.end()) return nullptr;
  return &sensors_[it->second];
}

}  // namespace bng_ros2
