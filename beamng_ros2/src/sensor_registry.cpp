#include "sensor_registry.h"
#include "json.hpp"
#include <iostream>

namespace bng_ros2 {

SensorRegistry& SensorRegistry::instance() {
  static SensorRegistry inst;
  return inst;
}

// Compute the pre-allocated byte count from a blobLayout JSON object.
// Returns 0 if the layout is incomplete or unrecognised.
static uint32_t compute_max_bytes(const nlohmann::json& layout) {
  if (!layout.is_object()) return 0;

  // Image: width * height * bytes_per_pixel (3 for rgb8)
  if (layout.contains("width") && layout.contains("height")) {
    const uint32_t w = layout.at("width").get<uint32_t>();
    const uint32_t h = layout.at("height").get<uint32_t>();
    const std::string fmt = layout.value("format", "rgb8");
    if (fmt == "rgb8" || fmt == "rgb_uint8") return w * h * 3u;
    if (fmt == "rgba8") return w * h * 4u;
    if (fmt == "8UC1" || fmt == "mono8") return w * h * 1u;
    return w * h * 3u;
  }

  // Point cloud: maxPoints * pointStride
  if (layout.contains("maxPoints") && layout.contains("pointStride")) {
    const uint32_t pts    = layout.at("maxPoints").get<uint32_t>();
    const uint32_t stride = layout.at("pointStride").get<uint32_t>();
    return pts * stride;
  }

  // Mesh sensor: maxNodes * (10 f64 + u16 len prefix + avg partOrigin) + maxBeams * f64 + 2 u32 counts
  if (layout.contains("maxNodes") && layout.contains("maxBeams")) {
    const uint32_t nodes = layout.at("maxNodes").get<uint32_t>();
    const uint32_t beams = layout.at("maxBeams").get<uint32_t>();
    return nodes * (80 + 2 + 64) + beams * 8 + 8;
  }

  return 0;
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
      cfg.msg_file = s.at("msgFile").get<std::string>();
      cfg.frame_id = s.value("frameId", "");

      const std::string mode_str = s.value("dataMode", "scalar");
      cfg.data_mode = (mode_str == "blob") ? DataMode::Blob : DataMode::ScalarFrame;

      if (cfg.data_mode == DataMode::Blob) {
        if (s.contains("blobLayout")) {
          const auto& bl = s["blobLayout"];
          cfg.blob_layout.format       = bl.value("format",      "");
          cfg.blob_layout.width        = bl.value("width",        uint32_t{0});
          cfg.blob_layout.height       = bl.value("height",       uint32_t{0});
          cfg.blob_layout.point_stride = bl.value("pointStride",  uint32_t{0});
          cfg.blob_layout.max_bytes    = compute_max_bytes(bl);

          if (cfg.blob_layout.max_bytes == 0) {
            std::cerr << "[ROS2 WARN] SensorRegistry: blob sensor '" << cfg.id
                      << "' blobLayout could not determine max_bytes "
                          "(check format, maxPoints/pointStride or width/height).\n";
          }
        } else {
          // Fixed-size blob message types: pre-allocate without requiring blobLayout in JSON.
          const std::string& mf = cfg.msg_file;
          if (mf == "Range") {
            cfg.blob_layout.max_bytes = 17u;  // radiation_type(1) + 4*float(16)
          }
          // Image: require blobLayout with width/height in JSON for camera
          if (cfg.blob_layout.max_bytes == 0) {
            std::cerr << "[ROS2 WARN] SensorRegistry: blob sensor '" << cfg.id
                      << "' has no 'blobLayout' - buffer will not be pre-allocated.\n";
          }
        }
      }

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
