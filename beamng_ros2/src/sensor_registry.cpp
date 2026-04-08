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

  // maxPoints-driven layouts first so Radar/LiDAR are not mis-sized as images when
  if (layout.contains("maxPoints")) {
    uint32_t stride = layout.value("pointStride", uint32_t{0});

    if (stride == 0) {
      const std::string format = layout.value("format", "");
      if (format == "radar_returns_f32") stride = 28u;
      else if (format == "xyz_float32") stride = 12u;
    }

    if (stride > 0) {
      return layout.at("maxPoints").get<uint32_t>() * stride;
    }
  }

  // Image: width * height * bytes_per_pixel (3 for rgb8)
  if (layout.contains("width") && layout.contains("height")) {
    const uint32_t width = layout.at("width").get<uint32_t>();
    const uint32_t height = layout.at("height").get<uint32_t>();

    const std::string format = layout.value("format", "rgb8");

    if (format == "rgb8" || format == "rgb_uint8") return width * height * 3u;
    if (format == "rgba8")                         return width * height * 4u;
    if (format == "8UC1" || format == "mono8")     return width * height * 1u;

    return width * height * 3u;
  }

  // Point cloud: maxPoints * pointStride (both explicit)
  if (layout.contains("maxPoints") && layout.contains("pointStride")) {
    const uint32_t points = layout.at("maxPoints").get<uint32_t>();
    const uint32_t stride = layout.at("pointStride").get<uint32_t>();
    return points * stride;
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

    for (const auto& sensor : root["sensors"]) {
      SensorConfig sensor_config;
      sensor_config.id       = sensor.at("id").get<std::string>();
      sensor_config.msg_file = sensor.at("msgFile").get<std::string>();
      sensor_config.frame_id = sensor.value("frameId", "");

      const std::string mode_str = sensor.value("dataMode", "scalar");
      sensor_config.data_mode = (mode_str == "blob") ? DataMode::Blob : DataMode::ScalarFrame;

      if (sensor_config.data_mode == DataMode::Blob) {
        if (sensor.contains("blobLayout")) {
          const auto& blob_layout = sensor["blobLayout"];
          sensor_config.blob_layout.format       = blob_layout.value("format",      "");
          sensor_config.blob_layout.width        = blob_layout.value("width",        uint32_t{0});
          sensor_config.blob_layout.height       = blob_layout.value("height",       uint32_t{0});
          sensor_config.blob_layout.point_stride = blob_layout.value("pointStride",  uint32_t{0});
          sensor_config.blob_layout.publish_mode = blob_layout.value("publishMode", std::string{});
          sensor_config.blob_layout.publish_rate_hz = blob_layout.value("publishRateHz", 0.0f);

          if (blob_layout.contains("streams") && blob_layout["streams"].is_array()) {
            for (const auto& stream : blob_layout["streams"]) {
              ImageStreamSpec spec;
              spec.id       = stream.value("id", std::string{});
              spec.encoding = stream.value("encoding", std::string{"rgb8"});
              spec.role     = stream.value("role", std::string{});
              if (!spec.id.empty()) {
                sensor_config.blob_layout.image_streams.push_back(std::move(spec));
              }
            }
          }

          sensor_config.blob_layout.max_bytes = compute_max_bytes(blob_layout);

          if (sensor_config.blob_layout.max_bytes == 0) {
            std::cerr << "[ROS2 WARN] SensorRegistry: blob sensor '" << sensor_config.id
                      << "' blobLayout could not determine max_bytes "
                          "(check format, maxPoints/pointStride or width/height).\n";
          }
        } else {
          // Fixed-size blob message types: pre-allocate without requiring blobLayout in JSON.
          const std::string& msg_file = sensor_config.msg_file;
          if (msg_file == "Range") {
            sensor_config.blob_layout.max_bytes = 17u;  // radiation_type(1) + 4*float(16)
          }
          // Image: require blobLayout with width/height in JSON for camera
          if (sensor_config.blob_layout.max_bytes == 0) {
            std::cerr << "[ROS2 WARN] SensorRegistry: blob sensor '" << sensor_config.id
                      << "' has no 'blobLayout' - buffer will not be pre-allocated.\n";
          }
        }
      }

      if (sensor.contains("fields") && sensor["fields"].is_array()) {
        for (const auto& field : sensor["fields"]) {
          sensor_config.fields.push_back({
            field.at("name").get<std::string>(),
            field.at("path").get<std::string>(),
          });
        }
      }

      index_[sensor_config.id] = sensors_.size();
      sensors_.push_back(std::move(sensor_config));
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
