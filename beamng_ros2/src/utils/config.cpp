#include "utils/config.h"

#include "json.hpp"

#include <iostream>

namespace bng_ros2 {

namespace {

uint32_t default_max_bytes_for(LayoutId layout, const BlobLayoutSpec& blob_layout) {
    switch (layout) {
        case LayoutId::BlobImu:         return 80;
        case LayoutId::BlobNavSatFix:   return 24;
        case LayoutId::BlobIdealRadar:  return 4 * (4 + 15 * 8);
        case LayoutId::BlobRoads:       return 54 * 8;
        case LayoutId::BlobDamage:      return 65536;
        case LayoutId::BlobPowertrain:  return 65536;
        case LayoutId::BlobMesh:        return 64u * 1024u * 1024u;
        case LayoutId::GpuCamera:       return blob_layout.width * blob_layout.height * 4u;
        default:                        return 0;
    }
}

}  // namespace

bool parse_config_from_json(const std::string& json, Config& out) {
    out.sensors.clear();
    try {
        const auto root = nlohmann::json::parse(json);
        if (!root.contains("sensors") || !root["sensors"].is_array()) {
            std::cerr << "[ROS2] config: missing sensors array\n";
            return false;
        }

        for (const auto& sensor : root["sensors"]) {
            SensorEntry entry;
            entry.id = sensor.at("id").get<std::string>();
            entry.frame_id = sensor.value("frameId", std::string{});
            entry.layout = parse_layout_id(sensor.value("layoutId", std::string{}));
            entry.category = category_for_layout(entry.layout);

            if (sensor.contains("blobLayout") && sensor["blobLayout"].is_object()) {
                const auto& blob_layout = sensor["blobLayout"];
                entry.blob_layout.format = blob_layout.value("format", std::string{});
                entry.blob_layout.width = blob_layout.value("width", uint32_t{0});
                entry.blob_layout.height = blob_layout.value("height", uint32_t{0});
                entry.blob_layout.point_stride = blob_layout.value("pointStride", uint32_t{0});
                entry.blob_layout.max_bytes = blob_layout.value("max_bytes", blob_layout.value("maxBytes", uint32_t{0}));
                entry.blob_layout.publish_rate_hz = blob_layout.value("publishRateHz", blob_layout.value("publish_rate_hz", 0.0));

                if (entry.blob_layout.publish_rate_hz > 0.0) {
                    entry.blob_layout.inv_publish_rate_hz = 1.0 / entry.blob_layout.publish_rate_hz;
                } else {
                    entry.blob_layout.inv_publish_rate_hz = 0.0;
                }
                
                entry.blob_layout.publish_mode = blob_layout.value("publishMode", blob_layout.value("publish_mode", std::string{}));
                entry.blob_layout.range_min = blob_layout.value("rangeMin", 0.0);
                entry.blob_layout.range_max = blob_layout.value("rangeMax", 0.0);
                entry.blob_layout.field_of_view = blob_layout.value("fieldOfView", 0.0);

                if (blob_layout.contains("streams") && blob_layout["streams"].is_array()) {
                    for (const auto& stream : blob_layout["streams"]) {
                        CameraStreamSpec spec;
                        spec.id = stream.value("id", std::string{});
                        spec.encoding = stream.value("encoding", std::string{"rgb8"});
                        spec.role = stream.value("role", std::string{"colour"});
                        entry.blob_layout.camera_streams.push_back(std::move(spec));
                    }
                }
                if (entry.blob_layout.max_bytes == 0) {
                    entry.blob_layout.max_bytes = default_max_bytes_for(entry.layout, entry.blob_layout);
                }
            }

            if (sensor.contains("fields") && sensor["fields"].is_array()) {
                for (const auto& field : sensor["fields"]) {
                    FieldSpec field_spec;
                    if (field.is_string()) {
                        field_spec.name = field.get<std::string>();
                    } else if (field.is_object()) {
                        field_spec.name = field.value("name", std::string{});
                    }
                    if (!field_spec.name.empty()) entry.fields.push_back(std::move(field_spec));
                }
            }

            out.sensors.push_back(std::move(entry));
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[ROS2] config parse error: " << e.what() << "\n";
        out.sensors.clear();
        return false;
    }
}

}  // namespace bng_ros2
