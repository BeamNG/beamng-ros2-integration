#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "utils/sensor_registry.h"

namespace bng_ros2 {

struct CameraStreamSpec {
    std::string id;
    std::string encoding;
    std::string role;
};

struct BlobLayoutSpec {
    std::string format;
    uint32_t width{0};
    uint32_t height{0};
    uint32_t point_stride{0};
    uint32_t max_bytes{0};
    double publish_rate_hz{0.0};
    double inv_publish_rate_hz{0.0};
    std::string publish_mode;
    std::vector<CameraStreamSpec> camera_streams;
};

struct FieldSpec {
    std::string name;
};

struct SensorEntry {
    std::string id;
    std::string frame_id;
    LayoutId layout{LayoutId::Invalid};
    SensorCategory category{SensorCategory::Scalar};
    BlobLayoutSpec blob_layout;
    std::vector<FieldSpec> fields;
};

struct Config {
    std::vector<SensorEntry> sensors;
};

bool parse_config_from_json(const std::string& json, Config& out);

}  // namespace bng_ros2
