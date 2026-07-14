#include "publishers/gpu_publishers.h"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <cstring>
#include <string>
#include <stdexcept>

namespace bng_ros2 {

namespace {

std::string frame_or_node(rclcpp::Node& node, const std::string& frame) {
    return frame.empty() ? std::string(node.get_name()) : frame;
}

uint32_t image_step_bytes(const std::string& encoding, uint32_t width) {
    if (encoding == "rgb8" || encoding == "bgr8") return width * 3u;
    if (encoding == "32FC1" || encoding == "32SC1" || encoding == "rgba8") return width * 4u;
    if (encoding == "mono8" || encoding == "8UC1") return width;
    return width * 3u;
}

}

BlobPublishFn make_image(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor, uint32_t width, uint32_t height, const std::string& encoding) {
    using sensor_msgs::msg::Image;
    auto pub = node.create_publisher<Image>(topic, 10);
    Image camera_template;
    camera_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    camera_template.width = width;
    camera_template.height = height;
    camera_template.encoding = encoding;
    camera_template.is_bigendian = 0;
    camera_template.step = image_step_bytes(encoding, width);
    camera_template.data.resize(static_cast<std::size_t>(camera_template.step) * height);

    return [pub, msg = std::move(camera_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        if (len == 0) return;
        msg.header.stamp = stamp;
        if (msg.data.size() != len) msg.data.resize(len);
        std::memcpy(msg.data.data(), data, len);

        pub->publish(msg);
    };
}

/// PointCloud2 from raw shmem (xyz_float32 when `blob_layout.format` is empty or `xyz_float32`).
BlobPublishFn make_pointcloud2(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    auto pub = node.create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
    sensor_msgs::msg::PointCloud2 pointcloud_template;
    pointcloud_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    pointcloud_template.height = 1;
    pointcloud_template.is_bigendian = false;
    pointcloud_template.is_dense = false;

    const uint32_t stride = sensor.blob_layout.point_stride > 0 ? sensor.blob_layout.point_stride : 12u;
    pointcloud_template.point_step = stride;

    if (sensor.blob_layout.format.empty() || sensor.blob_layout.format == "xyz_float32") {
        sensor_msgs::msg::PointField fx, fy, fz;
        fx.name = "x"; fx.offset = 0; fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
        fy.name = "y"; fy.offset = 4; fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
        fz.name = "z"; fz.offset = 8; fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
        pointcloud_template.fields = {fx, fy, fz};
    }

    return [pub, msg = std::move(pointcloud_template), stride](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        msg.header.stamp = stamp;
        msg.width = (stride > 0) ? (len / stride) : 0;
        msg.row_step = len;
        if (msg.data.size() != len) msg.data.resize(len);
        if (len > 0) std::memcpy(msg.data.data(), data, len);

        pub->publish(msg);
    };
}

/// Ultrasonic shmem holds a single raw f32 (the measured distance), range limits/FOV are static from config.
BlobPublishFn make_range(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using sensor_msgs::msg::Range;
    auto pub = node.create_publisher<Range>(topic, 10);
    Range range_template;
    range_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    range_template.radiation_type = Range::ULTRASOUND;
    range_template.field_of_view = static_cast<float>(sensor.blob_layout.field_of_view);
    range_template.min_range = static_cast<float>(sensor.blob_layout.range_min);
    range_template.max_range = static_cast<float>(sensor.blob_layout.range_max);

    return [pub, msg = std::move(range_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        if (len < sizeof(float)) return;
        float distance;
        std::memcpy(&distance, data, sizeof(float));
        msg.header.stamp = stamp;
        msg.range = distance;

        pub->publish(msg);
    };
}


void build_gpu_streams(rclcpp::Node& node, GpuSensor& gpu, const SensorEntry& sensor) {
    const uint32_t img_width = sensor.blob_layout.width  > 0 ? sensor.blob_layout.width  : 512u;
    const uint32_t img_height = sensor.blob_layout.height > 0 ? sensor.blob_layout.height : 512u;
    const std::string topic = std::string("~/") + sensor.id;

    if (sensor.layout == LayoutId::GpuCamera) {
        const auto& specs_in = sensor.blob_layout.camera_streams;
        const bool plain_topic = specs_in.empty() ? true : (specs_in.size() <= 1);

        auto append_camera_stream = [&](const CameraStreamSpec& spec) {
            GpuStream stream;
            stream.width = img_width;
            stream.height = img_height;
            stream.role = spec.role.empty() ? std::string("colour") : spec.role;
            stream.encoding = spec.encoding.empty() ? std::string("rgb8") : spec.encoding;
            stream.needs_rgb_conversion = (stream.role == "colour" || stream.role == "annotation");
            stream.byte_size = img_width * img_height * 4u;

            const std::string stream_topic = plain_topic ? topic : (topic + "_" + spec.id);
            stream.publish_fn = make_image(node, stream_topic, sensor, img_width, img_height, stream.encoding);
            gpu.streams.push_back(std::move(stream));
        };

        if (specs_in.empty()) {
            CameraStreamSpec stream_spec;
            stream_spec.id = "rgb";
            append_camera_stream(stream_spec);
        } else {
            for (const auto& spec : specs_in) {
                append_camera_stream(spec);
            }
        }
        return;
    }

    if (sensor.layout == LayoutId::GpuLidar) {
        const uint32_t pointcloud_bytes = sensor.blob_layout.max_bytes > 0
            ? sensor.blob_layout.max_bytes : 64u * 1024u * 1024u;
        GpuStream points;
        points.byte_size = pointcloud_bytes;
        points.publish_fn = make_pointcloud2(node, topic, sensor);
        gpu.streams.push_back(std::move(points));
        return;
    }

    else if (sensor.layout == LayoutId::GpuRadar) {
        GpuStream stream;
        stream.byte_size = sensor.blob_layout.max_bytes > 0
            ? sensor.blob_layout.max_bytes : 4u * 1024u * 1024u;
        stream.publish_fn = make_pointcloud2(node, topic, sensor);
        gpu.streams.push_back(std::move(stream));
        return;
    }

    else if (sensor.layout == LayoutId::GpuUltrasonic) {
        GpuStream stream;
        stream.byte_size = sizeof(float);
        stream.publish_fn = make_range(node, topic, sensor);
        gpu.streams.push_back(std::move(stream));
        return;
    }

    else {
        throw std::runtime_error("Unsupported GPU sensor layout");
    }
}

}  // namespace bng_ros2
