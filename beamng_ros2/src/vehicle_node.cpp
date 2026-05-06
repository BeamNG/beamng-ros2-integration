#include "vehicle_node.h"

#include "publishers/blob_publishers.h"
#include "publishers/gpu_publishers.h"
#include "publishers/scalar_publishers.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>

#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>

namespace bng_ros2 {

namespace {

// Windows (and some libc setups) quantize sleep_for(<~1ms) to ~1–2ms, so a 500µs sleep caps the loop near 500–1000Hz.
// Pace with a short spin + PAUSE so scalar/TF work can actually reach ~2000Hz without changing the rest of the design.
inline void spin_until(std::chrono::steady_clock::time_point deadline) {
    using clock = std::chrono::steady_clock;
    while (clock::now() < deadline) {
#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
        _mm_pause();
#else
        std::this_thread::yield();
#endif
    }
}

}  // namespace

VehicleNode::VehicleNode(const std::string& name)
    : Node(name), name_(name) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 10);
}

VehicleNode::~VehicleNode() {
    running_.store(false, std::memory_order_release);
    if (publish_thread_.joinable()) publish_thread_.join();
}

int VehicleNode::apply_config(const std::string& json) {
    // Stop the existing publish thread (if any) before clearing state.
    running_.store(false, std::memory_order_release);
    if (publish_thread_.joinable()) publish_thread_.join();

    scalar_publishers_.clear();
    blob_slots_.clear();
    gpu_sensors_.clear();
    string_field_index_.clear();

    Config config;
    if (!parse_config_from_json(json, config)) {
        return -1;
    }

    uint32_t scalar_cursor = 0;
    uint32_t string_cursor = 0;

    for (const auto& sensor : config.sensors) {
        const std::string topic = std::string("~/") + sensor.id;
        switch (sensor.category) {
            case SensorCategory::Scalar: {
                auto built = build_scalar_publisher(*this, topic, sensor, scalar_cursor, string_cursor);
                if (!built.publish_fn) break;
                scalar_publishers_.push_back(std::move(built.publish_fn));
                for (const auto& binding : built.string_bindings) {
                    string_field_index_[binding.first] = binding.second;
                }
                scalar_cursor += built.scalar_slots_used;
                string_cursor += built.string_slots_used;
                break;
            }
            case SensorCategory::Blob: {
                BlobSlot slot;
                slot.publish_fn = build_blob_publisher(*this, topic, sensor);
                if (!slot.publish_fn) break;
                const uint32_t cap = sensor.blob_layout.max_bytes > 0 ? sensor.blob_layout.max_bytes : 65536u;
                slot.buffer = std::make_unique<BlobBuffer>(cap);
                if (sensor.blob_layout.inv_publish_rate_hz > 0.0) {
                    const double period = sensor.blob_layout.inv_publish_rate_hz;
                    slot.min_publish_interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(period));
                }
                // Mesh payloads are enormous: unthrottled publish (0 ns interval) runs every loop tick and blows RAM/CPU.
                if (sensor.layout == LayoutId::BlobMesh && slot.min_publish_interval.count() == 0) {
                    slot.min_publish_interval = std::chrono::milliseconds(50);
                }
                blob_slots_.push_back(std::move(slot));
                break;
            }
            case SensorCategory::Gpu: {
                GpuSensor gpu;
                build_gpu_streams(*this, gpu, sensor);
                if (sensor.blob_layout.inv_publish_rate_hz > 0.0) {
                    const double period = sensor.blob_layout.inv_publish_rate_hz;
                    gpu.min_publish_interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(period));
                }
                gpu_sensors_.push_back(std::move(gpu));
                break;
            }
        }
    }

    scalar_buf_.resize_both(scalar_cursor, 0.0);
    {
        std::lock_guard<std::mutex> lk(string_mutex_);
        string_store_.assign(string_cursor, std::string{});
    }
    has_string_fields_ = string_cursor > 0;
    string_store_version_.store(0, std::memory_order_relaxed);

    running_.store(true, std::memory_order_release);
    publish_thread_ = std::thread([this]() { publish_loop(); });
    return static_cast<int>(blob_slots_.size());
}

int VehicleNode::get_blob_slot(uint32_t slot, BngBlobSlotFfiView* out) {
    if (slot >= blob_slots_.size() || !out) return -1;
    BlobBuffer& b = *blob_slots_[slot].buffer;
    out->buf0 = b.data_a();
    out->buf1 = b.data_b();
    out->state_atomic = static_cast<void*>(b.state_atomic());
    out->capacity_bytes = static_cast<uint32_t>(b.capacity());
    return 0;
}

uint32_t VehicleNode::blob_state_acquire(uint32_t slot) {
    if (slot >= blob_slots_.size()) return 0;
    return blob_slots_[slot].buffer->state_acquire();
}

void VehicleNode::blob_commit_side(uint32_t slot, uint32_t side, uint32_t length_bytes) {
    if (slot >= blob_slots_.size()) return;
    BlobBuffer& b = *blob_slots_[slot].buffer;
    if (length_bytes > static_cast<uint32_t>(b.capacity())) return;
    if (side == 0u) b.commit_write_a(length_bytes);
    else if (side == 1u) b.commit_write_b(length_bytes);
}

void VehicleNode::bind_gpu_sensor(uint32_t gpu_slot, const BngGpuStreamDesc* descs, uint32_t count) {
    if (gpu_slot >= gpu_sensors_.size() || !descs) {
        return;
    }
    GpuSensor& gpu = gpu_sensors_[gpu_slot];
    for (uint32_t i = 0; i < count; ++i) {
        const BngGpuStreamDesc& stream_desc = descs[i];
        if (stream_desc.stream_index >= gpu.streams.size()) continue;
        GpuStream& stream = gpu.streams[stream_desc.stream_index];
        if (stream_desc.width > 0)  stream.width = stream_desc.width;
        if (stream_desc.height > 0) stream.height = stream_desc.height;
        if (stream_desc.size_bytes > 0) stream.byte_size = stream_desc.size_bytes;
        else if (stream.width > 0 && stream.height > 0) stream.byte_size = stream.width * stream.height * 4u;
        if (stream_desc.role)     stream.role = stream_desc.role;
        stream.needs_rgb_conversion = (stream.role == "colour" || stream.role == "annotation");
        if (stream_desc.encoding) stream.encoding = stream_desc.encoding;
        if (!stream_desc.shared_memory_name) continue;
        stream.shared_memory_name = stream_desc.shared_memory_name;
        if (!stream.reader.open(stream.shared_memory_name, stream.byte_size)) {
            // Engine may create the shmem asynchronously, publish_loop will retry.
            std::cerr << "[ROS2] bind_gpu_sensor: shmem open failed (will retry): " << stream.shared_memory_name << "\n";
        }
        if (stream.needs_rgb_conversion) {
            stream.convert_buf.resize(static_cast<std::size_t>(stream.width) * stream.height * 3u);
        }
    }
}

void VehicleNode::unbind_gpu_sensor(uint32_t gpu_slot) {
    if (gpu_slot >= gpu_sensors_.size()) return;
    for (auto& stream : gpu_sensors_[gpu_slot].streams) {
        stream.reader.close();
        stream.shared_memory_name.clear();
    }
}

void VehicleNode::push_scalar_frame(const double* values, uint32_t count) {
    auto& slot = scalar_buf_.write_slot();
    const uint32_t n = std::min<uint32_t>(count, static_cast<uint32_t>(slot.size()));
    if (n == 0) return;
    std::memcpy(slot.data(), values, static_cast<std::size_t>(n) * sizeof(double));
    scalar_buf_.commit();
}

void VehicleNode::set_scalar_string(const char* sensor_id, const char* field_name, const char* value) {
    thread_local std::string key;
    key.clear();
    key.reserve(std::strlen(sensor_id) + std::strlen(field_name) + 2);
    key.append(sensor_id).append("::").append(field_name);
    auto it = string_field_index_.find(key);
    if (it == string_field_index_.end()) return;
    std::lock_guard<std::mutex> lk(string_mutex_);
    if (it->second < string_store_.size()) {
        const std::string new_value = value ? value : "";
        if (string_store_[it->second] != new_value) {
            string_store_[it->second] = new_value;
            string_store_version_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void VehicleNode::broadcast_tf(const double pos[3], const double rot[4]) {
    {
        std::lock_guard<std::mutex> lk(tf_state_.mutex);
        std::memcpy(tf_state_.pos, pos, sizeof(tf_state_.pos));
        std::memcpy(tf_state_.rot, rot, sizeof(tf_state_.rot));
    }
    tf_state_.dirty.store(true, std::memory_order_release);
}

void VehicleNode::set_vehicle_data(const double scale[3], const double color[4], const double arrow_color[4]) {
    {
        std::lock_guard<std::mutex> lk(marker_state_.mutex);
        std::memcpy(marker_state_.scale, scale, sizeof(marker_state_.scale));
        std::memcpy(marker_state_.color, color, sizeof(marker_state_.color));
        std::memcpy(marker_state_.arrow_color, arrow_color, sizeof(marker_state_.arrow_color));
    }
    marker_state_.dirty.store(true, std::memory_order_release);
}

void VehicleNode::publish_loop() {
    using namespace std::chrono;
    constexpr auto k_loop_period = microseconds(500);
    constexpr auto k_marker_period = milliseconds(100);

    auto last_marker = steady_clock::now();
    visualization_msgs::msg::MarkerArray marker_msg;
    bool marker_built = false;
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = name_;
    StringValues strings_copy;
    uint64_t last_string_version = 0;
    bool has_string_snapshot = false;

    while (running_.load(std::memory_order_acquire)) {
        const auto tick = steady_clock::now();
        if (!running_.load(std::memory_order_acquire)) break;

        const auto stamp_now = get_clock()->now();
        const auto steady_now = tick;

        // Publish scalar and string-based sensors (2kHz)
        if (!scalar_publishers_.empty()) {
            scalar_buf_.clear_dirty();
            const auto& scalars = scalar_buf_.read_slot();
            if (has_string_fields_) {
                const uint64_t version = string_store_version_.load(std::memory_order_relaxed);
                if (!has_string_snapshot || version != last_string_version) {
                    std::lock_guard<std::mutex> lk(string_mutex_);
                    strings_copy = string_store_;
                    last_string_version = string_store_version_.load(std::memory_order_relaxed);
                    has_string_snapshot = true;
                }
            }
            for (auto& fn : scalar_publishers_) fn(scalars, strings_copy, stamp_now);
        }

        // Publish non-GPU blob-based sensors
        for (auto& slot : blob_slots_) {
            if (!slot.buffer || !slot.publish_fn) continue;
            if (slot.min_publish_interval.count() > 0 &&
                steady_now - slot.last_publish < slot.min_publish_interval) continue;
            BlobBuffer& b = *slot.buffer;
            const uint32_t state = b.state_acquire();
            const bool ad = (state & BlobBuffer::k_dirty_a) != 0u;
            const bool bd = (state & BlobBuffer::k_dirty_b) != 0u;
            if (!ad && !bd) continue;
            if (ad) {
                slot.publish_fn(b.data_a(), b.payload_len_a(), stamp_now);
                b.clear_a_dirty();
            } else {
                slot.publish_fn(b.data_b(), b.payload_len_b(), stamp_now);
                b.clear_b_dirty();
            }
            slot.last_publish = steady_now;
        }

        // Publish GPU-based sensors
        for (auto& gpu : gpu_sensors_) {
            if (gpu.min_publish_interval.count() > 0 &&
                steady_now - gpu.last_publish < gpu.min_publish_interval) continue;
            bool any = false;
            for (auto& stream : gpu.streams) {
                if (!stream.publish_fn) continue;
                // Retry opening the shmem on each publish tick until it succeeds.
                // The engine may create the segment asynchronously after sensor creation.
                if (!stream.reader.is_open() && !stream.shared_memory_name.empty()) {
                    stream.reader.open(stream.shared_memory_name, stream.byte_size);
                }
                if (!stream.reader.is_open()) continue;
                const uint8_t* shm = stream.reader.data();
                if (stream.needs_rgb_conversion) {
                    const std::size_t pixels = static_cast<std::size_t>(stream.width) * stream.height;
                    const uint8_t* src = shm;
                    uint8_t* dst = stream.convert_buf.data();
                    for (std::size_t p = 0; p < pixels; ++p) {
                        dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
                        src += 4; dst += 3;
                    }
                    stream.publish_fn(stream.convert_buf.data(), static_cast<uint32_t>(pixels * 3u), stamp_now);
                } else {
                    stream.publish_fn(shm, stream.byte_size, stamp_now);
                }
                any = true;
            }
            if (any) gpu.last_publish = steady_now;
        }

        // Publish vehicle transform
        if (tf_state_.dirty.exchange(false, std::memory_order_acq_rel)) {
            {
                std::lock_guard<std::mutex> lk(tf_state_.mutex);
                tf_msg.transform.translation.x = tf_state_.pos[0];
                tf_msg.transform.translation.y = tf_state_.pos[1];
                tf_msg.transform.translation.z = tf_state_.pos[2];
                tf_msg.transform.rotation.x = tf_state_.rot[0];
                tf_msg.transform.rotation.y = tf_state_.rot[1];
                tf_msg.transform.rotation.z = tf_state_.rot[2];
                tf_msg.transform.rotation.w = tf_state_.rot[3];
            }
            tf_msg.header.stamp = stamp_now;
            tf_broadcaster_->sendTransform(tf_msg);
        }

        if (steady_now - last_marker >= k_marker_period) {
            last_marker = steady_now;
            if (marker_state_.dirty.exchange(false, std::memory_order_acq_rel) || !marker_built) {
                marker_built = true;
                marker_msg.markers.clear();
                marker_msg.markers.reserve(2);

                visualization_msgs::msg::Marker body;
                visualization_msgs::msg::Marker arrow;
                {
                    std::lock_guard<std::mutex> lk(marker_state_.mutex);
                    body.scale.x = marker_state_.scale[0];
                    body.scale.y = marker_state_.scale[1];
                    body.scale.z = marker_state_.scale[2];
                    body.color.r = static_cast<float>(marker_state_.color[0]);
                    body.color.g = static_cast<float>(marker_state_.color[1]);
                    body.color.b = static_cast<float>(marker_state_.color[2]);
                    body.color.a = static_cast<float>(marker_state_.color[3]);
                    arrow.color.r = static_cast<float>(marker_state_.arrow_color[0]);
                    arrow.color.g = static_cast<float>(marker_state_.arrow_color[1]);
                    arrow.color.b = static_cast<float>(marker_state_.arrow_color[2]);
                    arrow.color.a = static_cast<float>(marker_state_.arrow_color[3]);
                    arrow.pose.position.x = marker_state_.scale[0] * 0.5;
                }
                body.header.frame_id = name_;
                body.ns = name_ + "_pos";
                body.id = 0;
                body.type = visualization_msgs::msg::Marker::CUBE;
                body.action = visualization_msgs::msg::Marker::ADD;
                body.pose.orientation.w = 1.0;
                body.lifetime.sec = 5;

                arrow.header.frame_id = name_;
                arrow.ns = name_ + "_pos";
                arrow.id = 1;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose.orientation.w = 1.0;
                arrow.scale.x = 2.0; arrow.scale.y = 1.0; arrow.scale.z = 1.0;
                arrow.lifetime.sec = 5;

                marker_msg.markers.push_back(std::move(body));
                marker_msg.markers.push_back(std::move(arrow));
            }
            if (!marker_msg.markers.empty()) {
                marker_msg.markers[0].header.stamp = stamp_now;
                marker_msg.markers[1].header.stamp = stamp_now;
                marker_pub_->publish(marker_msg);
            }
        }

        if (running_.load(std::memory_order_acquire)) {
            spin_until(tick + k_loop_period);
        }
    }
}

}  // namespace bng_ros2
