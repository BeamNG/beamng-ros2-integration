#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "api.h"
#include "memory/buffers.h"
#include "memory/shared_memory_reader.h"
#include "utils/config.h"

namespace bng_ros2 {

using ScalarValues = std::vector<double>;
using StringValues = std::vector<std::string>;

using ScalarPublishFn = std::function<void(const ScalarValues&, const StringValues&, const rclcpp::Time&)>;
using BlobPublishFn   = std::function<void(const uint8_t*, uint32_t, const rclcpp::Time&)>;

struct ScalarPublisherBuild {
    ScalarPublishFn publish_fn;
    uint32_t scalar_slots_used{0};
    uint32_t string_slots_used{0};
    std::vector<std::pair<std::string, uint32_t>> string_bindings;
};

struct GpuStream {
    SharedMemoryReader reader;
    std::string shared_memory_name;
    uint32_t width{0};
    uint32_t height{0};
    uint32_t byte_size{0};
    std::string role;
    std::string encoding;
    bool needs_rgb_conversion{false};
    BlobPublishFn publish_fn;
    std::vector<uint8_t> convert_buf;
};

struct GpuSensor {
    std::vector<GpuStream> streams;
    std::chrono::nanoseconds min_publish_interval{0};
    std::chrono::steady_clock::time_point last_publish{std::chrono::steady_clock::now()};
};

struct BlobSlot {
    std::unique_ptr<BlobBuffer> buffer;
    BlobPublishFn publish_fn;
    std::chrono::nanoseconds min_publish_interval{0};
    std::chrono::steady_clock::time_point last_publish{std::chrono::steady_clock::now()};
};

class VehicleNode : public rclcpp::Node {
  public:
    explicit VehicleNode(const std::string& name);
    ~VehicleNode();

    int apply_config(const std::string& json);

    int get_blob_slot(uint32_t slot, BngBlobSlotFfiView* out);
    uint32_t blob_state_acquire(uint32_t slot);
    void blob_commit_side(uint32_t slot, uint32_t side, uint32_t length_bytes);

    void bind_gpu_sensor(uint32_t gpu_slot, const BngGpuStreamDesc* descs, uint32_t count);
    void unbind_gpu_sensor(uint32_t gpu_slot);

    void push_scalar_frame(const double* values, uint32_t count);
    void set_scalar_string(const char* sensor_id, const char* field_name, const char* value);

    void broadcast_tf(const double pos[3], const double rot[4]);
    void set_vehicle_data(const double scale[3], const double color[4], const double arrow_color[4]);

  private:
    void publish_loop();

    std::string name_;

    ScalarBuffer<double> scalar_buf_;
    StringValues string_store_;
    std::mutex string_mutex_;
    std::atomic<uint64_t> string_store_version_{0};
    bool has_string_fields_{false};
    std::unordered_map<std::string, uint32_t> string_field_index_;
    std::vector<ScalarPublishFn> scalar_publishers_;

    std::vector<BlobSlot> blob_slots_;
    std::vector<GpuSensor> gpu_sensors_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    struct {
        std::atomic<bool> dirty{false};
        std::mutex mutex;
        double pos[3]{0, 0, 0};
        double rot[4]{0, 0, 0, 1};
    } tf_state_;

    struct {
        std::atomic<bool> dirty{false};
        std::mutex mutex;
        double scale[3]{1, 1, 1};
        double color[4]{1, 1, 1, 1};
        double arrow_color[4]{1, 0, 0, 1};
    } marker_state_;

    std::thread publish_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace bng_ros2
