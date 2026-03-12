#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "definitions.h"
#include "sensor_factories.h"
#include "spsc_double_buffer.h"
#include "beamng_node.h"

namespace bng_ros2 {

struct VehicleData {
  tf2::Vector3 scale;
  std_msgs::msg::ColorRGBA color;
  std_msgs::msg::ColorRGBA arrow_color;
};

// One blob slot per Blob sensor.
// Both buffer slots are pre-sized to cfg.blob_layout.max_bytes at init time.
struct BlobSlot {
  SpscDoubleBuffer<std::vector<uint8_t>> buf;
  std::atomic<uint64_t>                 timestamp_ns{0};
  std::atomic<uint32_t>                 actual_byte_len{0};
  BlobPublishFn                         publish_fn;

  BlobSlot() = default;

  BlobSlot(BlobSlot&& o) noexcept
      : buf{std::move(o.buf)},
        timestamp_ns{o.timestamp_ns.load(std::memory_order_relaxed)},
        actual_byte_len{o.actual_byte_len.load(std::memory_order_relaxed)},
        publish_fn{std::move(o.publish_fn)} {}

  BlobSlot& operator=(BlobSlot&& o) noexcept {
    if (this != &o) {
      buf        = std::move(o.buf);
      timestamp_ns.store(o.timestamp_ns.load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
      actual_byte_len.store(o.actual_byte_len.load(std::memory_order_relaxed),
                            std::memory_order_relaxed);
      publish_fn = std::move(o.publish_fn);
    }
    return *this;
  }

  BlobSlot(const BlobSlot&)            = delete;
  BlobSlot& operator=(const BlobSlot&) = delete;
};

class VehicleNode : public rclcpp::Node {
  public:
    explicit VehicleNode(const std::string& name);
    ~VehicleNode();

    void broadcast_tf(double pos[3], double rot[4]);
    void set_data(double scale[3], double color[4], double arrow_color[4]);

    // Scalar path - registers the ordered mapping between Lua frame buffer positions
    // and their destination indices in the flat ScalarStore. Called once after construction.
    // Returns the number of slots registered.
    uint32_t register_frame_layout(const char* const* sensor_ids,
                                    const char* const* field_names,
                                    uint32_t count);

    // Scalar hot path - scatters values into the write slot, then commits.
    void push_frame(const double* values, uint32_t count);

    // String fields - low-frequency, not on the hot path.
    void set_sensor_field_string(std::string_view sensor_id,
                                  std::string_view field_name,
                                  std::string_view value);

    // Blob path - validates that each sensor id is known and returns the count of valid ones.
    // Called once after sensor creation in GE Lua.
    uint32_t register_blob_sensors(const char* const* sensor_ids, uint32_t count);

    // Blob hot path - copies all descriptors into write slots and commits them all at once.
    // Batching the commits prevents the publish thread from seeing a partial blob update.
    void push_blobs_batch(const BngBlobDesc* descs, uint32_t count);

    // Called from the C API init/destroy functions.
    void start_publish_thread();
    void stop_publish_thread();

    // Must be called once after make_shared (not from the constructor) because
    // scalar factories capture shared_from_this(), which requires the object to
    // already be managed by a shared_ptr.
    void init_sensor_publishers();

  private:
    void publish_loop();

    std::string  name_;
    VehicleData  data_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizer_;

    // Cached TF message - fixed string fields set once at init, mutable fields
    // (stamp, translation, rotation) updated per call. Avoids string heap allocation
    // on the 2000 Hz physics hot path.
    geometry_msgs::msg::TransformStamped tf_stamped_;

    // Scalar path
    SpscDoubleBuffer<ScalarStore> scalar_buf_;
    SpscDoubleBuffer<StringStore> string_buf_;
    // frame_layout_[i] = index into ScalarStore for the i-th Lua frame slot.
    std::vector<uint32_t> frame_layout_;
    // Maps "sensor_id::field_name" → index in StringStore.
    std::unordered_map<std::string, uint32_t> string_field_index_;
    // PublishFn vector in sensor registration order; called by publish_loop.
    std::vector<PublishFn> publish_fns_;

    // Blob path
    std::unordered_map<std::string, uint32_t> blob_sensor_index_;
    std::vector<BlobSlot>                     blob_slots_;

    // Marker visualizer
    visualization_msgs::msg::MarkerArray marker_array_;
    bool marker_dirty_{true};

    // Shared blob-dirty notification: publish_loop waits on this cv when no scalar data is pending, so blob commits wake the thread immediately.
    std::mutex              blob_cv_mutex_;
    std::condition_variable blob_cv_;
    std::atomic<bool>       blob_dirty_{false};

    // Cached node shared_ptr for spin_some - avoids atomic refcount on hot path.
    std::shared_ptr<rclcpp::Node> self_ptr_;

    // Publish thread
    std::thread       publish_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace bng_ros2

extern "C" {
BNG_ROS2_API char     bngros2_init_vehicle_node(uint32_t id, const char* name);
BNG_ROS2_API void     bngros2_destroy_vehicle_node(uint32_t id);
BNG_ROS2_API void     bngros2_destroy_all_vehicle_nodes();

// Returns an opaque handle to the VehicleNode for the given id, or nullptr.
// Cache this handle on the Lua side to avoid a hash-map lookup on every hot-path call.
BNG_ROS2_API void*    bngros2_get_vehicle_handle(uint32_t id);

// Handle-based hot-path functions - bypass the global nodes map entirely.
BNG_ROS2_API void     bngros2_handle_broadcast_tf(void* handle, double pos[3], double rot[4]);
BNG_ROS2_API void     bngros2_handle_push_frame(void* handle, const double* values, uint32_t count);
BNG_ROS2_API void     bngros2_handle_push_blobs_batch(void* handle, const BngBlobDesc* descs, uint32_t count);
BNG_ROS2_API void     bngros2_handle_set_sensor_field_string(void* handle, const char* sensor_id, const char* field_name, const char* value);

// Id-based setup functions - called once, map lookup cost is acceptable.
BNG_ROS2_API void     bngros2_broadcast_tf(uint32_t id, double pos[3], double rot[4]);
BNG_ROS2_API void     bngros2_vehicle_set_data(uint32_t id, double scale[3], double color[4], double arrow_color[4]);
BNG_ROS2_API uint32_t bngros2_register_sensor_fields(uint32_t id, const char* const* sensor_ids, const char* const* field_names, uint32_t count);
BNG_ROS2_API void     bngros2_vehicle_push_frame(uint32_t id, const double* values, uint32_t count);
BNG_ROS2_API void     bngros2_vehicle_set_sensor_field_string(uint32_t id, const char* sensor_id, const char* field_name, const char* value);
BNG_ROS2_API uint32_t bngros2_register_blob_sensors(uint32_t vehicle_id, const char* const* sensor_ids, uint32_t count);
BNG_ROS2_API void     bngros2_push_blobs_batch(uint32_t vehicle_id, const BngBlobDesc* descs, uint32_t count);
}
