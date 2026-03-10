#include "vehicle_node.h"
#include "sensor_registry.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "definitions.h"
#include "math_helpers.h"
#include <iostream>
#include <cstring>

namespace bng_ros2 {

VehicleNode::VehicleNode(const std::string& name)
    : Node(name, "beamng_bridge"), name_(name) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  visualizer_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 10);
  // NOTE: init_sensor_publishers() must be called AFTER make_shared() returns.
}

VehicleNode::~VehicleNode() {
  stop_publish_thread();
}

void VehicleNode::init_sensor_publishers() {
  const auto& registry = SensorRegistry::instance();
  if (!registry.is_loaded()) {
    std::cerr << "[ROS2 WARN] VehicleNode: SensorRegistry not loaded, no publishers created.\n";
    return;
  }

  auto& factory_reg = SensorFactoryRegistry::instance();

  uint32_t scalar_cursor = 0;
  uint32_t string_cursor = 0;

  for (const auto& sensor : registry.sensors()) {
    if (sensor.data_mode == DataMode::Blob) {
      const BlobRegisterFn* factory = factory_reg.find_blob(sensor.msg_file);
      if (!factory) {
        std::cerr << "[ROS2 WARN] VehicleNode: no blob factory for msgFile '"
                  << sensor.msg_file << "' (sensor '" << sensor.id << "'), skipping.\n";
        continue;
      }

      blob_slots_.emplace_back();
      BlobSlot& slot = blob_slots_.back();
      const uint32_t max_bytes = sensor.blob_layout.max_bytes;
      if (max_bytes > 0) {
        slot.buf.slot[0].resize(max_bytes);
        slot.buf.slot[1].resize(max_bytes);
      }

      try {
        slot.publish_fn = (*factory)(*this, sensor.topic, sensor);
        blob_sensor_index_[sensor.id] = static_cast<uint32_t>(blob_slots_.size() - 1);
        std::cout << "[ROS2 INFO] blob publisher: sensor '" << sensor.id
                  << "' on '" << sensor.topic << "'\n";
      } catch (const std::exception& e) {
        std::cerr << "[ROS2 ERROR] blob factory for sensor '" << sensor.id
                  << "': " << e.what() << "\n";
        blob_slots_.pop_back();
      }

    } else {
      const RegisterFn* sfac = factory_reg.find(sensor.msg_file);

      if (!sfac) {
        std::cerr << "[ROS2 WARN] VehicleNode: no scalar factory for msgFile '"
                  << sensor.msg_file << "' (sensor '" << sensor.id << "'), skipping.\n";
        continue;
      }

      try {
        auto result = (*sfac)(*this, sensor.topic, sensor, scalar_cursor, string_cursor);
        publish_fns_.push_back(std::move(result.publish_fn));
        std::cout << "[ROS2 INFO] scalar publisher: sensor '" << sensor.id
                  << "' on '" << sensor.topic
                  << "' scalar_base=" << scalar_cursor
                  << " slots=" << result.scalar_slots_used
                  << " string_slots=" << result.string_slots_used << "\n";
        scalar_cursor += result.scalar_slots_used;
        string_cursor += result.string_slots_used;
      } catch (const std::exception& e) {
        std::cerr << "[ROS2 ERROR] scalar factory for sensor '" << sensor.id
                  << "': " << e.what() << "\n";
      }
    }
  }

  if (scalar_cursor > 0) {
    scalar_buf_.slot[0].resize(scalar_cursor, 0.0);
    scalar_buf_.slot[1].resize(scalar_cursor, 0.0);
    std::cout << "[ROS2 INFO] ScalarStore pre-sized to " << scalar_cursor << " slots.\n";
  }
  if (string_cursor > 0) {
    string_buf_.slot[0].resize(string_cursor);
    string_buf_.slot[1].resize(string_cursor);
  }

  // Pre-populate the cached TF message with the fixed string fields so they are never heap-allocated on the hot path.
  tf_stamped_.header.frame_id = "world";
  tf_stamped_.child_frame_id  = name_;
}


void VehicleNode::broadcast_tf(double pos[3], double rot[4]) {
  // Only the stamp, translation, and rotation change per call; the string fields
  // were set once in init_sensor_publishers() and are reused from the cached message.
  tf_stamped_.header.stamp        = get_clock()->now();
  tf_stamped_.transform.translation = tf2::toMsg(to_vec3(pos));
  tf_stamped_.transform.rotation    = tf2::toMsg(to_quat(rot));
  tf_broadcaster_->sendTransform(tf_stamped_);
}

void VehicleNode::set_data(double scale[3], double color[4], double arrow_color[4]) {
  data_.scale       = to_vec3(scale);
  data_.color       = to_rgba(color);
  data_.arrow_color = to_rgba(arrow_color);
  marker_dirty_ = true;
}

uint32_t VehicleNode::register_frame_layout(const char* const* sensor_ids,
                                              const char* const* field_names,
                                              uint32_t count) {
  frame_layout_.clear();
  frame_layout_.reserve(count);

  // Assign sequential indices
  // The publish-time closures in publish_fns_ already captured their base indices, so we just record the position mapping here.
  // sensor_ids and field_names are not needed at this stage but are part of the API.
  for (uint32_t i = 0; i < count; ++i) {
    frame_layout_.push_back(i);
    (void)sensor_ids[i];
    (void)field_names[i];
  }

  // Ensure the scalar store slots are large enough.
  if (count > scalar_buf_.slot[0].size()) {
    scalar_buf_.slot[0].resize(count, 0.0);
    scalar_buf_.slot[1].resize(count, 0.0);
  }

  return count;
}

void VehicleNode::push_frame(const double* values, uint32_t count) {
  const uint32_t n = std::min(count, static_cast<uint32_t>(frame_layout_.size()));
  ScalarStore& ws = scalar_buf_.write_slot();
  for (uint32_t i = 0; i < n; ++i) {
    ws[frame_layout_[i]] = values[i];
  }
  scalar_buf_.commit();
}

void VehicleNode::set_sensor_field_string(std::string_view sensor_id,
                                            std::string_view field_name,
                                            std::string_view value) {
  // Construct the composite key on the stack, then build a std::string only if the lengths fit.
  // MSVC 2019 (C++17) does not support heterogeneous unordered_map lookup, so a proper std::string key is required for find().
  char key_buf[256];
  const size_t id_len    = sensor_id.size();
  const size_t field_len = field_name.size();
  if (id_len + field_len + 2 > sizeof(key_buf)) return;

  std::memcpy(key_buf, sensor_id.data(), id_len);
  key_buf[id_len]     = ':';
  key_buf[id_len + 1] = ':';
  std::memcpy(key_buf + id_len + 2, field_name.data(), field_len);
  // Construct the key string once from the stack buffer - single allocation.
  const std::string key(key_buf, id_len + 2 + field_len);
  auto it = string_field_index_.find(key);
  if (it == string_field_index_.end()) return;

  // Write into the current write slot. No commit here - string fields share the publish signal with the scalar frame.
  // The publish thread reads string_buf_ whenever scalar_buf_ is dirty, so committing once in push_frame is sufficient.
  string_buf_.write_slot()[it->second] = std::string(value);
}

uint32_t VehicleNode::register_blob_sensors(const char* const* sensor_ids,
                                              uint32_t count) {
  // Verify that each id was already indexed by init_sensor_publishers.
  // The sensor_ids here are rosConfig sensor IDs (e.g. "lidar_front"), not engine-level IDs.
  uint32_t registered = 0;
  for (uint32_t i = 0; i < count; ++i) {
    const std::string id(sensor_ids[i]);
    if (blob_sensor_index_.find(id) != blob_sensor_index_.end()) {
      ++registered;
    } else {
      std::cerr << "[ROS2 WARN] register_blob_sensors: sensor id '" << id
                << "' not found in blob_sensor_index_. "
                    "Check that it is declared as dataMode=blob in the config.\n";
    }
  }
  return registered;
}

void VehicleNode::push_blobs_batch(const BngBlobDesc* descs, uint32_t count) {
  // Write all blob slots first, commit all at the end.
  // This ensures the publish thread never wakes to a state where some slots have new data and
  // others still hold stale data from the previous frame.
  for (uint32_t i = 0; i < count; ++i) {
    const BngBlobDesc& d = descs[i];
    if (d.sensor_idx >= blob_slots_.size()) continue;
    BlobSlot& slot = blob_slots_[d.sensor_idx];

    std::vector<uint8_t>& ws = slot.buf.write_slot();
    // Grow only if payload exceeds pre-allocated capacity (should not happen in normal use).
    if (d.byte_len > ws.size()) ws.resize(d.byte_len);
    std::memcpy(ws.data(), d.data, d.byte_len);
    slot.timestamp_ns.store(d.timestamp_ns, std::memory_order_relaxed);
    slot.actual_byte_len.store(d.byte_len, std::memory_order_relaxed);
  }
  // Commit all written slots after all memcpys are done.
  for (uint32_t i = 0; i < count; ++i) {
    const BngBlobDesc& d = descs[i];
    if (d.sensor_idx < blob_slots_.size()) {
      blob_slots_[d.sensor_idx].buf.commit();
    }
  }
}

void VehicleNode::start_publish_thread() {
  if (running_.load()) return;
  running_.store(true);
  publish_thread_ = std::thread([this]() { publish_loop(); });
}

void VehicleNode::stop_publish_thread() {
  if (!running_.load()) return;
  running_.store(false);
  // Wake the thread so it can observe running_=false and exit cleanly.
  scalar_buf_.cv_.notify_all();
  if (publish_thread_.joinable()) publish_thread_.join();
}

void VehicleNode::publish_loop() {
  // Rate limiter for the marker visualizer
  auto last_marker_publish = std::chrono::steady_clock::now();
  constexpr auto kMarkerInterval = std::chrono::milliseconds(100);

  while (running_.load(std::memory_order_relaxed)) {

    // Sleep until a scalar commit wakes us, or the timeout expires.
    // The 1 ms timeout lets us still process blobs and markers when only blob sensors are active
    // (blobs have their own dirty flags but share the same thread).
    const bool has_scalar = scalar_buf_.wait_for_data(1);

    if (!running_.load(std::memory_order_relaxed)) break;

    if (has_scalar) {
      scalar_buf_.clear_dirty();
      const ScalarStore& scalars = scalar_buf_.read_slot();
      const StringStore& strings = string_buf_.read_slot();
      for (auto& fn : publish_fns_) {
        fn(scalars, strings);
      }
    }

    for (auto& slot : blob_slots_) {
      if (!slot.buf.is_dirty()) continue;
      slot.buf.clear_dirty();

      const std::vector<uint8_t>& data = slot.buf.read_slot();
      const uint64_t ts  = slot.timestamp_ns.load(std::memory_order_relaxed);
      const uint32_t len = slot.actual_byte_len.load(std::memory_order_relaxed);
      if (len > 0) {
        slot.publish_fn(data.data(), len, ts);
      }
    }

    // Marker visualizer
    const auto now = std::chrono::steady_clock::now();
    if (now - last_marker_publish >= kMarkerInterval) {
      last_marker_publish = now;

      if (marker_dirty_) {
        marker_array_.markers.clear();
        marker_array_.markers.reserve(2);

        visualization_msgs::msg::Marker vehicle;
        vehicle.header.frame_id  = name_;
        vehicle.ns               = name_ + "_pos";
        vehicle.id               = 0;
        vehicle.type             = visualization_msgs::msg::Marker::CUBE;
        vehicle.action           = visualization_msgs::msg::Marker::ADD;
        vehicle.pose.orientation.w = 1.0;
        vehicle.scale            = tf2::toMsg(data_.scale);
        vehicle.color            = data_.color;
        vehicle.lifetime.sec     = 5;

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id  = name_;
        arrow.ns               = name_ + "_pos";
        arrow.id               = 1;
        arrow.type             = visualization_msgs::msg::Marker::ARROW;
        arrow.action           = visualization_msgs::msg::Marker::ADD;
        arrow.pose.orientation.w = 1.0;
        arrow.pose.position.x += data_.scale[0] / 2;
        arrow.scale            = tf2::toMsg(tf2::Vector3(2.0, 1.0, 1.0));
        arrow.color            = data_.arrow_color;
        arrow.lifetime.sec     = 5;

        marker_array_.markers.emplace_back(std::move(vehicle));
        marker_array_.markers.emplace_back(std::move(arrow));
        marker_dirty_ = false;
      }

      if (!marker_array_.markers.empty()) {
        const auto stamp = get_clock()->now();
        marker_array_.markers[0].header.stamp = stamp;
        marker_array_.markers[1].header.stamp = stamp;
        visualizer_->publish(marker_array_);
      }
    }

    // ROS2 executor work (subscriptions, services, etc.)
    rclcpp::spin_some(std::static_pointer_cast<rclcpp::Node>(shared_from_this()));
  }
}

}  // namespace bng_ros2

static std::unordered_map<uint32_t, std::shared_ptr<bng_ros2::VehicleNode>> nodes;

BNG_ROS2_API char bngros2_init_vehicle_node(uint32_t id, const char* name) {
  try {
    auto node = std::make_shared<bng_ros2::VehicleNode>(name);
    node->init_sensor_publishers();
    const auto& [_, inserted] = nodes.emplace(id, node);
    if (inserted) {
      node->start_publish_thread();
      std::cout << "[ROS2 INFO] bngros2_init_vehicle_node: registered vid=" << id
                << " name='" << name << "' nodes.size()=" << nodes.size() << "\n";
    } else {
      std::cerr << "[ROS2 WARN] bngros2_init_vehicle_node: vid=" << id
                << " already registered, skipping.\n";
    }
    return static_cast<char>(inserted);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] bngros2_init_vehicle_node vid=" << id
              << ": " << e.what() << "\n";
  }
  return false;
}

BNG_ROS2_API void bngros2_destroy_vehicle_node(uint32_t id) {
  try {
    auto it = nodes.find(id);
    if (it != nodes.end()) {
      it->second->stop_publish_thread();
      nodes.erase(it);
    }
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API void bngros2_destroy_all_vehicle_nodes() {
  try {
    for (auto& [id, node] : nodes) node->stop_publish_thread();
    nodes.clear();
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

// Returns a raw (non-owning) pointer to the VehicleNode for the given vehicle id.
// The Lua side caches this once after init to avoid a map lookup on every hot-path call.
// The pointer remains valid for the lifetime of the node (until bngros2_destroy_vehicle_node).
BNG_ROS2_API void* bngros2_get_vehicle_handle(uint32_t id) {
  auto it = nodes.find(id);
  return (it != nodes.end()) ? it->second.get() : nullptr;
}

BNG_ROS2_API void bngros2_handle_broadcast_tf(void* handle, double pos[3], double rot[4]) {
  static_cast<bng_ros2::VehicleNode*>(handle)->broadcast_tf(pos, rot);
}

BNG_ROS2_API void bngros2_handle_push_frame(void* handle, const double* values, uint32_t count) {
  static_cast<bng_ros2::VehicleNode*>(handle)->push_frame(values, count);
}

BNG_ROS2_API void bngros2_handle_push_blobs_batch(void* handle,
                                                    const BngBlobDesc* descs,
                                                    uint32_t count) {
  static_cast<bng_ros2::VehicleNode*>(handle)->push_blobs_batch(descs, count);
}

BNG_ROS2_API void bngros2_handle_set_sensor_field_string(void* handle,
                                                          const char* sensor_id,
                                                          const char* field_name,
                                                          const char* value) {
  static_cast<bng_ros2::VehicleNode*>(handle)->set_sensor_field_string(
      sensor_id, field_name, value ? value : "");
}

BNG_ROS2_API void bngros2_broadcast_tf(uint32_t id, double pos[3], double rot[4]) {
  auto it = nodes.find(id);
  if (it == nodes.end()) return;
  try {
    it->second->broadcast_tf(pos, rot);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API void bngros2_vehicle_set_data(uint32_t id, double scale[3],
                                            double color[4], double arrow_color[4]) {
  auto it = nodes.find(id);
  if (it == nodes.end()) return;
  it->second->set_data(scale, color, arrow_color);
}

BNG_ROS2_API uint32_t bngros2_register_sensor_fields(uint32_t id,
                                                        const char* const* sensor_ids,
                                                        const char* const* field_names,
                                                        uint32_t count) {
  auto it = nodes.find(id);
  if (it == nodes.end()) {
    std::cerr << "[ROS2 WARN] bngros2_register_sensor_fields: vid=" << id
              << " not found in nodes map (size=" << nodes.size() << "). "
                  "Was bngros2_init_vehicle_node called first?\n";
    return 0;
  }
  try {
    return it->second->register_frame_layout(sensor_ids, field_names, count);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] bngros2_register_sensor_fields: " << e.what() << "\n";
    return 0;
  }
}

BNG_ROS2_API void bngros2_vehicle_push_frame(uint32_t id,
                                              const double* values,
                                              uint32_t count) {
  auto it = nodes.find(id);
  if (it == nodes.end()) return;
  it->second->push_frame(values, count);
}

BNG_ROS2_API void bngros2_vehicle_set_sensor_field_string(uint32_t id,
                                                            const char* sensor_id,
                                                            const char* field_name,
                                                            const char* value) {
  auto it = nodes.find(id);
  if (it == nodes.end()) return;
  try {
    it->second->set_sensor_field_string(sensor_id, field_name, value ? value : "");
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API uint32_t bngros2_register_blob_sensors(uint32_t vehicle_id,
                                                      const char* const* sensor_ids,
                                                      uint32_t count) {
  auto it = nodes.find(vehicle_id);
  if (it == nodes.end()) return 0;
  try {
    return it->second->register_blob_sensors(sensor_ids, count);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] bngros2_register_blob_sensors: " << e.what() << "\n";
    return 0;
  }
}

BNG_ROS2_API void bngros2_push_blobs_batch(uint32_t vehicle_id,
                                            const BngBlobDesc* descs,
                                            uint32_t count) {
  auto it = nodes.find(vehicle_id);
  if (it == nodes.end()) return;
  try {
    it->second->push_blobs_batch(descs, count);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] bngros2_push_blobs_batch: " << e.what() << "\n";
  }
}
