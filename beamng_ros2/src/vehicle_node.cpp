#include "vehicle_node.h"
#include "sensor_registry.h"
#include <beamng_msgs/msg/vehicle_control.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "definitions.h"
#include "math_helpers.h"
#include "sensor_factories.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <string_view>

namespace {

// Minimum wall-clock spacing between ROS publishes when publishMode is "fixed"; zero means no throttle.
std::chrono::nanoseconds interval_ns_for_fixed_publish_hz(const std::string& publish_mode, double rate_hz) {
  if (publish_mode != "fixed" || rate_hz <= 0.0) return std::chrono::nanoseconds{0};
  const double period_sec = 1.0 / rate_hz;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec));
}

// Engine shmem size per stream role (RGBA or float32 plane is 4 bytes per pixel, annotations RGB is 3).
size_t camera_shmem_byte_size(std::string_view role, uint32_t width, uint32_t height) {
  const size_t pixels = static_cast<size_t>(width) * height;
  return (role == "annotation") ? (pixels * 3u) : (pixels * 4u);
}

void copy_rgba8_to_rgb8(const uint8_t* rgba, uint8_t* rgb, size_t pixel_count) {
  const uint8_t* s = rgba;
  uint8_t* d = rgb;
  for (size_t i = 0; i < pixel_count; ++i) {
    d[0] = s[0];
    d[1] = s[1];
    d[2] = s[2];
    s += 4;
    d += 3;
  }
}

}  // namespace

namespace bng_ros2 {

VehicleNode::VehicleNode(const std::string& name)
    : Node(name, "beamng_bridge"), name_(name) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  visualizer_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 10);
  control_sub_ = create_subscription<beamng_msgs::msg::VehicleControl>(
      "~/control", 10, [this](const beamng_msgs::msg::VehicleControl::SharedPtr msg) {
        control_callback(msg);
      });
  // NOTE: init_sensor_publishers() must be called AFTER make_shared() returns.
}

VehicleNode::~VehicleNode() {
  stop_publish_thread();
}

void VehicleNode::prepare_destroy() {
  stop_publish_thread();
  publish_fns_.clear();
  for (auto& slot : blob_slots_) {
    slot.publish_fn = nullptr;
  }
  blob_slots_.clear();
  blob_sensor_index_.clear();
  camera_slots_.clear();
  control_sub_.reset();
  visualizer_.reset();
  tf_broadcaster_.reset();
  self_ptr_.reset();
}

void VehicleNode::control_callback(const beamng_msgs::msg::VehicleControl::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  last_control_.steering = msg->steering;
  last_control_.throttle = msg->throttle;
  last_control_.brake = msg->brake;
  last_control_.parkingbrake = msg->parkingbrake;
  last_control_.clutch = msg->clutch;
  last_control_.gear = msg->gear;
  last_control_time_ = std::chrono::steady_clock::now();
}

bool VehicleNode::get_control(double* steering, double* throttle, double* brake,
                                double* parkingbrake, double* clutch, int8_t* gear,
                                int timeout_ms) const {
  std::lock_guard<std::mutex> lock(control_mutex_);
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_control_time_).count();
  if (elapsed > timeout_ms) return false;
  if (steering) *steering = last_control_.steering;
  if (throttle) *throttle = last_control_.throttle;
  if (brake) *brake = last_control_.brake;
  if (parkingbrake) *parkingbrake = last_control_.parkingbrake;
  if (clutch) *clutch = last_control_.clutch;
  if (gear) *gear = last_control_.gear;
  return true;
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
      const std::string base = "/beamng_bridge/" + name_ + "/";

      if (sensor.msg_file == "Image") {
        blob_slots_.emplace_back();
        const uint32_t slot_idx = static_cast<uint32_t>(blob_slots_.size() - 1);
        blob_sensor_index_[sensor.id] = slot_idx;
        BlobSlot& slot = blob_slots_.back();
        const uint32_t max_bytes = sensor.blob_layout.max_bytes;
        if (max_bytes > 0) {
          slot.max_blob_bytes = max_bytes;
          slot.buf.slot[0].resize(max_bytes);
          slot.buf.slot[1].resize(max_bytes);
        }

        const uint32_t img_w = sensor.blob_layout.width  > 0 ? sensor.blob_layout.width  : 512u;
        const uint32_t img_h = sensor.blob_layout.height > 0 ? sensor.blob_layout.height : 512u;

        CameraSlot cam;
        cam.min_publish_interval =
            interval_ns_for_fixed_publish_hz(sensor.blob_layout.publish_mode, sensor.blob_layout.publish_rate_hz);

        const auto& stream_specs = sensor.blob_layout.image_streams;
        try {
          const auto add_stream = [&](const std::string& topic, const std::string& role,
                                      const std::string& enc) {
            CameraShmemStream stream{};
            stream.width = img_w;
            stream.height = img_h;
            stream.role = role;
            stream.publish_fn = make_camera_sensor_factory(*this, topic, sensor, enc, img_w, img_h);
            cam.streams.push_back(std::move(stream));
          };

          if (stream_specs.empty()) {
            add_stream(base + sensor.id, "colour", "rgb8");
          } else {
            const bool use_plain_sensor_topic = (stream_specs.size() == 1u);
            for (const auto& spec : stream_specs) {
              const std::string enc = spec.encoding.empty() ? "rgb8" : spec.encoding;
              const std::string topic =
                  use_plain_sensor_topic ? (base + sensor.id) : (base + sensor.id + "_" + spec.id);
              add_stream(topic, spec.role, enc);
            }
          }
          const size_t stream_count = cam.streams.size();
          camera_slots_[slot_idx] = std::move(cam);
          std::cout << "[ROS2 INFO] camera publisher: sensor '" << sensor.id
                    << "' slot " << slot_idx << " streams=" << stream_count << "\n";
        } catch (const std::exception& e) {
          std::cerr << "[ROS2 ERROR] camera init for sensor '" << sensor.id
                    << "': " << e.what() << "\n";
          blob_slots_.pop_back();
          blob_sensor_index_.erase(sensor.id);
        }
        continue;
      }

      const std::string topic = "/beamng_bridge/" + name_ + "/" + sensor.id;
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
        slot.max_blob_bytes = max_bytes;
        slot.buf.slot[0].resize(max_bytes);
        slot.buf.slot[1].resize(max_bytes);
      }

      try {
        slot.publish_fn = (*factory)(*this, topic, sensor);
        blob_sensor_index_[sensor.id] = static_cast<uint32_t>(blob_slots_.size() - 1);
        std::cout << "[ROS2 INFO] blob publisher: sensor '" << sensor.id
                  << "' on '" << topic << "'\n";
      } catch (const std::exception& e) {
        std::cerr << "[ROS2 ERROR] blob factory for sensor '" << sensor.id
                  << "': " << e.what() << "\n";
        blob_slots_.pop_back();
      }

    } else {
      const std::string topic = "/beamng_bridge/" + name_ + "/" + sensor.id;
      const RegisterFn* sfac = factory_reg.find(sensor.msg_file);

      if (!sfac) {
        std::cerr << "[ROS2 WARN] VehicleNode: no scalar factory for msgFile '"
                  << sensor.msg_file << "' (sensor '" << sensor.id << "'), skipping.\n";
        continue;
      }

      try {
        auto result = (*sfac)(*this, topic, sensor, scalar_cursor, string_cursor);
        publish_fns_.push_back(std::move(result.publish_fn));
        std::cout << "[ROS2 INFO] scalar publisher: sensor '" << sensor.id
                  << "' on '" << topic
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

  // Cache a shared_ptr to self for use in the publish loop (spin_some).
  // Must be assigned after init_sensor_publishers since shared_from_this() requires the object to already be managed by a shared_ptr at that point.
  self_ptr_ = shared_from_this();

  // Pre-populate the cached TF message with the fixed string fields so they are never heap-allocated on the hot path.
  tf_stamped_.header.frame_id = "world";
  tf_stamped_.child_frame_id  = name_;
}


void VehicleNode::broadcast_tf(double pos[3], double rot[4]) {
  // Only the stamp, translation, and rotation change per call; the string fields were set once in init_sensor_publishers() and are reused from the cached message.
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
  (void)sensor_ids;
  (void)field_names;

  frame_layout_.clear();
  frame_layout_.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    frame_layout_.push_back(i);
  }

  if (count > scalar_buf_.slot[0].size()) {
    scalar_buf_.slot[0].resize(count, 0.0);
    scalar_buf_.slot[1].resize(count, 0.0);
  }

  return count;
}

void VehicleNode::push_frame(const double* values, uint32_t count) {
  const uint32_t n = std::min(count, static_cast<uint32_t>(frame_layout_.size()));
  if (n == 0) return;
  ScalarStore& ws = scalar_buf_.write_slot();
  std::memcpy(ws.data(), values, static_cast<size_t>(n) * sizeof(double));
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
  // Guard: avoid dereferencing if no blob slots (e.g. node created before config loaded).
  if (blob_slots_.empty() || count == 0) return;

  // Write all blob slots first, commit all at the end.
  // This ensures the publish thread never wakes to a state where some slots have new data and
  // others still hold stale data from the previous frame.
  for (uint32_t i = 0; i < count; ++i) {
    const BngBlobDesc& d = descs[i];
    if (d.sensor_idx >= blob_slots_.size()) continue;
    BlobSlot& slot = blob_slots_[d.sensor_idx];

    if (d.data == nullptr) continue;

    uint32_t payload = d.byte_len;
    if (slot.max_blob_bytes > 0 && payload > slot.max_blob_bytes) {
      payload = slot.max_blob_bytes;
    }

    std::vector<uint8_t>& ws = slot.buf.write_slot();
    // Grow only if payload exceeds pre-allocated capacity (should not happen in normal use).
    if (payload > ws.size()) ws.resize(payload);
    if (payload > 0) {
      std::memcpy(ws.data(), d.data, payload);
    }
    slot.timestamp_ns.store(d.timestamp_ns, std::memory_order_relaxed);
    slot.actual_byte_len.store(payload, std::memory_order_relaxed);
  }
  // Commit all written slots after all memcpys are done.
  for (uint32_t i = 0; i < count; ++i) {
    const BngBlobDesc& d = descs[i];
    if (d.sensor_idx < blob_slots_.size()) {
      blob_slots_[d.sensor_idx].buf.commit();
    }
  }
  // Wake the publish thread once after all slots are committed.
  blob_dirty_.store(true, std::memory_order_release);
  blob_cv_.notify_one();
}

void VehicleNode::set_camera_streams(uint32_t sensor_idx, const BngCameraStreamDesc* streams, uint32_t count) {
  if (!streams || count == 0) return;
  auto it = camera_slots_.find(sensor_idx);
  if (it == camera_slots_.end()) {
    std::cerr << "[ROS2 WARN] set_camera_streams: unknown camera slot index " << sensor_idx << ".\n";
    return;
  }
  CameraSlot& cam = it->second;
  for (uint32_t i = 0; i < count; ++i) {
    const BngCameraStreamDesc& desc = streams[i];
    if (desc.stream_index >= cam.streams.size()) continue;
    CameraShmemStream& stream = cam.streams[desc.stream_index];
    stream.width = desc.width;
    stream.height = desc.height;
    const std::string shmem_name = desc.shmem_name ? desc.shmem_name : "";
    const size_t region_bytes = camera_shmem_byte_size(stream.role, desc.width, desc.height);
    if (!stream.reader.open(shmem_name, region_bytes)) {
      std::cerr << "[ROS2 WARN] set_camera_streams: failed to open shmem '" << shmem_name
                << "' (size " << region_bytes << ").\n";
      continue;
    }
    if (stream.role == "colour") {
      stream.convert_buf.resize(static_cast<size_t>(desc.width) * desc.height * 3u);
    }
    std::cout << "[ROS2 INFO] camera slot " << sensor_idx << " stream " << desc.stream_index
              << " shmem '" << shmem_name << "' " << desc.width << "x" << desc.height
              << " role=" << stream.role << "\n";
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
  // Wake the thread on both CVs so it can observe running_=false and exit cleanly.
  scalar_buf_.cv_.notify_all();
  blob_cv_.notify_all();
  if (publish_thread_.joinable()) publish_thread_.join();
}

void VehicleNode::publish_loop() {
  // Rate limiter for the marker visualizer
  auto last_marker_publish = std::chrono::steady_clock::now();
  constexpr auto kMarkerInterval = std::chrono::milliseconds(100);

  // Max wait when idle
  constexpr auto kMaxWaitForTf = std::chrono::microseconds(500);

  constexpr auto kMinSpinSomeInterval = std::chrono::milliseconds(2);
  auto last_spin_some = std::chrono::steady_clock::now() - kMinSpinSomeInterval;
  // Cap drain iterations so a bad producer cannot starve blob/image/spin work forever.
  constexpr int kMaxScalarDrainPerLoop = 128;

  auto publish_pending_scalars = [this] {
    if (!scalar_buf_.is_dirty()) return;
    scalar_buf_.clear_dirty();
    const ScalarStore& scalars = scalar_buf_.read_slot();
    const StringStore& strings = string_buf_.read_slot();
    const rclcpp::Time stamp = get_clock()->now();
    for (auto& fn : publish_fns_) {
      fn(scalars, strings, stamp);
    }
  };

  while (running_.load(std::memory_order_relaxed)) {

    const auto wait_deadline = std::chrono::steady_clock::now();
    std::chrono::microseconds wait_budget = kMaxWaitForTf;
    if (!camera_slots_.empty()) {
      std::chrono::microseconds soonest_wakeup = kMaxWaitForTf;
      bool any_camera_unthrottled = false;
      for (const auto& slot_entry : camera_slots_) {
        const CameraSlot& camera = slot_entry.second;
        if (camera.min_publish_interval.count() <= 0) {
          any_camera_unthrottled = true;
          break;
        }
        const auto next_publish_at = camera.last_publish + camera.min_publish_interval;
        if (wait_deadline >= next_publish_at) {
          soonest_wakeup = std::chrono::microseconds(0);
          break;
        }
        const auto time_until_publish =
            std::chrono::duration_cast<std::chrono::microseconds>(next_publish_at - wait_deadline);
        if (time_until_publish < soonest_wakeup) soonest_wakeup = time_until_publish;
      }
      wait_budget = any_camera_unthrottled ? std::chrono::microseconds(0)
                                            : std::min(soonest_wakeup, kMaxWaitForTf);
    }
    scalar_buf_.wait_for_data(wait_budget);

    if (!running_.load(std::memory_order_relaxed)) break;

    // Publish scalars before camera work so physics-rate topics are not delayed by RGB conversion.
    for (int drain = 0; drain < kMaxScalarDrainPerLoop && scalar_buf_.is_dirty(); ++drain) {
      publish_pending_scalars();
    }

    if (!camera_slots_.empty()) {
      const auto steady_now = std::chrono::steady_clock::now();
      const int64_t stamp_ns = get_clock()->now().nanoseconds();
      for (auto& slot_entry : camera_slots_) {
        const uint32_t blob_slot_index = slot_entry.first;
        CameraSlot& camera = slot_entry.second;
        if (blob_slot_index >= blob_slots_.size()) continue;
        if (camera.min_publish_interval.count() > 0) {
          if (steady_now - camera.last_publish < camera.min_publish_interval) continue;
        }
        camera.last_publish = steady_now;

        for (auto& shmem_stream : camera.streams) {
          if (!shmem_stream.publish_fn || !shmem_stream.reader.is_open()) continue;
          const uint8_t* shmem = shmem_stream.reader.data();
          const std::string& role = shmem_stream.role;
          if (role == "colour") {
            const size_t pixels = static_cast<size_t>(shmem_stream.width) * shmem_stream.height;
            copy_rgba8_to_rgb8(shmem, shmem_stream.convert_buf.data(), pixels);
            shmem_stream.publish_fn(shmem_stream.convert_buf.data(), static_cast<uint32_t>(pixels * 3u),
                                    static_cast<uint64_t>(stamp_ns));
          } else if (role == "annotation") {
            const size_t byte_len = static_cast<size_t>(shmem_stream.width) * shmem_stream.height * 3u;
            shmem_stream.publish_fn(shmem, static_cast<uint32_t>(byte_len), static_cast<uint64_t>(stamp_ns));
          } else if (role == "depth" || role == "instance") {
            const size_t byte_len = static_cast<size_t>(shmem_stream.width) * shmem_stream.height * 4u;
            shmem_stream.publish_fn(shmem, static_cast<uint32_t>(byte_len), static_cast<uint64_t>(stamp_ns));
          }
        }
      }
    }

    for (int drain = 0; drain < kMaxScalarDrainPerLoop && scalar_buf_.is_dirty(); ++drain) {
      publish_pending_scalars();
    }

    // Non-camera blob sensors only; cameras use shared memory in the block above.
    if (blob_dirty_.load(std::memory_order_acquire)) {
      blob_dirty_.store(false, std::memory_order_relaxed);
      for (uint32_t i = 0; i < blob_slots_.size(); ++i) {
        if (camera_slots_.count(i)) continue;
        BlobSlot& slot = blob_slots_[i];
        if (!slot.buf.is_dirty()) continue;
        slot.buf.clear_dirty();
        const std::vector<uint8_t>& data = slot.buf.read_slot();
        const uint64_t ts  = slot.timestamp_ns.load(std::memory_order_relaxed);
        const uint32_t len = slot.actual_byte_len.load(std::memory_order_relaxed);
        if (len > 0 && slot.publish_fn) {
          slot.publish_fn(data.data(), len, ts);
        }
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

    // ROS2 executor (subscriptions, e.g. ~/control). Must not run every loop iteration at physics rate.
    const auto now_spin = std::chrono::steady_clock::now();
    if (now_spin - last_spin_some >= kMinSpinSomeInterval) {
      last_spin_some = now_spin;
      rclcpp::spin_some(self_ptr_);
    }
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
      it->second->prepare_destroy();
      nodes.erase(it);
    }
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API void bngros2_destroy_all_vehicle_nodes() {
  try {
    for (auto& [id, node] : nodes) node->prepare_destroy();
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
  if (!handle || !descs) return;
  static_cast<bng_ros2::VehicleNode*>(handle)->push_blobs_batch(descs, count);
}

BNG_ROS2_API void bngros2_handle_set_camera_streams(void* handle,
                                                     uint32_t sensor_idx,
                                                     const BngCameraStreamDesc* streams,
                                                     uint32_t count) {
  if (!handle || !streams) return;
  static_cast<bng_ros2::VehicleNode*>(handle)->set_camera_streams(sensor_idx, streams, count);
}

BNG_ROS2_API void bngros2_handle_set_sensor_field_string(void* handle,
                                                          const char* sensor_id,
                                                          const char* field_name,
                                                          const char* value) {
  static_cast<bng_ros2::VehicleNode*>(handle)->set_sensor_field_string(
      sensor_id, field_name, value ? value : "");
}

BNG_ROS2_API char bngros2_handle_get_control(void* handle, double* steering, double* throttle,
                                              double* brake, double* parkingbrake, double* clutch,
                                              int8_t* gear) {
  if (!handle) return 0;
  return static_cast<char>(static_cast<bng_ros2::VehicleNode*>(handle)->get_control(
      steering, throttle, brake, parkingbrake, clutch, gear, 300)); // 300ms timeout
}

BNG_ROS2_API void bngros2_broadcast_tf(uint32_t id, double pos[3], double rot[4]) {
  auto it = nodes.find(id);
  if (it != nodes.end()) it->second->broadcast_tf(pos, rot);
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
