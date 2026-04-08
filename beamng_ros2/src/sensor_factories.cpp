#include "sensor_factories.h"
#include "sensor_registry.h"
#include <beamng_msgs/msg/electrics_sensor.hpp>
#include <beamng_msgs/msg/g_force_sensor.hpp>
#include <beamng_msgs/msg/state_sensor.hpp>
#include <beamng_msgs/msg/damage_sensor.hpp>
#include <beamng_msgs/msg/time_sensor.hpp>
#include <beamng_msgs/msg/radar_scan.hpp>
#include <beamng_msgs/msg/ideal_radar_sensor.hpp>
#include <beamng_msgs/msg/powertrain_sensor.hpp>
#include <beamng_msgs/msg/roads_sensor.hpp>
#include <beamng_msgs/msg/mesh_sensor.hpp>
#include <beamng_msgs/msg/mesh_sensor_node.hpp>
#include <beamng_msgs/msg/mesh_sensor_beam.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>
#include <cstdint>
#include <cstring>

namespace bng_ros2 {

namespace intr = rosidl_typesupport_introspection_cpp;

// Empty sensor_config.frame_id means "use the ROS node name" for std_msgs/Header.frame_id.
static std::string config_frame_id(rclcpp::Node& node, const std::string& frame_id_cfg) {
  return frame_id_cfg.empty() ? std::string(node.get_name()) : frame_id_cfg;
}

SensorFactoryRegistry& SensorFactoryRegistry::instance() {
  static SensorFactoryRegistry inst;
  return inst;
}

void SensorFactoryRegistry::register_factory(const std::string& msg_file, RegisterFn fn) {
  factories_[msg_file] = std::move(fn);
}

void SensorFactoryRegistry::register_blob_factory(const std::string& msg_file, BlobRegisterFn fn) {
  blob_factories_[msg_file] = std::move(fn);
}

const RegisterFn* SensorFactoryRegistry::find(const std::string& msg_file) const {
  auto it = factories_.find(msg_file);
  if (it == factories_.end()) return nullptr;
  return &it->second;
}

const BlobRegisterFn* SensorFactoryRegistry::find_blob(const std::string& msg_file) const {
  auto it = blob_factories_.find(msg_file);
  if (it == blob_factories_.end()) return nullptr;
  return &it->second;
}

// Describes one active field: resolved once at init, used on every publish.
struct FieldWriter {
  uint32_t value_idx;  // index into ScalarStore (numeric) or StringStore (string)
  size_t   offset;     // byte offset into the message struct
  uint8_t  type_id;    // rosidl primitive type id
  bool     is_string;
};

struct FieldResolution {
  size_t  offset{0};
  uint8_t type_id{0};
  bool    found{false};
};

// Recursively resolve a dot-separated field path (e.g. "g.x", "wheelthermalsfl.brake_core_temperature") against a MessageMembers tree.
// Returns the accumulated byte offset and leaf type id on success.
static FieldResolution resolve_field_path(
    const intr::MessageMembers* members,
    const std::string& dot_path,
    size_t base_offset = 0)
{
  auto dot = dot_path.find('.');
  const std::string head = (dot == std::string::npos) ? dot_path : dot_path.substr(0, dot);
  const std::string tail = (dot == std::string::npos) ? ""       : dot_path.substr(dot + 1);

  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto& member = members->members_[i];
    if (head != member.name_) continue;

    size_t field_offset = base_offset + member.offset_;

    if (tail.empty()) {
      return {field_offset, member.type_id_, true};
    }

    if (member.type_id_ == intr::ROS_TYPE_MESSAGE && member.members_ != nullptr) {
      const auto* nested = reinterpret_cast<const intr::MessageMembers*>(member.members_->data);
      return resolve_field_path(nested, tail, field_offset);
    }

    std::cerr << "[ROS2 WARN] '" << head << "' is not a nested message; "
                  "cannot resolve sub-path '" << tail << "'.\n";
    return {};
  }
  return {};
}

// Write a double value into a raw message buffer, casting to the field's type.
static void write_numeric(uint8_t* base, size_t offset, uint8_t type_id, double v) {
  uint8_t* ptr = base + offset;
  switch (type_id) {
    case intr::ROS_TYPE_FLOAT:   *reinterpret_cast<float*>   (ptr) = static_cast<float>(v);    break;
    case intr::ROS_TYPE_DOUBLE:  *reinterpret_cast<double*>  (ptr) = v;                        break;
    case intr::ROS_TYPE_BOOLEAN: *reinterpret_cast<bool*>    (ptr) = static_cast<bool>(v);     break;
    case intr::ROS_TYPE_INT8:    *reinterpret_cast<int8_t*>  (ptr) = static_cast<int8_t>(v);   break;
    case intr::ROS_TYPE_UINT8:   *reinterpret_cast<uint8_t*> (ptr) = static_cast<uint8_t>(v);  break;
    case intr::ROS_TYPE_INT16:   *reinterpret_cast<int16_t*> (ptr) = static_cast<int16_t>(v);  break;
    case intr::ROS_TYPE_UINT16:  *reinterpret_cast<uint16_t*>(ptr) = static_cast<uint16_t>(v); break;
    case intr::ROS_TYPE_INT32:   *reinterpret_cast<int32_t*> (ptr) = static_cast<int32_t>(v);  break;
    case intr::ROS_TYPE_UINT32:  *reinterpret_cast<uint32_t*>(ptr) = static_cast<uint32_t>(v); break;
    case intr::ROS_TYPE_INT64:   *reinterpret_cast<int64_t*> (ptr) = static_cast<int64_t>(v);  break;
    case intr::ROS_TYPE_UINT64:  *reinterpret_cast<uint64_t*>(ptr) = static_cast<uint64_t>(v); break;
    default: break;
  }
}

// Write a string into a raw message buffer at the given offset.
// The field in the compiled struct is a std::string, so we assign through a pointer.
static void write_string(uint8_t* base, size_t offset, const std::string& value) {
  *reinterpret_cast<std::string*>(base + offset) = value;
}

// C++17-compatible detection of a message type having a 'header' member.
template<typename T, typename = void>
struct has_header : std::false_type {};
template<typename T>
struct has_header<T, std::void_t<decltype(std::declval<T&>().header)>> : std::true_type {};

/* Generic factory for any compiled beamng_msgs message type.
  
  At init time: retrieves rosidl introspection metadata, resolves each field from the
  JSON sensor_config to a byte offset and type id, and builds a FieldWriter vector with integer indices into ScalarStore/StringStore.
  
  At publish time: iterates the FieldWriters using direct array reads (no string hashing),
  writes values into the message struct, and publishes.
*/
template<typename MessageType>
static ScalarRegisterResult make_generic_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config,
    uint32_t scalar_base_idx,
    uint32_t string_base_idx)
{
  const rosidl_message_type_support_t* type_support = rosidl_typesupport_cpp::get_message_type_support_handle<MessageType>();
  const rosidl_message_type_support_t* introspection_type_support = get_message_typesupport_handle(type_support, intr::typesupport_identifier);
  const auto* members = reinterpret_cast<const intr::MessageMembers*>(introspection_type_support->data);

  std::vector<FieldWriter> field_writers;
  field_writers.reserve(sensor_config.fields.size());

  uint32_t scalar_local_idx = 0;
  uint32_t string_local_idx = 0;

  for (const auto& field : sensor_config.fields) {
    auto result = resolve_field_path(members, field.name);

    if (!result.found) {
      std::cerr << "[ROS2 WARN] " << sensor_config.msg_file << ": field '"
                << field.name << "' not found in introspection, skipping.\n";
      continue;
    }

    const bool is_str = (result.type_id == intr::ROS_TYPE_STRING);
    const uint32_t idx = is_str
        ? (string_base_idx + string_local_idx++)
        : (scalar_base_idx + scalar_local_idx++);

    field_writers.push_back({idx, result.offset, result.type_id, is_str});
  }

  auto publisher = node.create_publisher<MessageType>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  // Capture the expected store sizes so the publish-time bounds checks are simple integer comparisons against a captured constant,
  // not a virtual call to vector::size() inside the hot loop.
  const uint32_t expected_scalar_slots = scalar_base_idx + scalar_local_idx;
  const uint32_t expected_string_slots = string_base_idx + string_local_idx;

  PublishFn publish_fn = [publisher, frame_id, field_writers = std::move(field_writers), expected_scalar_slots, expected_string_slots](
      const ScalarStore& scalars, const StringStore& strings, const rclcpp::Time& stamp)
  {
    MessageType msg;
    if constexpr (has_header<MessageType>::value) {
      msg.header.stamp    = stamp;
      msg.header.frame_id = frame_id;
    }

    if (scalars.size() < expected_scalar_slots || strings.size() < expected_string_slots)
      return;

    auto* base = reinterpret_cast<uint8_t*>(&msg);
    for (const auto& writer : field_writers) {
      if (writer.is_string) {
        write_string(base, writer.offset, strings[writer.value_idx]);
      } else {
        write_numeric(base, writer.offset, writer.type_id, scalars[writer.value_idx]);
      }
    }
    
    publisher->publish(msg);
  };
  
  return {std::move(publish_fn), scalar_local_idx, string_local_idx};
}

// Reads a uint8 from a byte buffer at *pos and advances *pos.
static inline uint8_t read_u8(const uint8_t* data, uint32_t* pos) {
  uint8_t value = data[*pos];
  *pos += 1;
  return value;
}

// Reads a uint16 from a byte buffer at *pos and advances *pos.
static inline uint16_t read_u16(const uint8_t* data, uint32_t* pos) {
  uint16_t value;
  std::memcpy(&value, data + *pos, sizeof(value));
  *pos += sizeof(value);
  return value;
}

// Reads a uint32 from a byte buffer at *pos and advances *pos.
static inline uint32_t read_u32(const uint8_t* data, uint32_t* pos) {
  uint32_t value;
  std::memcpy(&value, data + *pos, sizeof(value));
  *pos += sizeof(value);
  return value;
}

// Reads a int32 from a byte buffer at *pos and advances *pos.
static inline int32_t read_i32(const uint8_t* data, uint32_t* pos) {
  int32_t value;
  std::memcpy(&value, data + *pos, sizeof(value));
  *pos += sizeof(value);
  return value;
}

// Reads a float32 from a byte buffer at *pos and advances *pos.
static inline float read_f32(const uint8_t* data, uint32_t* pos) {
  float value;
  std::memcpy(&value, data + *pos, sizeof(value));
  *pos += sizeof(value);
  return value;
}

// Reads a float64 from a byte buffer at *pos and advances *pos.
static inline double read_f64(const uint8_t* data, uint32_t* pos) {
  double value;
  std::memcpy(&value, data + *pos, sizeof(value));
  *pos += sizeof(value);
  return value;
}

// Reads a length-prefixed string (uint16 len + bytes) at *pos and advances *pos.
static inline std::string read_str(const uint8_t* data, uint32_t* pos) {
  const uint16_t len = read_u16(data, pos);
  std::string value(reinterpret_cast<const char*>(data + *pos), len);
  *pos += len;
  return value;
}

// Decode one length-prefixed parallel array section from the blob into a message.
// Reads [uint32 count] then calls each field_reader count times in order.
template<typename MessageType>
static void decode_section(
    MessageType& msg,
    const std::vector<std::function<void(MessageType&, const uint8_t*, uint32_t*)>>& field_readers,
    const uint8_t* data,
    uint32_t len,
    uint32_t* pos)
{
  if (*pos + sizeof(uint32_t) > len) return;

  const uint32_t count = read_u32(data, pos);

  for (uint32_t i = 0; i < count; ++i) {
    for (const auto& reader : field_readers) {
      if (*pos >= len) return;
      reader(msg, data, pos);
    }
  }
}

// Generic factory for messages with variable-length parallel array fields.
// section_readers: one entry per array section; each entry is a list of per-field reader functions.
// Wire format per section: [uint32 count] then count entries, each field in order:
//   string  → [uint16 len][bytes]
//   float64 → [float64]
template<typename MessageType>
static BlobPublishFn make_dynamic_array_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config,
    std::vector<std::vector<std::function<void(MessageType&, const uint8_t*, uint32_t*)>>> section_readers)
{
  auto publisher = node.create_publisher<MessageType>(topic, 10);
  auto node_ptr = node.shared_from_this();
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  return [publisher, node_ptr, frame_id, section_readers = std::move(section_readers)]
    (const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    MessageType msg;

    if constexpr (has_header<MessageType>::value) {
      msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
      msg.header.frame_id = frame_id;
    }

    uint32_t pos = 0;
    for (const auto& section_reader : section_readers) {
      decode_section(msg, section_reader, data, len, &pos);
    }
    publisher->publish(msg);
  };
}

static BlobPublishFn make_dynamic_array_sensor_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using beamng_msgs::msg::DamageSensor;
  using Msg = DamageSensor;
  using FieldFn = std::function<void(Msg&, const uint8_t*, uint32_t*)>;

  std::vector<std::vector<FieldFn>> readers = {
    {
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.deformgroup_id.push_back(read_str(data, pos)); },
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.inv_max_events.push_back(read_f64(data, pos)); },
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.damage.push_back(read_f64(data, pos)); },
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.event_count.push_back(read_f64(data, pos)); },
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.max_events.push_back(read_f64(data, pos)); },
    },
    {
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.part_id.push_back(read_str(data, pos)); },
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.name.push_back(read_str(data, pos)); },
      [](Msg& msg, const uint8_t* data, uint32_t* pos) { msg.part_damage.push_back(read_f64(data, pos)); },
    },
  };

  return make_dynamic_array_factory<Msg>(node, topic, sensor_config, std::move(readers));
}

// Wire format for PowertrainSensor (virtual blob, vehicle-side encoder):
// [uint32 device_count] then for each device:
//   name (u16 len + bytes), input_av (f64), gear_ratio (f64), is_broken (u8),
//   mode (u16 + bytes), parent_name (u16 + bytes), parent_output_index (i32),
//   output_torque_1, output_av_1, output_torque_2, output_av_2 (f64).
static BlobPublishFn make_powertrain_sensor_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using beamng_msgs::msg::PowertrainSensor;
  using beamng_msgs::msg::PowertrainSensorDevice;

  auto publisher = node.create_publisher<PowertrainSensor>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  return [publisher, frame_id](const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    PowertrainSensor msg;
    msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.header.frame_id = frame_id;

    uint32_t pos = 0;
    if (pos + sizeof(uint32_t) > len) return;
    const uint32_t count = read_u32(data, &pos);

    for (uint32_t i = 0; i < count && pos < len; ++i) {
      PowertrainSensorDevice dev;
      dev.name                = read_str(data, &pos);
      dev.input_av            = read_f64(data, &pos);
      dev.gear_ratio          = read_f64(data, &pos);
      dev.is_broken           = (read_u8(data, &pos) != 0);
      dev.mode                = read_str(data, &pos);
      dev.parent_name         = read_str(data, &pos);
      dev.parent_output_index = read_i32(data, &pos);
      dev.output_torque_1     = read_f64(data, &pos);
      dev.output_av_1         = read_f64(data, &pos);
      dev.output_torque_2     = read_f64(data, &pos);
      dev.output_av_2         = read_f64(data, &pos);
      msg.devices.push_back(dev);
    }
    publisher->publish(msg);
  };
}

static BlobPublishFn make_pointcloud2_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  auto publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

  // Build a template message with all fixed fields set once at init time.
  sensor_msgs::msg::PointCloud2 template_msg;
  template_msg.header.frame_id = config_frame_id(node, sensor_config.frame_id);
  template_msg.height          = 1;      // unorganised point cloud
  template_msg.is_bigendian    = false;
  template_msg.is_dense        = false;

  const uint32_t stride = sensor_config.blob_layout.point_stride > 0
      ? sensor_config.blob_layout.point_stride : 12u;  // default: xyz float32 = 12 bytes

  template_msg.point_step = stride;

  // "xyz_float32" → three float32 fields at byte offsets 0, 4, 8.
  if (sensor_config.blob_layout.format == "xyz_float32" || sensor_config.blob_layout.format.empty()) {
    sensor_msgs::msg::PointField fx, fy, fz;
    fx.name = "x"; fx.offset = 0; fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4; fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8; fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
    template_msg.fields = {fx, fy, fz};
  }

  // Pre-size the data buffer to max capacity so assign() never reallocates at publish time.
  const uint32_t max_bytes = sensor_config.blob_layout.max_bytes;
  if (max_bytes > 0) {
    template_msg.data.resize(max_bytes);
  }

  // Capture msg (not template_msg) directly so the lambda reuses the same allocation every call.
  // Only the per-frame mutable fields (stamp, width, row_step, data) are updated;
  // the fixed fields (frame_id, height, is_bigendian, is_dense, point_step, fields) are set once here and never touched again.
  return [publisher, msg = std::move(template_msg), stride]
    (const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.width        = (stride > 0) ? (len / stride) : 0;
    msg.row_step     = len;
    msg.data.assign(data, data + len);
    publisher->publish(msg);
  };
}

// Wire format from Research.Radar.getLastReadingsBuffer:
// flat array of returns, each return is 7 consecutive float32 values:
//   [0] range                (float32)
//   [1] doppler_velocity     (float32)
//   [2] azimuth              (float32)
//   [3] elevation            (float32)
//   [4] radar_cross_section  (float32)
//   [5] signal_to_noise_ratio(float32)
//   [6] facing_factor        (float32)
// Total: 28 bytes per return. Count = len / 28.
static BlobPublishFn make_radar_scan_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using beamng_msgs::msg::RadarScan;
  using beamng_msgs::msg::RadarReturn;

  auto publisher = node.create_publisher<RadarScan>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  static constexpr uint32_t RETURN_STRIDE = 7 * sizeof(float);  // 28 bytes
  static_assert(sizeof(RadarReturn) == RETURN_STRIDE, "RadarReturn must match wire layout (7xfloat32, no padding) for bulk memcpy");
  static_assert(std::is_trivially_copyable_v<RadarReturn>, "RadarReturn must be memcpy-safe");

  // Pre-size the sequence once, publish path only adjusts length and copies wire bytes.
  const uint32_t max_returns = sensor_config.blob_layout.max_bytes > 0
      ? sensor_config.blob_layout.max_bytes / RETURN_STRIDE : 1000u;

  RadarScan template_msg;
  template_msg.header.frame_id = frame_id;
  template_msg.returns.reserve(max_returns);

  return [publisher, msg = std::move(template_msg), max_returns](const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    const uint32_t count = len / RETURN_STRIDE;
    if (count == 0) {
      return;
    }
    const uint32_t use_count = (count > max_returns) ? max_returns : count;

    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.returns.resize(use_count);
    std::memcpy(msg.returns.data(), data, static_cast<size_t>(use_count) * RETURN_STRIDE);

    publisher->publish(msg);
  };
}

// Wire format for AdvancedIMU (virtual blob, vehicle-side encoder):
// Fixed 80-byte payload (10 × float64):
//   [0..2]  angVelSmooth.xyz  (3×f64, 24 bytes)
//   [3..5]  accSmooth.xyz     (3×f64, 24 bytes)
//   [6..9]  orientation xyzw  (4×f64, 32 bytes) -- quaternion computed in Lua
// All covariance fields are set to -1 (unknown).
static BlobPublishFn make_imu_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using sensor_msgs::msg::Imu;

  auto publisher = node.create_publisher<Imu>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  static constexpr uint32_t PAYLOAD_SIZE = 10 * sizeof(double);  // 80 bytes

  Imu template_msg;
  template_msg.header.frame_id = frame_id;
  template_msg.orientation_covariance[0]         = -1.0;
  template_msg.angular_velocity_covariance[0]    = -1.0;
  template_msg.linear_acceleration_covariance[0] = -1.0;

  return [publisher, msg = std::move(template_msg)](const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    double values[10]; std::memcpy(values, data, sizeof(values));
    msg.header.stamp          = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.angular_velocity.x    = values[0];
    msg.angular_velocity.y    = values[1];
    msg.angular_velocity.z    = values[2];
    msg.linear_acceleration.x = values[3];
    msg.linear_acceleration.y = values[4];
    msg.linear_acceleration.z = values[5];
    msg.orientation.x         = values[6];
    msg.orientation.y         = values[7];
    msg.orientation.z         = values[8];
    msg.orientation.w         = values[9];

    publisher->publish(msg);
  };
}

// Wire format for RoadsSensor (virtual blob, vehicle-side encoder):
// Fixed 432-byte payload (54 × float64), order matches RoadsSensor.msg:
//   [0..5]   dist2_cl, dist2_left, dist2_right, half_width, road_radius, heading_angle
//   [6..17]  p0_on_cl..p3_on_cl (4× Point xyz)
//   [18..25] u_cl, v_cl (2× CubicPolynomial abcd)
//   [26..41] u_left_re, v_left_re, u_right_re, v_right_re (4× abcd)
//   [42..50] start_cl, start_l, start_r (3× Point xyz)
//   [51..53] drivability, speed_limit, flag1way
static BlobPublishFn make_roads_sensor_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using beamng_msgs::msg::RoadsSensor;

  auto publisher = node.create_publisher<RoadsSensor>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  static constexpr uint32_t PAYLOAD_SIZE = 54 * sizeof(double);  // 432 bytes

  RoadsSensor template_msg;
  template_msg.header.frame_id = frame_id;

  return [publisher, msg = std::move(template_msg)](
      const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    const double* values = reinterpret_cast<const double*>(data);
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));

    msg.dist2_cl      = values[0];
    msg.dist2_left    = values[1];
    msg.dist2_right   = values[2];
    msg.half_width   = values[3];
    msg.road_radius  = values[4];
    msg.heading_angle = values[5];

    msg.p0_on_cl.x = values[6];  msg.p0_on_cl.y = values[7];  msg.p0_on_cl.z = values[8];
    msg.p1_on_cl.x = values[9];  msg.p1_on_cl.y = values[10]; msg.p1_on_cl.z = values[11];
    msg.p2_on_cl.x = values[12]; msg.p2_on_cl.y = values[13]; msg.p2_on_cl.z = values[14];
    msg.p3_on_cl.x = values[15]; msg.p3_on_cl.y = values[16]; msg.p3_on_cl.z = values[17];

    msg.u_cl.a = values[18]; msg.u_cl.b = values[19]; msg.u_cl.c = values[20]; msg.u_cl.d = values[21];
    msg.v_cl.a = values[22]; msg.v_cl.b = values[23]; msg.v_cl.c = values[24]; msg.v_cl.d = values[25];

    msg.u_left_re.a = values[26];  msg.u_left_re.b = values[27];  msg.u_left_re.c = values[28];  msg.u_left_re.d = values[29];
    msg.v_left_re.a = values[30];  msg.v_left_re.b = values[31];  msg.v_left_re.c = values[32];  msg.v_left_re.d = values[33];
    msg.u_right_re.a = values[34]; msg.u_right_re.b = values[35]; msg.u_right_re.c = values[36]; msg.u_right_re.d = values[37];
    msg.v_right_re.a = values[38]; msg.v_right_re.b = values[39]; msg.v_right_re.c = values[40]; msg.v_right_re.d = values[41];

    msg.start_cl.x = values[42]; msg.start_cl.y = values[43]; msg.start_cl.z = values[44];
    msg.start_l.x  = values[45]; msg.start_l.y  = values[46]; msg.start_l.z  = values[47];
    msg.start_r.x  = values[48]; msg.start_r.y  = values[49]; msg.start_r.z  = values[50];

    msg.drivability = values[51];
    msg.speed_limit = values[52];
    msg.flag1way    = values[53];

    publisher->publish(msg);
  };
}

// Wire format for IdealRadar (virtual blob, vehicle-side encoder):
// Fixed payload of 4 vehicle entries, each 124 bytes:
//   [0]  vehicle_id              (int32,   4 bytes)
//   [1]  width                   (float64, 8 bytes)
//   [2]  length                  (float64, 8 bytes)
//   [3]  dist_to_player_vehicle_sq (float64, 8 bytes)
//   [4]  rel_dist_x              (float64, 8 bytes)
//   [5]  rel_dist_y              (float64, 8 bytes)
//   [6]  vel_bb.x                (float64, 8 bytes)
//   [7]  vel_bb.y                (float64, 8 bytes)
//   [8]  vel_bb.z                (float64, 8 bytes)
//   [9]  rel_vel_x               (float64, 8 bytes)
//   [10] rel_vel_y               (float64, 8 bytes)
//   [11] acc.x                   (float64, 8 bytes)
//   [12] acc.y                   (float64, 8 bytes)
//   [13] acc.z                   (float64, 8 bytes)
//   [14] rel_acc_x               (float64, 8 bytes)
//   [15] rel_acc_y               (float64, 8 bytes)
// Total: 4 + 15*8 = 124 bytes/vehicle, 496 bytes total.
static BlobPublishFn make_ideal_radar_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using beamng_msgs::msg::IdealRadarSensor;
  using beamng_msgs::msg::IdealRadarSensorVehicle;

  auto publisher = node.create_publisher<IdealRadarSensor>(topic, 10);
  auto node_ptr = node.shared_from_this();
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  static constexpr uint32_t VEHICLE_STRIDE = 4 + 15 * sizeof(double);  // 124 bytes
  static constexpr uint32_t PAYLOAD_SIZE   = 4 * VEHICLE_STRIDE;        // 496 bytes

  IdealRadarSensor template_msg;
  template_msg.header.frame_id = frame_id;

  return [publisher, node_ptr, msg = std::move(template_msg)](
      const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));

    auto decode_vehicle = [&](uint32_t offset) -> IdealRadarSensorVehicle {
      IdealRadarSensorVehicle v;
      int32_t vehicle_id; std::memcpy(&vehicle_id, data + offset, 4); offset += 4;
      double values[15]; std::memcpy(values, data + offset, 15 * sizeof(double));
      v.vehicle_id                = vehicle_id;
      v.width                     = values[0];
      v.length                    = values[1];
      v.dist_to_player_vehicle_sq = values[2];
      v.rel_dist_x                = values[3];
      v.rel_dist_y                = values[4];
      v.vel_bb.x                  = values[5];
      v.vel_bb.y                  = values[6];
      v.vel_bb.z                  = values[7];
      v.rel_vel_x                 = values[8];
      v.rel_vel_y                 = values[9];
      v.acc.x                     = values[10];
      v.acc.y                     = values[11];
      v.acc.z                     = values[12];
      v.rel_acc_x                 = values[13];
      v.rel_acc_y                 = values[14];
      return v;
    };

    msg.closest_vehicles1 = decode_vehicle(0 * VEHICLE_STRIDE);
    msg.closest_vehicles2 = decode_vehicle(1 * VEHICLE_STRIDE);
    msg.closest_vehicles3 = decode_vehicle(2 * VEHICLE_STRIDE);
    msg.closest_vehicles4 = decode_vehicle(3 * VEHICLE_STRIDE);

    publisher->publish(msg);
  };
}

// Wire format for GPS (virtual blob, vehicle-side encoder):
//   [0] latitude   (float64)
//   [1] longitude  (float64)
//   [2] altitude   (float64, NaN when unavailable)
// Total: 24 bytes, always fixed-size.
static BlobPublishFn make_nav_sat_fix_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using sensor_msgs::msg::NavSatFix;
  using sensor_msgs::msg::NavSatStatus;

  auto publisher = node.create_publisher<NavSatFix>(topic, 10);
  auto node_ptr = node.shared_from_this();
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  static constexpr uint32_t PAYLOAD_SIZE = 3 * sizeof(double);  // 24 bytes

  NavSatFix template_msg;
  template_msg.header.frame_id               = frame_id;
  template_msg.status.status                 = NavSatStatus::STATUS_FIX;
  template_msg.status.service                = NavSatStatus::SERVICE_GPS;
  template_msg.position_covariance_type      = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  template_msg.position_covariance           = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  return [publisher, node_ptr, msg = std::move(template_msg)](
      const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    double vals[3];
    std::memcpy(vals, data, PAYLOAD_SIZE);

    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.latitude     = vals[0];
    msg.longitude    = vals[1];
    msg.altitude     = vals[2];

    publisher->publish(msg);
  };
}

// Wire format for Ultrasonic (Range): produced in GE Lua from getUltrasonicReadings.
// Fixed 17-byte payload: radiation_type (uint8), field_of_view (float), min_range (float), max_range (float), range (float).
static BlobPublishFn make_range_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using sensor_msgs::msg::Range;

  auto publisher = node.create_publisher<Range>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  static constexpr uint32_t PAYLOAD_SIZE = 1 + 4 * sizeof(float);  // 17 bytes

  Range template_msg;
  template_msg.header.frame_id   = frame_id;
  template_msg.radiation_type    = Range::ULTRASOUND;
  template_msg.field_of_view    = 0.1f;
  template_msg.min_range        = 0.0f;
  template_msg.max_range        = 0.0f;
  template_msg.range            = 0.0f;

  return [publisher, msg = std::move(template_msg)](
      const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    uint32_t pos = 0;
    msg.header.stamp     = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.radiation_type   = data[pos]; pos += 1;
    msg.field_of_view    = read_f32(data, &pos);
    msg.min_range        = read_f32(data, &pos);
    msg.max_range        = read_f32(data, &pos);
    msg.range            = read_f32(data, &pos);

    publisher->publish(msg);
  };
}

static uint32_t image_row_step_bytes(const std::string& encoding, uint32_t width) {
  if (encoding == "rgb8" || encoding == "bgr8") return width * 3u;
  if (encoding == "32FC1" || encoding == "32SC1" || encoding == "rgba8") return width * 4u;
  if (encoding == "mono8" || encoding == "8UC1") return width;
  return width * 3u;
}

// Payload is already in ROS row layout; VehicleNode converts engine shared_memory where needed.
BlobPublishFn make_camera_sensor_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config,
    const std::string& encoding,
    uint32_t width,
    uint32_t height)
{
  using sensor_msgs::msg::Image;

  auto publisher = node.create_publisher<Image>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);

  const uint32_t step = image_row_step_bytes(encoding, width);
  const uint32_t max_bytes = step * height;

  Image template_msg;
  template_msg.header.frame_id = frame_id;
  template_msg.height          = height;
  template_msg.width           = width;
  template_msg.encoding        = encoding;
  template_msg.is_bigendian    = 0;
  template_msg.step            = step;
  template_msg.data.resize(max_bytes);

  return [publisher, msg = std::move(template_msg)](
      const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    if (len == 0) return;

    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    
    if (msg.data.size() != len) msg.data.resize(len);
    std::memcpy(msg.data.data(), data, len);
    publisher->publish(msg);
  };
}

static BlobPublishFn make_mesh_sensor_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_config)
{
  using beamng_msgs::msg::MeshSensor;
  using beamng_msgs::msg::MeshSensorNode;
  using beamng_msgs::msg::MeshSensorBeam;

  auto publisher = node.create_publisher<MeshSensor>(topic, 10);
  const std::string frame_id = config_frame_id(node, sensor_config.frame_id);
  // Reuse one message to avoid per-publish allocations (nodes/beams vectors keep capacity after clear).
  MeshSensor msg;

  return [publisher, frame_id, msg](const uint8_t* data, uint32_t len, uint64_t timestamp_ns) mutable
  {
    msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.header.frame_id = frame_id;
    msg.nodes.clear();
    msg.beams.clear();

    uint32_t pos = 0;
    if (pos + sizeof(uint32_t) > len) return;

    const uint32_t node_count = read_u32(data, &pos);
    msg.nodes.reserve(node_count);

    for (uint32_t i = 0; i < node_count && pos + 80 <= len; ++i) {
      MeshSensorNode n;
      const double* values = reinterpret_cast<const double*>(data + pos);
      n.pos.x   = values[0]; n.pos.y   = values[1]; n.pos.z   = values[2];
      n.vel.x   = values[3]; n.vel.y   = values[4]; n.vel.z   = values[5];
      n.force.x = values[6]; n.force.y = values[7]; n.force.z = values[8];
      n.mass    = values[9];
      pos += 80; // 80 bytes per node
      n.part_origin = read_str(data, &pos);
      msg.nodes.push_back(std::move(n));
    }

    if (pos + sizeof(uint32_t) > len) {
      publisher->publish(msg);
      return;
    }
    
    const uint32_t beam_count = read_u32(data, &pos);
    msg.beams.reserve(beam_count);
    if (pos + beam_count * sizeof(double) > len) {
      publisher->publish(msg); return;
    }
    
    const double* beam_stresses = reinterpret_cast<const double*>(data + pos);
    
    for (uint32_t i = 0; i < beam_count; ++i) {
      MeshSensorBeam b;
      b.stress = beam_stresses[i];
      msg.beams.push_back(b);
    }
    pos += beam_count * sizeof(double);

    publisher->publish(msg);
  };
}

void register_all_sensor_factories() {
  using namespace beamng_msgs::msg;
  auto& reg = SensorFactoryRegistry::instance();

  // Scalar factories - one entry per beamng_msgs message type.
  #define REGISTER(MessageTypeype) \
    reg.register_factory(#MessageTypeype, \
      [](rclcpp::Node& node, const std::string& topic, \
          const SensorConfig& sensor_config, \
          uint32_t scalar_base_idx, uint32_t string_base_idx) -> ScalarRegisterResult { \
        return make_generic_factory<MessageTypeype>( \
            node, topic, sensor_config, scalar_base_idx, string_base_idx); \
      })

  REGISTER(ElectricsSensor);
  REGISTER(GForceSensor);
  REGISTER(StateSensor);
  REGISTER(TimeSensor);

  #undef REGISTER

  reg.register_blob_factory("DamageSensor",    make_dynamic_array_sensor_factory);
  reg.register_blob_factory("Imu",             make_imu_factory);
  reg.register_blob_factory("PointCloud2",     make_pointcloud2_factory);
  reg.register_blob_factory("RadarScan",       make_radar_scan_factory);
  reg.register_blob_factory("NavSatFix",       make_nav_sat_fix_factory);
  reg.register_blob_factory("RoadsSensor",       make_roads_sensor_factory);
  reg.register_blob_factory("PowertrainSensor",  make_powertrain_sensor_factory);
  reg.register_blob_factory("IdealRadarSensor",  make_ideal_radar_factory);
  reg.register_blob_factory("MeshSensor",      make_mesh_sensor_factory);
  reg.register_blob_factory("Range",           make_range_factory);
  // Image (camera): created in VehicleNode::init_sensor_publishers : multi-stream + shared_memory.
}

}  // namespace bng_ros2
