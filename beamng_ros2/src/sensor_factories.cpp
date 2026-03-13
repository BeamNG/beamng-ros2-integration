#include "sensor_factories.h"
#include "sensor_registry.h"
#include <beamng_msgs/msg/electrics_sensor.hpp>
#include <beamng_msgs/msg/g_force_sensor.hpp>
#include <beamng_msgs/msg/state_sensor.hpp>
#include <beamng_msgs/msg/damage_sensor.hpp>
#include <beamng_msgs/msg/time_sensor.hpp>
#include <beamng_msgs/msg/radar_scan.hpp>
#include <beamng_msgs/msg/ideal_radar_sensor.hpp>
#include <beamng_msgs/msg/roads_sensor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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
#include <cmath>

namespace bng_ros2 {

namespace intr = rosidl_typesupport_introspection_cpp;


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
    const auto& m = members->members_[i];
    if (head != m.name_) continue;

    size_t field_offset = base_offset + m.offset_;

    if (tail.empty()) {
      return {field_offset, m.type_id_, true};
    }
    if (m.type_id_ == intr::ROS_TYPE_MESSAGE && m.members_ != nullptr) {
      const auto* nested = reinterpret_cast<const intr::MessageMembers*>(m.members_->data);
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
static void write_string(uint8_t* base, size_t offset, const std::string& v) {
  *reinterpret_cast<std::string*>(base + offset) = v;
}

// C++17 detection trait: does MsgT have a 'header' member?
template<typename T, typename = void>
struct has_header : std::false_type {};
template<typename T>
struct has_header<T, std::void_t<decltype(std::declval<T&>().header)>> : std::true_type {};

// Generic factory for any compiled beamng_msgs message type.
//
// At init time: retrieves rosidl introspection metadata, resolves each field from the
// JSON config to a byte offset and type id, and builds a FieldWriter vector with integer indices into ScalarStore/StringStore.
//
// At publish time: iterates the FieldWriters using direct array reads (no string hashing),
// writes values into the message struct, and publishes.
template<typename MsgT>
static ScalarRegisterResult make_generic_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& sensor_cfg,
    uint32_t scalar_base_idx,
    uint32_t string_base_idx)
{
  const rosidl_message_type_support_t* ts =
      rosidl_typesupport_cpp::get_message_type_support_handle<MsgT>();
  const rosidl_message_type_support_t* intr_ts =
      get_message_typesupport_handle(ts, intr::typesupport_identifier);
  const auto* members = reinterpret_cast<const intr::MessageMembers*>(intr_ts->data);

  std::vector<FieldWriter> writers;
  writers.reserve(sensor_cfg.fields.size());

  uint32_t scalar_local_idx = 0;
  uint32_t string_local_idx = 0;

  for (const auto& field : sensor_cfg.fields) {
    auto res = resolve_field_path(members, field.name);
    if (!res.found) {
      std::cerr << "[ROS2 WARN] " << sensor_cfg.msg_file << ": field '"
                << field.name << "' not found in introspection, skipping.\n";
      continue;
    }
    const bool is_str = (res.type_id == intr::ROS_TYPE_STRING);
    const uint32_t idx = is_str
        ? (string_base_idx + string_local_idx++)
        : (scalar_base_idx + scalar_local_idx++);

    writers.push_back({idx, res.offset, res.type_id, is_str});
  }

  auto pub = node.create_publisher<MsgT>(topic, 10);
  // Capture node as shared_ptr so the lambda never holds a dangling reference.
  auto node_ptr  = node.shared_from_this();
  const std::string frame_id = sensor_cfg.frame_id.empty()
      ? std::string(node.get_name()) : sensor_cfg.frame_id;

  // Capture the expected store sizes so the publish-time bounds checks are simple integer comparisons against a captured constant,
  // not a virtual call to vector::size() inside the hot loop.
  const uint32_t expected_scalar_slots = scalar_base_idx + scalar_local_idx;
  const uint32_t expected_string_slots = string_base_idx + string_local_idx;

  PublishFn fn = [pub, node_ptr, frame_id, writers = std::move(writers),
                  expected_scalar_slots, expected_string_slots](
      const ScalarStore& scalars, const StringStore& strings)
  {
    MsgT msg;
    if constexpr (has_header<MsgT>::value) {
      msg.header.stamp    = node_ptr->get_clock()->now();
      msg.header.frame_id = frame_id;
    }

    if (scalars.size() < expected_scalar_slots || strings.size() < expected_string_slots)
      return;

    auto* base = reinterpret_cast<uint8_t*>(&msg);
    for (const auto& w : writers) {
      if (w.is_string) {
        write_string(base, w.offset, strings[w.value_idx]);
      } else {
        write_numeric(base, w.offset, w.type_id, scalars[w.value_idx]);
      }
    }

    pub->publish(msg);
  };

  return {std::move(fn), scalar_local_idx, string_local_idx};
}

// Reads a uint16 from a byte buffer at *pos and advances *pos.
static inline uint16_t read_u16(const uint8_t* data, uint32_t* pos) {
  uint16_t v;
  std::memcpy(&v, data + *pos, sizeof(v));
  *pos += sizeof(v);
  return v;
}

// Reads a uint32 from a byte buffer at *pos and advances *pos.
static inline uint32_t read_u32(const uint8_t* data, uint32_t* pos) {
  uint32_t v;
  std::memcpy(&v, data + *pos, sizeof(v));
  *pos += sizeof(v);
  return v;
}

// Reads a float64 from a byte buffer at *pos and advances *pos.
static inline double read_f64(const uint8_t* data, uint32_t* pos) {
  double v;
  std::memcpy(&v, data + *pos, sizeof(v));
  *pos += sizeof(v);
  return v;
}

// Reads a length-prefixed string (uint16 len + bytes) at *pos and advances *pos.
static inline std::string read_str(const uint8_t* data, uint32_t* pos) {
  const uint16_t len = read_u16(data, pos);
  std::string s(reinterpret_cast<const char*>(data + *pos), len);
  *pos += len;
  return s;
}

// Decode one length-prefixed parallel array section from the blob into a message.
// Reads [uint32 count] then calls each field_reader count times in order.
template<typename MsgT>
static void decode_section(
    MsgT& msg,
    const std::vector<std::function<void(MsgT&, const uint8_t*, uint32_t*)>>& field_readers,
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
template<typename MsgT>
static BlobPublishFn make_dynamic_array_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& cfg,
    std::vector<std::vector<std::function<void(MsgT&, const uint8_t*, uint32_t*)>>> section_readers)
{
  auto pub      = node.create_publisher<MsgT>(topic, 10);
  auto node_ptr = node.shared_from_this();
  const std::string frame_id = cfg.frame_id.empty() ? std::string(node.get_name()) : cfg.frame_id;

  return [pub, node_ptr, frame_id,
          readers = std::move(section_readers)](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    MsgT msg;
    if constexpr (has_header<MsgT>::value) {
      msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(ts_ns));
      msg.header.frame_id = frame_id;
    }
    uint32_t pos = 0;
    for (const auto& section_readers_i : readers) {
      decode_section(msg, section_readers_i, data, len, &pos);
    }
    pub->publish(msg);
  };
}

static BlobPublishFn make_dynamic_array_sensor_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& cfg)
{
  using beamng_msgs::msg::DamageSensor;
  using Msg = DamageSensor;
  using FieldFn = std::function<void(Msg&, const uint8_t*, uint32_t*)>;

  std::vector<std::vector<FieldFn>> readers = {
    {
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.deformgroup_id.push_back(read_str(d, p)); },
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.inv_max_events.push_back(read_f64(d, p)); },
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.damage.push_back(read_f64(d, p)); },
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.event_count.push_back(read_f64(d, p)); },
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.max_events.push_back(read_f64(d, p)); },
    },
    {
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.part_id.push_back(read_str(d, p)); },
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.name.push_back(read_str(d, p)); },
      [](Msg& m, const uint8_t* d, uint32_t* p) { m.part_damage.push_back(read_f64(d, p)); },
    },
  };

  return make_dynamic_array_factory<Msg>(node, topic, cfg, std::move(readers));
}

static BlobPublishFn make_pointcloud2_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& cfg)
{
  auto pub = node.create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

  // Build a template message with all fixed fields set once at init time.
  sensor_msgs::msg::PointCloud2 tmpl;
  tmpl.header.frame_id = cfg.frame_id.empty() ? node.get_name() : cfg.frame_id;
  tmpl.height          = 1;      // unorganised point cloud
  tmpl.is_bigendian    = false;
  tmpl.is_dense        = false;

  const uint32_t stride = cfg.blob_layout.point_stride > 0
      ? cfg.blob_layout.point_stride : 12u;  // default: xyz float32 = 12 bytes

  tmpl.point_step = stride;

  // "xyz_float32" → three float32 fields at byte offsets 0, 4, 8.
  if (cfg.blob_layout.format == "xyz_float32" || cfg.blob_layout.format.empty()) {
    sensor_msgs::msg::PointField fx, fy, fz;
    fx.name = "x"; fx.offset = 0;  fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4;  fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8;  fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
    tmpl.fields = {fx, fy, fz};
  }

  // Pre-size the data buffer to max capacity so assign() never reallocates at publish time.
  const uint32_t max_bytes = cfg.blob_layout.max_bytes;
  if (max_bytes > 0) {
    tmpl.data.resize(max_bytes);
  }

  // Capture msg (not tmpl) directly so the lambda reuses the same allocation every call.
  // Only the per-frame mutable fields (stamp, width, row_step, data) are updated;
  // the fixed fields (frame_id, height, is_bigendian, is_dense, point_step, fields) are set once here and never touched again.
  return [pub, msg = std::move(tmpl), stride](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ts_ns));
    msg.width        = (stride > 0) ? (len / stride) : 0;
    msg.row_step     = len;
    // assign() writes exactly len bytes into the pre-sized buffer - no reallocation.
    msg.data.assign(data, data + len);
    pub->publish(msg);
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
    const SensorConfig& cfg)
{
  using beamng_msgs::msg::RadarScan;
  using beamng_msgs::msg::RadarReturn;

  auto pub      = node.create_publisher<RadarScan>(topic, 10);
  auto node_ptr = node.shared_from_this();
  const std::string frame_id = cfg.frame_id.empty() ? std::string(node.get_name()) : cfg.frame_id;

  static constexpr uint32_t RETURN_STRIDE = 7 * sizeof(float);  // 28 bytes

  // Pre-allocate the message and size the returns vector to max capacity once.
  // On each publish we only resize to the actual count (no-op if unchanged) and memcpy directly into each RadarReturn, avoiding per-call heap allocation.
  const uint32_t max_returns = cfg.blob_layout.max_bytes > 0
      ? cfg.blob_layout.max_bytes / RETURN_STRIDE : 1000u;

  RadarScan tmpl;
  tmpl.header.frame_id = frame_id;
  tmpl.returns.reserve(max_returns);

  return [pub, node_ptr, msg = std::move(tmpl)](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    const uint32_t count = len / RETURN_STRIDE;

    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ts_ns));
    msg.returns.resize(count);

    // Cast the wire buffer directly into each RadarReturn's fields.
    // RadarReturn contains 7 consecutive float32 members which are laid out identically to the wire format, so a single memcpy per return suffices.
    const float* src = reinterpret_cast<const float*>(data);
    for (uint32_t i = 0; i < count; ++i) {
      auto& r = msg.returns[i];
      const float* p = src + i * 7;
      r.range                 = p[0];
      r.doppler_velocity      = p[1];
      r.azimuth               = p[2];
      r.elevation             = p[3];
      r.radar_cross_section   = p[4];
      r.signal_to_noise_ratio = p[5];
      r.facing_factor         = p[6];
    }

    pub->publish(msg);
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
    const SensorConfig& cfg)
{
  using sensor_msgs::msg::Imu;

  auto pub      = node.create_publisher<Imu>(topic, 10);
  const std::string frame_id = cfg.frame_id.empty() ? std::string(node.get_name()) : cfg.frame_id;

  static constexpr uint32_t PAYLOAD_SIZE = 10 * sizeof(double);  // 80 bytes

  Imu tmpl;
  tmpl.header.frame_id = frame_id;
  tmpl.orientation_covariance[0]         = -1.0;
  tmpl.angular_velocity_covariance[0]    = -1.0;
  tmpl.linear_acceleration_covariance[0] = -1.0;

  return [pub, msg = std::move(tmpl)](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    double f[10]; std::memcpy(f, data, sizeof(f));
    msg.header.stamp          = rclcpp::Time(static_cast<int64_t>(ts_ns));
    msg.angular_velocity.x    = f[0];
    msg.angular_velocity.y    = f[1];
    msg.angular_velocity.z    = f[2];
    msg.linear_acceleration.x = f[3];
    msg.linear_acceleration.y = f[4];
    msg.linear_acceleration.z = f[5];
    msg.orientation.x         = f[6];
    msg.orientation.y         = f[7];
    msg.orientation.z         = f[8];
    msg.orientation.w         = f[9];

    pub->publish(msg);
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
    const SensorConfig& cfg)
{
  using beamng_msgs::msg::RoadsSensor;

  auto pub = node.create_publisher<RoadsSensor>(topic, 10);
  const std::string frame_id = cfg.frame_id.empty() ? std::string(node.get_name()) : cfg.frame_id;

  static constexpr uint32_t PAYLOAD_SIZE = 54 * sizeof(double);  // 432 bytes

  RoadsSensor tmpl;
  tmpl.header.frame_id = frame_id;

  return [pub, msg = std::move(tmpl)](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    const double* f = reinterpret_cast<const double*>(data);
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ts_ns));

    msg.dist2_cl      = f[0];
    msg.dist2_left    = f[1];
    msg.dist2_right   = f[2];
    msg.half_width   = f[3];
    msg.road_radius  = f[4];
    msg.heading_angle = f[5];

    msg.p0_on_cl.x = f[6];  msg.p0_on_cl.y = f[7];  msg.p0_on_cl.z = f[8];
    msg.p1_on_cl.x = f[9];  msg.p1_on_cl.y = f[10]; msg.p1_on_cl.z = f[11];
    msg.p2_on_cl.x = f[12]; msg.p2_on_cl.y = f[13]; msg.p2_on_cl.z = f[14];
    msg.p3_on_cl.x = f[15]; msg.p3_on_cl.y = f[16]; msg.p3_on_cl.z = f[17];

    msg.u_cl.a = f[18]; msg.u_cl.b = f[19]; msg.u_cl.c = f[20]; msg.u_cl.d = f[21];
    msg.v_cl.a = f[22]; msg.v_cl.b = f[23]; msg.v_cl.c = f[24]; msg.v_cl.d = f[25];
    msg.u_left_re.a = f[26]; msg.u_left_re.b = f[27]; msg.u_left_re.c = f[28]; msg.u_left_re.d = f[29];
    msg.v_left_re.a = f[30]; msg.v_left_re.b = f[31]; msg.v_left_re.c = f[32]; msg.v_left_re.d = f[33];
    msg.u_right_re.a = f[34]; msg.u_right_re.b = f[35]; msg.u_right_re.c = f[36]; msg.u_right_re.d = f[37];
    msg.v_right_re.a = f[38]; msg.v_right_re.b = f[39]; msg.v_right_re.c = f[40]; msg.v_right_re.d = f[41];

    msg.start_cl.x = f[42]; msg.start_cl.y = f[43]; msg.start_cl.z = f[44];
    msg.start_l.x  = f[45]; msg.start_l.y  = f[46]; msg.start_l.z  = f[47];
    msg.start_r.x  = f[48]; msg.start_r.y  = f[49]; msg.start_r.z  = f[50];

    msg.drivability = f[51];
    msg.speed_limit = f[52];
    msg.flag1way    = f[53];

    pub->publish(msg);
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
    const SensorConfig& cfg)
{
  using beamng_msgs::msg::IdealRadarSensor;
  using beamng_msgs::msg::IdealRadarSensorVehicle;

  auto pub      = node.create_publisher<IdealRadarSensor>(topic, 10);
  auto node_ptr = node.shared_from_this();
  const std::string frame_id = cfg.frame_id.empty() ? std::string(node.get_name()) : cfg.frame_id;

  static constexpr uint32_t VEHICLE_STRIDE = 4 + 15 * sizeof(double);  // 124 bytes
  static constexpr uint32_t PAYLOAD_SIZE   = 4 * VEHICLE_STRIDE;        // 496 bytes

  IdealRadarSensor tmpl;
  tmpl.header.frame_id = frame_id;

  return [pub, node_ptr, msg = std::move(tmpl)](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ts_ns));

    auto decode_vehicle = [&](uint32_t offset) -> IdealRadarSensorVehicle {
      IdealRadarSensorVehicle v;
      int32_t vid; std::memcpy(&vid, data + offset, 4); offset += 4;
      double f[15]; std::memcpy(f, data + offset, 15 * sizeof(double));
      v.vehicle_id                = vid;
      v.width                     = f[0];
      v.length                    = f[1];
      v.dist_to_player_vehicle_sq = f[2];
      v.rel_dist_x                = f[3];
      v.rel_dist_y                = f[4];
      v.vel_bb.x                  = f[5];
      v.vel_bb.y                  = f[6];
      v.vel_bb.z                  = f[7];
      v.rel_vel_x                 = f[8];
      v.rel_vel_y                 = f[9];
      v.acc.x                     = f[10];
      v.acc.y                     = f[11];
      v.acc.z                     = f[12];
      v.rel_acc_x                 = f[13];
      v.rel_acc_y                 = f[14];
      return v;
    };

    msg.closest_vehicles1 = decode_vehicle(0 * VEHICLE_STRIDE);
    msg.closest_vehicles2 = decode_vehicle(1 * VEHICLE_STRIDE);
    msg.closest_vehicles3 = decode_vehicle(2 * VEHICLE_STRIDE);
    msg.closest_vehicles4 = decode_vehicle(3 * VEHICLE_STRIDE);

    pub->publish(msg);
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
    const SensorConfig& cfg)
{
  using sensor_msgs::msg::NavSatFix;
  using sensor_msgs::msg::NavSatStatus;

  auto pub      = node.create_publisher<NavSatFix>(topic, 10);
  auto node_ptr = node.shared_from_this();
  const std::string frame_id = cfg.frame_id.empty() ? std::string(node.get_name()) : cfg.frame_id;

  static constexpr uint32_t PAYLOAD_SIZE = 3 * sizeof(double);  // 24 bytes

  NavSatFix tmpl;
  tmpl.header.frame_id               = frame_id;
  tmpl.status.status                 = NavSatStatus::STATUS_FIX;
  tmpl.status.service                = NavSatStatus::SERVICE_GPS;
  tmpl.position_covariance_type      = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  tmpl.position_covariance           = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  return [pub, node_ptr, msg = std::move(tmpl)](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    if (len < PAYLOAD_SIZE) return;

    double vals[3];
    std::memcpy(vals, data, PAYLOAD_SIZE);

    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ts_ns));
    msg.latitude     = vals[0];
    msg.longitude    = vals[1];
    msg.altitude     = vals[2];

    pub->publish(msg);
  };
}

static BlobPublishFn make_image_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const SensorConfig& cfg)
{
  auto pub = node.create_publisher<sensor_msgs::msg::Image>(topic, 10);

  sensor_msgs::msg::Image tmpl;
  tmpl.header.frame_id = cfg.frame_id.empty() ? node.get_name() : cfg.frame_id;
  tmpl.width           = cfg.blob_layout.width;
  tmpl.height          = cfg.blob_layout.height;
  tmpl.is_bigendian    = false;

  if (cfg.blob_layout.format == "rgba_uint8") {
    tmpl.encoding = "rgba8";
    tmpl.step     = cfg.blob_layout.width * 4;
  } else if (cfg.blob_layout.format == "rgb_uint8") {
    tmpl.encoding = "rgb8";
    tmpl.step     = cfg.blob_layout.width * 3;
  } else if (cfg.blob_layout.format == "depth_float32") {
    tmpl.encoding = "32FC1";
    tmpl.step     = cfg.blob_layout.width * 4;
  } else if (cfg.blob_layout.format == "mono_uint8") {
    tmpl.encoding = "mono8";
    tmpl.step     = cfg.blob_layout.width;
  } else {
    std::cerr << "[ROS2 WARN] make_image_factory: unknown format '"
              << cfg.blob_layout.format << "' for sensor '" << cfg.id
              << "', using raw passthrough.\n";
    tmpl.encoding = "rgb8";
    tmpl.step     = cfg.blob_layout.width * 3;
  }

  const uint32_t max_bytes = cfg.blob_layout.max_bytes;
  if (max_bytes > 0) {
    tmpl.data.resize(max_bytes);
  }

  // Capture msg directly so the lambda reuses the same allocation every call.
  // Only stamp and data change per frame; all other fields are fixed at init.
  return [pub, msg = std::move(tmpl)](
      const uint8_t* data, uint32_t len, uint64_t ts_ns) mutable
  {
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ts_ns));
    msg.data.assign(data, data + len);
    pub->publish(msg);
  };
}

void register_all_sensor_factories() {
  using namespace beamng_msgs::msg;
  auto& reg = SensorFactoryRegistry::instance();

  // Scalar factories - one entry per beamng_msgs message type.
  #define REGISTER(MsgType) \
    reg.register_factory(#MsgType, \
      [](rclcpp::Node& node, const std::string& topic, \
          const SensorConfig& cfg, \
          uint32_t scalar_base_idx, uint32_t string_base_idx) -> ScalarRegisterResult { \
        return make_generic_factory<MsgType>( \
            node, topic, cfg, scalar_base_idx, string_base_idx); \
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
  reg.register_blob_factory("RoadsSensor",     make_roads_sensor_factory);
  reg.register_blob_factory("IdealRadarSensor",make_ideal_radar_factory);
  reg.register_blob_factory("Image",           make_image_factory);
}

}  // namespace bng_ros2
