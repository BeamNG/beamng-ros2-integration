#include "sensor_factories.h"
#include "sensor_registry.h"
#include <beamng_msgs/msg/electrics_sensor.hpp>
#include <beamng_msgs/msg/g_force_sensor.hpp>
#include <beamng_msgs/msg/state_sensor.hpp>
#include <beamng_msgs/msg/damage_sensor.hpp>
#include <beamng_msgs/msg/time_sensor.hpp>
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

  PublishFn fn = [pub, node_ptr, frame_id, writers = std::move(writers)](
      const ScalarStore& scalars, const StringStore& strings)
  {
    MsgT msg;
    if constexpr (has_header<MsgT>::value) {
      msg.header.stamp    = node_ptr->get_clock()->now();
      msg.header.frame_id = frame_id;
    }

    auto* base = reinterpret_cast<uint8_t*>(&msg);
    for (const auto& w : writers) {
      if (w.is_string) {
        if (w.value_idx < strings.size())
          write_string(base, w.offset, strings[w.value_idx]);
      } else {
        if (w.value_idx < scalars.size())
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

  reg.register_blob_factory("DamageSensor",
    [](rclcpp::Node& node, const std::string& topic,
        const SensorConfig& cfg) -> BlobPublishFn {
      return make_dynamic_array_sensor_factory(node, topic, cfg);
    });

  reg.register_blob_factory("PointCloud2",
    [](rclcpp::Node& node, const std::string& topic,
        const SensorConfig& cfg) -> BlobPublishFn {
      return make_pointcloud2_factory(node, topic, cfg);
    });

  reg.register_blob_factory("Image",
    [](rclcpp::Node& node, const std::string& topic,
        const SensorConfig& cfg) -> BlobPublishFn {
      return make_image_factory(node, topic, cfg);
    });
}

}  // namespace bng_ros2
