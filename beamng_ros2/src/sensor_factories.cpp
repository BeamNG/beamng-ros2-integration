#include "sensor_factories.h"
#include "sensor_registry.h"
#include <beamng_msgs/msg/electrics_sensor.hpp>
#include <beamng_msgs/msg/g_force_sensor.hpp>
#include <beamng_msgs/msg/state_sensor.hpp>
#include <beamng_msgs/msg/damage_sensor.hpp>
#include <beamng_msgs/msg/time_sensor.hpp>
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

namespace bng_ros2 {

namespace intr = rosidl_typesupport_introspection_cpp;


SensorFactoryRegistry& SensorFactoryRegistry::instance() {
  static SensorFactoryRegistry inst;
  return inst;
}

void SensorFactoryRegistry::register_factory(const std::string& msg_file, RegisterFn fn) {
  factories_[msg_file] = std::move(fn);
}

const RegisterFn* SensorFactoryRegistry::find(const std::string& msg_file) const {
  auto it = factories_.find(msg_file);
  if (it == factories_.end()) return nullptr;
  return &it->second;
}


// Describes one active field: resolved once at init, used on every publish.
struct FieldWriter {
  std::string value_key;  // key into ValueMap or StringMap
  size_t      offset;     // byte offset into the message struct
  uint8_t     type_id;    // rosidl primitive type id
  bool        is_string;
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

static double get_num(const ValueMap& m, const std::string& key) {
  auto it = m.find(key);
  return it != m.end() ? it->second : 0.0;
}

static const std::string& get_str(const StringMap& m, const std::string& key) {
  static const std::string empty;
  auto it = m.find(key);
  return it != m.end() ? it->second : empty;
}

// C++17 detection trait: does MsgT have a 'header' member?
template<typename T, typename = void>
struct has_header : std::false_type {};
template<typename T>
struct has_header<T, std::void_t<decltype(std::declval<T&>().header)>> : std::true_type {};


// Works for any compiled beamng_msgs message type.
// At init time: retrieves introspection metadata, resolves field offsets for every field listed in the JSON config, builds a FieldWriter vector.
// At publish time: iterates FieldWriters and writes values by raw offset.
template<typename MsgT>
static PublishFn make_generic_factory(
    rclcpp::Node& node,
    const std::string& topic,
    const ValueMap& values,
    const StringMap& string_values,
    const SensorConfig& sensor_cfg)
{
  // rosidl_typesupport_cpp provides the generic type support handle
  // get_message_typesupport_handle extracts the introspection-specific variant.
  const rosidl_message_type_support_t* ts =
      rosidl_typesupport_cpp::get_message_type_support_handle<MsgT>();
  const rosidl_message_type_support_t* intr_ts =
      get_message_typesupport_handle(ts, intr::typesupport_identifier);
  const auto* members = reinterpret_cast<const intr::MessageMembers*>(intr_ts->data);

  std::vector<FieldWriter> writers;
  writers.reserve(sensor_cfg.fields.size());
  for (const auto& field : sensor_cfg.fields) {
    auto res = resolve_field_path(members, field.name);
    if (!res.found) {
      std::cerr << "[ROS2 WARN] " << sensor_cfg.msg_file << ": field '"
                << field.name << "' not found in introspection, skipping.\n";
      continue;
    }
    writers.push_back({field.name, res.offset, res.type_id,
                        res.type_id == intr::ROS_TYPE_STRING});
  }

  auto pub = node.create_publisher<MsgT>(topic, 10);

  return [pub, &values, &string_values, &node, writers = std::move(writers)]() {
    MsgT msg;
    if constexpr (has_header<MsgT>::value) {
      msg.header.stamp    = node.get_clock()->now();
      msg.header.frame_id = node.get_name();
    }

    auto* base = reinterpret_cast<uint8_t*>(&msg);
    for (const auto& w : writers) {
      if (w.is_string) {
        write_string(base, w.offset, get_str(string_values, w.value_key));
      } else {
        write_numeric(base, w.offset, w.type_id, get_num(values, w.value_key));
      }
    }

    pub->publish(msg);
  };
}


// To support a new message type: add its #include above and one REGISTER line.
void register_all_sensor_factories() {
  using namespace beamng_msgs::msg;
  auto& reg = SensorFactoryRegistry::instance();

  #define REGISTER(MsgType) \
    reg.register_factory(#MsgType, \
      [](rclcpp::Node& node, const std::string& topic, \
          const ValueMap& values, const StringMap& svalues, \
          const SensorConfig& cfg) -> PublishFn { \
        return make_generic_factory<MsgType>(node, topic, values, svalues, cfg); \
      })

  REGISTER(ElectricsSensor);
  REGISTER(GForceSensor);
  REGISTER(StateSensor);
  REGISTER(DamageSensor);
  REGISTER(TimeSensor);

  #undef REGISTER
}

}  // namespace bng_ros2
