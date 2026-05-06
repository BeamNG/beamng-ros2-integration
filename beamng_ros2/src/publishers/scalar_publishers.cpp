#include "publishers/scalar_publishers.h"

#include <beamng_msgs/msg/electrics_sensor.hpp>
#include <beamng_msgs/msg/g_force_sensor.hpp>
#include <beamng_msgs/msg/state_sensor.hpp>
#include <beamng_msgs/msg/time_sensor.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rclcpp/qos.hpp>

#include <iostream>
#include <string>
#include <vector>

namespace bng_ros2 {

namespace introspection = rosidl_typesupport_introspection_cpp;

namespace {

// rclcpp::SensorDataQoS() uses a small KeepLast depth (~5).
// At ~2 kHz the writer stales for a few ms
// Best-effort then drops samples before tools like ros2 bag pull them, worse after writer restart.
// Larger history : more RAM usage but optimized delivery
constexpr std::size_t k_scalar_publisher_history_depth = 256;

rclcpp::QoS scalar_publisher_qos() {
    rclcpp::QoS qos(k_scalar_publisher_history_depth);
    qos.best_effort();
    qos.durability_volatile();
    return qos;
}

struct FieldWriter {
    uint32_t value_idx;
    std::size_t offset;
    bool is_string;
    void (*numeric_writer)(uint8_t*, double);
};

bool resolve_field_path(const introspection::MessageMembers* members, const std::string& dot_path,
                        std::size_t base_offset, std::size_t* out_offset, uint8_t* out_type) {
    auto dot = dot_path.find('.');
    const std::string head = (dot == std::string::npos) ? dot_path : dot_path.substr(0, dot);
    const std::string tail = (dot == std::string::npos) ? std::string{} : dot_path.substr(dot + 1);

    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto& member = members->members_[i];
        if (head != member.name_) continue;
        
        const std::size_t offset = base_offset + member.offset_;
        if (tail.empty()) { *out_offset = offset; *out_type = member.type_id_; return true; }
        if (member.type_id_ == introspection::ROS_TYPE_MESSAGE && member.members_) {
            const auto* nested = reinterpret_cast<const introspection::MessageMembers*>(member.members_->data);
            return resolve_field_path(nested, tail, offset, out_offset, out_type);
        }
        return false;
    }
    return false;
}

template <typename T>
void write_numeric_cast(uint8_t* ptr, double v) {
    *reinterpret_cast<T*>(ptr) = static_cast<T>(v);
}

void write_numeric_bool(uint8_t* ptr, double v) {
    *reinterpret_cast<bool*>(ptr) = v != 0.0;
}

void (*resolve_numeric_writer(uint8_t type_id))(uint8_t*, double) {
    switch (type_id) {
        case introspection::ROS_TYPE_FLOAT:   return &write_numeric_cast<float>;
        case introspection::ROS_TYPE_DOUBLE:  return &write_numeric_cast<double>;
        case introspection::ROS_TYPE_BOOLEAN: return &write_numeric_bool;
        case introspection::ROS_TYPE_INT8:    return &write_numeric_cast<int8_t>;
        case introspection::ROS_TYPE_UINT8:   return &write_numeric_cast<uint8_t>;
        case introspection::ROS_TYPE_INT16:   return &write_numeric_cast<int16_t>;
        case introspection::ROS_TYPE_UINT16:  return &write_numeric_cast<uint16_t>;
        case introspection::ROS_TYPE_INT32:   return &write_numeric_cast<int32_t>;
        case introspection::ROS_TYPE_UINT32:  return &write_numeric_cast<uint32_t>;
        case introspection::ROS_TYPE_INT64:   return &write_numeric_cast<int64_t>;
        case introspection::ROS_TYPE_UINT64:  return &write_numeric_cast<uint64_t>;
        default:                              return nullptr;
    }
}

template <typename Msg>
ScalarPublisherBuild build_introspected(rclcpp::Node& node, const std::string& topic,
                                        const SensorEntry& sensor,
                                        uint32_t scalar_base, uint32_t string_base) {
    const auto* type_support = rosidl_typesupport_cpp::get_message_type_support_handle<Msg>();
    const auto* introspection_ts = get_message_typesupport_handle(type_support, introspection::typesupport_identifier);
    const auto* members = reinterpret_cast<const introspection::MessageMembers*>(introspection_ts->data);

    std::vector<FieldWriter> writers;
    writers.reserve(sensor.fields.size());
    uint32_t scalar_local = 0;
    uint32_t string_local = 0;
    std::vector<std::pair<std::string, uint32_t>> string_bindings;

    for (const auto& f : sensor.fields) {
        std::size_t offset = 0; uint8_t type = 0;
        if (!resolve_field_path(members, f.name, 0, &offset, &type)) {
            std::cerr << "[ROS2] scalar publisher: field '" << f.name << "' not found on " << topic << "\n";
            continue;
        }
        const bool is_str = (type == introspection::ROS_TYPE_STRING);
        const uint32_t idx = is_str ? (string_base + string_local++) : (scalar_base + scalar_local++);
        if (is_str) {
            string_bindings.push_back({sensor.id + "::" + f.name, idx});
            writers.push_back({idx, offset, true, nullptr});
        } else {
            writers.push_back({idx, offset, false, resolve_numeric_writer(type)});
        }
    }

    auto publisher = node.create_publisher<Msg>(topic, scalar_publisher_qos());
    const std::string frame_id = sensor.frame_id.empty() ? std::string(node.get_name()) : sensor.frame_id;
    const uint32_t expected_scalars = scalar_base + scalar_local;
    const uint32_t expected_strings = string_base + string_local;
    Msg msg_template;
    msg_template.header.frame_id = frame_id;

    ScalarPublishFn fn = [publisher, msg = std::move(msg_template), writers = std::move(writers), expected_scalars, expected_strings]
        (const ScalarValues& scalars, const StringValues& strings, const rclcpp::Time& stamp) mutable {
        if (scalars.size() < expected_scalars || strings.size() < expected_strings) return;
        msg.header.stamp = stamp;
        auto* base = reinterpret_cast<uint8_t*>(&msg);
        for (const auto& writer : writers) {
            if (writer.is_string) {
                *reinterpret_cast<std::string*>(base + writer.offset) = strings[writer.value_idx];
            } else if (writer.numeric_writer) {
                writer.numeric_writer(base + writer.offset, scalars[writer.value_idx]);
            }
        }
        publisher->publish(msg);
    };

    ScalarPublisherBuild out;
    out.publish_fn = std::move(fn);
    out.scalar_slots_used = scalar_local;
    out.string_slots_used = string_local;
    out.string_bindings = std::move(string_bindings);
    return out;
}

}  // namespace

ScalarPublisherBuild build_scalar_publisher(rclcpp::Node& node, const std::string& topic,
                                            const SensorEntry& sensor,
                                            uint32_t scalar_base, uint32_t string_base) {
    switch (sensor.layout) {
        case LayoutId::ScalarElectrics:
            return build_introspected<beamng_msgs::msg::ElectricsSensor>(node, topic, sensor, scalar_base, string_base);
        case LayoutId::ScalarGForce:
            return build_introspected<beamng_msgs::msg::GForceSensor>(node, topic, sensor, scalar_base, string_base);
        case LayoutId::ScalarState:
            return build_introspected<beamng_msgs::msg::StateSensor>(node, topic, sensor, scalar_base, string_base);
        case LayoutId::ScalarTime:
            return build_introspected<beamng_msgs::msg::TimeSensor>(node, topic, sensor, scalar_base, string_base);
        default:
            return {};
    }
}

}  // namespace bng_ros2
