#include "publishers/blob_publishers.h"

#include "utils/binary_read_helpers.h"

#include <beamng_msgs/msg/damage_sensor.hpp>
#include <beamng_msgs/msg/ideal_radar_sensor.hpp>
#include <beamng_msgs/msg/mesh_sensor.hpp>
#include <beamng_msgs/msg/mesh_sensor_node.hpp>
#include <beamng_msgs/msg/powertrain_sensor.hpp>
#include <beamng_msgs/msg/powertrain_sensor_device.hpp>
#include <beamng_msgs/msg/roads_sensor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <algorithm>
#include <cstring>
#include <string>

#include <rclcpp/qos.hpp>

namespace bng_ros2 {

using br::read_f64;
using br::read_i32;
using br::read_str_into;
using br::read_u32;
using br::read_u8;

namespace {

// Ensures the frame is not empty, if it is, use the node name
std::string frame_or_node(rclcpp::Node& node, const std::string& frame) {
    return frame.empty() ? std::string(node.get_name()) : frame;
}

}  // namespace

BlobPublishFn make_damage(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using beamng_msgs::msg::DamageSensor;
    auto publisher = node.create_publisher<DamageSensor>(topic, 10);
    DamageSensor damage_template;
    damage_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    return [publisher, msg = std::move(damage_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        msg.header.stamp = stamp;
        msg.deformgroup_id.clear();
        msg.inv_max_events.clear();
        msg.damage.clear();
        msg.event_count.clear();
        msg.max_events.clear();
        msg.part_id.clear();
        msg.name.clear();
        msg.part_damage.clear();

        uint32_t pos = 0;
        if (pos + 4 <= len) { // avoid u32 overflow
            const uint32_t deform_group_count = read_u32(data, &pos);
            msg.deformgroup_id.reserve(deform_group_count);
            msg.inv_max_events.reserve(deform_group_count);
            msg.damage.reserve(deform_group_count);
            msg.event_count.reserve(deform_group_count);
            msg.max_events.reserve(deform_group_count);
            for (uint32_t i = 0; i < deform_group_count && pos < len; ++i) {
                msg.deformgroup_id.emplace_back();
                read_str_into(data, &pos, msg.deformgroup_id.back());
                msg.inv_max_events.push_back(read_f64(data, &pos));
                msg.damage.push_back(read_f64(data, &pos));
                msg.event_count.push_back(read_f64(data, &pos));
                msg.max_events.push_back(read_f64(data, &pos));
            }
        }
        if (pos + 4 <= len) { // avoid u32 overflow
            const uint32_t part_damage_count = read_u32(data, &pos);
            msg.part_id.reserve(part_damage_count);
            msg.name.reserve(part_damage_count);
            msg.part_damage.reserve(part_damage_count);
            for (uint32_t i = 0; i < part_damage_count && pos < len; ++i) {
                msg.part_id.emplace_back();
                read_str_into(data, &pos, msg.part_id.back());
                msg.name.emplace_back();
                read_str_into(data, &pos, msg.name.back());
                msg.part_damage.push_back(read_f64(data, &pos));
            }
        }
        publisher->publish(msg);
    };
}

BlobPublishFn make_imu(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using sensor_msgs::msg::Imu;
    auto publisher = node.create_publisher<Imu>(topic, 10);
    Imu imu_template;
    imu_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    imu_template.orientation_covariance[0] = -1.0;
    imu_template.angular_velocity_covariance[0] = -1.0;
    imu_template.linear_acceleration_covariance[0] = -1.0;

    return [publisher, msg = std::move(imu_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        if (len < 10 * sizeof(double)) return;
        double values[10];
        std::memcpy(values, data, sizeof(values));

        msg.header.stamp = stamp;
        msg.angular_velocity.x = values[0];
        msg.angular_velocity.y = values[1];
        msg.angular_velocity.z = values[2];
        msg.linear_acceleration.x = values[3];
        msg.linear_acceleration.y = values[4];
        msg.linear_acceleration.z = values[5];
        msg.orientation.x = values[6];
        msg.orientation.y = values[7];
        msg.orientation.z = values[8];
        msg.orientation.w = values[9];

        publisher->publish(msg);
    };
}

BlobPublishFn make_nav_sat(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using sensor_msgs::msg::NavSatFix;
    using sensor_msgs::msg::NavSatStatus;
    auto publisher = node.create_publisher<NavSatFix>(topic, 10);
    NavSatFix gps_template;
    gps_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    gps_template.status.status = NavSatStatus::STATUS_FIX;
    gps_template.status.service = NavSatStatus::SERVICE_GPS;
    gps_template.position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    gps_template.position_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    return [publisher, msg = std::move(gps_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        if (len < 3 * sizeof(double)) return;
        double values[3]; std::memcpy(values, data, sizeof(values));

        msg.header.stamp = stamp;
        msg.latitude = values[0];
        msg.longitude = values[1];
        msg.altitude = values[2];

        publisher->publish(msg);
    };
}

BlobPublishFn make_ideal_radar(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using beamng_msgs::msg::IdealRadarSensor;
    using beamng_msgs::msg::IdealRadarSensorVehicle;
    auto publisher = node.create_publisher<IdealRadarSensor>(topic, 10);
    IdealRadarSensor idealradar_template;
    idealradar_template.header.frame_id = frame_or_node(node, sensor.frame_id);

    return [publisher, msg = std::move(idealradar_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        constexpr uint32_t k_stride = 4 + 15 * sizeof(double);
        constexpr uint32_t k_payload = 4 * k_stride;
        if (len < k_payload) return;
        msg.header.stamp = stamp;
        auto decode = [&](uint32_t off) {
            IdealRadarSensorVehicle vehicle;
            int32_t vid; std::memcpy(&vid, data + off, 4); off += 4;
            double values[15]; std::memcpy(values, data + off, sizeof(values));

            vehicle.vehicle_id = vid;
            vehicle.width = values[0];
            vehicle.length = values[1];
            vehicle.dist_to_player_vehicle_sq = values[2];
            vehicle.rel_dist_x = values[3];
            vehicle.rel_dist_y = values[4];
            vehicle.vel_bb.x = values[5];
            vehicle.vel_bb.y = values[6];
            vehicle.vel_bb.z = values[7];
            vehicle.rel_vel_x = values[8];
            vehicle.rel_vel_y = values[9];
            vehicle.acc.x = values[10];
            vehicle.acc.y = values[11];
            vehicle.acc.z = values[12];
            vehicle.rel_acc_x = values[13];
            vehicle.rel_acc_y = values[14];
            return vehicle;
        };

        msg.closest_vehicles1 = decode(0 * k_stride);
        msg.closest_vehicles2 = decode(1 * k_stride);
        msg.closest_vehicles3 = decode(2 * k_stride);
        msg.closest_vehicles4 = decode(3 * k_stride);
        publisher->publish(msg);
    };
}

BlobPublishFn make_roads(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using beamng_msgs::msg::RoadsSensor;
    auto publisher = node.create_publisher<RoadsSensor>(topic, 10);
    RoadsSensor roads_template;
    roads_template.header.frame_id = frame_or_node(node, sensor.frame_id);

    return [publisher, msg = std::move(roads_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        constexpr uint32_t k_payload = 54 * sizeof(double);
        if (len < k_payload) return;
        const double* values = reinterpret_cast<const double*>(data);

        msg.header.stamp = stamp;
        msg.dist2_cl = values[0];
        msg.dist2_left = values[1];
        msg.dist2_right = values[2];
        msg.half_width = values[3];
        msg.road_radius = values[4];
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
        msg.flag1way = values[53];

        publisher->publish(msg);
    };
}

BlobPublishFn make_powertrain(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using beamng_msgs::msg::PowertrainSensor;
    using beamng_msgs::msg::PowertrainSensorDevice;
    auto publisher = node.create_publisher<PowertrainSensor>(topic, 10);
    PowertrainSensor powertrain_template;
    powertrain_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    return [publisher, msg = std::move(powertrain_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        msg.header.stamp = stamp;
        msg.devices.clear();
        uint32_t pos = 0;
        if (pos + 4 > len) return;

        const uint32_t device_count = read_u32(data, &pos);
        msg.devices.reserve(device_count);
        for (uint32_t i = 0; i < device_count && pos < len; ++i) {
            msg.devices.emplace_back();
            PowertrainSensorDevice& device = msg.devices.back();
            read_str_into(data, &pos, device.name);
            device.input_av = read_f64(data, &pos);
            device.gear_ratio = read_f64(data, &pos);
            device.is_broken = read_u8(data, &pos) != 0;
            read_str_into(data, &pos, device.mode);
            read_str_into(data, &pos, device.parent_name);
            device.parent_output_index = read_i32(data, &pos);
            device.output_torque_1 = read_f64(data, &pos);
            device.output_av_1 = read_f64(data, &pos);
            device.output_torque_2 = read_f64(data, &pos);
            device.output_av_2 = read_f64(data, &pos);
        }
        publisher->publish(msg);
    };
}

BlobPublishFn make_mesh(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    using beamng_msgs::msg::MeshSensor;
    using beamng_msgs::msg::MeshSensorNode;
    // Mesh messages are huge, this is to prevent OOM.
    auto publisher = node.create_publisher<MeshSensor>(topic, rclcpp::QoS(rclcpp::KeepLast(1)));
    MeshSensor mesh_template;
    mesh_template.header.frame_id = frame_or_node(node, sensor.frame_id);
    return [publisher, msg = std::move(mesh_template)](const uint8_t* data, uint32_t len, const rclcpp::Time& stamp) mutable {
        msg.header.stamp = stamp;
        msg.nodes.clear();
        msg.beams.clear();
        uint32_t pos = 0;
        if (pos + 4 > len) return;
        uint32_t node_count = read_u32(data, &pos);
        msg.nodes.reserve(node_count);

        for (uint32_t i = 0; i < node_count && pos + 80 <= len; ++i) {
            MeshSensorNode node;
            const double* values = reinterpret_cast<const double*>(data + pos);
            node.pos.x = values[0];
            node.pos.y = values[1];
            node.pos.z = values[2];
            node.vel.x = values[3];
            node.vel.y = values[4];
            node.vel.z = values[5];
            node.force.x = values[6];
            node.force.y = values[7];
            node.force.z = values[8];
            node.mass = values[9];
            pos += 80;
            read_str_into(data, &pos, node.part_origin);
            msg.nodes.push_back(std::move(node));
        }

        if (pos + 4 > len) { publisher->publish(msg); return; }
        uint32_t beam_count = read_u32(data, &pos);
        if (pos + beam_count * sizeof(double) > len) { publisher->publish(msg); return; }
        const double* stresses = reinterpret_cast<const double*>(data + pos);

        msg.beams.resize(beam_count);
        for (uint32_t i = 0; i < beam_count; ++i) {
            msg.beams[i].stress = stresses[i];
        }
        publisher->publish(msg);
    };
}

BlobPublishFn build_blob_publisher(rclcpp::Node& node, const std::string& topic, const SensorEntry& sensor) {
    switch (sensor.layout) {
        case LayoutId::BlobDamage:      return make_damage(node, topic, sensor);
        case LayoutId::BlobImu:         return make_imu(node, topic, sensor);
        case LayoutId::BlobNavSatFix:   return make_nav_sat(node, topic, sensor);
        case LayoutId::BlobIdealRadar:  return make_ideal_radar(node, topic, sensor);
        case LayoutId::BlobRoads:       return make_roads(node, topic, sensor);
        case LayoutId::BlobPowertrain:  return make_powertrain(node, topic, sensor);
        case LayoutId::BlobMesh:        return make_mesh(node, topic, sensor);
        default:                        return {};
    }
}

}  // namespace bng_ros2
