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

namespace bng_ros2 {

VehicleNode::VehicleNode(const std::string& name)
    : Node(name, "beamng_bridge"), name_(name) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  visualizer_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 10);
  init_sensor_publishers();
}

void VehicleNode::init_sensor_publishers() {
  const auto& registry = SensorRegistry::instance();
  if (!registry.is_loaded()) {
    std::cerr << "[ROS2 WARN] VehicleNode: SensorRegistry not loaded, no sensor publishers created.\n";
    return;
  }

  auto& factory_reg = SensorFactoryRegistry::instance();

  for (const auto& sensor : registry.sensors()) {
    // Emplace both maps before use so their addresses are stable for lambda capture.
    sensor_values_.emplace(sensor.id, ValueMap{});
    sensor_string_values_.emplace(sensor.id, StringMap{});
    const ValueMap&  values        = sensor_values_.at(sensor.id);
    const StringMap& string_values = sensor_string_values_.at(sensor.id);

    const RegisterFn* factory = factory_reg.find(sensor.msg_file);
    if (!factory) {
      std::cerr << "[ROS2 WARN] VehicleNode: no factory for msgFile '"
                << sensor.msg_file << "' (sensor '" << sensor.id << "'), skipping.\n";
      continue;
    }

    publish_fns_[sensor.id] = (*factory)(*this, sensor.topic, values, string_values, sensor);

    std::cout << "[ROS2 INFO] VehicleNode '" << name_
              << "': publisher registered for sensor '" << sensor.id
              << "' on topic '" << sensor.topic << "'.\n";
  }
}

void VehicleNode::broadcast_tf(double pos[3], double rot[4]) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id  = name_;
  t.transform.translation = tf2::toMsg(to_vec3(pos));
  t.transform.rotation    = tf2::toMsg(to_quat(rot));
  tf_broadcaster_->sendTransform(t);
}

void VehicleNode::set_data(double scale[3], double color[4], double arrow_color[4]) {
  data_.scale       = to_vec3(scale);
  data_.color       = to_rgba(color);
  data_.arrow_color = to_rgba(arrow_color);
}

void VehicleNode::set_sensor_field(const std::string& sensor_id,
                                    const std::string& field_name,
                                    double value) {
  auto it = sensor_values_.find(sensor_id);
  if (it == sensor_values_.end()) return;
  it->second[field_name] = value;
}

void VehicleNode::set_sensor_field_string(const std::string& sensor_id,
                                          const std::string& field_name,
                                          const std::string& value) {
  auto it = sensor_string_values_.find(sensor_id);
  if (it == sensor_string_values_.end()) return;
  it->second[field_name] = value;
}

void VehicleNode::publish() {
  auto stamp = get_clock()->now();

  visualization_msgs::msg::MarkerArray ma;

  visualization_msgs::msg::Marker vehicle;
  vehicle.header.frame_id  = name_;
  vehicle.header.stamp     = stamp;
  vehicle.ns               = name_ + "_pos";
  vehicle.id               = 0;
  vehicle.type             = visualization_msgs::msg::Marker::CUBE;
  vehicle.action           = visualization_msgs::msg::Marker::ADD;
  vehicle.pose.orientation = quat_forward();
  vehicle.scale            = tf2::toMsg(data_.scale);
  vehicle.color            = data_.color;
  vehicle.lifetime.sec     = 5.0;

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id  = name_;
  arrow.header.stamp     = stamp;
  arrow.ns               = name_ + "_pos";
  arrow.id               = 1;
  arrow.type             = visualization_msgs::msg::Marker::ARROW;
  arrow.action           = visualization_msgs::msg::Marker::ADD;
  arrow.pose.orientation = quat_forward();
  arrow.pose.position.x += data_.scale[0] / 2;
  arrow.scale            = tf2::toMsg(tf2::Vector3(2.0, 1.0, 1.0));
  arrow.color            = data_.arrow_color;
  arrow.lifetime.sec     = 5.0;

  ma.markers.emplace_back(std::move(vehicle));
  ma.markers.emplace_back(std::move(arrow));
  visualizer_->publish(ma);

  for (auto& [sensor_id, fn] : publish_fns_) {
    fn();
  }
}

}  // namespace bng_ros2


static std::unordered_map<uint32_t, std::shared_ptr<bng_ros2::VehicleNode>> nodes;

BNG_ROS2_API char bngros2_init_vehicle_node(uint32_t id, const char* name) {
  try {
    const auto& [_, inserted] = nodes.emplace(id, std::make_shared<bng_ros2::VehicleNode>(name));
    return inserted;
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
  return false;
}

BNG_ROS2_API void bngros2_destroy_vehicle_node(uint32_t id) {
  try {
    nodes.erase(id);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API void bngros2_destroy_all_vehicle_nodes() {
  try {
    nodes.clear();
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
}

BNG_ROS2_API char bngros2_spin_vehicle_node(uint32_t id) {
  auto it = nodes.find(id);
  if (it == nodes.end()) return false;
  try {
    rclcpp::spin_some(it->second);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
    return false;
  }
  return true;
}

BNG_ROS2_API void bngros2_vehicle_publish(uint32_t id) {
  auto it = nodes.find(id);
  if (it == nodes.end()) return;
  try {
    it->second->publish();
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
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

BNG_ROS2_API void bngros2_vehicle_set_sensor_field(uint32_t id,
                                                    const char* sensor_id,
                                                    const char* field_name,
                                                    double value) {
  auto it = nodes.find(id);
  if (it == nodes.end()) return;
  try {
    it->second->set_sensor_field(sensor_id, field_name, value);
  } catch (std::exception& e) {
    std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
  }
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
