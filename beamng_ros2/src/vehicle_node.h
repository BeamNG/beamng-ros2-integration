#pragma once
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include <unordered_map>
#include "definitions.h"
#include "sensor_factories.h"

namespace bng_ros2 {

struct VehicleData {
  tf2::Vector3 scale;
  geometry_msgs::msg::Point offset;
  std_msgs::msg::ColorRGBA color;
  std_msgs::msg::ColorRGBA arrow_color;
};

class VehicleNode : public rclcpp::Node {
  public:
    explicit VehicleNode(const std::string& name);
    void broadcast_tf(double pos[3], double rot[4]);
    void set_data(double scale[3], double color[4], double arrow_color[4]);
    void set_sensor_field(const std::string& sensor_id,
                          const std::string& field_name,
                          double value);
    void set_sensor_field_string(const std::string& sensor_id,
                                  const std::string& field_name,
                                  const std::string& value);
    void publish();

  private:
    void init_sensor_publishers();

    std::string name_;
    VehicleData data_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizer_;

    // Value maps are allocated once and captured by reference in PublishFn lambdas.
    std::unordered_map<std::string, ValueMap>  sensor_values_;
    std::unordered_map<std::string, StringMap> sensor_string_values_;

    std::unordered_map<std::string, PublishFn> publish_fns_;
};

}  // namespace bng_ros2

extern "C" {
BNG_ROS2_API char bngros2_init_vehicle_node(uint32_t id, const char* name);
BNG_ROS2_API void bngros2_destroy_vehicle_node(uint32_t id);
BNG_ROS2_API void bngros2_destroy_all_vehicle_nodes();
BNG_ROS2_API char bngros2_spin_vehicle_node(uint32_t id);
BNG_ROS2_API void bngros2_broadcast_tf(uint32_t id, double pos[3], double rot[4]);
BNG_ROS2_API void bngros2_vehicle_set_data(uint32_t id, double scale[3],
                                            double color[4], double arrow_color[4]);
BNG_ROS2_API void bngros2_vehicle_set_sensor_field(uint32_t id,
                                                    const char* sensor_id,
                                                    const char* field_name,
                                                    double value);
BNG_ROS2_API void bngros2_vehicle_set_sensor_field_string(uint32_t id,
                                                            const char* sensor_id,
                                                            const char* field_name,
                                                            const char* value);
BNG_ROS2_API void bngros2_vehicle_publish(uint32_t id);
}
