#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class BeamNGBridge : public rclcpp::Node {
 public:
  BeamNGBridge() : Node("beamng_bridge") {}
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeamNGBridge>());
  rclcpp::shutdown();
  return 0;
}
