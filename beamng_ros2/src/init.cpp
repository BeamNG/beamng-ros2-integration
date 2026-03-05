#include "rclcpp/rclcpp.hpp"
#include "sensor_factories.h"

static void ros2_init() {
  try {
    rclcpp::init(0, nullptr);
  } catch (const rclcpp::ContextAlreadyInitialized&) {
    // Silently ignore: another DLL or the host app already called rclcpp::init.
  }
  bng_ros2::register_all_sensor_factories();
}

#ifdef __linux__
  __attribute__((constructor)) static void initialize_bngros2() {
    ros2_init();
}
#elif _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include "windows.h"
  static BOOL APIENTRY DllMain(HMODULE, DWORD, LPVOID) {
    ros2_init();
    return TRUE;
  }
#endif
