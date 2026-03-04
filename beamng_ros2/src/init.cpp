#include "rclcpp/rclcpp.hpp"
#include <iostream>

void ros2_init() {
  try {
    rclcpp::init(0, nullptr);
  } catch (const rclcpp::ContextAlreadyInitialized&) {
    // Another component (host app or DLL load order) already initialized.
    // Nothing to do.
  }
}

#ifdef __linux__
__attribute__((constructor)) static void initialize_bngros2() {
  ros2_init();
}
#elif _WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
static BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call,
                             LPVOID lpReserved) {
  ros2_init();
  printf("ROS2 initialized\n");
  return TRUE;
}
#endif
