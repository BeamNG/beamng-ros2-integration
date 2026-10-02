#include "api.h"

#include "rclcpp/rclcpp.hpp"

#include <iostream>

extern "C" void bngros2_ros_init(void) {
    try {
        rclcpp::init(0, nullptr);
    } catch (const rclcpp::ContextAlreadyInitialized&) {
        // Silently ignore: another DLL or the host app already called rclcpp::init.
    }
}

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
static BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID lpReserved) {
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(hModule);
    } else if (reason == DLL_PROCESS_DETACH && lpReserved == nullptr) {
        // lpReserved == nullptr means FreeLibrary was called (not process exit)
        try {
            rclcpp::shutdown();
        } catch (const std::exception& e) {
            std::cerr << "[ROS2 ERROR] " << e.what() << "\n";
            return FALSE;
        }
    }
    return TRUE;
}
#endif
