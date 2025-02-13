#include "rclcpp/rclcpp.hpp"

void ros2_init() {
    rclcpp::init(0, nullptr);
}

#ifdef __linux__
__attribute__((constructor))
static void initialize_bngros2() {
    ros2_init();
}
#elif _WIN32
#include "stdafx.h"
static BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    ros2_init();
    return TRUE;
}
#endif