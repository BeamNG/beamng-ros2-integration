#pragma once

#if defined(_MSC_VER)
#define BNG_EXPORT __declspec(dllexport)
#define BNG_IMPORT __declspec(dllimport)
#else
#define BNG_EXPORT __attribute__((visibility("default")))
#define BNG_IMPORT
#endif

#if BNG_ROS2_COMPILING
#define BNG_ROS2_API BNG_EXPORT
#else
#define BNG_ROS2_API BNG_IMPORT
#endif
