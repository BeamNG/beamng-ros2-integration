#pragma once

#include <string>
#include <string_view>

namespace bng_ros2 {

enum class SensorCategory { Scalar, Blob, Gpu };

enum class LayoutId {
    Invalid = 0,
    ScalarElectrics,
    ScalarGForce,
    ScalarState,
    ScalarTime,
    BlobDamage,
    BlobImu,
    BlobNavSatFix,
    BlobIdealRadar,
    BlobRoads,
    BlobPowertrain,
    BlobMesh,
    GpuCamera,
    GpuLidar,
    GpuRadar,
    GpuUltrasonic,
};

inline LayoutId parse_layout_id(std::string_view s) {
    if (s == "electrics_sensor")  return LayoutId::ScalarElectrics;
    if (s == "gforce_sensor")     return LayoutId::ScalarGForce;
    if (s == "state_sensor")      return LayoutId::ScalarState;
    if (s == "time_sensor")       return LayoutId::ScalarTime;
    if (s == "blob_damage")       return LayoutId::BlobDamage;
    if (s == "blob_imu")          return LayoutId::BlobImu;
    if (s == "blob_nav_sat_fix")  return LayoutId::BlobNavSatFix;
    if (s == "blob_ideal_radar")  return LayoutId::BlobIdealRadar;
    if (s == "blob_roads")        return LayoutId::BlobRoads;
    if (s == "blob_powertrain")   return LayoutId::BlobPowertrain;
    if (s == "blob_mesh")         return LayoutId::BlobMesh;
    if (s == "gpu_camera")        return LayoutId::GpuCamera;
    if (s == "gpu_lidar")         return LayoutId::GpuLidar;
    if (s == "gpu_radar")         return LayoutId::GpuRadar;
    if (s == "gpu_ultrasonic")    return LayoutId::GpuUltrasonic;
    return LayoutId::Invalid;
}

inline SensorCategory category_for_layout(LayoutId layout) {
    switch (layout) {
        case LayoutId::ScalarElectrics:
        case LayoutId::ScalarGForce:
        case LayoutId::ScalarState:
        case LayoutId::ScalarTime:
            return SensorCategory::Scalar;
        case LayoutId::GpuCamera:
        case LayoutId::GpuLidar:
        case LayoutId::GpuRadar:
        case LayoutId::GpuUltrasonic:
            return SensorCategory::Gpu;
        default:
            return SensorCategory::Blob;
    }
}

}  // namespace bng_ros2
