#pragma once

#include <cstdint>
#include "utils/definitions.h"

extern "C" {

struct BngBlobSlotFfiView {
    uint8_t* buf0;
    uint8_t* buf1;
    void* state_atomic;
    uint32_t capacity_bytes;
};

struct BngGpuStreamDesc {
    const char* shared_memory_name;
    uint32_t size_bytes;
    uint32_t width;
    uint32_t height;
    uint32_t stream_index;
    const char* role;
    const char* encoding;
};

struct BngControlView {
    uint64_t version;
    double steering;
    double throttle;
    double brake;
    double parkingbrake;
    double clutch;
    int32_t gear;
};

BNG_ROS2_API char bngros2_init_vehicle_node(uint32_t id, const char* name);
BNG_ROS2_API void bngros2_destroy_vehicle_node(uint32_t id);
BNG_ROS2_API void bngros2_destroy_all_vehicle_nodes(void);
BNG_ROS2_API void* bngros2_get_vehicle_handle(uint32_t id);

BNG_ROS2_API int bngros2_handle_apply_config(void* handle, const char* json);

BNG_ROS2_API int bngros2_handle_get_blob_slot(void* handle, uint32_t slot, BngBlobSlotFfiView* out);
BNG_ROS2_API uint32_t bngros2_handle_blob_state_acquire(void* handle, uint32_t slot);
BNG_ROS2_API void bngros2_handle_blob_commit_side(void* handle, uint32_t slot, uint32_t side, uint32_t length_bytes);

BNG_ROS2_API void bngros2_handle_bind_gpu_sensor(void* handle, uint32_t gpu_slot, const BngGpuStreamDesc* descs, uint32_t count);
BNG_ROS2_API void bngros2_handle_unbind_gpu_sensor(void* handle, uint32_t gpu_slot);

BNG_ROS2_API void bngros2_handle_push_scalar_frame(void* handle, const double* values, uint32_t count);
BNG_ROS2_API void bngros2_handle_set_scalar_string(void* handle, const char* sensor_id, const char* field_name, const char* value);

BNG_ROS2_API void bngros2_handle_broadcast_tf(void* handle, const double pos[3], const double rot[4]);
BNG_ROS2_API void bngros2_handle_set_vehicle_data(void* handle, const double scale[3], const double color[4], const double arrow_color[4]);

BNG_ROS2_API uint64_t bngros2_handle_get_control_version(void* handle);
BNG_ROS2_API int bngros2_handle_get_control_frame(void* handle, BngControlView* out);

}  // extern "C"
