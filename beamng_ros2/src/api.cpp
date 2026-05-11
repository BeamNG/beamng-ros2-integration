#include "api.h"

#include <memory>
#include <mutex>
#include <unordered_map>

#include "vehicle_node.h"

namespace {
std::unordered_map<uint32_t, std::unique_ptr<bng_ros2::VehicleNode>> g_nodes;
std::mutex g_mutex;
}  // namespace

extern "C" {

BNG_ROS2_API char bngros2_init_vehicle_node(uint32_t id, const char* name) {
    const char* nm = name ? name : "bng_vehicle";
    std::lock_guard<std::mutex> lk(g_mutex);
    if (g_nodes.count(id)) {
        return 0;
    }
    g_nodes[id] = std::make_unique<bng_ros2::VehicleNode>(nm);
    return 1;
}

BNG_ROS2_API void bngros2_destroy_vehicle_node(uint32_t id) {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_nodes.erase(id);
}

BNG_ROS2_API void bngros2_destroy_all_vehicle_nodes(void) {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_nodes.clear();
}

BNG_ROS2_API void* bngros2_get_vehicle_handle(uint32_t id) {
    std::lock_guard<std::mutex> lk(g_mutex);
    auto it = g_nodes.find(id);
    return it == g_nodes.end() ? nullptr : it->second.get();
}

BNG_ROS2_API int bngros2_handle_apply_config(void* handle, const char* json) {
    if (!handle || !json) {
        return -1;
    }
    return static_cast<bng_ros2::VehicleNode*>(handle)->apply_config(json);
}

BNG_ROS2_API int bngros2_handle_get_blob_slot(void* handle, uint32_t slot, BngBlobSlotFfiView* out) {
    if (!handle) return -1;
    return static_cast<bng_ros2::VehicleNode*>(handle)->get_blob_slot(slot, out);
}

BNG_ROS2_API uint32_t bngros2_handle_blob_state_acquire(void* handle, uint32_t slot) {
    if (!handle) return 0;
    return static_cast<bng_ros2::VehicleNode*>(handle)->blob_state_acquire(slot);
}

BNG_ROS2_API void bngros2_handle_blob_commit_side(void* handle, uint32_t slot, uint32_t side, uint32_t length_bytes) {
    if (!handle) return;
    static_cast<bng_ros2::VehicleNode*>(handle)->blob_commit_side(slot, side, length_bytes);
}

BNG_ROS2_API void bngros2_handle_bind_gpu_sensor(void* handle, uint32_t gpu_slot, const BngGpuStreamDesc* descs, uint32_t count) {
    if (!handle) return;
    static_cast<bng_ros2::VehicleNode*>(handle)->bind_gpu_sensor(gpu_slot, descs, count);
}

BNG_ROS2_API void bngros2_handle_unbind_gpu_sensor(void* handle, uint32_t gpu_slot) {
    if (!handle) return;
    static_cast<bng_ros2::VehicleNode*>(handle)->unbind_gpu_sensor(gpu_slot);
}

BNG_ROS2_API void bngros2_handle_push_scalar_frame(void* handle, const double* values, uint32_t count) {
    if (!handle) return;
    static_cast<bng_ros2::VehicleNode*>(handle)->push_scalar_frame(values, count);
}

BNG_ROS2_API void bngros2_handle_set_scalar_string(void* handle, const char* sensor_id, const char* field_name, const char* value) {
    if (!handle) return;
    static_cast<bng_ros2::VehicleNode*>(handle)->set_scalar_string(sensor_id, field_name, value);
}

BNG_ROS2_API void bngros2_handle_broadcast_tf(void* handle, const double pos[3], const double rot[4]) {
    if (!handle) return;
    static_cast<bng_ros2::VehicleNode*>(handle)->broadcast_tf(pos, rot);
}

BNG_ROS2_API void bngros2_handle_set_vehicle_data(void* handle, const double scale[3], const double color[4], const double arrow_color[4]) {
    if (!handle) return;
    static_cast<bng_ros2::VehicleNode*>(handle)->set_vehicle_data(scale, color, arrow_color);
}

BNG_ROS2_API uint64_t bngros2_handle_get_control_version(void* handle) {
    if (!handle) return 0;
    return static_cast<bng_ros2::VehicleNode*>(handle)->get_control_version();
}

BNG_ROS2_API int bngros2_handle_get_control_frame(void* handle, BngControlView* out) {
    if (!handle) return -1;
    return static_cast<bng_ros2::VehicleNode*>(handle)->get_control_frame(out);
}

}  // extern "C"
