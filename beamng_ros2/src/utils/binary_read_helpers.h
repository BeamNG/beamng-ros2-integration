#pragma once

#include <cstdint>
#include <cstring>
#include <string>

namespace bng_ros2 {
namespace br {

inline uint8_t read_u8(const uint8_t* data, uint32_t* pos) {
    const uint8_t value = data[*pos];
    *pos += 1;
    return value;
}

inline uint16_t read_u16(const uint8_t* data, uint32_t* pos) {
    uint16_t value;
    std::memcpy(&value, data + *pos, 2);
    *pos += 2;
    return value;
}

inline uint32_t read_u32(const uint8_t* data, uint32_t* pos) {
    uint32_t value;
    std::memcpy(&value, data + *pos, 4);
    *pos += 4;
    return value;
}

inline int32_t read_i32(const uint8_t* data, uint32_t* pos) {
    int32_t value;
    std::memcpy(&value, data + *pos, 4);
    *pos += 4;
    return value;
}

inline float read_f32(const uint8_t* data, uint32_t* pos) {
    float value;
    std::memcpy(&value, data + *pos, 4);
    *pos += 4;
    return value;
}

inline double read_f64(const uint8_t* data, uint32_t* pos) {
    double value;
    std::memcpy(&value, data + *pos, 8);
    *pos += 8;
    return value;
}

inline void read_str_into(const uint8_t* data, uint32_t* pos, std::string& out) {
    const uint16_t len = read_u16(data, pos);
    out.assign(reinterpret_cast<const char*>(data + *pos), len);
    *pos += len;
}

}  // namespace br
}  // namespace bng_ros2
