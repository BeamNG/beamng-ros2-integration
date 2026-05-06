#pragma once

#include <cstdint>
#include <string>

namespace bng_ros2 {

// Named shared memory mapping (engine writes GPU/camera frames; publish thread reads).
class SharedMemoryReader {
  public:
    SharedMemoryReader() = default;
    ~SharedMemoryReader();
    SharedMemoryReader(SharedMemoryReader&&) noexcept;
    SharedMemoryReader& operator=(SharedMemoryReader&&) noexcept;
    SharedMemoryReader(const SharedMemoryReader&) = delete;
    SharedMemoryReader& operator=(const SharedMemoryReader&) = delete;

    bool open(const std::string& name, size_t size_bytes);

    const uint8_t* data() const { return ptr_; }
    size_t size() const { return size_; }
    bool is_open() const { return ptr_ != nullptr; }

    void close();

  private:
    void* handle_{nullptr};
    uint8_t* ptr_{nullptr};
    size_t size_{0};
};

}  // namespace bng_ros2
