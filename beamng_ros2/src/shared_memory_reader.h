#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace bng_ros2 {

// Cross-platform open/read/close for named shared memory (engine writes, we read).
// Used for camera Image sensors in the beamngpy-style streaming workflow.
class SharedMemoryReader {
  public:
    SharedMemoryReader() = default;
    ~SharedMemoryReader();
    SharedMemoryReader(SharedMemoryReader&&) noexcept;
    SharedMemoryReader& operator=(SharedMemoryReader&&) noexcept;
    SharedMemoryReader(const SharedMemoryReader&) = delete;
    SharedMemoryReader& operator=(const SharedMemoryReader&) = delete;

    // Open existing named shared memory for read. Returns true on success.
    bool open(const std::string& name, size_t size_bytes);

    // Return pointer to shared memory (read-only), or nullptr if not open. Size = size_bytes passed to open().
    const uint8_t* data() const { return ptr_; }
    size_t size() const { return size_; }
    bool is_open() const { return ptr_ != nullptr; }

    void close();

  private:
    void* handle_{nullptr};  // platform-specific mapping handle
    uint8_t* ptr_{nullptr};
    size_t size_{0};
};

}  // namespace bng_ros2
