#include "shmem_reader.h"
#include <cstring>

#if defined(_WIN32) || defined(_WIN64)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace bng_ros2 {

#if defined(_WIN32) || defined(_WIN64)

bool ShmemReader::open(const std::string& name, size_t size_bytes) {
  close();
  HANDLE h = ::OpenFileMappingA(FILE_MAP_READ, FALSE, name.c_str());
  if (!h || h == INVALID_HANDLE_VALUE) return false;
  void* p = ::MapViewOfFile(h, FILE_MAP_READ, 0, 0, size_bytes);
  if (!p) {
    ::CloseHandle(h);
    return false;
  }
  handle_ = h;
  ptr_ = static_cast<uint8_t*>(p);
  size_ = size_bytes;
  return true;
}

void ShmemReader::close() {
  if (ptr_) {
    ::UnmapViewOfFile(ptr_);
    ptr_ = nullptr;
  }
  if (handle_) {
    ::CloseHandle(static_cast<HANDLE>(handle_));
    handle_ = nullptr;
  }
  size_ = 0;
}

#else

bool ShmemReader::open(const std::string& name, size_t size_bytes) {
  close();
  int fd = ::shm_open(name.c_str(), O_RDONLY, 0666);
  if (fd < 0) return false;
  void* p = ::mmap(nullptr, size_bytes, PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (p == MAP_FAILED) return false;
  handle_ = p;
  ptr_ = static_cast<uint8_t*>(p);
  size_ = size_bytes;
  return true;
}

void ShmemReader::close() {
  if (ptr_ && size_ > 0) {
    ::munmap(ptr_, size_);
  }
  ptr_ = nullptr;
  handle_ = nullptr;
  size_ = 0;
}

#endif

ShmemReader::~ShmemReader() {
  close();
}

ShmemReader::ShmemReader(ShmemReader&& other) noexcept
    : handle_(other.handle_),
      ptr_(other.ptr_),
      size_(other.size_) {
  other.handle_ = nullptr;
  other.ptr_ = nullptr;
  other.size_ = 0;
}

ShmemReader& ShmemReader::operator=(ShmemReader&& other) noexcept {
  if (this != &other) {
    close();
    handle_ = other.handle_;
    ptr_ = other.ptr_;
    size_ = other.size_;
    other.handle_ = nullptr;
    other.ptr_ = nullptr;
    other.size_ = 0;
  }
  return *this;
}

}  // namespace bng_ros2
