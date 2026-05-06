#include "memory/shared_memory_reader.h"

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

#if defined(_WIN32) || defined(_WIN64) // Windows

bool SharedMemoryReader::open(const std::string& name, size_t size_bytes) {
	close();
	HANDLE handle = ::OpenFileMappingA(FILE_MAP_READ, FALSE, name.c_str());
	if (!handle || handle == INVALID_HANDLE_VALUE) return false;

	void* ptr = ::MapViewOfFile(handle, FILE_MAP_READ, 0, 0, size_bytes);
	if (!ptr) {
		::CloseHandle(handle);
		return false;
	}

	handle_ = handle;
	ptr_ = static_cast<uint8_t*>(ptr);
	size_ = size_bytes;
	return true;
}

void SharedMemoryReader::close() {
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

#else  // Linux

bool SharedMemoryReader::open(const std::string& name, size_t size_bytes) {
	close();
	int file_descriptor = ::shm_open(name.c_str(), O_RDONLY, 0666);
	if (file_descriptor < 0) return false;

	void* ptr = ::mmap(nullptr, size_bytes, PROT_READ, MAP_SHARED, file_descriptor, 0);
	::close(file_descriptor);

	if (ptr == MAP_FAILED) return false;

	handle_ = ptr;
	ptr_ = static_cast<uint8_t*>(ptr);
	size_ = size_bytes;
	return true;
}

void SharedMemoryReader::close() {
	if (ptr_ && size_ > 0) {
		::munmap(ptr_, size_);
	}

	ptr_ = nullptr;
	handle_ = nullptr;
	size_ = 0;
}

#endif

SharedMemoryReader::~SharedMemoryReader() {
	close();
}

SharedMemoryReader::SharedMemoryReader(SharedMemoryReader&& other) noexcept : handle_(other.handle_), ptr_(other.ptr_), size_(other.size_) {
	other.handle_ = nullptr;
	other.ptr_ = nullptr;
	other.size_ = 0;
}

SharedMemoryReader& SharedMemoryReader::operator=(SharedMemoryReader&& other) noexcept {
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
