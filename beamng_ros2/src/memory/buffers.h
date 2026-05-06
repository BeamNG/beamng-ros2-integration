#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

namespace bng_ros2 {

// Producer (Lua physics step) writes into write_slot(), then commit() flips the active slot and raises the dirty flag.
// Consumer (publish thread) reads read_slot() while is_dirty() is true and calls clear_dirty() after publishing.
template <typename T>
class ScalarBuffer {
  public:
    std::vector<T>& write_slot() {
        const int front = read_front_.load(std::memory_order_relaxed);
        return buffers_[1 - front];
    }

    const std::vector<T>& read_slot() const {
        return buffers_[read_front_.load(std::memory_order_acquire)];
    }

    void commit() {
        const int front = read_front_.load(std::memory_order_relaxed);
        read_front_.store(1 - front, std::memory_order_release);
        dirty_.store(true, std::memory_order_release);
    }

    void clear_dirty() { dirty_.store(false, std::memory_order_release); }

    void resize_both(std::size_t n, const T& value) {
        buffers_[0].resize(n, value);
        buffers_[1].resize(n, value);
    }

  private:
    std::vector<T>   buffers_[2];
    std::atomic<int>  read_front_{0};
    std::atomic<bool> dirty_{false};
};

// Two fixed-capacity byte buffers, with a single atomic holding two dirty flags.
// Lua picks whichever side is clean and commits it
// The publish thread drains the dirty side(s) and clears them.
class BlobBuffer {
  public:
    // flags are public because shared directly to Lua with FFI
    static constexpr uint32_t k_dirty_a = 1u;
    static constexpr uint32_t k_dirty_b = 2u;

    explicit BlobBuffer(std::size_t capacity)
        : capacity_(capacity), buf_{ std::vector<uint8_t>(capacity), std::vector<uint8_t>(capacity) } {}

    std::size_t capacity() const { return capacity_; }
    uint8_t* data_a() { return buf_[0].data(); }
    uint8_t* data_b() { return buf_[1].data(); }

    std::atomic<uint32_t>* state_atomic() { return &state_; }
    uint32_t state_acquire() const { return state_.load(std::memory_order_acquire); }

    uint32_t payload_len_a() const { return len_a_.load(std::memory_order_acquire); }
    uint32_t payload_len_b() const { return len_b_.load(std::memory_order_acquire); }

    void commit_write_a(uint32_t length_bytes) {
        len_a_.store(length_bytes, std::memory_order_release);
        state_.fetch_or(k_dirty_a, std::memory_order_release);
    }
    void commit_write_b(uint32_t length_bytes) {
        len_b_.store(length_bytes, std::memory_order_release);
        state_.fetch_or(k_dirty_b, std::memory_order_release);
    }

    void clear_a_dirty() { state_.fetch_and(~k_dirty_a, std::memory_order_acq_rel); }
    void clear_b_dirty() { state_.fetch_and(~k_dirty_b, std::memory_order_acq_rel); }

  private:
    std::size_t capacity_;
    std::array<std::vector<uint8_t>, 2> buf_;
    std::atomic<uint32_t> state_{0};
    std::atomic<uint32_t> len_a_{0};
    std::atomic<uint32_t> len_b_{0};
};

}  // namespace bng_ros2
