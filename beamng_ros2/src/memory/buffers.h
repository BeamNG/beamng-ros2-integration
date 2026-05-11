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

struct ControlMessage {
    double steering{0};
    double throttle{0};
    double brake{0};
    double parkingbrake{0};
    double clutch{0};
    int32_t gear{0};
};

class ControlBuffer {
  public:
    void commit(const ControlMessage& message) {
        const int write_slot_index = 1 - read_slot_.load(std::memory_order_relaxed);
        slots_[static_cast<std::size_t>(write_slot_index)] = message;
        read_slot_.store(write_slot_index, std::memory_order_release);
        sequence_.fetch_add(1, std::memory_order_release);
    }

    uint64_t sequence() const { return sequence_.load(std::memory_order_acquire); }

    int read_snapshot(ControlMessage* out, uint64_t* out_version) const {
        if (!out || !out_version) return -1;
        constexpr int k_max_tries = 4;
        for (int attempt = 0; attempt < k_max_tries; ++attempt) {
            const uint64_t begin_sequence = sequence_.load(std::memory_order_acquire);
            if (begin_sequence == 0) return -1;
            
            const int read_index = read_slot_.load(std::memory_order_acquire);
            *out = slots_[static_cast<std::size_t>(read_index)];
            const uint64_t end_sequence = sequence_.load(std::memory_order_acquire);
            const int read_index2 = read_slot_.load(std::memory_order_acquire);
            if (begin_sequence == end_sequence && read_index == read_index2) {
                *out_version = end_sequence;
                return 0;
            }
        }
        return -1;
    }

  private:
    std::array<ControlMessage, 2> slots_{};
    std::atomic<int> read_slot_{0};
    std::atomic<uint64_t> sequence_{0};
};

}  // namespace bng_ros2
