#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <condition_variable>

namespace bng_ros2 {

// Lock-free single-producer single-consumer double buffer.
//
// Writer (physics/GFX thread): call write_slot() to get the current write slot,
// fill it, then call commit() to atomically flip write_idx and mark dirty.
//
// Reader (publish thread): wait on the condition variable via wait_for_data(),
// then call read_slot() to get the stable slot (the one NOT currently being written),
// then call clear_dirty() after consuming.
//
// Atomicity guarantee: write_idx and the dirty flag are packed into a single atomic uint32_t (state_).
// Bit 0 = write_idx; bit 1 = dirty. commit() performs a single atomic store
// so the reader can never observe dirty=true with a stale write_idx (the torn-read hazard present when the two values are separate atomics).
//
// alignas(64) on slot[] keeps the two slots on separate cache lines and aligns
// the struct to a cache-line boundary.
template<typename T>
struct SpscDoubleBuffer {
  alignas(64) T slot[2];

  // Packed state: bit 0 = write_idx, bit 1 = dirty.
  std::atomic<uint32_t> state_{0};

  // Condition variable used by the publish thread to sleep when there is no data, eliminating the busy-spin CPU cost.
  // notify() is called by commit().
  // The mutex is only contended at the sleep/wake boundary, not on the hot path.
  std::mutex              cv_mutex_;
  std::condition_variable cv_;

  SpscDoubleBuffer() = default;

  // std::atomic and std::condition_variable are not movable, so we provide explicit
  // move operations that transfer slot data and reset synchronisation state.
  // Safe only during init before any concurrent access.
  SpscDoubleBuffer(SpscDoubleBuffer&& o) noexcept
      : slot{std::move(o.slot[0]), std::move(o.slot[1])},
        state_{o.state_.load(std::memory_order_relaxed)} {}

  SpscDoubleBuffer& operator=(SpscDoubleBuffer&& o) noexcept {
    if (this != &o) {
      slot[0] = std::move(o.slot[0]);
      slot[1] = std::move(o.slot[1]);
      state_.store(o.state_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
    return *this;
  }

  SpscDoubleBuffer(const SpscDoubleBuffer&)            = delete;
  SpscDoubleBuffer& operator=(const SpscDoubleBuffer&) = delete;

  // Writer side - call, fill, then commit().
  T& write_slot() {
    return slot[state_.load(std::memory_order_relaxed) & 1];
  }

  // Atomically flip write_idx and set dirty in a single store, then wake the reader.
  // state_ bit layout after commit: new write_idx = old write_idx ^ 1, dirty = 1.
  void commit() {
    const uint32_t prev  = state_.load(std::memory_order_relaxed);
    const uint32_t next  = (prev ^ 1u) | 2u;  // flip bit 0 (write_idx), set bit 1 (dirty)
    state_.store(next, std::memory_order_release);
    cv_.notify_one();
  }

  // Reader side: returns true immediately if dirty, otherwise blocks until commit() is called or the timeout expires.
  // max_wait_ms allows the publish thread to process other work (e.g. markers, spin_some) even with no sensor data.
  bool wait_for_data(int max_wait_ms = 1) {
    if (state_.load(std::memory_order_acquire) & 2u) return true;
    std::unique_lock<std::mutex> lk(cv_mutex_);
    cv_.wait_for(lk, std::chrono::milliseconds(max_wait_ms),
                 [this]{ return (state_.load(std::memory_order_acquire) & 2u) != 0; });
    return (state_.load(std::memory_order_acquire) & 2u) != 0;
  }

  bool is_dirty() const {
    return (state_.load(std::memory_order_acquire) & 2u) != 0;
  }

  void clear_dirty() {
    // Clear only bit 1 (dirty), preserve bit 0 (write_idx).
    state_.fetch_and(~2u, std::memory_order_release);
  }

  // Reader side - only call after observing dirty == true.
  // Returns the slot that is NOT currently being written.
  const T& read_slot() const {
    return slot[(state_.load(std::memory_order_acquire) & 1u) ^ 1u];
  }
};

}  // namespace bng_ros2
