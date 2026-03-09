#pragma once
#include <atomic>
#include <cstdint>

namespace bng_ros2 {

// Lock-free single-producer single-consumer double buffer.
//
// Writer (physics/GFX thread): call write_slot() to get the current write slot,
// fill it, then call commit() to atomically flip write_idx and mark dirty.
//
// Reader (publish thread): spin on dirty, call read_slot() to get the stable slot
// (the one NOT currently being written), then clear dirty after consuming.
//
// Invariant: write_slot() and read_slot() always return different slots because
// read_slot() returns slot[write_idx ^ 1].
//
// alignas(64) on slot[] keeps the two slots on separate cache lines and aligns
// the struct to a cache-line boundary.
template<typename T>
struct SpscDoubleBuffer {
  alignas(64) T slot[2];
  std::atomic<int>  write_idx{0};
  std::atomic<bool> dirty{false};

  SpscDoubleBuffer() = default;

  // std::atomic is not movable, so we provide explicit move operations that transfer
  // slot data and reset atomics. Safe only during init before any concurrent access
  // (the only time a vector reallocation can move BlobSlots).
  SpscDoubleBuffer(SpscDoubleBuffer&& o) noexcept
      : slot{std::move(o.slot[0]), std::move(o.slot[1])},
        write_idx{o.write_idx.load(std::memory_order_relaxed)},
        dirty{o.dirty.load(std::memory_order_relaxed)} {}

  SpscDoubleBuffer& operator=(SpscDoubleBuffer&& o) noexcept {
    if (this != &o) {
      slot[0] = std::move(o.slot[0]);
      slot[1] = std::move(o.slot[1]);
      write_idx.store(o.write_idx.load(std::memory_order_relaxed),
                      std::memory_order_relaxed);
      dirty.store(o.dirty.load(std::memory_order_relaxed),
                  std::memory_order_relaxed);
    }
    return *this;
  }

  SpscDoubleBuffer(const SpscDoubleBuffer&)            = delete;
  SpscDoubleBuffer& operator=(const SpscDoubleBuffer&) = delete;

  // Writer side — call, fill, then commit().
  T& write_slot() {
    return slot[write_idx.load(std::memory_order_relaxed)];
  }

  // Flip write_idx so the just-filled slot becomes the stable read slot, then signal the reader.
  void commit() {
    const int next = write_idx.load(std::memory_order_relaxed) ^ 1;
    write_idx.store(next, std::memory_order_release);
    dirty.store(true, std::memory_order_release);
  }

  // Reader side — only call after observing dirty == true.
  // Returns the slot that is NOT currently being written.
  const T& read_slot() const {
    return slot[write_idx.load(std::memory_order_acquire) ^ 1];
  }

  // Mutable variant for cases where the reader needs to move-assign data out.
  T& read_slot_mut() {
    return slot[write_idx.load(std::memory_order_acquire) ^ 1];
  }
};

}  // namespace bng_ros2
