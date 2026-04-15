#ifndef RTC_MPC_COMM_TRIPLE_BUFFER_HPP_
#define RTC_MPC_COMM_TRIPLE_BUFFER_HPP_

/// @file triple_buffer.hpp
/// @brief Lock-free single-producer / single-consumer triple buffer.
///
/// Three storage slots are rotated through three roles:
///   * `writing` — producer is currently populating this slot
///   * `latest`  — most-recently published slot, not yet observed by consumer
///   * `reading` — slot held by the consumer for zero-copy reads
///
/// Publication and acquisition are each a single atomic exchange on
/// @ref latest_, so the producer and consumer can never simultaneously name
/// the same buffer. The consumer receives a `const T*` into the shared array
/// and holds exclusive access to that slot until the next acquire — there is
/// no memcpy on either side.
///
/// The `latest_` word packs both the slot index (bits 0..1) and a "dirty"
/// flag (bit 2) that signals "newly published, not yet consumed". This lets
/// both publish and acquire be single XCHG operations, which is the key to
/// eliminating the classic two-step triple-buffer race where a consumer can
/// acquire between the producer's two stores.
///
/// Contract:
///   * Exactly one producer thread, one consumer thread.
///   * `T` must be trivially copyable (buffers are default-constructed in
///     the container, not placement-new'd).

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {

template <typename T>
class TripleBuffer {
  static_assert(std::is_trivially_copyable_v<T>,
                "TripleBuffer requires a trivially copyable type");

 public:
  TripleBuffer() noexcept = default;
  ~TripleBuffer() = default;

  TripleBuffer(const TripleBuffer&) = delete;
  TripleBuffer& operator=(const TripleBuffer&) = delete;
  TripleBuffer(TripleBuffer&&) = delete;
  TripleBuffer& operator=(TripleBuffer&&) = delete;

  // ── Producer API (single thread, wait-free) ────────────────────────

  /// @brief Writable reference to the producer-owned slot.
  /// @note Valid until the next @ref FinishWrite call.
  [[nodiscard]] T& StartWrite() noexcept {
    return buffers_[writing_];
  }

  /// @brief Publish the currently-written slot as `latest`.
  ///
  /// Single atomic exchange: swap our `writing` index into `latest_`
  /// (with the dirty bit set) and take whatever was there as our next
  /// scratchpad. No intermediate state is observable — the consumer
  /// either sees the previous `latest` or the new one, never an alias.
  void FinishWrite() noexcept {
    const uint8_t new_latest =
        static_cast<uint8_t>(writing_) | kDirtyBit;
    const uint8_t previous = latest_.exchange(new_latest,
                                              std::memory_order_acq_rel);
    writing_ = static_cast<uint8_t>(previous & kIndexMask);
  }

  // ── Consumer API (single thread, zero-copy, wait-free) ─────────────

  /// @brief Try to acquire the newest published buffer.
  /// @return `nullptr` if no new data has been published since the previous
  ///         acquire; otherwise a pointer into the shared array that stays
  ///         valid until the next @ref TryAcquireLatest call.
  [[nodiscard]] const T* TryAcquireLatest() noexcept {
    // Fast path: if dirty bit is clear, no new data is available.
    if ((latest_.load(std::memory_order_acquire) & kDirtyBit) == 0) {
      return nullptr;
    }
    // Swap our current `reading` slot into `latest` with dirty=0. Take
    // whatever was in `latest` as our new read slot. Single atomic op.
    const uint8_t new_latest = reading_;  // dirty bit implicitly 0
    const uint8_t previous = latest_.exchange(new_latest,
                                              std::memory_order_acq_rel);
    if ((previous & kDirtyBit) == 0) {
      // Lost the race with another (nonexistent) consumer, or another
      // thread cleared the dirty bit. Unreachable under the single-consumer
      // contract — restore state and report no data.
      latest_.store(previous, std::memory_order_release);
      return nullptr;
    }
    reading_ = static_cast<uint8_t>(previous & kIndexMask);
    return &buffers_[reading_];
  }

  /// @brief Access the slot most recently acquired by the consumer.
  /// @warning Only meaningful after a successful @ref TryAcquireLatest.
  ///          Before the first successful acquire the returned buffer is
  ///          default-constructed.
  [[nodiscard]] const T& Current() const noexcept {
    return buffers_[reading_];
  }

  /// @brief Query whether a new buffer is waiting.
  /// @note Advisory only — `TryAcquireLatest` is the authoritative path.
  [[nodiscard]] bool HasNewData() const noexcept {
    return (latest_.load(std::memory_order_acquire) & kDirtyBit) != 0;
  }

 private:
  static constexpr uint8_t kIndexMask = 0b0000'0011;
  static constexpr uint8_t kDirtyBit  = 0b0000'0100;

  // The three indices {writing_, latest_, reading_} are always distinct.
  // `writing_` and `reading_` are owned by the producer and consumer
  // respectively (single-threaded access), so they need not be atomic.
  std::array<T, 3> buffers_{};
  uint8_t writing_{0};
  std::atomic<uint8_t> latest_{1};
  uint8_t reading_{2};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_COMM_TRIPLE_BUFFER_HPP_
