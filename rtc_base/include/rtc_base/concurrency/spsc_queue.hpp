#ifndef RTC_BASE_CONCURRENCY_SPSC_QUEUE_HPP_
#define RTC_BASE_CONCURRENCY_SPSC_QUEUE_HPP_

// Lock-free single-producer / single-consumer (SPSC) ring buffer of
// fixed-size POD entries. Generalized from the previous `SpscLogBuffer` /
// `SpscPublishBuffer` patterns — payload type is now a template parameter.
//
// Constraints:
//   - Exactly ONE producer thread (Push).
//   - Exactly ONE consumer thread (Pop).
//   - Capacity N must be a power of 2 (enforced via static_assert).
//   - T must be trivially copyable so `buffer_[i] = entry` does not allocate
//     and so the bounded-storage `std::array<T, N>` is RT-safe to construct.
//
// Memory ordering is the same scheme used by the original two buffers:
//   - Producer publishes the new head_ with release; consumer reads head_
//     with acquire when its cached value is stale.
//   - Consumer publishes the new tail_ with release; producer reads tail_
//     with acquire when its cached value is stale.
//   - drop_count_ uses relaxed ordering — diagnostic only.
//
// Cache-line padding mirrors the original layout: head/tail/drop_count
// each on their own line to avoid false sharing.

#include "rtc_base/types/types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc
{

template<typename T, std::size_t N>
class SpscQueue {
  static_assert(N > 0 && (N & (N - 1)) == 0,
                "SpscQueue capacity N must be a power of 2");
  static_assert(std::is_trivially_copyable_v<T>,
                "SpscQueue payload T must be trivially copyable");

public:
  using value_type = T;
  static constexpr std::size_t kCapacity = N;

  // Producer side. Returns false (and drops the entry) when the buffer is
  // full — no blocking, no allocation.
  [[nodiscard]] bool Push(const T & entry) noexcept
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & (N - 1);

    if (next == cached_tail_) [[unlikely]] {
      cached_tail_ = tail_.load(std::memory_order_acquire);
      if (next == cached_tail_) [[unlikely]] {
        drop_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
    }

    buffer_[head] = entry;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Consumer side. Returns false when the buffer is empty.
  [[nodiscard]] bool Pop(T & out) noexcept
  {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);

    if (tail == cached_head_) {
      cached_head_ = head_.load(std::memory_order_acquire);
      if (tail == cached_head_) {
        return false;
      }
    }

    out = buffer_[tail];
    tail_.store((tail + 1) & (N - 1), std::memory_order_release);
    return true;
  }

  // Lifetime drop counter. Safe to call from any thread.
  [[nodiscard]] std::uint64_t drop_count() const noexcept
  {
    return drop_count_.load(std::memory_order_relaxed);
  }

private:
  std::array<T, N> buffer_{};

  alignas(kCacheLineSize) std::atomic<std::size_t> head_{0};
  std::size_t cached_tail_{0};

  alignas(kCacheLineSize) std::atomic<std::size_t> tail_{0};
  std::size_t cached_head_{0};

  alignas(kCacheLineSize) std::atomic<std::uint64_t> drop_count_{0};
};

} // namespace rtc

#endif // RTC_BASE_CONCURRENCY_SPSC_QUEUE_HPP_
