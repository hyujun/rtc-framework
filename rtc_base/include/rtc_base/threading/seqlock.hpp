#ifndef RTC_BASE_THREADING_SEQLOCK_HPP_
#define RTC_BASE_THREADING_SEQLOCK_HPP_

// SeqLock: lock-free single-writer / multi-reader synchronisation.
//
// Designed for RT threads where the writer must never block.
// The writer is wait-free (two atomic stores + memcpy).
// The reader retries on torn reads (writer was active during read).
//
// Requirements: T must be trivially copyable.
//
// Usage:
//   SeqLock<MyData> sl;
//   // Writer (single thread, e.g. EventLoop):
//   sl.Store(new_data);
//   // Reader (any thread):
//   MyData snapshot = sl.Load();

#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace rtc {

inline constexpr std::size_t kSeqLockCacheLineSize = 64;

template <typename T>
class SeqLock {
  static_assert(std::is_trivially_copyable_v<T>,
                "SeqLock requires a trivially copyable type");

 public:
  SeqLock() noexcept = default;

  explicit SeqLock(const T& initial) noexcept : data_(initial) {}

  // ── Writer (single-producer, wait-free) ──────────────────────────────
  void Store(const T& val) noexcept {
    // Odd sequence = write in progress
    const uint32_t seq = seq_.load(std::memory_order_relaxed);
    seq_.store(seq + 1, std::memory_order_release);           // → odd

    std::memcpy(&data_, &val, sizeof(T));

    seq_.store(seq + 2, std::memory_order_release);           // → even (next)
  }

  // ── Reader (multi-consumer, lock-free with retry) ────────────────────
  [[nodiscard]] T Load() const noexcept {
    T result;
    uint32_t s0, s1;
    do {
      s0 = seq_.load(std::memory_order_acquire);

      std::memcpy(&result, &data_, sizeof(T));

      s1 = seq_.load(std::memory_order_acquire);
    } while (s0 != s1 || (s0 & 1u));
    // Retry if: sequence changed during read, or writer was active (odd)
    return result;
  }

  // ── Sequence number (for external staleness checks) ──────────────────
  [[nodiscard]] uint32_t sequence() const noexcept {
    return seq_.load(std::memory_order_acquire);
  }

 private:
  alignas(kSeqLockCacheLineSize) std::atomic<uint32_t> seq_{0};
  alignas(kSeqLockCacheLineSize) T data_{};
};

}  // namespace rtc

#endif  // RTC_BASE_THREADING_SEQLOCK_HPP_
