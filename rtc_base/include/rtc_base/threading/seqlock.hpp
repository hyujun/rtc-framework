#ifndef RTC_BASE_THREADING_SEQLOCK_HPP_
#define RTC_BASE_THREADING_SEQLOCK_HPP_

// SeqLock: lock-free single-writer / multi-reader synchronisation.
//
// Designed for RT threads where the writer must never block.
// The writer is wait-free (two atomic stores, one fence, and a word-by-word
// copy of T). The reader retries on torn reads (writer was active during read).
//
// Requirements: T must be trivially copyable. Exactly one thread may call
// Store() at a time — two overlapping Store() calls can tear the payload or
// leave the sequence odd, and then every reader spins forever.
//
// Memory ordering (Boehm, "Can seqlocks get along with programming language
// memory models?", MSPC 2012; same shape as the E-STOP reason seqlock in
// rtc_controller_manager, #935):
//   writer  seq = odd (relaxed); fence(release); payload stores (relaxed);
//           seq = even (release)
//   reader  s0 = seq (acquire); payload loads (relaxed); fence(acquire);
//           s1 = seq (relaxed); accept iff s0 == s1 and s0 is even
// A release store only keeps EARLIER accesses before it, and an acquire load
// only keeps LATER accesses after it, so without the two fences the payload
// stores may become visible before the odd sequence and the payload loads may
// be satisfied after s1 — a mixed snapshot then passes s0 == s1. x86 (TSO)
// does not reorder these in hardware; ARMv8 does.
//
// The payload is held as an array of std::atomic<uint64_t> and copied with
// relaxed atomic accesses. The fence-to-fence synchronisation above is only
// defined for atomic accesses, and a plain memcpy racing with the writer is a
// data race (undefined behaviour) that ThreadSanitizer reports. Cost: the word
// copy is a few times slower than an inlined memcpy (x86: ~0.1-0.5 us for
// 4 KB, up to ~2 us for 16 KB, layout dependent; aarch64 not measured).
//
// Convergence: under the single-writer invariant a reader's retry window is
// bounded by the longest Store() observed across cores. In practice Load()
// converges in 1 iteration and worst-case in 2-3. The invariant "single writer
// + bounded write duration" must hold at the design level — there is no
// built-in retry cap.
//
// Usage:
//   SeqLock<MyData> sl;
//   // Writer (single thread, e.g. EventLoop):
//   sl.Store(new_data);
//   // Reader (any thread):
//   MyData snapshot = sl.Load();

#include "rtc_base/types/types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace rtc {

template <typename T>
class SeqLock {
  static_assert(std::is_trivially_copyable_v<T>, "SeqLock requires a trivially copyable type");

  using Word = std::uint64_t;
  static_assert(std::atomic<Word>::is_always_lock_free,
                "SeqLock payload words must be lock-free atomics");

  static constexpr std::size_t kFullWords = sizeof(T) / sizeof(Word);
  static constexpr std::size_t kTailBytes = sizeof(T) % sizeof(Word);
  static constexpr std::size_t kWords = kFullWords + (kTailBytes != 0 ? 1 : 0);

 public:
  // Holds the bytes of a value-initialised T, so default member initialisers
  // of T are honoured. Not constant-initialisable: a SeqLock with static
  // storage duration is initialised dynamically.
  SeqLock() noexcept : SeqLock(T{}) {}

  explicit SeqLock(const T& initial) noexcept { CopyIn(initial); }

  // ── Writer (single-producer, wait-free) ──────────────────────────────
  /// Publishes `val`. Only one thread may call this at a time (see the
  /// header comment); readers never block it.
  void Store(const T& val) noexcept {
    // Odd sequence = write in progress
    const uint32_t seq = seq_.load(std::memory_order_relaxed);
    seq_.store(seq + 1, std::memory_order_relaxed);  // → odd
    // Keeps the payload stores below from becoming visible before the odd
    // sequence: a reader that sees any of them also sees seq odd or newer.
    std::atomic_thread_fence(std::memory_order_release);

    CopyIn(val);

    seq_.store(seq + 2, std::memory_order_release);  // → even (next)
  }

  // ── Reader (multi-consumer, lock-free with retry) ────────────────────
  [[nodiscard]] T Load() const noexcept {
    T result;
    LoadInto(result);
    return result;
  }

  /// Same read, copied straight into caller-owned storage. For payloads large
  /// enough that the by-value Load() matters — Load() builds the copy in a
  /// local and returns it, so a caller assigning it to a member pays the
  /// payload once more on the stack. `out` holds a consistent snapshot on
  /// return; between retries it may briefly hold a torn one, so it must not be
  /// storage another thread reads. Retries without bound while a write is in
  /// progress (see Convergence in the header comment).
  void LoadInto(T& out) const noexcept {
    for (;;) {
      const uint32_t s0 = seq_.load(std::memory_order_acquire);
      if ((s0 & 1u) != 0) {
        continue;  // writer active — do not copy a payload known to be torn
      }

      CopyOut(out);

      // Keeps the payload loads above from being satisfied after the second
      // sequence load: if any of them saw a later Store(), s1 differs from s0.
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == s0) {
        return;
      }
    }
  }

  // ── Sequence number (for external staleness checks) ──────────────────
  [[nodiscard]] uint32_t sequence() const noexcept { return seq_.load(std::memory_order_acquire); }

 private:
  void CopyIn(const T& val) noexcept {
    const auto* src = reinterpret_cast<const unsigned char*>(&val);
    for (std::size_t i = 0; i < kFullWords; ++i) {
      Word w = 0;
      std::memcpy(&w, src + i * sizeof(Word), sizeof(Word));
      words_[i].store(w, std::memory_order_relaxed);
    }
    if constexpr (kTailBytes != 0) {
      Word w = 0;
      std::memcpy(&w, src + kFullWords * sizeof(Word), kTailBytes);
      words_[kFullWords].store(w, std::memory_order_relaxed);
    }
  }

  void CopyOut(T& out) const noexcept {
    auto* dst = reinterpret_cast<unsigned char*>(&out);
    for (std::size_t i = 0; i < kFullWords; ++i) {
      const Word w = words_[i].load(std::memory_order_relaxed);
      std::memcpy(dst + i * sizeof(Word), &w, sizeof(Word));
    }
    if constexpr (kTailBytes != 0) {
      const Word w = words_[kFullWords].load(std::memory_order_relaxed);
      std::memcpy(dst + kFullWords * sizeof(Word), &w, kTailBytes);
    }
  }

  alignas(kCacheLineSize) std::atomic<uint32_t> seq_{0};
  alignas(kCacheLineSize) std::array<std::atomic<Word>, kWords> words_{};
};

}  // namespace rtc

#endif  // RTC_BASE_THREADING_SEQLOCK_HPP_
