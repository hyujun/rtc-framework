#ifndef RTC_BASE_LOGGING_THREAD_CSV_PRODUCER_HPP_
#define RTC_BASE_LOGGING_THREAD_CSV_PRODUCER_HPP_

// Generic per-tick CSV producer for controller-owned data logging.
//
// Sibling of rtc::ThreadTimingProducer (rtc_base/timing/), but unlike
// the timing variant this template:
//   - Does NOT emit `t_wall_ns` / `tick_count` auto-columns. The caller's
//     Payload owns the entire CSV row — every column the writer emits
//     comes from the Payload itself. Controllers that want a timestamp
//     embed `state.t_relative_s` (filled by CM RT loop) into the Payload
//     before pushing.
//   - Imposes no schema contract beyond "Payload must be trivially
//     copyable" (forced by SpscQueue<T> at compile time).
//
// One template covers state-log POD, sensor-log POD, inference-log POD,
// and any future per-controller CSV channel. Each Payload defines its
// own header/row writer — typically free functions next to the POD
// type (see ur5e_bringup/include/ur5e_bringup/logging/).
//
// Threading contract:
//   - Push() called from the producer thread only (the controller's RT
//     Compute() chain — Q-ACTIVITY-GATING).
//   - Drain() / DropCount() called from the consumer thread only
//     (controller's own non-RT drain timer).
//
// Header-only.

#include "rtc_base/concurrency/spsc_queue.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc {

template <typename Payload, std::size_t N>
class ThreadCsvProducer {
 public:
  static_assert(std::is_trivially_copyable_v<Payload>,
                "ThreadCsvProducer Payload must be trivially copyable");
  static constexpr std::size_t kCapacity = N;

  /// Push one Payload onto the SPSC ring. Wait-free; on a full ring the
  /// sample is dropped and `DropCount()` increments. RT-safe.
  [[nodiscard]] bool Push(const Payload& payload) noexcept { return queue_.Push(payload); }

  /// Drain pending samples in FIFO order. Returns the number drained.
  /// Non-RT.
  template <typename Fn>
  std::size_t Drain(Fn&& on_sample) noexcept {
    std::size_t n = 0;
    Payload p{};
    while (queue_.Pop(p)) {
      on_sample(p);
      ++n;
    }
    return n;
  }

  /// Lifetime count of samples dropped due to a full ring. Non-RT.
  [[nodiscard]] std::uint64_t DropCount() const noexcept { return queue_.drop_count(); }

 private:
  SpscQueue<Payload, N> queue_;
};

}  // namespace rtc

#endif  // RTC_BASE_LOGGING_THREAD_CSV_PRODUCER_HPP_
