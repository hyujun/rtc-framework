#ifndef RTC_MPC_THREAD_SOLVE_TRIGGER_CHANNEL_HPP_
#define RTC_MPC_THREAD_SOLVE_TRIGGER_CHANNEL_HPP_

/// @file solve_trigger_channel.hpp
/// @brief The handshake between a consumer tick and an MPC thread it
///        triggers (`MpcThreadLaunchConfig::trigger == kExternal`): "solve
///        now" one way, "that solve is done" the other, over two eventfds and
///        two ids — no mutex, no condition variable (RT-4, RT-10).
///
/// Ordering: the consumer increments `requested_id` and writes the trigger
/// fd. The thread wakes, drains the fd, loads `requested_id` (several
/// requests behind one wake collapse into one solve of the latest id),
/// solves, stores `completed_id` and writes the done fd. The consumer waits
/// for `completed_id == requested_id` (@ref WaitIdle) on the done fd, in
/// bounded slices so its caller can look at its own stop conditions.
///
/// Arming: @ref Arm aligns the ids and drains both fds before the first
/// request; @ref Disarm clears the flag and nudges both fds so a waiter on
/// either side wakes and sees it. A request while disarmed is ignored and
/// counted; a wait while disarmed returns `kCancelled` at once.
///
/// Ownership: the consumer owns the channel (configure → cleanup) and the
/// thread holds a raw pointer for the channel's lifetime — the consumer's
/// tick never touches the thread object. Threads: one requester, one
/// solver; `Arm` / `Disarm` from the consumer's lifecycle thread, never
/// concurrently with its tick.

#include <atomic>
#include <cstdint>

namespace rtc::mpc {

class SolveTriggerChannel {
 public:
  enum class WaitStatus : std::uint8_t {
    kIdle,       ///< every request has been answered
    kTimeout,    ///< the timeout passed with a request outstanding
    kCancelled,  ///< the channel is not armed (or not open)
  };

  SolveTriggerChannel() = default;

  ~SolveTriggerChannel() { Close(); }

  SolveTriggerChannel(const SolveTriggerChannel&) = delete;
  SolveTriggerChannel& operator=(const SolveTriggerChannel&) = delete;
  SolveTriggerChannel(SolveTriggerChannel&&) = delete;
  SolveTriggerChannel& operator=(SolveTriggerChannel&&) = delete;

  /// @brief Create both eventfds (non-blocking, close-on-exec). Not RT.
  /// @return false when the kernel refused one (the channel stays closed).
  [[nodiscard]] bool Open() noexcept;

  /// @brief Disarm and close the fds. Not RT; no waiter may be inside.
  void Close() noexcept;

  [[nodiscard]] bool Opened() const noexcept { return trigger_fd_ >= 0 && done_fd_ >= 0; }

  // ── Consumer side ─────────────────────────────────────────────────────

  /// @brief Align the ids, drain both fds, set the flag. Lifecycle thread,
  ///        with the tick not running. A closed channel stays disarmed.
  void Arm() noexcept;

  /// @brief Clear the flag and wake both sides. Any non-RT thread.
  void Disarm() noexcept;

  [[nodiscard]] bool Armed() const noexcept { return armed_.load(std::memory_order_acquire); }

  /// @brief Ask for one solve. RT: an increment and an eventfd write. While
  ///        disarmed: nothing (counted, @ref IgnoredRequests). With a request
  ///        still outstanding: counted (@ref CoalescedRequests) — the thread
  ///        answers the latest id with one solve.
  void RequestSolve() noexcept;

  /// @brief Wait until every request is answered, up to @p timeout_ns. RT:
  ///        poll on the done fd, EINTR retried; the slot's count drained.
  [[nodiscard]] WaitStatus WaitIdle(std::int64_t timeout_ns) noexcept;

  [[nodiscard]] bool Idle() const noexcept {
    return completed_id_.load(std::memory_order_acquire) ==
           requested_id_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::uint64_t RequestedId() const noexcept {
    return requested_id_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::uint64_t CompletedId() const noexcept {
    return completed_id_.load(std::memory_order_acquire);
  }

  // ── Solver side ───────────────────────────────────────────────────────

  /// @brief Wait up to @p timeout_ms for a request. Drains the trigger fd.
  /// @return true when a request is outstanding (`RequestedId() >
  ///         CompletedId()`), false on timeout, a poll error or a nudge
  ///         without a request (the caller looks at its own stop flag).
  [[nodiscard]] bool WaitTrigger(int timeout_ms) noexcept;

  /// @brief Answer the request @p id: store it and write the done fd.
  void MarkCompleted(std::uint64_t id) noexcept;

  /// @brief Wake both sides without a request (a stop). Any thread.
  void Nudge() noexcept;

  // ── Counters (relaxed; any thread) ────────────────────────────────────

  [[nodiscard]] std::uint64_t IgnoredRequests() const noexcept {
    return ignored_requests_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t CoalescedRequests() const noexcept {
    return coalesced_requests_.load(std::memory_order_relaxed);
  }

 private:
  static void Drain(int fd) noexcept;
  static void Write(int fd) noexcept;

  int trigger_fd_{-1};
  int done_fd_{-1};
  std::atomic<bool> armed_{false};
  std::atomic<std::uint64_t> requested_id_{0};
  std::atomic<std::uint64_t> completed_id_{0};
  std::atomic<std::uint64_t> ignored_requests_{0};
  std::atomic<std::uint64_t> coalesced_requests_{0};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_THREAD_SOLVE_TRIGGER_CHANNEL_HPP_
