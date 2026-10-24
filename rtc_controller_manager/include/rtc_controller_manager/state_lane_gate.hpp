#ifndef RTC_CONTROLLER_MANAGER_STATE_LANE_GATE_H_
#define RTC_CONTROLLER_MANAGER_STATE_LANE_GATE_H_

#include <atomic>
#include <chrono>
#include <thread>

namespace rtc {

// ── StateLaneGate ────────────────────────────────────────────────────────────
//
// Admission gate plus in-flight counter for callbacks that one thread runs and
// another must be able to retire (issue #941). The retiring thread calls
// Close(): from then on TryEnter() refuses, and Close() returns once every
// admitted caller has left — or false when one is still inside at the timeout.
//
// Why this and not the executor: rclcpp keeps only the Subscription alive for
// the duration of a dispatch, and nothing short of cancel()+join() on the
// executor waits for a running callback. CM holds no executor handle on its
// teardown paths (RtControllerMain owns them), so the wait has to live with
// the callback itself. Each DeviceBackend owns one gate and wraps every
// state-lane callback with it (DeviceBackend::GuardStateLane); the gate is
// shared with the wrapper by shared_ptr, so it outlives the backend for as
// long as a subscription that may still dispatch does.
//
// Memory order (Dekker pair, both seq_cst so the two sides sit in one total
// order): TryEnter publishes in_flight BEFORE it reads closed; Close publishes
// closed BEFORE it reads in_flight. Either the caller sees the close and backs
// out, or Close sees the caller and waits. Exit is a release so the body's
// writes happen-before the destroy that follows a successful Close.
//
// RT contract (TryEnter / Exit run on cb_group_rt_callback_, SCHED_FIFO):
// two atomic RMWs and one load per message, no allocation, no lock, no wait.
// Close polls with a sleep and is for the lifecycle thread only.
class StateLaneGate {
 public:
  /// Admits the caller unless the gate is closed. A refused caller owes no Exit.
  [[nodiscard]] bool TryEnter() noexcept {
    in_flight_.fetch_add(1, std::memory_order_seq_cst);
    if (closed_.load(std::memory_order_seq_cst)) {
      in_flight_.fetch_sub(1, std::memory_order_release);
      return false;
    }
    return true;
  }

  /// Pairs with a TryEnter that returned true.
  void Exit() noexcept { in_flight_.fetch_sub(1, std::memory_order_release); }

  /// Closes the gate and waits up to `timeout` for the admitted callers to
  /// leave. Returns false when one is still inside at the deadline — the gate
  /// stays closed either way, so a later TryEnter is still refused. Not RT.
  [[nodiscard]] bool Close(std::chrono::steady_clock::duration timeout) noexcept {
    closed_.store(true, std::memory_order_seq_cst);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (in_flight_.load(std::memory_order_seq_cst) != 0) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return in_flight_.load(std::memory_order_seq_cst) == 0;
      }
      std::this_thread::sleep_for(kClosePollInterval);
    }
    return true;
  }

  [[nodiscard]] bool IsClosed() const noexcept { return closed_.load(std::memory_order_acquire); }

 private:
  // Short against the measured callback body (mean ~7 us, p99 < 200 us —
  // issue #941), long enough not to burn the lifecycle thread's core.
  static constexpr std::chrono::microseconds kClosePollInterval{20};

  std::atomic<bool> closed_{false};
  std::atomic<int> in_flight_{0};
};

/// RAII admission: `if (!entry) return;` at the top of the guarded body, and
/// the Exit is paid on every path out of it. Not copyable or movable — one
/// entry per dispatch, by construction.
class StateLaneEntry {
 public:
  explicit StateLaneEntry(StateLaneGate& gate) noexcept : gate_(gate), admitted_(gate.TryEnter()) {}

  ~StateLaneEntry() noexcept {
    if (admitted_) {
      gate_.Exit();
    }
  }

  [[nodiscard]] explicit operator bool() const noexcept { return admitted_; }

  StateLaneEntry(const StateLaneEntry&) = delete;
  StateLaneEntry& operator=(const StateLaneEntry&) = delete;
  StateLaneEntry(StateLaneEntry&&) = delete;
  StateLaneEntry& operator=(StateLaneEntry&&) = delete;

 private:
  StateLaneGate& gate_;
  bool admitted_;
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_STATE_LANE_GATE_H_
