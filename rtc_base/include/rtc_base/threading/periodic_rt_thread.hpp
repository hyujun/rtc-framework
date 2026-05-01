#ifndef RTC_BASE_THREADING_PERIODIC_RT_THREAD_HPP_
#define RTC_BASE_THREADING_PERIODIC_RT_THREAD_HPP_

// PeriodicRtThread — shared skeleton for fixed-frequency RT/soft-RT loops.
//
// Both the controller-manager 500 Hz RT loop and the MPC solve thread used
// to carry their own bespoke jthread + clock_nanosleep + overrun-detection
// + (in MPC's case) Pause/Resume scaffolding. The shape was identical:
//
//   1. ApplyThreadConfig at thread entry.
//   2. Sleep until the next deadline (clock_nanosleep TIMER_ABSTIME).
//   3. Detect lag > period, count overruns / missed ticks, recover.
//   4. Run a subclass-supplied tick body.
//   5. Capture per-phase wall-clock timing and push one
//      rtc::RtTickTimingPayload onto an SPSC ring (Sprint 1 schema).
//
// This base owns 1–4 + the per-tick t0..t3 capture for 5; subclasses bring
// the tick body + the optional timing producer + per-channel reactions
// (E-STOP, sim-CV wakeup).
//
// Header-only: rtc_base ships header-only and we want subclasses (rtc_mpc,
// rtc_controller_manager) to instantiate freely without taking another
// link-time dependency.
//
// Thread model:
//   - Start() spawns one std::jthread that runs RunLoop until RequestStop()
//     is observed (cooperative cancellation via stop_token).
//   - The loop calls subclass's OnTick() every period.
//   - Pause() makes the loop block on a condition_variable at the top of
//     the next iteration; Resume() / RequestStop() wake it.
//   - Subclasses may override WaitForNextTick (e.g. simulation CV wakeup)
//     and OnOverrun (E-STOP trigger). Defaults preserve the
//     deterministic clock_nanosleep + counter-only behaviour.

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"

#include <time.h>  // clock_nanosleep, clock_gettime, CLOCK_MONOTONIC, TIMER_ABSTIME

#include <atomic>
#include <chrono>
#include <cmath>  // std::abs
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stop_token>
#include <thread>

namespace rtc {

/// Configuration accepted by PeriodicRtThread::Start().
///
/// `frequency_hz` must be positive and fits in a 64-bit nanosecond period
/// (i.e. ~1 mHz lower bound). The thread config covers CPU affinity /
/// scheduler policy / priority / debug name and is forwarded to
/// rtc::ApplyThreadConfig at thread entry.
struct PeriodicRtThreadConfig {
  ThreadConfig thread_config{};
  double frequency_hz{0.0};
};

class PeriodicRtThread {
 public:
  PeriodicRtThread() = default;

  virtual ~PeriodicRtThread() { Join(); }

  PeriodicRtThread(const PeriodicRtThread&) = delete;
  PeriodicRtThread& operator=(const PeriodicRtThread&) = delete;
  PeriodicRtThread(PeriodicRtThread&&) = delete;
  PeriodicRtThread& operator=(PeriodicRtThread&&) = delete;

  /// Spawn the loop thread. Idempotent once running. `cfg.frequency_hz`
  /// must be > 0; otherwise the call is a no-op and Running() stays false.
  void Start(const PeriodicRtThreadConfig& cfg) {
    if (running_.exchange(true)) {
      return;
    }
    if (!(cfg.frequency_hz > 0.0)) {
      running_.store(false);
      return;
    }
    cfg_ = cfg;
    period_ns_ = static_cast<std::uint64_t>(1.0e9 / cfg.frequency_hz + 0.5);
    budget_us_ = static_cast<double>(period_ns_) / 1000.0;
    thread_ = std::jthread([this](std::stop_token stoken) { RunLoop(std::move(stoken)); });
  }

  /// Request the loop to stop and wake any pending pause/wait. Non-blocking.
  /// Safe to call from any non-RT thread, including the destructor.
  void RequestStop() noexcept {
    if (thread_.joinable()) {
      thread_.request_stop();
    }
    // Wake pause-cv so the loop observes stop_requested promptly.
    { std::lock_guard<std::mutex> lock(pause_mutex_); }
    pause_cv_.notify_all();
    // Subclasses with a custom WaitForNextTick (e.g. CM sim CV wakeup)
    // observe the stop via OnRequestStop() and wake their own primitives.
    OnRequestStop();
  }

  /// Join the loop thread. Idempotent; safe to call from the destructor.
  void Join() noexcept {
    RequestStop();
    if (thread_.joinable()) {
      thread_.join();
    }
    running_.store(false);
  }

  [[nodiscard]] bool Running() const noexcept { return running_.load(); }

  /// Suspend the loop after the current iteration completes. Idempotent.
  /// Safe to call from any non-RT thread.
  void Pause() noexcept { paused_.store(true); }

  /// Resume the loop. Idempotent.
  void Resume() noexcept {
    {
      std::lock_guard<std::mutex> lock(pause_mutex_);
      paused_.store(false);
    }
    pause_cv_.notify_all();
  }

  [[nodiscard]] bool Paused() const noexcept { return paused_.load(); }

  // ── Telemetry (drain-thread / non-RT) ───────────────────────────────────

  [[nodiscard]] std::uint64_t TickCount() const noexcept {
    return tick_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t OverrunCount() const noexcept {
    return overrun_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t SkipCount() const noexcept {
    return skip_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t ConsecutiveOverruns() const noexcept {
    return consecutive_overruns_.load(std::memory_order_relaxed);
  }

  /// Attach a per-tick timing producer. After each OnTick, base pushes one
  /// RtTickTimingPayload {t_state_us, t_compute_us, t_publish_us, t_total_us,
  /// jitter_us} via the producer's Push(). Pass any
  /// ThreadTimingProducer<RtTickTimingPayload, N> instance (CmTimingBuffer or
  /// MpcTimingBuffer); Push() is wait-free.
  ///
  /// Must be called before Start(); the loop reads `timing_push_` once per
  /// tick without locking.
  template <std::size_t N>
  void SetTimingProducer(ThreadTimingProducer<RtTickTimingPayload, N>* producer) noexcept {
    if (producer == nullptr) {
      timing_push_ = {};
      return;
    }
    timing_push_ = [producer](const RtTickTimingPayload& p) noexcept { (void)producer->Push(p); };
  }

 protected:
  enum class WaitResult { kProceed, kAbort };

  /// Subclass tick body. Base captures t0 before this is called and t3
  /// immediately after. Subclasses should call MarkStateAcquired() after
  /// the state-acquisition phase and MarkComputeDone() after the compute
  /// phase to record the intermediate breakpoints.
  ///
  /// OnTick is called only when the loop is not paused and stop has not
  /// been requested.
  virtual void OnTick() = 0;

  /// Stamp t1 — end of the state-acquisition phase. Call once per tick
  /// from inside OnTick().
  void MarkStateAcquired() noexcept { t1_ = std::chrono::steady_clock::now(); }

  /// Stamp t2 — end of the compute phase. Call once per tick from inside
  /// OnTick().
  void MarkComputeDone() noexcept { t2_ = std::chrono::steady_clock::now(); }

  /// Sleep until the next tick deadline. Default: clock_nanosleep
  /// TIMER_ABSTIME on CLOCK_MONOTONIC, advancing `next_wake_` by
  /// `period_ns_`. Detects lag > period and increments overrun / skip
  /// counters; calls OnOverrun() on each event.
  ///
  /// Subclasses with a non-deadline wakeup (e.g. CM simulation mode that
  /// blocks on a CV until /joint_states arrives) should override this
  /// method, perform their own wait, and return:
  ///   - kProceed: proceed to OnTick().
  ///   - kAbort:   exit the loop. OnLoopAborted() will be invoked once.
  ///
  /// Overrun detection is provided by the default implementation only.
  /// CV-driven subclasses get no overrun bookkeeping (timeouts there are
  /// handled by the abort path).
  virtual WaitResult WaitForNextTick() noexcept { return DefaultWaitClockNanosleep(); }

  /// Called once per overrun event (lag > period in the default wait).
  /// `consecutive` is the running count, reset to 0 on the next on-time
  /// tick. Default no-op. CM subclass overrides to trigger a global E-STOP
  /// once the count exceeds its threshold.
  virtual void OnOverrun(std::uint64_t /*consecutive*/) noexcept {}

  /// Called once when WaitForNextTick returned kAbort. Subclass logs /
  /// triggers a shutdown. Default no-op.
  virtual void OnLoopAborted() noexcept {}

  /// Called from RequestStop() so subclasses with custom wait primitives
  /// (CV, eventfd, …) can wake them. Runs on the thread that called
  /// RequestStop(), not the loop thread. Default no-op.
  virtual void OnRequestStop() noexcept {}

  /// Period in nanoseconds, derived from cfg.frequency_hz. Available to
  /// subclasses that override WaitForNextTick.
  [[nodiscard]] std::uint64_t PeriodNs() const noexcept { return period_ns_; }

 private:
  void RunLoop(std::stop_token stoken) {
    static_cast<void>(rtc::ApplyThreadConfig(cfg_.thread_config));

    // Anchor the absolute-time wake schedule to "now".
    clock_gettime(CLOCK_MONOTONIC, &next_wake_);
    wake_initialised_ = true;
    have_prev_t0_ = false;

    while (!stoken.stop_requested()) {
      // ── Pause gate ──────────────────────────────────────────────────────
      if (paused_.load()) {
        std::unique_lock<std::mutex> lock(pause_mutex_);
        pause_cv_.wait(lock,
                       [this, &stoken]() { return !paused_.load() || stoken.stop_requested(); });
        if (stoken.stop_requested()) {
          break;
        }
        // Re-anchor wake schedule + break the jitter chain so the first
        // post-resume tick isn't reported as a giant jitter against the
        // last pre-pause t0.
        clock_gettime(CLOCK_MONOTONIC, &next_wake_);
        have_prev_t0_ = false;
      }

      // ── Wait for next deadline (subclass-overridable) ───────────────────
      const WaitResult wr = WaitForNextTick();
      if (stoken.stop_requested()) {
        break;
      }
      if (wr == WaitResult::kAbort) {
        OnLoopAborted();
        break;
      }

      // ── Tick body ───────────────────────────────────────────────────────
      t0_ = std::chrono::steady_clock::now();
      t1_ = t0_;
      t2_ = t0_;

      OnTick();

      const auto t3 = std::chrono::steady_clock::now();
      tick_count_.fetch_add(1, std::memory_order_relaxed);

      // ── Per-tick payload push ───────────────────────────────────────────
      if (timing_push_) {
        RtTickTimingPayload p{};
        p.t_state_us = std::chrono::duration<double, std::micro>(t1_ - t0_).count();
        p.t_compute_us = std::chrono::duration<double, std::micro>(t2_ - t1_).count();
        p.t_publish_us = std::chrono::duration<double, std::micro>(t3 - t2_).count();
        p.t_total_us = std::chrono::duration<double, std::micro>(t3 - t0_).count();
        if (have_prev_t0_) {
          const double actual_period_us =
              std::chrono::duration<double, std::micro>(t0_ - prev_t0_).count();
          p.jitter_us = std::abs(actual_period_us - budget_us_);
        }
        timing_push_(p);
      }
      prev_t0_ = t0_;
      have_prev_t0_ = true;
    }
  }

  /// Default deadline wait: advance `next_wake_` by one period and sleep
  /// to it. Detect lag > period and recover by skipping ahead.
  WaitResult DefaultWaitClockNanosleep() noexcept {
    if (!wake_initialised_) {
      clock_gettime(CLOCK_MONOTONIC, &next_wake_);
      wake_initialised_ = true;
    }

    const auto p_ns = static_cast<std::int64_t>(period_ns_);
    next_wake_.tv_nsec += p_ns;
    if (next_wake_.tv_nsec >= 1'000'000'000L) {
      next_wake_.tv_sec += next_wake_.tv_nsec / 1'000'000'000L;
      next_wake_.tv_nsec %= 1'000'000'000L;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake_, nullptr);

    // Overrun detection.
    struct timespec now_ts {};

    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    const std::int64_t now_ns =
        static_cast<std::int64_t>(now_ts.tv_sec) * 1'000'000'000L + now_ts.tv_nsec;
    const std::int64_t wake_ns =
        static_cast<std::int64_t>(next_wake_.tv_sec) * 1'000'000'000L + next_wake_.tv_nsec;
    const std::int64_t lag_ns = now_ns - wake_ns;

    if (lag_ns > p_ns) {
      const std::int64_t missed_ticks = lag_ns / p_ns;
      const std::int64_t advance_ns = missed_ticks * p_ns;
      next_wake_.tv_nsec += static_cast<long>(advance_ns);
      if (next_wake_.tv_nsec >= 1'000'000'000L) {
        next_wake_.tv_sec += next_wake_.tv_nsec / 1'000'000'000L;
        next_wake_.tv_nsec %= 1'000'000'000L;
      }
      overrun_count_.fetch_add(1, std::memory_order_relaxed);
      skip_count_.fetch_add(static_cast<std::uint64_t>(missed_ticks), std::memory_order_relaxed);
      const auto consecutive = consecutive_overruns_.fetch_add(1, std::memory_order_relaxed) + 1;
      OnOverrun(consecutive);
      // Break jitter chain — next on-time tick reports 0 jitter against a
      // missing predecessor rather than a giant catch-up gap.
      have_prev_t0_ = false;
    } else {
      consecutive_overruns_.store(0, std::memory_order_relaxed);
    }
    return WaitResult::kProceed;
  }

  // ── State ───────────────────────────────────────────────────────────────
  PeriodicRtThreadConfig cfg_{};
  std::jthread thread_;
  std::atomic<bool> running_{false};

  // Pause / Resume.
  std::atomic<bool> paused_{false};
  std::mutex pause_mutex_;
  std::condition_variable pause_cv_;

  // Period / wakeup.
  std::uint64_t period_ns_{0};
  double budget_us_{0.0};

  struct timespec next_wake_ {};

  bool wake_initialised_{false};

  // Per-tick timestamps (loop-thread only — no synchronisation).
  std::chrono::steady_clock::time_point t0_{};
  std::chrono::steady_clock::time_point t1_{};
  std::chrono::steady_clock::time_point t2_{};
  std::chrono::steady_clock::time_point prev_t0_{};
  bool have_prev_t0_{false};

  // Counters (consumed off-loop).
  std::atomic<std::uint64_t> tick_count_{0};
  std::atomic<std::uint64_t> overrun_count_{0};
  std::atomic<std::uint64_t> skip_count_{0};
  std::atomic<std::uint64_t> consecutive_overruns_{0};

  // Optional timing producer — bound via SetTimingProducer<N>().
  std::function<void(const RtTickTimingPayload&)> timing_push_;
};

}  // namespace rtc

#endif  // RTC_BASE_THREADING_PERIODIC_RT_THREAD_HPP_
