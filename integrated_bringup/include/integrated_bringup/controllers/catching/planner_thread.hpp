#ifndef INTEGRATED_BRINGUP_CONTROLLERS_CATCHING_PLANNER_THREAD_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_CATCHING_PLANNER_THREAD_HPP_

// ── Catching planner thread (dynamic_catching S6-A, D-7, L3 §5.3) ───────────
//
// A `rtc::PeriodicRtThread` sibling of `rtc::mpc::MPCThread`, not a subclass of
// it: the MPC thread's interface is built around `MPCSolution`, and forcing a
// PlanSnapshot through it would couple the two for nothing (plan §6). The
// thread is thin on purpose — the whole iteration body is
// `rtc::catching::PlannerCycle::Run` (rtc_controllers, ROS-free), so what the
// thread adds is only the WAKE SOURCE and the scheduling.
//
// PLACEMENT (E-7 decision J, 2026-09-23). The planner plays the same role as
// the MPC solver — a solver thread feeding the RT loop — so it takes the
// existing `mpc` layout role (`SelectThreadConfigs().mpc.main`) and its thread
// name, `mpc_main`. There is no layout role of its own. A `ps -L` that shows
// two `mpc_main` rows while a catching controller is active is expected: one
// is this thread, the other a DemoWbc MPC thread paused by its controller's
// deactivation. CM keeps one controller active at a time and each pauses its
// solver on deactivate, so only one of them RUNS on that core.
//
// WAKE SOURCE. Event driven (D-7c): the vision subscription writes the wake
// eventfd whenever it accepts a trajectory, and `WaitForNextTick` polls it with
// `planner.wake_timeout_s` as the upper bound, so the planner also re-plans
// against a moving RT state when vision is quiet. A non-semaphore eventfd read
// drains the whole counter: any number of signals between two wakes coalesce
// into ONE wake, and that wake always reads the newest snapshot (G3-L). The RT
// tick never touches the eventfd.
//
// PAUSE DOES NOT STOP AN ITERATION IN FLIGHT. `Pause()` is a flag the loop
// checks at the top of the next iteration; a wake already past it runs to the
// end and can publish once after `on_deactivate`. The RT refuses that plan by
// its activation generation (D-23, JudgePlan (b)), which is why this thread
// does not try to make Pause synchronous.

#include "integrated_bringup/logging/planner_events_csv.hpp"
#include "rtc_base/concurrency/spsc_queue.hpp"
#include "rtc_base/threading/periodic_rt_thread.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_base/timing/thread_timing_producer.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace integrated_bringup {

class CatchingPlannerThread final : public rtc::PeriodicRtThread {
 public:
  /// Per-wake timing ring (drained by the controller's aux timer into
  /// `<session>/timing/planner_timing_log.csv`). Sized for ~25 s of 20 Hz
  /// wakes, several drain periods.
  static constexpr std::size_t kTimingCapacity = 512;
  using TimingBuffer = rtc::ThreadTimingProducer<rtc::RtTickTimingPayload, kTimingCapacity>;
  /// Per-wake records for planner_events.csv (decision E). Same drain as the
  /// timing ring; a full ring drops the NEWEST row and counts it.
  static constexpr std::size_t kEventCapacity = 512;
  using EventQueue = rtc::SpscQueue<rtc::catching::PlannerCycleRecord, kEventCapacity>;

  /// `cycle`, the eventfd and the timing ring are owned by the controller and
  /// outlive this thread (the controller joins it before destroying any of
  /// them). The ring lives with the controller, not here, so the aux drain
  /// timer never has to touch this object — it can be joined and replaced
  /// while the timer keeps running. `wake_fd` must be a valid non-blocking
  /// eventfd.
  CatchingPlannerThread(rtc::catching::PlannerCycle& cycle, int wake_fd, double wake_timeout_s,
                        TimingBuffer& timing, EventQueue& events) noexcept;

  /// Joins BEFORE the members go: the base destructor also joins, but by then
  /// this class's members — and its OnRequestStop override — are gone, and the
  /// loop could still be inside OnTick.
  ~CatchingPlannerThread() override;

  CatchingPlannerThread(const CatchingPlannerThread&) = delete;
  CatchingPlannerThread& operator=(const CatchingPlannerThread&) = delete;
  CatchingPlannerThread(CatchingPlannerThread&&) = delete;
  CatchingPlannerThread& operator=(CatchingPlannerThread&&) = delete;

  /// Start the loop with the layout role's thread config. The period handed
  /// to the base is `1 / wake_timeout_s` (the base refuses a non-positive
  /// frequency); the deadline scheduling itself is replaced by the eventfd
  /// wait below.
  void StartWith(const rtc::ThreadConfig& thread_config);

  /// Signal a wake. Any thread; a non-blocking eventfd write. Returns false if
  /// the write failed (a full counter, which cannot happen in practice, or a
  /// closed fd).
  static bool Signal(int wake_fd) noexcept;

  // ── Observation (any thread, relaxed) ────────────────────────────────────
  /// Wakes caused by the eventfd, and by the timeout.
  [[nodiscard]] std::uint64_t SignalWakeCount() const noexcept {
    return signal_wakes_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t TimeoutWakeCount() const noexcept {
    return timeout_wakes_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t PublishedCount() const noexcept {
    return published_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint64_t SupersededCount() const noexcept {
    return superseded_.load(std::memory_order_relaxed);
  }

  /// Wakes that saw a trial reset in the RT state (L7 §4.8).
  [[nodiscard]] std::uint64_t ResetSeenCount() const noexcept {
    return resets_seen_.load(std::memory_order_relaxed);
  }

  /// Event rows the ring had no room for.
  [[nodiscard]] std::uint64_t EventDropCount() const noexcept {
    return event_drops_.load(std::memory_order_relaxed);
  }

  /// The last wake's record (SeqLock: a torn read is impossible).
  [[nodiscard]] rtc::catching::PlannerCycleRecord LastRecord() const noexcept {
    return last_record_.Load();
  }

 protected:
  WaitResult WaitForNextTick() noexcept override;
  void OnRequestStop() noexcept override;
  void OnTick() noexcept override;

  /// The wake cadence is set by vision and by the timeout, not by a deadline,
  /// so |actual period − budget| is not jitter (same reason as the CM's sim
  /// mode).
  [[nodiscard]] bool JitterMeaningful() const noexcept override { return false; }

 private:
  /// Drain the counter. Returns true if it held any signal.
  bool Drain() noexcept;

  rtc::catching::PlannerCycle& cycle_;
  int wake_fd_;
  int timeout_ms_;
  double frequency_hz_;
  TimingBuffer& timing_;
  EventQueue& events_;
  std::atomic<std::uint64_t> event_drops_{0};
  rtc::SeqLock<rtc::catching::PlannerCycleRecord> last_record_{};
  std::atomic<std::uint64_t> signal_wakes_{0};
  std::atomic<std::uint64_t> timeout_wakes_{0};
  std::atomic<std::uint64_t> published_{0};
  std::atomic<std::uint64_t> superseded_{0};
  std::atomic<std::uint64_t> resets_seen_{0};
};

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_CATCHING_PLANNER_THREAD_HPP_
