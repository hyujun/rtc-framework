#ifndef RTC_MPC_THREAD_MPC_THREAD_HPP_
#define RTC_MPC_THREAD_MPC_THREAD_HPP_

/// @file mpc_thread.hpp
/// @brief Abstract MPC solve loop + worker frame.
///
/// `MPCThread` drives `Solve()` once per period on a dedicated thread and
/// publishes the result via @ref MPCSolutionManager::PublishSolution. The
/// fixed-frequency loop, lifecycle (Start / RequestStop / Join /
/// Pause / Resume), per-tick timing capture, and overrun bookkeeping are
/// all provided by @ref rtc::PeriodicRtThread base; this class only adds
/// MPC-specific concerns: worker thread management, the
/// `Solve(state, out, workers)` virtual, and the wiring of the timing
/// producer onto the base.
///
/// Threading model:
/// - The main solve thread is owned by @ref rtc::PeriodicRtThread.
/// - Optional worker threads (up to @ref kMaxMpcWorkers) are owned by this
///   class and exposed to `Solve()` via a `std::span<std::jthread>`.
///   Solvers that do not exploit parallelism can simply ignore the span.
/// - Thread affinity / priority is applied by the caller-supplied
///   @ref rtc::ThreadConfig values, one per thread; the main thread's
///   config is forwarded to base, workers apply their own at thread entry.
///
/// Lifecycle:
///   1. Construct subclass.
///   2. @ref Init with the @ref MPCSolutionManager and launch config.
///   3. @ref Start — spawns workers + base main loop.
///   4. @ref RequestStop / destructor — workers + base join cleanly.

#include "rtc_base/threading/periodic_rt_thread.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <array>
#include <span>
#include <thread>

namespace rtc::mpc {

/// Max number of worker threads that can be managed alongside the main
/// solve thread. Matches the parallel capacity of 12/16-core layouts.
inline constexpr int kMaxMpcWorkers = 2;

/// Configuration struct for launching an MPC thread. Populated by the
/// caller (typically from `rtc::SystemThreadConfigs::mpc`) and passed into
/// @ref MPCThread::Init.
struct MpcThreadLaunchConfig {
  /// Main solve thread scheduling / affinity.
  rtc::ThreadConfig main{};
  /// Number of active worker configs in @ref workers.
  int num_workers{0};
  /// Per-worker scheduling / affinity (first `num_workers` entries valid).
  std::array<rtc::ThreadConfig, kMaxMpcWorkers> workers{};
  /// Target solve frequency in Hz.
  double target_frequency_hz{20.0};
};

class MPCThread : public rtc::PeriodicRtThread {
public:
  MPCThread() = default;
  ~MPCThread() override;

  MPCThread(const MPCThread &) = delete;
  MPCThread &operator=(const MPCThread &) = delete;
  MPCThread(MPCThread &&) = delete;
  MPCThread &operator=(MPCThread &&) = delete;

  /// @brief Configure the thread before @ref Start.
  /// @param manager        shared solution manager (state read, solution
  ///                       publish)
  /// @param launch_config  thread affinity / priority / frequency
  void Init(MPCSolutionManager &manager,
            const MpcThreadLaunchConfig &launch_config) noexcept;

  /// @brief Spawn workers + the base main solve loop. No-op if already
  ///        running or not initialised.
  void Start();

  /// @brief Stop and join the main loop + workers. Idempotent.
  void StopAndJoin() noexcept;

  // ── Per-tick timing producer ───────────────────────────────────────────
  //
  // Base captures t0..t3 around OnTick (which delegates to Solve()) and
  // pushes one rtc::RtTickTimingPayload onto this SPSC ring per iteration.
  // A non-RT consumer (e.g. the controller LifecycleNode's 1 Hz aux timer)
  // drains via Drain(...) into <session>/timing/mpc_timing_log.csv.
  // Push is wait-free; on overflow the sample is dropped (DropCount()
  // increments).

  /// @brief Direct accessor for the per-tick MPC-loop timing producer.
  [[nodiscard]] rtc::MpcTimingBuffer &TimingProducer() noexcept {
    return timing_producer_;
  }
  [[nodiscard]] const rtc::MpcTimingBuffer &TimingProducer() const noexcept {
    return timing_producer_;
  }

protected:
  /// @brief Perform one MPC solve.
  ///
  /// @param state     latest RT-thread state snapshot.
  /// @param out_sol   solution buffer to populate.
  /// @param workers   worker jthread handles (empty span if no workers).
  /// @return true if @p out_sol contains a usable solution (will be
  ///         published); false to skip publishing this cycle.
  virtual bool Solve(const MPCStateSnapshot &state, MPCSolution &out_sol,
                     std::span<std::jthread> workers) = 0;

  // PeriodicRtThread hook: drives one ReadState → Solve → PublishSolution
  // iteration on the base's thread.
  void OnTick() noexcept override;

private:
  MPCSolutionManager *manager_{nullptr};
  MpcThreadLaunchConfig launch_config_{};
  bool initialised_{false};

  // Worker threads owned by this class — spawned by Start(), joined by
  // dtor / Join (via base's RequestStop chain).
  std::array<std::jthread, kMaxMpcWorkers> workers_{};

  // Scratch solution buffer reused across ticks (trivially copyable POD).
  MPCSolution scratch_{};

  // Per-tick timing SPSC ring. Producer is the base main loop (inside
  // OnTick), consumer is the non-RT drain thread.
  rtc::MpcTimingBuffer timing_producer_;
};

} // namespace rtc::mpc

#endif // RTC_MPC_THREAD_MPC_THREAD_HPP_
