#ifndef RTC_MPC_THREAD_MPC_THREAD_HPP_
#define RTC_MPC_THREAD_MPC_THREAD_HPP_

/// @file mpc_thread.hpp
/// @brief Abstract MPC solve loop.
///
/// `MPCThread` drives `Solve()` once per period on a dedicated thread and
/// publishes the result via @ref MPCSolutionManager::PublishSolution. The
/// fixed-frequency loop, lifecycle (Start / RequestStop / Join /
/// Pause / Resume), per-tick timing capture, and overrun bookkeeping are
/// all provided by @ref rtc::PeriodicRtThread base; this class only adds
/// MPC-specific concerns: the `Solve(state, out)` virtual and the wiring
/// of the timing producer onto the base.
///
/// Threading model:
/// - The solve thread is owned by @ref rtc::PeriodicRtThread; affinity /
///   priority come from the caller-supplied @ref rtc::ThreadConfig.
/// - Solve() runs single-threaded. Parallel solving, if ever introduced,
///   goes through the solver's own OpenMP pool
///   (`aligator::SolverProxDDP::setNumThreads`) — OpenMP owns and spawns
///   its threads, so externally owned thread handles cannot serve it
///   (#380 removed the earlier span-of-jthreads worker frame for exactly
///   that reason).
///
/// Lifecycle:
///   1. Construct subclass.
///   2. @ref Init with the @ref MPCSolutionManager and launch config.
///   3. @ref Start — spawns the base main loop.
///   4. @ref RequestStop / destructor — base joins cleanly.

#include "rtc_base/threading/periodic_rt_thread.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

namespace rtc::mpc {

/// Configuration struct for launching an MPC thread. Populated by the
/// caller (typically from `rtc::SystemThreadConfigs::mpc`) and passed into
/// @ref MPCThread::Init.
struct MpcThreadLaunchConfig {
  /// Main solve thread scheduling / affinity.
  rtc::ThreadConfig main{};
  /// Target solve frequency in Hz.
  double target_frequency_hz{20.0};
};

class MPCThread : public rtc::PeriodicRtThread {
 public:
  MPCThread() = default;
  ~MPCThread() override;

  MPCThread(const MPCThread&) = delete;
  MPCThread& operator=(const MPCThread&) = delete;
  MPCThread(MPCThread&&) = delete;
  MPCThread& operator=(MPCThread&&) = delete;

  /// @brief Configure the thread before @ref Start.
  /// @param manager        shared solution manager (state read, solution
  ///                       publish)
  /// @param launch_config  thread affinity / priority / frequency
  void Init(MPCSolutionManager& manager, const MpcThreadLaunchConfig& launch_config) noexcept;

  /// @brief Spawn the base main solve loop. No-op if already running or
  ///        not initialised.
  void Start();

  /// @brief Stop and join the main loop. Idempotent.
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
  [[nodiscard]] rtc::MpcTimingBuffer& TimingProducer() noexcept { return timing_producer_; }

  [[nodiscard]] const rtc::MpcTimingBuffer& TimingProducer() const noexcept {
    return timing_producer_;
  }

 protected:
  /// @brief Perform one MPC solve.
  ///
  /// @param state     latest RT-thread state snapshot.
  /// @param out_sol   solution buffer to populate.
  /// @return true if @p out_sol contains a usable solution (will be
  ///         published); false to skip publishing this cycle.
  virtual bool Solve(const MPCStateSnapshot& state, MPCSolution& out_sol) = 0;

  // PeriodicRtThread hook: drives one ReadState → Solve → PublishSolution
  // iteration on the base's thread.
  void OnTick() noexcept override;

 private:
  MPCSolutionManager* manager_{nullptr};
  MpcThreadLaunchConfig launch_config_{};
  bool initialised_{false};

  // Scratch solution buffer reused across ticks (trivially copyable POD).
  MPCSolution scratch_{};

  // Per-tick timing SPSC ring. Producer is the base main loop (inside
  // OnTick), consumer is the non-RT drain thread.
  rtc::MpcTimingBuffer timing_producer_;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_THREAD_MPC_THREAD_HPP_
