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
/// - Solve() runs on this thread, plus — when `solver_threads` > 1 — the
///   solver's OpenMP pool. OpenMP creates that pool from this thread, so its
///   workers start with this thread's affinity, policy and name; at loop
///   entry, before the first tick, the thread runs one parallel region in
///   which each worker renames itself `mpc_worker_<i>` and pins itself to
///   `workers.slots[i]`. libgomp keeps the same pool for later regions of the
///   same size, so the solver's regions run on those pinned threads. A pool
///   that does not come up exactly as configured is a refusal: the loop ends
///   before its first tick and @ref PoolState reports `kFailed` — never a
///   solve on a misplaced pool.
/// - Every MPC thread sets its OpenMP ICVs at loop entry — dynamic off,
///   default team size 1 — the one-thread case included. Only regions that
///   name their team size (the loop entry's, the solver's) use the pool; a
///   region sized by default (Eigen's own products) stays serial. That keeps
///   the layout's thread set exact: libgomp retires surplus pool threads when
///   a smaller team runs and spawns fresh, unpinned ones for the next full
///   one, so a default-sized region would undo the pinning.
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

#include <sys/types.h>

#include <array>
#include <atomic>
#include <cstdint>

namespace rtc::mpc {

/// Configuration struct for launching an MPC thread. Populated by the
/// caller (typically from `rtc::SystemThreadConfigs::mpc`) and passed into
/// @ref MPCThread::Init.
struct MpcThreadLaunchConfig {
  /// Main solve thread scheduling / affinity.
  rtc::ThreadConfig main{};
  /// Target solve frequency in Hz.
  double target_frequency_hz{20.0};
  /// Solver threads, this one included — must equal the handler's
  /// `MPCSolverConfig::num_threads`. 1 = no pool.
  int solver_threads{1};
  /// Slots for the pool's `solver_threads - 1` workers
  /// (`rtc::SelectMpcWorkerSlots(profile)`); worker i takes `slots[i]`.
  rtc::MpcWorkerSlots workers{};
};

/// @brief Whether the MPC thread's OpenMP pool came up as configured.
enum class SolverPoolState : std::uint8_t {
  kPending,  ///< not started, or the loop entry has not finished
  kReady,    ///< pool (or the single thread) set up; ticks run
  kFailed,   ///< refused before the first tick; the loop has ended
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

  /// @brief The loop entry's verdict on the solver pool (acquire). Reset to
  ///        `kPending` by @ref Start. `Running()` is not a substitute: it is
  ///        true from Start until a refused loop has finished unwinding.
  [[nodiscard]] SolverPoolState PoolState() const noexcept {
    return pool_state_.load(std::memory_order_acquire);
  }

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

  // ── Overruns by budget ─────────────────────────────────────────────────
  //
  // The base counts an overrun (OverrunCount: the wait found the deadline
  // missed by more than one period) after the tick that caused it. A tick
  // the subclass marked as a standing-solve tick (MarkStandingTick — a solve
  // that runs to convergence by design) is not an online overrun: the two
  // are counted apart here, and their sum is OverrunCount once the thread is
  // joined (while it runs, a reader can see the base's count before this
  // split's). Any thread.

  /// @return overruns after a tick on the online budget.
  [[nodiscard]] std::uint64_t OnlineOverrunCount() const noexcept {
    return online_overruns_.load(std::memory_order_relaxed);
  }

  /// @return overruns after a standing-solve tick.
  [[nodiscard]] std::uint64_t StandingOverrunCount() const noexcept {
    return standing_overruns_.load(std::memory_order_relaxed);
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
  // iteration on the base's thread. A tick that reads a snapshot the RT
  // side has never stored (`valid == false`) skips the solve.
  void OnTick() noexcept override;

  /// @brief Mark the current tick as a standing-solve tick, from inside
  ///        Solve(): an overrun it causes is counted in StandingOverrunCount.
  ///        Loop thread only.
  void MarkStandingTick() noexcept { standing_tick_ = true; }

  // PeriodicRtThread hook: attribute the overrun to the tick before it.
  void OnOverrun(std::uint64_t consecutive) noexcept override;

  // PeriodicRtThread hook: set the OpenMP ICVs and, with solver_threads > 1,
  // create, name, pin and check the pool before the first tick.
  void OnLoopEntry(bool thread_config_applied) noexcept override;

  /// @brief Whether the solver this thread drives runs as many threads as the
  ///        launch config's `solver_threads` (the pool is built for that
  ///        count). Checked at loop entry; a mismatch refuses the loop.
  ///        Default true — a thread without a solver has nothing to match.
  [[nodiscard]] virtual bool SolverThreadsMatch() const noexcept { return true; }

  /// @brief The launch config given to @ref Init.
  [[nodiscard]] const MpcThreadLaunchConfig& LaunchConfig() const noexcept {
    return launch_config_;
  }

  /// @brief Write this thread's OpenMP ICVs (dynamic off, default team size
  ///        1). Loop thread only; no allocation. For a code path that may
  ///        have run something writing ICVs on this thread (a handler rebuilt
  ///        in place).
  static void ApplySolverThreadIcvs() noexcept;

 private:
  MPCSolutionManager* manager_{nullptr};
  MpcThreadLaunchConfig launch_config_{};
  bool initialised_{false};

  // Scratch solution buffer reused across ticks (trivially copyable POD).
  MPCSolution scratch_{};

  // Per-tick timing SPSC ring. Producer is the base main loop (inside
  // OnTick), consumer is the non-RT drain thread.
  rtc::MpcTimingBuffer timing_producer_;

  // Loop thread only: set by MarkStandingTick, cleared at the top of OnTick —
  // so the wait after a tick still reads that tick's mark.
  bool standing_tick_{false};
  std::atomic<std::uint64_t> online_overruns_{0};
  std::atomic<std::uint64_t> standing_overruns_{0};

  // What each worker found after naming and pinning itself, written inside
  // the loop entry's parallel region (relaxed) and read by this thread after
  // the region's implicit barrier.
  struct WorkerReport {
    std::atomic<int> apply_rc{-1};  // 0 = ApplyThreadConfig succeeded
    std::atomic<int> on_slot{0};    // 1 = affinity is exactly the slot's CPU
    std::atomic<int> policy{-1};    // sched policy after the pin
    std::atomic<pid_t> tid{0};
  };

  std::array<WorkerReport, rtc::kMaxMpcWorkers> worker_reports_{};
  std::atomic<SolverPoolState> pool_state_{SolverPoolState::kPending};

  [[nodiscard]] bool SetUpPool(bool thread_config_applied) noexcept;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_THREAD_MPC_THREAD_HPP_
