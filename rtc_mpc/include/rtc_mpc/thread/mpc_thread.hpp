#ifndef RTC_MPC_THREAD_MPC_THREAD_HPP_
#define RTC_MPC_THREAD_MPC_THREAD_HPP_

/// @file mpc_thread.hpp
/// @brief Abstract MPC solve loop + worker frame.
///
/// `MPCThread` is a reusable skeleton that drives `Solve()` at a fixed
/// frequency on a dedicated jthread. Downstream packages (e.g. an Aligator
/// integration) subclass it and implement `Solve()`; see
/// @ref MockMPCThread below for a deterministic test implementation.
///
/// Threading model:
/// - The main solve thread runs `Solve()` once per period, publishing the
///   result via @ref MPCSolutionManager::PublishSolution.
/// - Optional worker threads are started before the solve loop and exposed
///   to `Solve()` via a `std::span<std::jthread>`. Solvers that do not
///   exploit parallelism can simply ignore the span.
/// - Thread affinity / priority is applied by the caller-supplied
///   @ref rtc::ThreadConfig values, one per thread; the base class calls
///   `rtc::ApplyThreadConfig` at thread entry.
/// - Cooperative cancellation is via `std::jthread`'s stop_token.
///
/// Lifecycle:
///   1. Construct subclass.
///   2. @ref Init with the @ref MPCSolutionManager, thread configs, and
///      target frequency.
///   3. @ref Start — spawns main + workers.
///   4. @ref RequestStop / destructor — stop_token signals, joins cleanly.

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <array>
#include <atomic>
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

class MPCThread {
 public:
  MPCThread() = default;
  virtual ~MPCThread();

  MPCThread(const MPCThread&) = delete;
  MPCThread& operator=(const MPCThread&) = delete;
  MPCThread(MPCThread&&) = delete;
  MPCThread& operator=(MPCThread&&) = delete;

  /// @brief Configure the thread before @ref Start.
  /// @param manager        shared solution manager (state read, solution
  ///                       publish)
  /// @param launch_config  thread affinity / priority / frequency
  void Init(MPCSolutionManager& manager,
            const MpcThreadLaunchConfig& launch_config) noexcept;

  /// @brief Spawn main + worker threads. No-op if already running or not
  ///        initialised.
  void Start();

  /// @brief Signal the main thread (and workers) to stop. Non-blocking.
  void RequestStop() noexcept;

  /// @brief Join all threads. Idempotent; safe to call from the destructor.
  void Join() noexcept;

  [[nodiscard]] bool Running() const noexcept { return running_.load(); }

 protected:
  /// @brief Perform one MPC solve.
  ///
  /// @param state     latest RT-thread state snapshot.
  /// @param out_sol   solution buffer to populate.
  /// @param workers   worker jthread handles (empty span if no workers).
  /// @return true if @p out_sol contains a usable solution (will be
  ///         published); false to skip publishing this cycle.
  virtual bool Solve(const MPCStateSnapshot& state,
                     MPCSolution& out_sol,
                     std::span<std::jthread> workers) = 0;

 private:
  void RunMain(std::stop_token stoken);

  MPCSolutionManager* manager_{nullptr};
  MpcThreadLaunchConfig launch_config_{};
  std::jthread main_thread_;
  std::array<std::jthread, kMaxMpcWorkers> workers_{};
  std::atomic<bool> running_{false};
  bool initialised_{false};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_THREAD_MPC_THREAD_HPP_
