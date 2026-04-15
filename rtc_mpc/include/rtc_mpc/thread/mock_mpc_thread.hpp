#ifndef RTC_MPC_THREAD_MOCK_MPC_THREAD_HPP_
#define RTC_MPC_THREAD_MOCK_MPC_THREAD_HPP_

/// @file mock_mpc_thread.hpp
/// @brief Deterministic MPC solver stand-in for testing.
///
/// `MockMPCThread` implements @ref MPCThread::Solve with a trivial
/// controller: from the current state, it builds a horizon-length linear
/// trajectory towards a fixed target `q_target`. Velocity is set to a
/// constant equal to `(q_target - q0) / horizon_sec`. Riccati gains are
/// set to identity on the position rows.
///
/// This suffices to exercise the end-to-end MPC↔RT pipeline (TripleBuffer
/// publish, interpolation, Riccati feedback, stale counter) without
/// depending on a real trajectory optimiser.

#include <Eigen/Core>
#include <mutex>

#include "rtc_mpc/thread/mpc_thread.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

namespace rtc::mpc {

class MockMPCThread final : public MPCThread {
 public:
  /// @brief Configure the mock planner.
  /// @param nq               position dim (≤ kMaxNq)
  /// @param nv               velocity dim (≤ kMaxNv)
  /// @param horizon          node count to populate per solve
  /// @param dt_node          OCP node spacing [s]
  void Configure(int nq, int nv, int horizon, double dt_node) noexcept;

  /// @brief Set / update the target pose. Safe to call while the thread
  ///        is running.
  void SetTarget(const Eigen::Ref<const Eigen::VectorXd>& q_target) noexcept;

 protected:
  bool Solve(const MPCStateSnapshot& state,
             MPCSolution& out_sol,
             std::span<std::jthread> workers) override;

 private:
  mutable std::mutex mutex_;  // protects q_target_; called off RT path
  Eigen::VectorXd q_target_;
  int nq_{0};
  int nv_{0};
  int horizon_{10};
  double dt_node_{0.01};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_THREAD_MOCK_MPC_THREAD_HPP_
