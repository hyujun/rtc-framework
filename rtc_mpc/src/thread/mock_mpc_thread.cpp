#include "rtc_mpc/thread/mock_mpc_thread.hpp"

#include <algorithm>

namespace rtc::mpc {

void MockMPCThread::Configure(int nq, int nv, int horizon,
                              double dt_node) noexcept {
  const std::lock_guard lock(mutex_);
  nq_ = std::clamp(nq, 0, kMaxNq);
  nv_ = std::clamp(nv, 0, kMaxNv);
  horizon_ = std::clamp(horizon, 1, kMaxHorizon);
  dt_node_ = dt_node > 0.0 ? dt_node : 0.01;
  if (q_target_.size() != nq_) {
    q_target_.setZero(nq_);
  }
}

void MockMPCThread::SetTarget(
    const Eigen::Ref<const Eigen::VectorXd>& q_target) noexcept {
  const std::lock_guard lock(mutex_);
  if (q_target_.size() != nq_) {
    q_target_.setZero(nq_);
  }
  const int n = std::min<int>(static_cast<int>(q_target.size()), nq_);
  q_target_.head(n) = q_target.head(n);
}

bool MockMPCThread::Solve(const MPCStateSnapshot& state,
                          MPCSolution& out_sol,
                          std::span<std::jthread> /*workers*/) {
  // Snapshot target under lock so SetTarget() can't tear the read. The
  // Solve method runs off the RT path, so a mutex here is acceptable.
  Eigen::VectorXd target;
  int nq = 0;
  int nv = 0;
  int horizon = 10;
  double dt = 0.01;
  {
    const std::lock_guard lock(mutex_);
    nq = nq_;
    nv = nv_;
    horizon = horizon_;
    dt = dt_node_;
    target = q_target_;
  }
  if (nq <= 0 || nv <= 0) {
    return false;
  }

  // Start from the RT snapshot (if valid); else start from zero.
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nq);
  if (state.nq >= nq) {
    for (int i = 0; i < nq; ++i) {
      q0(i) = state.q[static_cast<std::size_t>(i)];
    }
  }

  const double horizon_sec = static_cast<double>(horizon) * dt;
  Eigen::VectorXd velocity = (target.head(nq) - q0) / horizon_sec;

  out_sol = MPCSolution{};  // zero-init
  out_sol.horizon_length = horizon;
  out_sol.dt_node = dt;
  out_sol.converged = true;
  out_sol.nq = nq;
  out_sol.nv = nv;
  out_sol.nu = nv;
  out_sol.nx = nq + nv;
  out_sol.n_contact_vars = 0;
  out_sol.timestamp_ns = state.timestamp_ns;

  const int n_coupled = std::min(nq, nv);
  for (int k = 0; k <= horizon; ++k) {
    const double alpha = static_cast<double>(k) /
                         static_cast<double>(horizon);
    for (int i = 0; i < nq; ++i) {
      out_sol.q_traj[static_cast<std::size_t>(k)]
                    [static_cast<std::size_t>(i)] =
          q0(i) + alpha * (target(i) - q0(i));
    }
    for (int i = 0; i < n_coupled; ++i) {
      out_sol.v_traj[static_cast<std::size_t>(k)]
                    [static_cast<std::size_t>(i)] = velocity(i);
    }
  }

  // Identity Riccati (position rows only) for the first node.
  for (int i = 0; i < n_coupled; ++i) {
    const std::size_t flat = static_cast<std::size_t>(i) *
                             static_cast<std::size_t>(out_sol.nx) +
                             static_cast<std::size_t>(i);
    out_sol.K_riccati[0][flat] = 1.0;
  }
  return true;
}

}  // namespace rtc::mpc
