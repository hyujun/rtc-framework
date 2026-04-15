#include "rtc_mpc/interpolation/trajectory_interpolator.hpp"

#include <algorithm>
#include <cmath>

namespace rtc::mpc {

void TrajectoryInterpolator::Init(int nq, int nv, int n_contact_vars) {
  // Bounds check against the compile-time capacity of MPCSolution. These
  // are capacity limits, not per-robot limits, so they are runtime-invariant.
  if (nq < 0 || nq > kMaxNq || nv < 0 || nv > kMaxNv ||
      n_contact_vars < 0 || n_contact_vars > kMaxContactVars) {
    nq_ = 0;
    nv_ = 0;
    n_contact_vars_ = 0;
    has_solution_ = false;
    return;
  }
  nq_ = nq;
  nv_ = nv;
  n_contact_vars_ = n_contact_vars;
  has_solution_ = false;
}

void TrajectoryInterpolator::SetSolution(const MPCSolution& sol,
                                         uint64_t receive_ns) noexcept {
  if (!sol.IsValid()) {
    has_solution_ = false;
    return;
  }
  sol_ = sol;  // trivially copyable; compiler lowers to memcpy
  receive_ns_ = receive_ns;
  has_solution_ = true;
}

void TrajectoryInterpolator::Interpolate(
    uint64_t now_ns,
    Eigen::Ref<Eigen::VectorXd> q_ref,
    Eigen::Ref<Eigen::VectorXd> v_ref,
    Eigen::Ref<Eigen::VectorXd> a_ff,
    Eigen::Ref<Eigen::VectorXd> lambda_ref,
    InterpMeta& meta_out) noexcept {
  meta_out.valid = false;
  meta_out.beyond_horizon = false;
  meta_out.progress = 0.0;

  if (!has_solution_) {
    return;
  }

  const double dt = sol_.dt_node;
  const int horizon = sol_.horizon_length;
  if (horizon <= 0 || dt <= 0.0) {
    return;
  }

  // Elapsed time since node 0 (in seconds). Clamp to non-negative to
  // tolerate minor clock drift between MPC publish and RT consume.
  const int64_t delta_ns =
      static_cast<int64_t>(now_ns) - static_cast<int64_t>(receive_ns_);
  const double t_local =
      std::max<double>(0.0, static_cast<double>(delta_ns) * 1e-9);

  const double horizon_sec =
      static_cast<double>(horizon) * dt;

  if (t_local >= horizon_sec) {
    // Beyond horizon: freeze on the final node and zero the feedforward.
    const int last = horizon;  // q_traj index (length horizon+1)
    const int last_u = horizon - 1;  // u/lambda index
    const int nq = std::min(nq_, sol_.nq);
    const int nv = std::min(nv_, sol_.nv);
    const int nc = std::min(n_contact_vars_, sol_.n_contact_vars);

    for (int i = 0; i < nq; ++i) {
      q_ref(i) = sol_.q_traj[static_cast<std::size_t>(last)]
                           [static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < nv; ++i) {
      v_ref(i) = sol_.v_traj[static_cast<std::size_t>(last)]
                           [static_cast<std::size_t>(i)];
      a_ff(i) = 0.0;
    }
    for (int i = 0; i < nc; ++i) {
      lambda_ref(i) = sol_.lambda_traj[static_cast<std::size_t>(last_u)]
                                     [static_cast<std::size_t>(i)];
    }
    meta_out.valid = true;
    meta_out.beyond_horizon = true;
    meta_out.progress = 1.0;
    return;
  }

  // Locate the containing node: k = floor(t_local / dt), alpha = fractional.
  const double t_over_dt = t_local / dt;
  int node_k = static_cast<int>(std::floor(t_over_dt));
  if (node_k < 0) {
    node_k = 0;
  }
  if (node_k > horizon - 1) {
    node_k = horizon - 1;
  }
  const double alpha = t_over_dt - static_cast<double>(node_k);

  InterpolateHermite(node_k, alpha, q_ref, v_ref, a_ff, lambda_ref);
  meta_out.valid = true;
  meta_out.beyond_horizon = false;
  meta_out.progress = t_local / horizon_sec;
}

void TrajectoryInterpolator::InterpolateHermite(
    int node_k, double alpha,
    Eigen::Ref<Eigen::VectorXd> q_ref,
    Eigen::Ref<Eigen::VectorXd> v_ref,
    Eigen::Ref<Eigen::VectorXd> a_ff,
    Eigen::Ref<Eigen::VectorXd> lambda_ref) noexcept {
  // Cubic Hermite basis evaluated at α ∈ [0, 1].
  const double a2 = alpha * alpha;
  const double a3 = a2 * alpha;
  const double h00 = 2.0 * a3 - 3.0 * a2 + 1.0;
  const double h10 = a3 - 2.0 * a2 + alpha;
  const double h01 = -2.0 * a3 + 3.0 * a2;
  const double h11 = a3 - a2;

  // First derivatives w.r.t. α (used for velocity; divided by Δt below).
  const double h00p = 6.0 * a2 - 6.0 * alpha;
  const double h10p = 3.0 * a2 - 4.0 * alpha + 1.0;
  const double h01p = -6.0 * a2 + 6.0 * alpha;
  const double h11p = 3.0 * a2 - 2.0 * alpha;

  // Second derivatives w.r.t. α (used for acceleration; divided by Δt²).
  const double h00pp = 12.0 * alpha - 6.0;
  const double h10pp = 6.0 * alpha - 4.0;
  const double h01pp = -12.0 * alpha + 6.0;
  const double h11pp = 6.0 * alpha - 2.0;

  const double dt = sol_.dt_node;
  const double inv_dt = 1.0 / dt;
  const double inv_dt2 = inv_dt * inv_dt;

  const int nq = std::min(nq_, sol_.nq);
  const int nv = std::min(nv_, sol_.nv);
  const int nc = std::min(n_contact_vars_, sol_.n_contact_vars);
  const auto kk = static_cast<std::size_t>(node_k);
  const auto kk1 = static_cast<std::size_t>(node_k + 1);

  // Position: q(α) = h00·q_k + h10·Δt·v_k + h01·q_{k+1} + h11·Δt·v_{k+1}
  // Requires nv == nq for the velocity coupling (true for revolute-joint
  // fixed-base manipulators; see README.md).
  const int n_coupled = std::min(nq, nv);
  for (int i = 0; i < n_coupled; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const double q_k = sol_.q_traj[kk][idx];
    const double q_k1 = sol_.q_traj[kk1][idx];
    const double v_k = sol_.v_traj[kk][idx];
    const double v_k1 = sol_.v_traj[kk1][idx];
    q_ref(i) = h00 * q_k + h10 * dt * v_k + h01 * q_k1 + h11 * dt * v_k1;
    // v(t) = dq/dt = (dq/dα) · (dα/dt) = (dq/dα) / Δt
    v_ref(i) = (h00p * q_k + h10p * dt * v_k + h01p * q_k1 +
                h11p * dt * v_k1) * inv_dt;
    // a(t) = d²q/dt² = (d²q/dα²) / Δt²
    a_ff(i) = (h00pp * q_k + h10pp * dt * v_k + h01pp * q_k1 +
               h11pp * dt * v_k1) * inv_dt2;
  }
  // If nq > nv (rare: config with un-modelled velocity components), fall
  // back to linear position interpolation for the tail and zero velocity /
  // acceleration on those dims.
  for (int i = n_coupled; i < nq; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    q_ref(i) = (1.0 - alpha) * sol_.q_traj[kk][idx] +
               alpha * sol_.q_traj[kk1][idx];
  }
  // If nv > nq (also rare), zero the extra velocity entries.
  for (int i = n_coupled; i < nv; ++i) {
    v_ref(i) = 0.0;
    a_ff(i) = 0.0;
  }

  // Contact force: linear interpolation (non-smooth allowed).
  const double one_minus_alpha = 1.0 - alpha;
  for (int i = 0; i < nc; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    lambda_ref(i) = one_minus_alpha * sol_.lambda_traj[kk][idx] +
                    alpha * sol_.lambda_traj[kk1][idx];
  }
}

double TrajectoryInterpolator::RemainingHorizonSec(
    uint64_t now_ns) const noexcept {
  if (!has_solution_ || sol_.dt_node <= 0.0 || sol_.horizon_length <= 0) {
    return 0.0;
  }
  const int64_t delta_ns =
      static_cast<int64_t>(now_ns) - static_cast<int64_t>(receive_ns_);
  const double t_local =
      std::max<double>(0.0, static_cast<double>(delta_ns) * 1e-9);
  const double horizon_sec =
      static_cast<double>(sol_.horizon_length) * sol_.dt_node;
  const double remaining = horizon_sec - t_local;
  return remaining > 0.0 ? remaining : 0.0;
}

}  // namespace rtc::mpc
