// ── Piecewise-constant-jerk joint trajectory: closed-form evaluation ─────────
// (dynamic_catching MPC plan E1-F01; formulation §1.5)
//
// The segment MPC (mpc_segment_core.hpp) publishes joint NODES (q_k, q̇_k, q̈_k) at a
// fixed spacing Δ and nothing between them (MD-9). Between
// node k and k+1 the jerk is constant, u_k = (q̈_{k+1} − q̈_k)/Δ, so for
// τ ∈ [0, Δ]:
//
//   q(τ)  = q_k + q̇_k τ + ½ q̈_k τ² + ⅙ u_k τ³
//   q̇(τ)  = q̇_k + q̈_k τ + ½ u_k τ²
//   q̈(τ)  = q̈_k + u_k τ
//
// This is EXACT for nodes produced by the MPC's own triple-integrator model
// (the nodes are consistent), which is why the RT sampler (E1-F02) and the
// planner's own-segment prediction (E1-F03, MD-28 path (i)) evaluate this
// instead of interpolating: both then reproduce the trajectory the QP
// optimised, and the result is C² at every node. (The planner's warm-start
// shift needs no evaluation: the stop end is fixed, so it is a column copy.)
// For inconsistent nodes (hand-made, or corrupted) position and velocity jump
// at node k+1 — the caller that builds nodes owns consistency.
//
// Pure numeric core: Eigen only, no heap, noexcept, fail-closed.
//  • Δ ≤ 0 or non-finite, and non-finite t, are invalid (NUM-2) — outputs are
//    left untouched. Node VALUES are not checked (the RT sampler reads nodes
//    the planner already validated); a NaN node yields NaN outputs.
//  • An output may alias only its own input (q with q0 …): the update is
//    element-wise, but q written before q̇0 is read would break q ↔ q̇0.
//  • t < 0 is invalid, not clamped: a query before the first node is a caller
//    bug (same resolution as decel_target.hpp ambiguity 2).
//  • t ≥ N·Δ HOLDS node N as stored. The MPC's node N is at rest (terminal
//    equality q̇_N = q̈_N = 0), so holding it is the physically consistent
//    continuation; a caller that feeds a non-resting node N gets that node back.
#pragma once

#include <Eigen/Core>

#include <cmath>

namespace rtc::catching {

/// @brief Evaluate one constant-jerk segment starting at node k.
///
/// Column-free form for callers that already hold the two nodes (the shift).
/// @param q0,qd0,qdd0 node k (n each)
/// @param qdd1 q̈ at node k+1 (n) — fixes the segment's jerk
/// @param dt node spacing Δ > 0 [s]
/// @param tau time since node k, 0 ≤ τ ≤ Δ [s]
/// @param[out] q,qd,qdd state at τ (n each). Untouched on failure.
/// @return false on Δ ≤ 0, τ outside [0, Δ], non-finite scalars, or a size
///         mismatch.
[[nodiscard]] inline bool EvaluateJerkSegment(const Eigen::Ref<const Eigen::VectorXd>& q0,
                                              const Eigen::Ref<const Eigen::VectorXd>& qd0,
                                              const Eigen::Ref<const Eigen::VectorXd>& qdd0,
                                              const Eigen::Ref<const Eigen::VectorXd>& qdd1,
                                              double dt, double tau, Eigen::Ref<Eigen::VectorXd> q,
                                              Eigen::Ref<Eigen::VectorXd> qd,
                                              Eigen::Ref<Eigen::VectorXd> qdd) noexcept {
  const Eigen::Index n = q0.size();
  if (qd0.size() != n || qdd0.size() != n || qdd1.size() != n || q.size() != n || qd.size() != n ||
      qdd.size() != n) {
    return false;
  }
  if (!std::isfinite(dt) || !(dt > 0.0) || !std::isfinite(tau) || tau < 0.0 || tau > dt) {
    return false;
  }
  const double t2 = tau * tau;
  const double t3 = t2 * tau;
  for (Eigen::Index j = 0; j < n; ++j) {
    const double u = (qdd1[j] - qdd0[j]) / dt;
    q[j] = q0[j] + qd0[j] * tau + 0.5 * qdd0[j] * t2 + u * t3 / 6.0;
    qd[j] = qd0[j] + qdd0[j] * tau + 0.5 * u * t2;
    qdd[j] = qdd0[j] + u * tau;
  }
  return true;
}

/// @brief Sample a node trajectory (n × (N+1) columns) at time t from node 0.
///
/// @param Q,Qd,Qdd node matrices, column k = node k (same shape, ≥ 1 column)
/// @param dt node spacing Δ > 0 [s]
/// @param t query time, t ≥ 0 [s]; t ≥ N·Δ holds node N (header note)
/// @param[out] q,qd,qdd state at t (n each). Untouched on failure.
/// @return false on Δ ≤ 0, t < 0, non-finite scalars, or a shape mismatch.
[[nodiscard]] inline bool SampleJerkTrajectory(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                                               const Eigen::Ref<const Eigen::MatrixXd>& Qd,
                                               const Eigen::Ref<const Eigen::MatrixXd>& Qdd,
                                               double dt, double t, Eigen::Ref<Eigen::VectorXd> q,
                                               Eigen::Ref<Eigen::VectorXd> qd,
                                               Eigen::Ref<Eigen::VectorXd> qdd) noexcept {
  const Eigen::Index n = Q.rows();
  const Eigen::Index cols = Q.cols();
  if (cols < 1 || Qd.rows() != n || Qd.cols() != cols || Qdd.rows() != n || Qdd.cols() != cols ||
      q.size() != n || qd.size() != n || qdd.size() != n) {
    return false;
  }
  if (!std::isfinite(dt) || !(dt > 0.0) || !std::isfinite(t) || t < 0.0) {
    return false;
  }
  const Eigen::Index n_seg = cols - 1;
  const double k_real = std::floor(t / dt);
  if (k_real >= static_cast<double>(n_seg)) {
    q = Q.col(n_seg);
    qd = Qd.col(n_seg);
    qdd = Qdd.col(n_seg);
    return true;
  }
  const auto k = static_cast<Eigen::Index>(k_real);
  // t − k·Δ can round a hair past Δ at an exact node; the segment accepts
  // [0, Δ] only, so bound it (t ≥ 0 and k = ⌊t/Δ⌋ keep it ≥ 0 up to rounding).
  double tau = t - static_cast<double>(k) * dt;
  tau = tau < 0.0 ? 0.0 : (tau > dt ? dt : tau);
  return EvaluateJerkSegment(Q.col(k), Qd.col(k), Qdd.col(k), Qdd.col(k + 1), dt, tau, q, qd, qdd);
}

}  // namespace rtc::catching
