#include "rtc_tsid/kinematics/clik_reference.hpp"

#include "rtc_tsid/kinematics/se3_error.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rtc::tsid {

void ClikReferenceGenerator::Init(int nv, const Config& config) {
  if (nv <= 0) {
    throw std::runtime_error("ClikReferenceGenerator: nv must be positive, got " +
                             std::to_string(nv));
  }
  if (config.arm_v_idx.empty()) {
    throw std::runtime_error("ClikReferenceGenerator: arm_v_idx must not be empty");
  }
  if (!(config.damping_sq > 0.0)) {
    throw std::runtime_error("ClikReferenceGenerator: damping_sq must be > 0, got " +
                             std::to_string(config.damping_sq));
  }
  if (!(config.w_task > 0.0)) {
    throw std::runtime_error("ClikReferenceGenerator: w_task must be > 0, got " +
                             std::to_string(config.w_task));
  }
  if (config.w_arm < 0.0 || config.w_hand < 0.0) {
    throw std::runtime_error("ClikReferenceGenerator: w_arm / w_hand must be >= 0");
  }
  // Position-limit box is optional; when supplied each side must be full-nv.
  if (config.q_min.size() != 0 && config.q_min.size() != nv) {
    throw std::runtime_error("ClikReferenceGenerator: q_min size " +
                             std::to_string(config.q_min.size()) + " != nv " + std::to_string(nv));
  }
  if (config.q_max.size() != 0 && config.q_max.size() != nv) {
    throw std::runtime_error("ClikReferenceGenerator: q_max size " +
                             std::to_string(config.q_max.size()) + " != nv " + std::to_string(nv));
  }
  if (config.q_min.size() != config.q_max.size()) {
    throw std::runtime_error(
        "ClikReferenceGenerator: q_min / q_max must both be set or both empty");
  }

  // Index validation: range, duplicates, arm/hand overlap.
  std::vector<bool> seen(static_cast<size_t>(nv), false);
  auto check_indices = [&](const std::vector<int>& idx, const char* label) {
    for (const int i : idx) {
      if (i < 0 || i >= nv) {
        throw std::runtime_error(std::string("ClikReferenceGenerator: ") + label + " index " +
                                 std::to_string(i) + " out of range [0, " + std::to_string(nv) +
                                 ")");
      }
      if (seen[static_cast<size_t>(i)]) {
        throw std::runtime_error(std::string("ClikReferenceGenerator: ") + label + " index " +
                                 std::to_string(i) + " duplicated or overlaps the other set");
      }
      seen[static_cast<size_t>(i)] = true;
    }
  };
  check_indices(config.arm_v_idx, "arm_v_idx");
  check_indices(config.hand_v_idx, "hand_v_idx");

  nv_ = nv;
  arm_v_idx_ = config.arm_v_idx;
  hand_v_idx_ = config.hand_v_idx;
  n_arm_ = static_cast<int>(arm_v_idx_.size());
  n_hand_ = static_cast<int>(hand_v_idx_.size());
  damping_sq_ = config.damping_sq;
  v_limit_ = config.v_limit;
  w_task_ = config.w_task;
  w_arm_ = config.w_arm;
  w_hand_ = config.w_hand;
  q_min_ = config.q_min;
  q_max_ = config.q_max;

  manipulability_ = 0.0;
  tcp_error_norm_ = 0.0;

  q_ref_.setZero(nv_);
  v_ref_.setZero(nv_);
  j_task_.setZero(6, nv_);
  v_post_arm_.setZero(n_arm_);
  v_post_hand_.setZero(n_hand_);
  j_arm_.setZero(6, n_arm_);
  m6_.setZero();
  e_x_.setZero();
  r_task_.setZero();

  // Fixed-dim box QP: N variables, no equality, N inequality rows (C = Iₙ).
  // C is constant (per-joint velocity box) → set once here; only l/u/H/g change
  // per tick. Dim fields are fixed so QPSolverWrapper only update()s on the RT
  // path (no re-init / heap alloc).
  qp_data_.Init(nv_, 0, nv_);
  qp_data_.C.topLeftCorner(nv_, nv_).setIdentity();
  qp_data_.n_vars = nv_;
  qp_data_.n_eq = 0;
  qp_data_.n_ineq = nv_;

  QPSolverConfig solver_cfg;
  qp_solver_.Init(nv_, 0, nv_, solver_cfg);
}

bool ClikReferenceGenerator::Compute(const PinocchioCache& cache, int tcp_frame_idx,
                                     int base_frame_idx, const pinocchio::SE3& placement_des,
                                     const Eigen::VectorXd& q_posture_des, double dt) noexcept {
  // Preconditions. nq == nv (reduced revolute/prismatic tree) is required so
  // velocity indices address q directly and q_ref = q + v·dt is valid.
  const int n_registered = static_cast<int>(cache.registered_frames.size());
  if (nv_ == 0 || tcp_frame_idx < 0 || tcp_frame_idx >= n_registered ||
      base_frame_idx >= n_registered) {
    return false;
  }
  if (cache.q.size() != nv_ || cache.v.size() != nv_ || q_posture_des.size() != nv_ ||
      !(dt > 0.0)) {
    return false;
  }

  const auto& rf = cache.registered_frames[static_cast<size_t>(tcp_frame_idx)];

  // 현재 TCP pose를 base 기준으로 (SE3Task 와 동일한 contract/fast-path).
  const pinocchio::SE3 tip_in_base =
      (base_frame_idx < 0)
          ? rf.oMf
          : cache.registered_frames[static_cast<size_t>(base_frame_idx)].oMf.actInv(rf.oMf);

  // ── L1: SE(3) 오차 (공용 규약) → task-velocity reference r_task = Kx ⊙ e_x ──
  e_x_ = ComputeSe3Error(tip_in_base, placement_des);
  // 회전 오차를 base 좌표로 정렬 (se3_error 규약 — header 참조).
  e_x_.tail<3>() = tip_in_base.rotation() * e_x_.tail<3>();
  tcp_error_norm_ = e_x_.norm();
  r_task_ = kx_.cwiseProduct(e_x_);

  const int N = nv_;

  // ── Task Jacobian J_task ∈ R⁶ˣᴺ: arm columns of rf.J, hand columns 0 ──
  // (also gather j_arm_ for the manipulability diagnostic).
  j_task_.setZero();
  for (int c = 0; c < n_arm_; ++c) {
    const auto vi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    j_task_.col(vi) = rf.J.col(vi);
    j_arm_.col(c) = rf.J.col(vi);
  }

  // ── L2/L3 posture velocity references v_p = K·(q_des − q) ──
  for (int c = 0; c < n_arm_; ++c) {
    const auto qi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    v_post_arm_(c) = ka_ * (q_posture_des(qi) - cache.q(qi));
  }
  for (int c = 0; c < n_hand_; ++c) {
    const auto qi = static_cast<Eigen::Index>(hand_v_idx_[static_cast<size_t>(c)]);
    v_post_hand_(c) = kh_ * (q_posture_des(qi) - cache.q(qi));
  }

  // ── H = w_task·JᵀJ + diag(w_arm@arm, w_hand@hand) + μ²·I ──
  auto H = qp_data_.H.topLeftCorner(N, N);
  auto g = qp_data_.g.head(N);
  H.setZero();
  H.noalias() += w_task_ * j_task_.transpose() * j_task_;
  for (int c = 0; c < n_arm_; ++c) {
    const auto vi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    H(vi, vi) += w_arm_;
  }
  for (int c = 0; c < n_hand_; ++c) {
    const auto vi = static_cast<Eigen::Index>(hand_v_idx_[static_cast<size_t>(c)]);
    H(vi, vi) += w_hand_;
  }
  H.diagonal().array() += damping_sq_;

  // ── g = −(w_task·Jᵀr_task + scatter(w_arm·v_post_arm, w_hand·v_post_hand)) ──
  g.noalias() = -w_task_ * (j_task_.transpose() * r_task_);
  for (int c = 0; c < n_arm_; ++c) {
    const auto vi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    g(vi) -= w_arm_ * v_post_arm_(c);
  }
  for (int c = 0; c < n_hand_; ++c) {
    const auto vi = static_cast<Eigen::Index>(hand_v_idx_[static_cast<size_t>(c)]);
    g(vi) -= w_hand_ * v_post_hand_(c);
  }

  // ── Box constraints lᵢ ≤ vᵢ ≤ uᵢ (per-joint velocity ∩ position) ──
  auto l = qp_data_.l.head(N);
  auto u = qp_data_.u.head(N);
  const bool vel_box = (v_limit_ > 0.0);
  const bool pos_box = (q_min_.size() == N && q_max_.size() == N);
  const double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < N; ++i) {
    double lo = vel_box ? -v_limit_ : -inf;
    double hi = vel_box ? v_limit_ : inf;
    if (pos_box) {
      const double q_i = cache.q(i);  // q-index == v-index (nq == nv contract)
      lo = std::max(lo, (q_min_(i) - q_i) / dt);
      hi = std::min(hi, (q_max_(i) - q_i) / dt);
    }
    // NUM guard: an already-limit-violating joint can invert the box (lo > hi).
    // Collapse onto the limit-respecting bound so only motion back toward the
    // feasible set is allowed (v_limit may be exceeded in that recovery
    // direction — intentional, the violation must be corrected).
    if (lo > hi) {
      lo = hi;
    }
    l(i) = lo;
    u(i) = hi;
  }

  // ── Solve (C = Iₙ already set in Init) ──
  const auto& res = qp_solver_.Solve(qp_data_);
  if (!res.converged) {
    q_ref_ = cache.q;
    v_ref_.setZero();
    return false;
  }
  v_ref_ = res.x_opt.head(N);

  // ── Damped manipulability √det(J_a·J_aᵀ + μ²·I) — diag continuity only
  //    (the solve no longer forms J♯; this 6×6 is purely diagnostic). ──
  m6_.noalias() = j_arm_ * j_arm_.transpose();
  m6_.diagonal().array() += damping_sq_;
  ldlt6_.compute(m6_);
  const double det = ldlt6_.vectorD().prod();
  manipulability_ = (det > 0.0) ? std::sqrt(det) : 0.0;

  // ── One-step target, measured anchoring: q_ref = q_meas + v_ref·dt ──
  q_ref_ = cache.q;
  q_ref_.noalias() += v_ref_ * dt;

  if (!q_ref_.allFinite() || !v_ref_.allFinite()) {
    // Safe outputs even if the caller ignores the return value.
    q_ref_ = cache.q;
    v_ref_.setZero();
    return false;
  }
  return true;
}

}  // namespace rtc::tsid
