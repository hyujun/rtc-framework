#include "rtc_tsid/kinematics/clik_reference.hpp"

#include "rtc_tsid/kinematics/se3_error.hpp"

#include <algorithm>
#include <cmath>
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

  manipulability_ = 0.0;
  tcp_error_norm_ = 0.0;

  q_ref_.setZero(nv_);
  v_ref_.setZero(nv_);
  j_arm_.setZero(6, n_arm_);
  v_post_arm_.setZero(n_arm_);
  v_arm_.setZero(n_arm_);
  m6_.setZero();
  e_x_.setZero();
  rhs6_.setZero();
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

  // ── L1: SE(3) 오차 (공용 규약) + task-space 목표 속도 ──
  e_x_ = ComputeSe3Error(tip_in_base, placement_des);
  // 회전 오차를 base 좌표로 정렬: log3(R_cᵀ·R_d) 는 tip(body) 좌표 벡터인데
  // rf.J 의 각속도 행은 LOCAL_WORLD_ALIGNED(world≈base) — body 좌표 그대로
  // 피드백하면 tip 이 base 대비 90°+ 회전한 자세에서 축 부호가 반전되어
  // 발산한다. R_tip_in_base·e_body = log3(R_d·R_cᵀ) (norm 불변 → SE3Task
  // 오차와 동일 척도, A/B 비교 유효).
  e_x_.tail<3>() = tip_in_base.rotation() * e_x_.tail<3>();
  tcp_error_norm_ = e_x_.norm();

  // Arm column gather: J_a = rf.J(:, arm_v_idx)
  for (int c = 0; c < n_arm_; ++c) {
    j_arm_.col(c) = rf.J.col(arm_v_idx_[static_cast<size_t>(c)]);
  }

  // ── L2: arm posture velocity v_p = Ka·(q_a_des − q_a) ──
  for (int c = 0; c < n_arm_; ++c) {
    const auto qi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    v_post_arm_(c) = ka_ * (q_posture_des(qi) - cache.q(qi));
  }

  // v_arm = v_p + J_aᵀ·(J_a·J_aᵀ + μ²I)⁻¹·(Kx⊙e − J_a·v_p)
  // — 우역행렬·projector 를 명시적으로 만들지 않는 동치식 (header 참조).
  rhs6_ = kx_.cwiseProduct(e_x_);
  rhs6_.noalias() -= j_arm_ * v_post_arm_;

  m6_.noalias() = j_arm_ * j_arm_.transpose();
  m6_.diagonal().array() += damping_sq_;
  ldlt6_.compute(m6_);

  // Damped manipulability from the LDLT diagonal pivots (det ≥ μ¹² > 0).
  const double det = ldlt6_.vectorD().prod();
  manipulability_ = (det > 0.0) ? std::sqrt(det) : 0.0;

  ldlt6_.solveInPlace(rhs6_);
  v_arm_ = v_post_arm_;
  v_arm_.noalias() += j_arm_.transpose() * rhs6_;

  // ── L3 + scatter: per-joint clamp 후 v_ref 에 산개, 나머지 0 ──
  v_ref_.setZero();
  const bool clamp_on = v_limit_ > 0.0;
  for (int c = 0; c < n_arm_; ++c) {
    const double vc = v_arm_(c);
    v_ref_(static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)])) =
        clamp_on ? std::clamp(vc, -v_limit_, v_limit_) : vc;
  }
  for (int c = 0; c < n_hand_; ++c) {
    const auto qi = static_cast<Eigen::Index>(hand_v_idx_[static_cast<size_t>(c)]);
    const double vc = kh_ * (q_posture_des(qi) - cache.q(qi));
    v_ref_(qi) = clamp_on ? std::clamp(vc, -v_limit_, v_limit_) : vc;
  }

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
