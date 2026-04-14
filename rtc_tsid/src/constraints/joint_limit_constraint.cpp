#include "rtc_tsid/constraints/joint_limit_constraint.hpp"

#include <cmath>

namespace rtc::tsid {

void JointLimitConstraint::init(
    const pinocchio::Model& /*model*/,
    const RobotModelInfo& robot_info,
    PinocchioCache& /*cache*/,
    const YAML::Node& constraint_config) {
  nv_ = robot_info.nv;
  floating_base_ = robot_info.floating_base;

  // YAML config
  if (constraint_config) {
    dt_ = constraint_config["dt"].as<double>(0.002);
    margin_pos_ = constraint_config["position_margin"].as<double>(0.05);
    margin_vel_ = constraint_config["velocity_margin"].as<double>(0.1);
  }

  // Position limits (margin 적용)
  // Note: robot_info.q_lower/q_upper는 [nq], 여기서는 [nv] 기준
  // Fixed-base: nq == nv, 직접 사용
  // Floating-base: 처음 7개 (quat) → nv에서는 6개, 무제약 처리
  q_min_.resize(nv_);
  q_max_.resize(nv_);

  // Position limits + margin (per-joint margin clamping)
  // margin이 joint range의 절반을 초과하면 0으로 clamp
  {
    const int offset = floating_base_ ? 6 : 0;
    const int q_offset = floating_base_ ? 7 : 0;  // quaternion 보정

    if (floating_base_) {
      q_min_.head(6).setConstant(-1e10);
      q_max_.head(6).setConstant(1e10);
    }

    const int na = nv_ - offset;
    for (int i = 0; i < na; ++i) {
      const double ql = robot_info.q_lower(q_offset + i);
      const double qu = robot_info.q_upper(q_offset + i);
      const double range = qu - ql;
      const double eff_margin = (range > 2.0 * margin_pos_) ? margin_pos_ : 0.0;
      q_min_(offset + i) = ql + eff_margin;
      q_max_(offset + i) = qu - eff_margin;
    }
  }

  // Velocity limits + margin (per-joint margin clamping)
  v_min_.resize(nv_);
  v_max_limit_.resize(nv_);

  {
    const int offset = floating_base_ ? 6 : 0;

    if (floating_base_) {
      v_min_.head(6).setConstant(-1e10);
      v_max_limit_.head(6).setConstant(1e10);
    }

    const int na = nv_ - offset;
    for (int i = 0; i < na; ++i) {
      const double vl = robot_info.v_max(offset + i);
      const double eff_margin = (vl > 2.0 * margin_vel_) ? margin_vel_ : 0.0;
      v_min_(offset + i) = -vl + eff_margin;
      v_max_limit_(offset + i) = vl - eff_margin;
    }
  }

  // Workspace pre-allocate
  a_pos_lb_.setZero(nv_);
  a_pos_ub_.setZero(nv_);
  a_vel_lb_.setZero(nv_);
  a_vel_ub_.setZero(nv_);
}

int JointLimitConstraint::eq_dim(
    const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int JointLimitConstraint::ineq_dim(
    const ContactState& /*contacts*/) const noexcept {
  return nv_;
}

void JointLimitConstraint::compute_equality(
    const PinocchioCache& /*cache*/,
    const ContactState& /*contacts*/,
    const RobotModelInfo& /*robot_info*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
    Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept {
  // No equality constraints
}

void JointLimitConstraint::compute_inequality(
    const PinocchioCache& cache,
    const ContactState& /*contacts*/,
    const RobotModelInfo& /*robot_info*/,
    int /*n_vars*/,
    Eigen::Ref<Eigen::MatrixXd> C_block,
    Eigen::Ref<Eigen::VectorXd> l_block,
    Eigen::Ref<Eigen::VectorXd> u_block) noexcept {
  const double dt2 = dt_ * dt_;

  // C = [I_{nv} | 0]
  for (int i = 0; i < nv_; ++i) {
    C_block(i, i) = 1.0;
  }

  // Floating-base 또는 fixed-base에 따라 q 접근 방식 결정
  // Fixed-base: cache.q는 [nv], 직접 사용
  // Floating-base: cache.q는 [nq=nv+1], v-space 매핑 필요
  // cache.v는 항상 [nv]

  for (int i = 0; i < nv_; ++i) {
    const double vi = cache.v(i);

    // Position-based bounds: a ≥ 2·(q_min - q - v·dt) / dt²
    // Floating-base 처음 6 DoF: q_min=-1e10, q_max=1e10 → bound 사실상 무제약
    double qi;
    if (floating_base_ && i < 6) {
      // Free-flyer DoF: 무제약 (q_min/q_max already ±1e10)
      qi = 0.0;  // 아무 값이나 (bound가 ±1e10이므로 무의미)
    } else {
      // Actuated joint: q index 계산
      // floating-base: q[7+i-6] = q[i+1] (quaternion 4D → velocity 3D 차이)
      const int q_idx = floating_base_ ? (i + 1) : i;
      qi = cache.q(q_idx);
    }

    a_pos_lb_(i) = 2.0 * (q_min_(i) - qi - vi * dt_) / dt2;
    a_pos_ub_(i) = 2.0 * (q_max_(i) - qi - vi * dt_) / dt2;

    // Velocity-based bounds: a ≥ (v_min - v) / dt
    a_vel_lb_(i) = (v_min_(i) - vi) / dt_;
    a_vel_ub_(i) = (v_max_limit_(i) - vi) / dt_;

    // 최종: tightening
    l_block(i) = std::max(a_pos_lb_(i), a_vel_lb_(i));
    u_block(i) = std::min(a_pos_ub_(i), a_vel_ub_(i));
  }
}

}  // namespace rtc::tsid
