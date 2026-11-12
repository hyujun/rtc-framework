#include "rtc_tsid/constraints/torque_limit_constraint.hpp"

#include "rtc_tsid/core/yaml_vector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc::tsid {

void TorqueLimitConstraint::Init(const pinocchio::Model& /*model*/,
                                 const RobotModelInfo& robot_info, PinocchioCache& /*cache*/,
                                 const YAML::Node& constraint_config) {
  nv_ = robot_info.nv;
  na_ = robot_info.n_actuated;
  for (int i = 0; i < na_; ++i) {
    if (!(robot_info.tau_min(i) <= robot_info.tau_max(i))) {
      throw std::invalid_argument("torque_limit: tau_min must not exceed tau_max");
    }
  }

  // YAML 의 tau_scale (0 < x ≤ 1). 누락 시 1.0 — raw URDF effort_limit 사용 (회귀 0).
  // 범위 위반 시 init 에서 throw — RT path 아님, silent margin 변경 방지.
  tau_scale_ = 1.0;
  if (constraint_config && constraint_config["tau_scale"]) {
    tau_scale_ = constraint_config["tau_scale"].as<double>();
    if (!(tau_scale_ > 0.0 && tau_scale_ <= 1.0)) {
      throw std::runtime_error("TorqueLimitConstraint: tau_scale must be in (0, 1], got " +
                               std::to_string(tau_scale_));
    }
  }

  // rate_max [N·m/s] (scalar or one per actuated joint, > 0) with dt (> 0).
  has_rate_ = false;
  has_prev_ = false;
  dt_ = 0.002;
  rate_dt_.setZero(na_);
  tau_prev_.setZero(na_);
  if (constraint_config && constraint_config["dt"]) {
    dt_ = constraint_config["dt"].as<double>();
    if (!(std::isfinite(dt_) && dt_ > 0.0)) {
      throw std::invalid_argument("torque_limit: dt must be finite and > 0");
    }
  }
  if (constraint_config && constraint_config["rate_max"]) {
    rate_dt_ = ReadScalarOrVector(constraint_config, "rate_max", na_, 0.0, /*strict=*/true,
                                  "torque_limit") *
               dt_;
    has_rate_ = true;
  }

  SM_.setZero(na_, nv_);
  Sh_.setZero(na_);
}

void TorqueLimitConstraint::OnTorqueApplied(const Eigen::VectorXd& tau) noexcept {
  if (!has_rate_ || tau.size() != na_ || !tau.allFinite()) {
    return;  // nothing to bound, or nothing usable to bound with
  }
  tau_prev_ = tau;
  has_prev_ = true;
}

void TorqueLimitConstraint::Reset() noexcept {
  has_prev_ = false;
}

int TorqueLimitConstraint::EqDim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int TorqueLimitConstraint::IneqDim(const ContactState& /*contacts*/) const noexcept {
  return na_;
}

int TorqueLimitConstraint::MaxEqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
  return 0;
}

int TorqueLimitConstraint::MaxIneqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
  return na_;
}

void TorqueLimitConstraint::ComputeEquality(const PinocchioCache& /*cache*/,
                                            const ContactState& /*contacts*/,
                                            const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                            Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
                                            Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept {}

void TorqueLimitConstraint::ComputeInequality(const PinocchioCache& cache,
                                              const ContactState& contacts,
                                              const RobotModelInfo& robot_info, int /*n_vars*/,
                                              Eigen::Ref<Eigen::MatrixXd> C_block,
                                              Eigen::Ref<Eigen::VectorXd> l_block,
                                              Eigen::Ref<Eigen::VectorXd> u_block) noexcept {
  // τ = S·(M·a + h - Jcᵀ·λ)
  // τ_min ≤ S·(M·a + h - Jcᵀ·λ) ≤ τ_max
  //
  // C·z = S·M·a - S·Jcᵀ·λ
  // l = τ_min - S·h
  // u = τ_max - S·h

  // C[:, 0:nv] = S·M
  SM_.noalias() = robot_info.S * cache.M;
  C_block.leftCols(nv_) = SM_;

  // Stage A-5a: fixed-dim QP — every contact slot's λ columns are written
  // (active: -S·Jc_iᵀ, inactive: zero left from caller's zero-init). Offset
  // always advances so active contacts land on stable column indices.
  if (manager_) {
    int lambda_offset = 0;
    for (size_t i = 0; i < contacts.contacts.size(); ++i) {
      const int cdim = manager_->contacts[i].contact_dim;

      if (!contacts.contacts[i].active || i >= cache.contact_frames.size()) {
        lambda_offset += cdim;
        continue;
      }

      const auto& Jc = cache.contact_frames[i].J;

      // −S·Jcᵀ, not −(S·Jcᵀ): a negated product is evaluated into a heap
      // temporary, a negated factor is folded into the GEMM (same bits).
      C_block.block(0, nv_ + lambda_offset, na_, cdim).noalias() =
          -robot_info.S * Jc.topRows(cdim).transpose();

      lambda_offset += cdim;
    }
  }

  // l = tau_scale·τ_min - S·h,  u = tau_scale·τ_max - S·h
  // tau_scale=1.0 (default) 면 기존 raw URDF effort_limit 사용.
  Sh_.noalias() = robot_info.S * cache.h;
  l_block.head(na_) = tau_scale_ * robot_info.tau_min - Sh_;
  u_block.head(na_) = tau_scale_ * robot_info.tau_max - Sh_;

  // Torque rate (header): the same rows, cut to τ_prev ± rate·dt once a
  // torque was applied; a box beyond the effort limits is pinned to them.
  int clamped = 0;
  if (has_prev_) {  // implies has_rate_: OnTorqueApplied stores nothing otherwise
    for (int i = 0; i < na_; ++i) {
      const double lim_lo = tau_scale_ * robot_info.tau_min(i);
      const double lim_hi = tau_scale_ * robot_info.tau_max(i);
      double lo = std::max(lim_lo, tau_prev_(i) - rate_dt_(i));
      double hi = std::min(lim_hi, tau_prev_(i) + rate_dt_(i));
      if (lo > hi) {
        ++clamped;
        lo = hi = (tau_prev_(i) - rate_dt_(i) > lim_hi) ? lim_hi : lim_lo;
      }
      l_block(i) = lo - Sh_(i);
      u_block(i) = hi - Sh_(i);
    }
  }
  clamped_count_ = clamped;
}

}  // namespace rtc::tsid
