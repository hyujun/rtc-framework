#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Torque limit constraint
//
// τ = S·(M·a + h - Jcᵀ·λ)
// τ_min ≤ τ ≤ τ_max
//
// C = [S·M | -S·Jcᵀ]    (n_actuated × n_vars)
// l = τ_min - S·h
// u = τ_max - S·h
//
// `rate_max` (optional, scalar or one value per actuated joint [N·m/s], > 0)
// adds the torque-rate bound on the SAME rows: with τ_prev the torque the
// owner last applied (OnTorqueApplied — a tau_valid tick only) and dt the
// control period (`dt`, default 0.002 s),
//   l = max(tau_scale·τ_min, τ_prev − rate·dt) − S·h
//   u = min(tau_scale·τ_max, τ_prev + rate·dt) − S·h.
// Before any torque was applied (the first tick, after Reset, after a run of
// non-converged ticks nothing was applied on) only the effort limits act. A
// τ_prev outside the effort limits would empty the row: it is then pinned to
// the limit it lies beyond (ClampedCount()). Turning the constraint off by a
// phase preset turns the rate bound off with it (it is one row).
// ────────────────────────────────────────────────
class TorqueLimitConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "torque_limit"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& constraint_config) override;

  [[nodiscard]] int EqDim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int IneqDim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int MaxEqDim(const ContactManagerConfig& contacts) const noexcept override;
  [[nodiscard]] int MaxIneqDim(const ContactManagerConfig& contacts) const noexcept override;

  void ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                       const RobotModelInfo& robot_info, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> A_block,
                       Eigen::Ref<Eigen::VectorXd> b_block) noexcept override;

  void ComputeInequality(const PinocchioCache& cache, const ContactState& contacts,
                         const RobotModelInfo& robot_info, int n_vars,
                         Eigen::Ref<Eigen::MatrixXd> C_block, Eigen::Ref<Eigen::VectorXd> l_block,
                         Eigen::Ref<Eigen::VectorXd> u_block) noexcept override;

  void SetContactManager(const ContactManagerConfig* manager) noexcept { manager_ = manager; }

  // The applied torque becomes τ_prev of the next tick (size n_actuated and
  // finite; anything else is ignored). It is the QP's own τ (TSIDController
  // forwards it on tau_valid ticks): an owner that saturates or filters the
  // torque before applying it bounds the rate of the QP output, not of what
  // reached the plant.
  void OnTorqueApplied(const Eigen::VectorXd& tau) noexcept override;

  // Forget τ_prev: the next tick bounds the effort limits only.
  void Reset() noexcept override;

  [[nodiscard]] bool RateMaxEnabled() const noexcept { return has_rate_; }

  [[nodiscard]] bool HasPreviousTorque() const noexcept { return has_prev_; }

  [[nodiscard]] double Dt() const noexcept { return dt_; }

  [[nodiscard]] const Eigen::VectorXd& PreviousTorque() const noexcept { return tau_prev_; }

  // Rows of the last ComputeInequality whose rate box lay beyond the effort
  // limits and was pinned to them.
  [[nodiscard]] int ClampedCount() const noexcept { return clamped_count_; }

 private:
  bool has_rate_{false};
  bool has_prev_{false};
  double dt_{0.002};
  int clamped_count_{0};
  Eigen::VectorXd rate_dt_;   // [na] rate_max·dt
  Eigen::VectorXd tau_prev_;  // [na]

  int nv_{0};
  int na_{0};
  // URDF effort_limit 에 곱하는 margin (0 < tau_scale ≤ 1). YAML 키 누락 시 1.0
  // (raw URDF 사용). init 에서 (0, 1] 범위 위반 시 throw — silent margin 변경 방지.
  double tau_scale_{1.0};
  const ContactManagerConfig* manager_{nullptr};

  // Pre-allocated workspace
  Eigen::MatrixXd SM_;  // [na × nv] S·M
  Eigen::VectorXd Sh_;  // [na] S·h
};

}  // namespace rtc::tsid
