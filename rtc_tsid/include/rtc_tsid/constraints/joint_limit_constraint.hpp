#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Acceleration-level joint limit constraint — ONE-TICK reachability of the
// position and velocity limits. (Not the multi-step viability bound of
// Del Prete 2018: this box only asks that the next sample respect q / v
// limits, which is why a joint can still be parked inside the margin band.)
//
// Position limit → acceleration bound. With Δ_u = q_max_i − q_i − v_i·dt and
// Δ_l = q_min_i − q_i − v_i·dt (where q lands at a = 0):
//   a_i ≤ Δ_u / dt²   if Δ_u ≥ 0,   a_i ≤ 2·Δ_u / dt²   if Δ_u < 0
//   a_i ≥ Δ_l / dt²   if Δ_l ≤ 0,   a_i ≥ 2·Δ_l / dt²   if Δ_l > 0
// Why (#1019): nothing in the stack integrates a_opt — the plant does. A
// plant that holds a over the control period in N semi-implicit substeps of
// dt/N moves q by v·dt + c·a·dt², c = (1 + 1/N)/2 ∈ (½, 1] (N = 1: MuJoCo's
// Euler at one step per period; N → ∞: a continuous plant). The bound that
// keeps q⁺ inside the limit for every such c is Δ/(c·dt²) at its tightest:
// c = 1 while the joint would stay inside at a = 0, c = ½ once it would not.
// Continuous at Δ = 0. Not covered: a plant whose step is longer than the
// control period, and the step after next (one-tick reachability only).
// Velocity limit → acceleration bound  (v⁺ = v + a·dt):
//   a_i ≥ (v_min_i - v_i) / dt
//   a_i ≤ (v_max_i - v_i) / dt
// One row per actuated joint: l_i = max(pos_lb, vel_lb), u_i = min(pos_ub,
// vel_ub), C row i = e_(n_base + i) (C = [I_nv | 0] on a fixed base).
//
// Inside the position-margin band the two cross (l_i > u_i). The row is then
// collapsed onto the velocity bound it crossed (l = u = vel_lb at the upper
// limit, vel_ub at the lower): the box stays non-empty and the QP is asked to
// reverse the joint toward the range at its velocity limit within one tick
// (#551). That acceleration, |v_lim − v| / dt, is a violent command: the QP
// stays solvable with hard torque limits (the other joints redistribute the
// coupled torque — test_tsid_wqp pins this), but the collapse is a safety net,
// not a substitute for keeping joints out of the band. InvertedCount() says
// how many rows needed it this tick. Config: dt (the control period: a
// consumer that knows its rate passes 1/control_rate and refuses a YAML dt
// that disagrees — DemoWbcController does), position_margin, velocity_margin.
// Keep position_margin SMALLER than the margin of whatever generates the
// position reference — a joint parked at that margin otherwise sits in this
// band on every tick.
//
// Floating-base: the free-flyer's DoF get no row (IneqDim = nv − n_base_v). Init
// requires every non-root joint (every joint on a fixed base) to be 1-DoF, so
// joint i's q and v entries pair up; anything else throws.
// ────────────────────────────────────────────────
class JointLimitConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "joint_limit"; }

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

  // Rows of the last ComputeInequality whose assembled box was empty
  // (l_i > u_i) before any repair. Rows with non-finite q_i / v_i are not
  // counted. Diagnostic only — read by the controller's wbc_diag log (#551).
  [[nodiscard]] int InvertedCount() const noexcept { return inverted_count_; }

  // The period the box integrates over (YAML `dt`, default 0.002 s).
  [[nodiscard]] double Dt() const noexcept { return dt_; }

 private:
  int nv_{0};
  int n_base_{0};  // velocity rows without a limit: RobotModelInfo::n_base_v
  int n_rows_{0};  // nv − n_base
  int inverted_count_{0};
  std::vector<int> q_index_;  // [n_rows] q entry of row i

  double dt_{0.002};         // 제어 주기 (s)
  double margin_pos_{0.05};  // position limit margin (rad)
  double margin_vel_{0.1};   // velocity limit margin (rad/s)

  Eigen::VectorXd q_min_;        // [n_rows] (margin 적용 후)
  Eigen::VectorXd q_max_;        // [n_rows] (margin 적용 후)
  Eigen::VectorXd v_min_;        // [n_rows] (margin 적용 후, = -v_max + margin)
  Eigen::VectorXd v_max_limit_;  // [n_rows] (margin 적용 후, = v_max - margin)

  // Pre-allocated workspace
  Eigen::VectorXd a_pos_lb_;  // [n_rows]
  Eigen::VectorXd a_pos_ub_;  // [n_rows]
  Eigen::VectorXd a_vel_lb_;  // [n_rows]
  Eigen::VectorXd a_vel_ub_;  // [n_rows]
};

}  // namespace rtc::tsid
