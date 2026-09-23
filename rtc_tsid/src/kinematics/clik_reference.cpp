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
  // `!(x > 0.0)` already rejects NaN (every NaN comparison is false), but it
  // admits +inf — and these two sit in the SAME soft-priority chain the
  // w_arm / w_hand gate below refuses infinities for (w_task >> w_arm, w_hand
  // >> damping_sq). An infinite member makes that ordering vacuous: +inf
  // damping_sq regularises H into uselessness, +inf w_task drowns every other
  // term. Finiteness first, so the positivity test never sees a NaN.
  if (!std::isfinite(config.damping_sq) || config.damping_sq <= 0.0) {
    throw std::runtime_error("ClikReferenceGenerator: damping_sq must be finite and > 0, got " +
                             std::to_string(config.damping_sq));
  }
  if (!std::isfinite(config.w_task) || config.w_task <= 0.0) {
    throw std::runtime_error("ClikReferenceGenerator: w_task must be finite and > 0, got " +
                             std::to_string(config.w_task));
  }
  // NUM-7: `x < 0.0` is a magnitude test, not a finiteness gate. Every
  // comparison against NaN is false, so the sign form walks a NaN weight
  // through a guard that READS as if it had validated it. Rewriting it as
  // `!(x >= 0.0)` catches NaN and -inf but NOT +inf, which satisfies `>= 0`
  // and has no sanctioned meaning here — an infinite posture weight lands on
  // H's diagonal (`H(vi, vi) += w_arm_`) and there is no finite task weight it
  // could be ranked against (the soft-priority contract is w_task >> w_arm,
  // w_hand >> damping_sq). Both halves are therefore needed: finiteness first,
  // so the sign test never sees a NaN.
  //
  // Of the two failure shapes this gate covers, this is the less bad one: a
  // poisoned H makes the QP fail downstream, loudly. The silent shape is
  // v_limit / anchor_drift_max below.
  if (!std::isfinite(config.w_arm) || !std::isfinite(config.w_hand) || config.w_arm < 0.0 ||
      config.w_hand < 0.0) {
    throw std::runtime_error("ClikReferenceGenerator: w_arm / w_hand must be finite and >= 0");
  }
  // NUM-7, the SILENT axis. Both of these are switches whose "off" position is
  // `<= 0`, and both are read back as a `> 0.0` predicate (`vel_box =
  // (v_limit_ > 0.0)`; `if (anchor_drift_max_ > 0.0)`). A NaN makes both
  // predicates false, so the feature turns ITSELF off indistinguishably from
  // the sanctioned off sentinel: the per-joint velocity clamp and the
  // carry-forward anti-windup clamp simply stop existing, q_ref / v_ref stay
  // finite and plausible, and Compute()'s own allFinite() exit guard therefore
  // never fires. Nothing downstream can tell that configuration from a
  // deliberate `v_limit: -1`.
  //
  // The guard is isfinite, NOT `> 0`: a FINITE non-positive value is a
  // legitimate request to disable the clamp and must keep passing (0 and -1
  // are both pinned by the tests). Neither infinity is that request — "off"
  // already has a sanctioned encoding here, and accepting ±inf as a second one
  // is exactly what the position box gate refused: -inf would reach the QP as
  // an unbounded side, +inf differs from off only while some other switch
  // happens to agree.
  if (!std::isfinite(config.v_limit)) {
    throw std::runtime_error("ClikReferenceGenerator: v_limit must be finite, got " +
                             std::to_string(config.v_limit));
  }
  if (config.v_limit_per_joint.size() != 0) {
    if (config.v_limit_per_joint.size() != nv) {
      throw std::runtime_error("ClikReferenceGenerator: v_limit_per_joint size " +
                               std::to_string(config.v_limit_per_joint.size()) + " != nv " +
                               std::to_string(nv));
    }
    // Per-joint limits have no "off" encoding: a joint without a bound belongs
    // in the scalar path. NaN fails both tests below (NUM-7).
    for (Eigen::Index i = 0; i < config.v_limit_per_joint.size(); ++i) {
      const double v = config.v_limit_per_joint(i);
      if (!std::isfinite(v) || !(v > 0.0)) {
        throw std::runtime_error(
            "ClikReferenceGenerator: v_limit_per_joint must be finite and > 0, got " +
            std::to_string(v) + " at index " + std::to_string(i));
      }
    }
  }
  if (config.a_max.size() != 0) {
    if (config.a_max.size() != nv) {
      throw std::runtime_error("ClikReferenceGenerator: a_max size " +
                               std::to_string(config.a_max.size()) + " != nv " +
                               std::to_string(nv));
    }
    if (nv > 64) {
      throw std::runtime_error(
          "ClikReferenceGenerator: acceleration box needs nv <= 64 (conflict_mask width), got " +
          std::to_string(nv));
    }
    for (Eigen::Index i = 0; i < config.a_max.size(); ++i) {
      const double a = config.a_max(i);
      if (!std::isfinite(a) || !(a > 0.0)) {
        throw std::runtime_error("ClikReferenceGenerator: a_max must be finite and > 0, got " +
                                 std::to_string(a) + " at index " + std::to_string(i));
      }
    }
  }
  if (!std::isfinite(config.w_smooth) || config.w_smooth < 0.0) {
    throw std::runtime_error("ClikReferenceGenerator: w_smooth must be finite and >= 0, got " +
                             std::to_string(config.w_smooth));
  }
  if (config.evaluate_at_command && config.anchor_drift_max > 0.0) {
    throw std::runtime_error(
        "ClikReferenceGenerator: anchor_drift_max needs the measured state and cannot be used "
        "with evaluate_at_command");
  }
  if (!std::isfinite(config.w_axis) || !(config.w_axis > 0.0)) {
    throw std::runtime_error("ClikReferenceGenerator: w_axis must be finite and > 0, got " +
                             std::to_string(config.w_axis));
  }
  if (config.max_iter < 1) {
    throw std::runtime_error("ClikReferenceGenerator: max_iter must be >= 1, got " +
                             std::to_string(config.max_iter));
  }
  if (!std::isfinite(config.anchor_drift_max)) {
    throw std::runtime_error("ClikReferenceGenerator: anchor_drift_max must be finite, got " +
                             std::to_string(config.anchor_drift_max));
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
  // NUM-7: the size/symmetry checks above are NOT a finiteness gate. A
  // non-finite component is laundered by the box assembly in Compute() —
  // std::max(lo, NaN) returns lo and std::min(hi, NaN) returns hi — so the
  // position bound silently vanishes for that joint while q_ref/v_ref stay
  // finite, which means Compute()'s own allFinite() output guard never fires.
  // The inverted-box collapse (lo > hi) is equally blind: every comparison
  // against NaN is false. ±inf is rejected too: the sanctioned encoding for
  // "no position bound" is an EMPTY q_min/q_max, and -inf is only benign while
  // the velocity box is on (with v_limit <= 0 it reaches the QP as l = -inf),
  // i.e. it is not a local property. A caller whose model reports unbounded
  // joints (Pinocchio's ±inf convention for continuous joints) must leave the
  // box empty rather than forward the infinities. The in-tree consumer never
  // reaches this: Pinocchio only emits ±inf limits for unbounded/free-flyer
  // joints, which make nq != nv, and InitClik's reduced-tree gate rejects that
  // model before it ever assembles a box.
  //
  // q_min(i) == q_max(i) is legitimate and must pass: it is a joint locked by
  // the caller's own safety margin (shipped panda finger [0, 0.04] under
  // position_margin 0.02 lands exactly on equality). Only strict inversion is
  // a contradiction — that box admits no velocity at all, and the collapse
  // guard would drive the joint at the full velocity limit forever.
  for (Eigen::Index i = 0; i < config.q_min.size(); ++i) {
    if (!std::isfinite(config.q_min(i)) || !std::isfinite(config.q_max(i))) {
      throw std::runtime_error("ClikReferenceGenerator: q_min / q_max must be finite, got [" +
                               std::to_string(config.q_min(i)) + ", " +
                               std::to_string(config.q_max(i)) + "] at index " + std::to_string(i));
    }
    if (config.q_min(i) > config.q_max(i)) {
      throw std::runtime_error("ClikReferenceGenerator: q_min must be <= q_max, got q_min " +
                               std::to_string(config.q_min(i)) + " > q_max " +
                               std::to_string(config.q_max(i)) + " at index " + std::to_string(i));
    }
  }

  // Acceleration constraint: exactly one of box | kinematic | dynamic, and a
  // field of another form set is a configuration that reads as if it were on.
  const bool kin_set = config.task_accel_max_linear != 0.0 || config.task_accel_max_angular != 0.0;
  const bool dyn_set = config.tau_max.size() != 0 || config.eta_tau != 0.0;
  switch (config.accel_constraint) {
    case AccelConstraint::kBox:
      if (kin_set || dyn_set) {
        throw std::runtime_error(
            "ClikReferenceGenerator: task_accel_max_* / tau_max / eta_tau are set but "
            "accel_constraint is box");
      }
      break;
    case AccelConstraint::kKinematic:
      if (config.a_max.size() != 0 || dyn_set) {
        throw std::runtime_error(
            "ClikReferenceGenerator: accel_constraint kinematic takes task_accel_max_* only "
            "(a_max / tau_max must be empty, eta_tau 0)");
      }
      if (!std::isfinite(config.task_accel_max_linear) || !(config.task_accel_max_linear > 0.0) ||
          !std::isfinite(config.task_accel_max_angular) || !(config.task_accel_max_angular > 0.0)) {
        throw std::runtime_error(
            "ClikReferenceGenerator: task_accel_max_linear / _angular must be finite and > 0");
      }
      break;
    case AccelConstraint::kDynamic:
      if (config.a_max.size() != 0 || kin_set) {
        throw std::runtime_error(
            "ClikReferenceGenerator: accel_constraint dynamic takes tau_max / eta_tau only "
            "(a_max / task_accel_max_* must be unset)");
      }
      if (config.tau_max.size() != nv) {
        throw std::runtime_error("ClikReferenceGenerator: tau_max size " +
                                 std::to_string(config.tau_max.size()) + " != nv " +
                                 std::to_string(nv));
      }
      if (!std::isfinite(config.eta_tau) || !(config.eta_tau > 0.0) || config.eta_tau > 1.0) {
        throw std::runtime_error("ClikReferenceGenerator: eta_tau must be in (0, 1], got " +
                                 std::to_string(config.eta_tau));
      }
      for (Eigen::Index i = 0; i < config.tau_max.size(); ++i) {
        if (!std::isfinite(config.tau_max(i)) || config.tau_max(i) < 0.0) {
          throw std::runtime_error("ClikReferenceGenerator: tau_max must be finite and >= 0, got " +
                                   std::to_string(config.tau_max(i)) + " at index " +
                                   std::to_string(i));
        }
      }
      for (const int i : config.arm_v_idx) {
        if (i >= 0 && i < nv && !(config.tau_max(i) > 0.0)) {
          throw std::runtime_error(
              "ClikReferenceGenerator: tau_max must be > 0 on every arm index, got 0 at index " +
              std::to_string(i));
        }
      }
      break;
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
  v_limit_per_joint_ = config.v_limit_per_joint;
  a_max_ = config.a_max;
  w_smooth_ = config.w_smooth;
  accel_constraint_ = config.accel_constraint;
  task_a_lin_ = config.task_accel_max_linear;
  task_a_ang_ = config.task_accel_max_angular;
  if (accel_constraint_ == AccelConstraint::kDynamic) {
    tau_bound_ = config.eta_tau * config.tau_max;
  } else {
    tau_bound_.resize(0);
  }
  switch (accel_constraint_) {
    case AccelConstraint::kBox:
      n_accel_rows_max_ = 0;
      break;
    case AccelConstraint::kKinematic:
      n_accel_rows_max_ = 6;  // the SE3 overload's rows; the axis overload uses 5
      break;
    case AccelConstraint::kDynamic:
      n_accel_rows_max_ = static_cast<int>(config.arm_v_idx.size());
      break;
  }
  n_accel_rows_ = 0;
  evaluate_at_command_ = config.evaluate_at_command;
  w_axis_ = config.w_axis;
  w_task_ = config.w_task;
  w_arm_ = config.w_arm;
  w_hand_ = config.w_hand;
  q_min_ = config.q_min;
  q_max_ = config.q_max;
  anchor_drift_max_ = config.anchor_drift_max;
  anchor_initialized_ = false;  // first Compute() re-anchors to measured
  command_check_armed_ = false;

  manipulability_ = 0.0;
  tcp_error_norm_ = 0.0;
  last_solve_ = SolveDiagnostics{};

  q_ref_.setZero(nv_);
  v_ref_.setZero(nv_);
  v_prev_.setZero(nv_);
  j_task_.setZero(6, nv_);
  j_pos_.setZero(3, nv_);
  j_axis_.setZero(2, nv_);
  v_post_arm_.setZero(n_arm_);
  v_post_hand_.setZero(n_hand_);
  j_arm_.setZero(6, n_arm_);
  m6_.setZero();
  e_x_.setZero();
  r_task_.setZero();

  // Fixed-dim box QP: N variables, no equality, N inequality rows (C = Iₙ).
  // C is constant (per-joint velocity box) → set once here; only l/u/H/g change
  // per tick. Dim fields are fixed so QPSolverWrapper only update()s on the RT
  // path (no re-init / heap alloc). kKinematic / kDynamic add their rows BELOW
  // the box (rewritten each tick); kBox adds none, so its QP is the legacy one.
  const int n_ineq = nv_ + n_accel_rows_max_;
  qp_data_.Init(nv_, 0, n_ineq);
  qp_data_.C.topLeftCorner(nv_, nv_).setIdentity();
  if (n_accel_rows_max_ > 0) {
    qp_data_.l.tail(n_accel_rows_max_).setConstant(-std::numeric_limits<double>::infinity());
    qp_data_.u.tail(n_accel_rows_max_).setConstant(std::numeric_limits<double>::infinity());
  }
  qp_data_.n_vars = nv_;
  qp_data_.n_eq = 0;
  qp_data_.n_ineq = n_ineq;

  QPSolverConfig solver_cfg;
  solver_cfg.max_iter = config.max_iter;
  qp_solver_.Init(nv_, 0, n_ineq, solver_cfg);
}

bool ClikReferenceGenerator::Compute(const PinocchioCache& cache, int tcp_frame_idx,
                                     int base_frame_idx, const pinocchio::SE3& placement_des,
                                     const Eigen::VectorXd& q_posture_des, double dt,
                                     bool reseed_anchor,
                                     const Eigen::Matrix<double, 6, 1>* twist_ff) noexcept {
  last_solve_ = SolveDiagnostics{};
  if (!PreconditionsHold(cache, tcp_frame_idx, base_frame_idx, q_posture_des, dt)) {
    return false;
  }
  if (twist_ff != nullptr && !twist_ff->allFinite()) {
    return false;
  }
  if (!CommandStateMatches(cache)) {
    last_solve_.command_mismatch = true;
    return false;
  }

  const auto& rf = cache.registered_frames[static_cast<size_t>(tcp_frame_idx)];

  // 현재 TCP pose를 base 기준으로 (SE3Task 와 동일한 contract/fast-path).
  const pinocchio::SE3 tip_in_base =
      (base_frame_idx < 0)
          ? rf.oMf
          : cache.registered_frames[static_cast<size_t>(base_frame_idx)].oMf.actInv(rf.oMf);

  // ── L1: SE(3) 오차 → task-velocity reference r_task = Kx ⊙ e_x ──
  // ComputeTaskPoseError = LWA BodyLog6 — SE3Task/ObjectSE3Task (dynamics WBC) 와
  // 동일 척도(U1 통일, A/B command-source 비교 전제). 속도 법칙은 1차 유지.
  e_x_ = ComputeTaskPoseError(tip_in_base, placement_des);
  tcp_error_norm_ = e_x_.norm();
  r_task_ = kx_.cwiseProduct(e_x_);
  if (twist_ff != nullptr) {
    r_task_ += *twist_ff;
  }

  const int N = nv_;

  // ── Task Jacobian J_task ∈ R⁶ˣᴺ: arm columns of rf.J, hand columns 0 ──
  // (also gather j_arm_ for the manipulability diagnostic).
  j_task_.setZero();
  for (int c = 0; c < n_arm_; ++c) {
    const auto vi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    j_task_.col(vi) = rf.J.col(vi);
    j_arm_.col(c) = rf.J.col(vi);
  }

  ComputePostureReferences(cache, q_posture_des);

  // ── H = w_task·JᵀJ + diag(w_arm@arm, w_hand@hand) + μ²·I ──
  auto H = qp_data_.H.topLeftCorner(N, N);
  auto g = qp_data_.g.head(N);
  H.setZero();
  H.noalias() += w_task_ * j_task_.transpose() * j_task_;
  // ── g = −(w_task·Jᵀr_task + scatter(w_arm·v_post_arm, w_hand·v_post_hand)) ──
  g.noalias() = -w_task_ * (j_task_.transpose() * r_task_);
  AddPostureAndDamping();

  AssembleBox(cache.q, dt);
  if (!AssembleAccelRows(cache, rf, dt, false)) {
    last_solve_.non_finite = true;  // a non-finite M, h, v or drift
    return Fail(cache);
  }
  return SolveAndIntegrate(cache, dt, reseed_anchor);
}

bool ClikReferenceGenerator::Compute(const PinocchioCache& cache, int frame_idx, int base_frame_idx,
                                     const PositionAxisTarget& target,
                                     const Eigen::VectorXd& q_posture_des, double dt,
                                     bool reseed_anchor) noexcept {
  namespace se3 = rtc::math::se3;
  last_solve_ = SolveDiagnostics{};
  axis_region_ = se3::AxisAlignRegion::kInvalidInput;
  if (!PreconditionsHold(cache, frame_idx, base_frame_idx, q_posture_des, dt)) {
    return false;
  }
  if (!target.position.allFinite() || !target.linear_velocity_ff.allFinite() ||
      !target.angular_velocity_ff.allFinite()) {
    return false;
  }
  if (!CommandStateMatches(cache)) {
    last_solve_.command_mismatch = true;
    return false;
  }

  const auto& rf = cache.registered_frames[static_cast<size_t>(frame_idx)];

  // ── Target → world-aligned axes (base_frame_idx < 0 → already world) ──
  Eigen::Vector3d p_des = target.position;
  Eigen::Vector3d a_des = target.axis;
  Eigen::Vector3d v_ff = target.linear_velocity_ff;
  Eigen::Vector3d w_ff = target.angular_velocity_ff;
  if (base_frame_idx >= 0) {
    const pinocchio::SE3& oMb = cache.registered_frames[static_cast<size_t>(base_frame_idx)].oMf;
    p_des = oMb.act(target.position);
    a_des.noalias() = oMb.rotation() * target.axis;
    v_ff.noalias() = oMb.rotation() * target.linear_velocity_ff;
    w_ff.noalias() = oMb.rotation() * target.angular_velocity_ff;
  }

  // ── Errors: position (world) and approach axis (rotation vector, world) ──
  const Eigen::Matrix3d& R_wc = rf.oMf.rotation();
  const Eigen::Vector3d z_c = R_wc.col(2);
  const se3::AxisAlignErrorResult axis = se3::AxisAlignError(z_c, a_des);
  axis_region_ = axis.region;
  if (!axis.IsValid()) {
    return false;  // non-unit / non-finite axis target
  }
  const Eigen::Vector3d e_p = p_des - rf.oMf.translation();
  position_error_norm_ = e_p.norm();
  axis_error_angle_ = axis.error.norm();
  tcp_error_norm_ = std::sqrt(position_error_norm_ * position_error_norm_ +
                              axis_error_angle_ * axis_error_angle_);

  // Task references. The axis rows live in the frame's LOCAL x, y (S·R_WCᵀ);
  // e_a ⟂ z_C, so dropping LOCAL z loses nothing (L5 §4.2).
  const Eigen::Vector3d r_p = kx_.head<3>().cwiseProduct(e_p) + v_ff;
  const Eigen::Vector3d w_ref_local = R_wc.transpose() * (k_axis_ * axis.error + w_ff);

  // ── Jacobian rows on the arm columns (hand columns 0) ──
  const int N = nv_;
  j_pos_.setZero();
  j_axis_.setZero();
  for (int c = 0; c < n_arm_; ++c) {
    const auto vi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    j_pos_.col(vi) = rf.J.col(vi).head<3>();
    const Eigen::Vector3d w_local = R_wc.transpose() * rf.J.col(vi).tail<3>();
    j_axis_.col(vi) = w_local.head<2>();
    j_arm_.col(c) = rf.J.col(vi);  // manipulability diagnostic
  }

  ComputePostureReferences(cache, q_posture_des);

  auto H = qp_data_.H.topLeftCorner(N, N);
  auto g = qp_data_.g.head(N);
  H.setZero();
  H.noalias() += w_task_ * j_pos_.transpose() * j_pos_;
  H.noalias() += w_axis_ * j_axis_.transpose() * j_axis_;
  g.noalias() = -w_task_ * (j_pos_.transpose() * r_p);
  g.noalias() -= w_axis_ * (j_axis_.transpose() * w_ref_local.head<2>());
  AddPostureAndDamping();

  AssembleBox(cache.q, dt);
  if (!AssembleAccelRows(cache, rf, dt, true)) {
    last_solve_.non_finite = true;  // a non-finite M, h, v or drift
    return Fail(cache);
  }
  return SolveAndIntegrate(cache, dt, reseed_anchor);
}

bool ClikReferenceGenerator::PreconditionsHold(const PinocchioCache& cache, int tcp_frame_idx,
                                               int base_frame_idx,
                                               const Eigen::VectorXd& q_posture_des,
                                               double dt) const noexcept {
  // nq == nv (reduced revolute/prismatic tree) is required so velocity indices
  // address q directly and q_ref = q + v·dt is valid.
  const int n_registered = static_cast<int>(cache.registered_frames.size());
  if (nv_ == 0 || tcp_frame_idx < 0 || tcp_frame_idx >= n_registered ||
      base_frame_idx >= n_registered) {
    return false;
  }
  return cache.q.size() == nv_ && cache.v.size() == nv_ && q_posture_des.size() == nv_ && dt > 0.0;
}

bool ClikReferenceGenerator::CommandStateMatches(const PinocchioCache& cache) const noexcept {
  // Armed by the first success, disarmed only by Init / ResetAnchor — NOT by a
  // failed call: a failure leaves q_ref = cache.q (the command state the
  // caller passed), so the check stays meaningful, and a measured q wired in
  // right after a failure must not slip through.
  if (!evaluate_at_command_ || !command_check_armed_) {
    return true;
  }
  constexpr double kTol = 1e-12;
  for (const int vi : arm_v_idx_) {
    const auto i = static_cast<Eigen::Index>(vi);
    if (!(std::abs(cache.q(i) - q_ref_(i)) <= kTol)) {  // NaN → mismatch
      return false;
    }
  }
  return true;
}

void ClikReferenceGenerator::ComputePostureReferences(
    const PinocchioCache& cache, const Eigen::VectorXd& q_posture_des) noexcept {
  // ── L2/L3 posture velocity references v_p = K·(q_des − q) ──
  for (int c = 0; c < n_arm_; ++c) {
    const auto qi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    v_post_arm_(c) = ka_ * (q_posture_des(qi) - cache.q(qi));
  }
  for (int c = 0; c < n_hand_; ++c) {
    const auto qi = static_cast<Eigen::Index>(hand_v_idx_[static_cast<size_t>(c)]);
    v_post_hand_(c) = kh_ * (q_posture_des(qi) - cache.q(qi));
  }
}

void ClikReferenceGenerator::AddPostureAndDamping() noexcept {
  // Adds the L2/L3 posture and μ² terms on top of the task terms the caller has
  // already written into H and g — the same accumulation order the single-task
  // path always had, so its output stays bit-identical.
  const int N = nv_;
  auto H = qp_data_.H.topLeftCorner(N, N);
  auto g = qp_data_.g.head(N);
  for (int c = 0; c < n_arm_; ++c) {
    const auto vi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    H(vi, vi) += w_arm_;
  }
  for (int c = 0; c < n_hand_; ++c) {
    const auto vi = static_cast<Eigen::Index>(hand_v_idx_[static_cast<size_t>(c)]);
    H(vi, vi) += w_hand_;
  }
  H.diagonal().array() += damping_sq_;

  for (int c = 0; c < n_arm_; ++c) {
    const auto vi = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
    g(vi) -= w_arm_ * v_post_arm_(c);
  }
  for (int c = 0; c < n_hand_; ++c) {
    const auto vi = static_cast<Eigen::Index>(hand_v_idx_[static_cast<size_t>(c)]);
    g(vi) -= w_hand_ * v_post_hand_(c);
  }

  // Smoothing (w_s/2)·‖v − v_prev‖²: skipped entirely when off so the legacy
  // accumulation is untouched.
  if (w_smooth_ > 0.0) {
    H.diagonal().array() += w_smooth_;
    g.noalias() -= w_smooth_ * v_prev_;
  }
}

void ClikReferenceGenerator::AssembleBox(const Eigen::VectorXd& q, double dt) noexcept {
  // ── Box constraints lᵢ ≤ vᵢ ≤ uᵢ (per-joint velocity ∩ position) ──
  const int N = nv_;
  auto l = qp_data_.l.head(N);
  auto u = qp_data_.u.head(N);
  const bool per_joint = (v_limit_per_joint_.size() == N);
  const bool vel_box = per_joint || (v_limit_ > 0.0);
  const bool pos_box = (q_min_.size() == N && q_max_.size() == N);
  const bool accel_box = (a_max_.size() == N);
  const double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < N; ++i) {
    const double v_max = per_joint ? v_limit_per_joint_(i) : v_limit_;
    double lo = vel_box ? -v_max : -inf;
    double hi = vel_box ? v_max : inf;
    if (pos_box) {
      const double q_i = q(i);  // q-index == v-index (nq == nv contract)
      lo = std::max(lo, (q_min_(i) - q_i) / dt);
      hi = std::min(hi, (q_max_(i) - q_i) / dt);
    }
    // NUM guard: an already-limit-violating joint can invert the box (lo > hi).
    // Collapse onto the limit-respecting bound so only motion back toward the
    // feasible set is allowed, then re-apply the velocity box to the collapsed
    // value. The collapse lands on `hi` in BOTH directions, and that is
    // +v_limit below q_min but the raw (q_max − qᵢ)/dt above q_max — unbounded
    // in the violation size and growing as dt shrinks (a joint ε past the
    // envelope would command −ε/dt, and v_ref leaves as the device command
    // velocity). Recovery still runs at the full limit; only its magnitude is
    // bounded, and the two directions become symmetric. With the velocity box
    // off (v_limit ≤ 0) there is no bound to fall back on, so it stays raw.
    if (lo > hi) {
      lo = vel_box ? std::clamp(hi, -v_max, v_max) : hi;
      hi = lo;
    }
    if (accel_box) {
      // Acceleration window around the previous command (L5 §4.3). [lo, hi] is
      // non-empty here (the collapse above guarantees it). If the window misses
      // it, keep the acceleration bound — a one-tick velocity jump on a
      // position interface is what a protective stop reacts to — and report
      // the conflict; the position overshoot it allows is what limit_margin is
      // for, and the supervisor aborts on the flag.
      const double a_dt = a_max_(i) * dt;
      const double w_lo = v_prev_(i) - a_dt;
      const double w_hi = v_prev_(i) + a_dt;
      if (w_lo > hi || w_hi < lo) {
        const double v = std::clamp(std::clamp(v_prev_(i), lo, hi), w_lo, w_hi);
        lo = v;
        hi = v;
        last_solve_.bound_conflict = true;
        last_solve_.conflict_mask |= (std::uint64_t{1} << static_cast<unsigned>(i));
      } else {
        lo = std::max(lo, w_lo);
        hi = std::min(hi, w_hi);
      }
    }
    l(i) = lo;
    u(i) = hi;
  }
}

bool ClikReferenceGenerator::AssembleAccelRows(const PinocchioCache& cache,
                                               const PinocchioCache::RegisteredFrame& rf, double dt,
                                               bool axis_rows) noexcept {
  n_accel_rows_ = 0;
  if (accel_constraint_ == AccelConstraint::kBox) {
    return true;  // the legacy QP: no rows, nothing touched
  }
  const int N = nv_;
  const int R = n_accel_rows_max_;
  const double inv_dt = 1.0 / dt;
  auto C = qp_data_.C.middleRows(N, R);
  auto l = qp_data_.l.segment(N, R);
  auto u = qp_data_.u.segment(N, R);

  if (accel_constraint_ == AccelConstraint::kDynamic) {
    // τ_i = Σ_{j∈arm} M_ij·(v_j − v_c,j)/dt + h_i,  |τ_i| ≤ b_i = η·τ_max,i.
    // The hand columns are left out: the hand is locked or commanded
    // elsewhere, so (v_hand − v_c,hand)/dt is not its acceleration; its
    // velocity still reaches the row through h(q, v_c).
    if (cache.M.rows() != N || cache.M.cols() != N || cache.h.size() != N || cache.v.size() != N) {
      return false;
    }
    for (int c = 0; c < n_arm_; ++c) {
      const auto i = static_cast<Eigen::Index>(arm_v_idx_[static_cast<size_t>(c)]);
      C.row(c).setZero();
      double m_vc = 0.0;
      for (const int vj : arm_v_idx_) {
        const auto j = static_cast<Eigen::Index>(vj);
        C(c, j) = cache.M(i, j) * inv_dt;
        m_vc += cache.M(i, j) * cache.v(j);
      }
      const double shift = m_vc * inv_dt - cache.h(i);
      l(c) = -tau_bound_(i) + shift;
      u(c) = tau_bound_(i) + shift;
    }
    n_accel_rows_ = n_arm_;
  } else {
    // ẍ = J·(v − v_c)/dt + J̇·v_c on the rows the cost tracks, in the axes the
    // cost uses: world-aligned for the SE3 rows and the position rows, the
    // frame's LOCAL x, y for the approach-axis rows (d/dt(Rᵀω) = Rᵀω̇, so the
    // local drift is Rᵀ times the world one). J has zero hand columns, so
    // J·v_c is the arm's part of v_c.
    if (cache.v.size() != N) {
      return false;
    }
    Eigen::Matrix<double, 6, 1> drift;
    int rows = 6;
    if (axis_rows) {
      C.topRows<3>() = j_pos_ * inv_dt;
      C.middleRows<2>(3) = j_axis_ * inv_dt;
      drift.head<3>() = rf.dJv.head<3>();
      const Eigen::Vector3d w_local = rf.oMf.rotation().transpose() * rf.dJv.tail<3>();
      drift.segment<2>(3) = w_local.head<2>();
      drift(5) = 0.0;
      rows = 5;
    } else {
      C.topRows<6>() = j_task_ * inv_dt;
      drift = rf.dJv;
    }
    for (int r = 0; r < rows; ++r) {
      const double bound = (r < 3) ? task_a_lin_ : task_a_ang_;
      const double shift = C.row(r).dot(cache.v) - drift(r);
      l(r) = -bound + shift;
      u(r) = bound + shift;
    }
    // A row the overload does not use stays inert (the other overload may
    // have written it on an earlier call).
    const double inf = std::numeric_limits<double>::infinity();
    for (int r = rows; r < R; ++r) {
      C.row(r).setZero();
      l(r) = -inf;
      u(r) = inf;
    }
    n_accel_rows_ = rows;
  }
  // Unit-norm rows (file header). A zero row (a joint that moves nothing) is
  // left as is: its bounds already say whether 0 is admissible.
  for (int r = 0; r < n_accel_rows_; ++r) {
    const double norm = C.row(r).norm();
    if (norm > 1e-12) {
      const double s = 1.0 / norm;
      C.row(r) *= s;
      l(r) *= s;
      u(r) *= s;
    }
  }
  last_solve_.accel_rows = n_accel_rows_;
  // A non-finite M, h, v or drift would reach the solver as a NaN row; refuse
  // it before the solve, like a non-finite feed-forward.
  return C.topRows(n_accel_rows_).allFinite() && l.head(n_accel_rows_).allFinite() &&
         u.head(n_accel_rows_).allFinite();
}

bool ClikReferenceGenerator::Fail(const PinocchioCache& cache) noexcept {
  q_ref_ = cache.q;
  v_ref_.setZero();
  anchor_initialized_ = false;  // force a measured re-anchor on recovery
  v_prev_.setZero();            // the command this tick was v_ref = 0
  if (n_accel_rows_max_ > 0) {
    // With coupling rows a failed solve's duals are no start for the next
    // tick: warm-started from them ProxQP kept reporting PRIMAL_INFEASIBLE
    // after one bad tick. kBox keeps the legacy warm start (golden).
    qp_solver_.ResetWarmStart();
  }
  return false;
}

bool ClikReferenceGenerator::AccelRowsHold(const Eigen::VectorXd& v) noexcept {
  // Rows are unit-norm, so the tolerance is in velocity units: the solver's
  // eps_abs plus a relative slack for a large shift.
  constexpr double kAbs = 1e-6;
  constexpr double kRel = 1e-6;
  const int N = nv_;
  int binding = 0;
  bool hold = true;
  for (int k = 0; k < n_accel_rows_; ++k) {
    const double r = qp_data_.C.row(N + k).dot(v);
    const double lo = qp_data_.l(N + k);
    const double hi = qp_data_.u(N + k);
    const double tol = kAbs + kRel * std::max(std::abs(lo), std::abs(hi));
    if (!(r >= lo - tol) || !(r <= hi + tol)) {  // NaN → broken
      hold = false;
    } else if (r <= lo + tol || r >= hi - tol) {
      ++binding;
    }
  }
  last_solve_.accel_rows_binding = binding;
  return hold;
}

bool ClikReferenceGenerator::SolveAndIntegrate(const PinocchioCache& cache, double dt,
                                               bool reseed_anchor) noexcept {
  const int N = nv_;
  // ── Solve (C = Iₙ already set in Init) ──
  const auto& res = qp_solver_.Solve(qp_data_);
  last_solve_.reached_solve = true;
  last_solve_.converged = res.converged;
  last_solve_.non_finite = res.non_finite;
  last_solve_.status = res.status;
  last_solve_.iterations = res.iterations;
  last_solve_.solve_time_us = res.solve_time_us;
  if (!res.converged) {
    return Fail(cache);
  }
  v_ref_ = res.x_opt.head(N);
  if (n_accel_rows_ > 0 && !AccelRowsHold(v_ref_)) {
    // Converged but off a row: the rows and the velocity ∩ position box could
    // not hold together. A command that breaks the constraint is not returned.
    last_solve_.accel_rows_violated = true;
    last_solve_.converged = false;
    return Fail(cache);
  }

  // ── Damped manipulability √det(J_a·J_aᵀ + μ²·I) — diag continuity only
  //    (the solve no longer forms J♯; this 6×6 is purely diagnostic). ──
  m6_.noalias() = j_arm_ * j_arm_.transpose();
  m6_.diagonal().array() += damping_sq_;
  ldlt6_.compute(m6_);
  const double det = ldlt6_.vectorD().prod();
  manipulability_ = (det > 0.0) ? std::sqrt(det) : 0.0;

  // ── One-step target: q_ref = q_anchor + v_ref·dt ──
  // Anchor at measured on a reseed (or the first / post-failure call); else
  // carry forward from the previous q_ref so the desired integrates open-loop
  // between goal/phase edges (DemoTaskController's desired_q_ pattern). Only the
  // anchor switches — v_ref above is a measured-based closed-loop correction
  // either way.
  if (evaluate_at_command_ || reseed_anchor || !anchor_initialized_) {
    q_ref_ = cache.q;  // measured (or, in command mode, the command state q_c)
  }  // else q_ref_ holds the previous desired — carry forward in place.
  q_ref_.noalias() += v_ref_ * dt;

  // Anti-windup: bound how far the carry-forward desired may lead/lag measured.
  if (anchor_drift_max_ > 0.0) {
    q_ref_ = q_ref_.array()
                 .max(cache.q.array() - anchor_drift_max_)
                 .min(cache.q.array() + anchor_drift_max_)
                 .matrix();
  }

  if (!q_ref_.allFinite() || !v_ref_.allFinite()) {
    last_solve_.non_finite = true;
    // Safe outputs even if the caller ignores the return value.
    return Fail(cache);
  }
  anchor_initialized_ = true;
  command_check_armed_ = true;
  v_prev_ = v_ref_;
  return true;
}

}  // namespace rtc::tsid
