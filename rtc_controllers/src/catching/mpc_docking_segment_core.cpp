#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"

#include "rtc_controllers/catching/mpc_block_grid.hpp"
#include "rtc_controllers/catching/mpc_segment_core_torque.hpp"
#include "rtc_controllers/gain_floor.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <span>

namespace rtc::catching {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// Entry-box comparison slack [rad] — rounding only.
constexpr double kBoxRoundingSlack = 1e-12;
// ‖d_line‖ must be 1 to this tolerance; P_⊥ = I − d dᵀ is a projector only then.
constexpr double kUnitTolerance = 1e-6;
// A covariance counts as PSD when its smallest eigenvalue is above
// −kPsdRelTolerance · trace: an estimator's covariance is PSD only to rounding.
constexpr double kPsdRelTolerance = 1e-9;
// Approach-window comparison slack [s] — node instants are products of
// doubles, so a node exactly at the window's edge must not fall out by an ulp.
constexpr double kWindowSlack = 1e-12;

[[nodiscard]] bool FinitePositive(double x) noexcept {
  return std::isfinite(x) && x > 0.0;
}

[[nodiscard]] double MicrosSince(std::chrono::steady_clock::time_point t0) noexcept {
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

[[nodiscard]] double Positive(double x) noexcept {
  return x > 0.0 ? x : 0.0;
}

}  // namespace

const char* MpcDockingReasonName(MpcDockingReason reason) noexcept {
  switch (reason) {
    case MpcDockingReason::kNone:
      return "none";
    case MpcDockingReason::kNotInitialized:
      return "not_initialized";
    case MpcDockingReason::kParamsInvalid:
      return "params_invalid";
    case MpcDockingReason::kBlocksTooFew:
      return "blocks_too_few";
    case MpcDockingReason::kBlocksAcrossCatch:
      return "blocks_across_catch";
    case MpcDockingReason::kLimitsInvalid:
      return "limits_invalid";
    case MpcDockingReason::kModelUnsupported:
      return "model_unsupported";
    case MpcDockingReason::kFrameUnknown:
      return "frame_unknown";
    case MpcDockingReason::kTerminalRankDeficient:
      return "terminal_rank_deficient";
    case MpcDockingReason::kTimingWindowInvalid:
      return "timing_window_invalid";
    case MpcDockingReason::kDimMismatch:
      return "dim_mismatch";
    case MpcDockingReason::kNonFinite:
      return "non_finite";
    case MpcDockingReason::kInputOutOfRange:
      return "input_out_of_range";
    case MpcDockingReason::kInitialStateOutsideBox:
      return "initial_state_outside_box";
    case MpcDockingReason::kBallInvalid:
      return "ball_invalid";
    case MpcDockingReason::kCovarianceInvalid:
      return "covariance_invalid";
    case MpcDockingReason::kTargetRequired:
      return "target_required";
    case MpcDockingReason::kLinearInfeasible:
      return "linear_infeasible";
    case MpcDockingReason::kConverged:
      return "converged";
    case MpcDockingReason::kIterationLimit:
      return "iteration_limit";
    case MpcDockingReason::kDeadline:
      return "deadline";
    case MpcDockingReason::kInfeasible:
      return "infeasible";
    case MpcDockingReason::kLineSearchFailed:
      return "line_search_failed";
    case MpcDockingReason::kQpFailed:
      return "qp_failed";
    case MpcDockingReason::kSolutionNonFinite:
      return "solution_non_finite";
  }
  return "unknown";
}

const char* DockingRowGroupName(DockingRowGroup group) noexcept {
  switch (group) {
    case DockingRowGroup::kTorque:
      return "torque";
    case DockingRowGroup::kGap:
      return "gap";
    case DockingRowGroup::kEntrance:
      return "entrance";
    case DockingRowGroup::kLateral:
      return "lateral";
    case DockingRowGroup::kTiming:
      return "timing";
    case DockingRowGroup::kVelocitySet:
      return "velocity_set";
    case DockingRowGroup::kImpact:
      return "impact";
    case DockingRowGroup::kBox:
      return "box";
    case DockingRowGroup::kTerminal:
      return "terminal";
  }
  return "unknown";
}

// ── Init (non-RT) ────────────────────────────────────────────────────────────

MpcDockingReason MpcDockingSegmentCore::Init(const pinocchio::Model& arm,
                                             pinocchio::FrameIndex catch_frame,
                                             const MpcDockingSegmentCoreParams& params,
                                             const MpcDockingSegmentCoreLimits& limits,
                                             ClockFn clock) {
  initialized_ = false;
  solver_warm_ = false;

  if (arm.nv < 1 || arm.nv > kMaxPlanNv || arm.nq != arm.nv) {
    return MpcDockingReason::kModelUnsupported;
  }
  if (catch_frame >= arm.frames.size()) {
    return MpcDockingReason::kFrameUnknown;
  }
  const int n = arm.nv;
  const auto nn = static_cast<Eigen::Index>(n);
  const MpcDockingSegmentCoreParams& p = params;

  // ── Grid ──
  if (p.n_pre < 1 || p.n_stop < 3 || p.n_stop > kMaxSegmentNodes ||
      p.n_pre > kMaxMpcNodes - p.n_stop || !FinitePositive(p.dt_pre) ||
      !FinitePositive(p.dt_stop) || !FinitePositive(p.u_scale)) {
    return MpcDockingReason::kParamsInvalid;
  }
  const int n_nodes = p.n_pre + p.n_stop;
  // The block partition: none may span the catch node, three after it
  // (mpc_block_grid.hpp — the rule MpcSegmentCore shares).
  switch (CheckBlockGrid(p.n_blocks, p.block_sizes, p.n_pre, n_nodes)) {
    case BlockGridCheck::kOk:
      break;
    case BlockGridCheck::kInvalid:
      return MpcDockingReason::kParamsInvalid;
    case BlockGridCheck::kTooFew:
      return MpcDockingReason::kBlocksTooFew;
    case BlockGridCheck::kAcrossCatch:
      return MpcDockingReason::kBlocksAcrossCatch;
  }

  // ── Weights ──
  const auto optional_nonneg = [nn](const Eigen::VectorXd& x) {
    return x.size() == 0 || (x.size() == nn && x.allFinite() && x.minCoeff() >= 0.0);
  };
  const auto optional_positive = [nn](const Eigen::VectorXd& x) {
    return x.size() == 0 || (x.size() == nn && x.allFinite() && x.minCoeff() > 0.0);
  };
  if (!optional_nonneg(p.r_tau) || !optional_nonneg(p.r_acc) || !optional_positive(p.r_jerk) ||
      !optional_positive(p.r_jerk_stop) || !optional_nonneg(p.w_q_nom) ||
      !optional_positive(p.manip_d_q)) {
    return MpcDockingReason::kParamsInvalid;
  }
  if (p.w_q_nom.size() != 0 && (p.q_nom.size() != nn || !p.q_nom.allFinite())) {
    return MpcDockingReason::kParamsInvalid;
  }
  if (!rtc::IsFiniteNonNegative(p.w_manip) || !FinitePositive(p.manip_d_lin) ||
      !FinitePositive(p.manip_d_ang) || !FinitePositive(p.manip_delta)) {
    return MpcDockingReason::kParamsInvalid;
  }
  const auto nonneg3 = [](const Eigen::Vector3d& x) {
    return x.allFinite() && x.minCoeff() >= 0.0;
  };
  if (!nonneg3(p.q_p) || !nonneg3(p.q_v) || !nonneg3(p.q_nu_f) || !p.q_rho_f.allFinite() ||
      p.q_rho_f.minCoeff() < 0.0 || !FinitePositive(p.sigma_T) || !p.rho_ref.allFinite() ||
      !p.nu_ref.allFinite() || !(-p.nu_ref.z() > 0.0) || !rtc::IsFiniteNonNegative(p.w_impact) ||
      !FinitePositive(p.e_ref)) {
    return MpcDockingReason::kParamsInvalid;
  }

  // ── Approach and capture set ──
  if (!rtc::IsFiniteNonNegative(p.approach_window) || !std::isfinite(p.s_ent) ||
      !FinitePositive(p.r_ent) || !rtc::IsFiniteNonNegative(p.tan_theta) ||
      !rtc::IsFiniteNonNegative(p.c_ent_max) || !rtc::IsFiniteNonNegative(p.a_brake) ||
      !rtc::IsFiniteNonNegative(p.lambda1_c) || !rtc::IsFiniteNonNegative(p.lambda2_c) ||
      !rtc::IsFiniteNonNegative(p.lambda1_v) || !rtc::IsFiniteNonNegative(p.lambda2_v) ||
      // A slack that costs nothing switches its row off without saying so.
      !(p.lambda1_c + p.lambda2_c > 0.0) || !(p.lambda1_v + p.lambda2_v > 0.0)) {
    return MpcDockingReason::kParamsInvalid;
  }
  if (p.n_faces < 0 || p.n_faces > kMaxDockingFaces || !FinitePositive(p.c_min) ||
      !std::isfinite(p.c_cap_max) || !(p.c_cap_max > p.c_min) || !FinitePositive(p.v_perp_max) ||
      p.speed_faces < 3 || p.speed_faces > kMaxDockingSpeedFaces ||
      // ε_σ enters squared, and σ_T too: the square must not underflow to 0.
      !FinitePositive(p.eps_sigma) || !(p.eps_sigma * p.eps_sigma > 0.0) ||
      !(p.sigma_T * p.sigma_T > 0.0) || !std::isfinite(1.0 / p.m_ball)) {
    return MpcDockingReason::kParamsInvalid;
  }
  // Risks → κ. A risk above ½ would give a NEGATIVE κ, i.e. loosen the row.
  const auto risk_ok = [](double eps) { return std::isfinite(eps) && eps > 0.0 && eps <= 0.5; };
  std::array<double, kMaxDockingFaces> kappa_face{};
  for (int i = 0; i < p.n_faces; ++i) {
    const auto k = static_cast<std::size_t>(i);
    // Unit normals: the row adds ε_σ² to aᵀΣ_ρa under one root, and its
    // violation is reported in metres — both only mean that for ‖a‖ = 1.
    if (!p.face_a[k].allFinite() || !(std::abs(p.face_a[k].norm() - 1.0) <= kUnitTolerance) ||
        !std::isfinite(p.face_b[k]) || !risk_ok(p.face_eps[k]) ||
        !NormalQuantile(1.0 - p.face_eps[k], kappa_face[k])) {
      return MpcDockingReason::kParamsInvalid;
    }
  }
  double kappa_nu = 0.0;
  if (!risk_ok(p.eps_nu) || !NormalQuantile(1.0 - p.eps_nu, kappa_nu)) {
    return MpcDockingReason::kParamsInvalid;
  }
  const bool timing_on = p.chance && p.timing_row;
  double sigma_max = 0.0;
  double k_timing = 0.0;
  if (timing_on) {
    if (!rtc::IsFiniteNonNegative(p.sigma_tau) ||
        !DockingTimingSigmaMax(p.delta_lo, p.delta_hi, p.delta_0, p.eps_t, sigma_max) ||
        !(sigma_max > p.sigma_tau)) {
      return MpcDockingReason::kTimingWindowInvalid;
    }
    k_timing = std::sqrt(sigma_max * sigma_max - p.sigma_tau * p.sigma_tau);
    if (!FinitePositive(k_timing)) {
      return MpcDockingReason::kTimingWindowInvalid;  // σ_max within rounding of σ_τ
    }
  }

  // ── Impact ──
  if (!p.contact_point_hand.allFinite() || !FinitePositive(p.m_ball) ||
      !std::isfinite(p.restitution) || p.restitution < 0.0 || p.restitution > 1.0 ||
      std::isnan(p.e_max) || !(p.e_max > 0.0) || std::isnan(p.p_max) || !(p.p_max > 0.0) ||
      !rtc::IsFiniteNonNegative(p.w_perp)) {
    return MpcDockingReason::kParamsInvalid;
  }

  // ── SQP ──
  if (p.max_iterations < 1 || !(p.armijo_eta > 0.0) || !(p.armijo_eta < 1.0) ||
      !(p.backtrack_beta > 0.0) || !(p.backtrack_beta < 1.0) || p.max_backtracks < 0 ||
      std::isnan(p.delta_tr) || !(p.delta_tr > 0.0) || !std::isfinite(p.mu_growth) ||
      !(p.mu_growth > 1.0) || !FinitePositive(p.mu_max) || !FinitePositive(p.tol_violation) ||
      !(p.mu_min_gain > 0.0) || !(p.mu_min_gain < 1.0) || p.stall_window < 1 ||
      p.stall_window > kDockingStallHistory || !(p.stall_reduction > 0.0) ||
      !(p.stall_reduction < 1.0) || !FinitePositive(p.tol_kkt) ||
      !FinitePositive(p.tol_complementarity) || !FinitePositive(p.tol_linear) ||
      !FinitePositive(p.init_w_q) || !rtc::IsFiniteNonNegative(p.init_w_v) ||
      !FinitePositive(p.init_pinv_damping) || !FinitePositive(p.solver.eps_abs) ||
      !rtc::IsFiniteNonNegative(p.solver.eps_rel) ||
      !rtc::IsFiniteNonNegative(p.solver.eps_primal_inf) || p.solver.max_iter < 1 ||
      p.solver.max_iter_in < 1) {
    return MpcDockingReason::kParamsInvalid;
  }
  for (const double mu : p.mu_init) {
    if (!FinitePositive(mu) || mu > p.mu_max) {
      return MpcDockingReason::kParamsInvalid;
    }
  }

  // ── Limits (finite first — max/min below would launder a NaN, NUM-7) ──
  const auto sized = [nn](const Eigen::VectorXd& x) { return x.size() == nn && x.allFinite(); };
  if (!sized(limits.q_min) || !sized(limits.q_max) || !sized(limits.qd_max) ||
      !sized(limits.tau_max) || !sized(limits.armature) ||
      (p.accel_box && !sized(limits.qdd_max)) || (p.jerk_box && !sized(limits.jerk_max))) {
    return MpcDockingReason::kLimitsInvalid;
  }
  const bool tau_bounds = limits.tau_lo.size() != 0 || limits.tau_hi.size() != 0;
  if (tau_bounds && (!sized(limits.tau_lo) || !sized(limits.tau_hi))) {
    return MpcDockingReason::kLimitsInvalid;
  }
  for (Eigen::Index j = 0; j < nn; ++j) {
    if (!(limits.q_min[j] <= limits.q_max[j]) || !(limits.qd_max[j] > 0.0) ||
        !(limits.tau_max[j] > 0.0) || !std::isfinite(1.0 / limits.tau_max[j]) ||
        !(limits.armature[j] >= 0.0) || (p.accel_box && !(limits.qdd_max[j] > 0.0)) ||
        (p.jerk_box && !(limits.jerk_max[j] > 0.0)) ||
        (tau_bounds && !(limits.tau_lo[j] < limits.tau_hi[j]))) {
      return MpcDockingReason::kLimitsInvalid;
    }
  }

  // ── Commit ──
  params_ = params;
  clock_ = clock;
  n_ = n;
  n_nodes_ = n_nodes;
  n_pre_ = p.n_pre;
  n_blocks_ = p.n_blocks;
  frame_ = catch_frame;
  kappa_face_ = kappa_face;
  kappa_nu_ = kappa_nu;
  sigma_max_ = sigma_max;
  k_timing_ = k_timing;
  timing_on_ = timing_on;
  torque_cost_ = p.r_tau.size() != 0 && p.r_tau.maxCoeff() > 0.0;
  acc_cost_ = p.r_acc.size() != 0 && p.r_acc.maxCoeff() > 0.0;
  posture_cost_ = p.w_q_nom.size() != 0 && p.w_q_nom.maxCoeff() > 0.0;
  near_cost_ = p.q_p.maxCoeff() > 0.0 || p.q_v.maxCoeff() > 0.0;
  manip_on_ = p.w_manip > 0.0;
  impact_rows_ = std::isfinite(p.e_max) || std::isfinite(p.p_max);
  impact_cost_ = p.w_impact > 0.0;
  impact_on_ = impact_rows_ || impact_cost_;
  perp_on_ = p.w_perp > 0.0;

  model_ = arm;
  model_.armature = arm.armature + limits.armature;
  data_ = pinocchio::Data(model_);

  q_lo_ = limits.q_min;
  q_hi_ = limits.q_max;
  v_hi_ = limits.qd_max;
  a_hi_ = p.accel_box ? limits.qdd_max : Eigen::VectorXd::Zero(nn);
  u_hi_ = p.jerk_box ? limits.jerk_max : Eigen::VectorXd::Zero(nn);
  inv_tau_ = limits.tau_max.cwiseInverse();
  tau_lo_ = tau_bounds ? limits.tau_lo : Eigen::VectorXd(-limits.tau_max);
  tau_hi_ = tau_bounds ? limits.tau_hi : limits.tau_max;
  r_tau_ = torque_cost_ ? p.r_tau : Eigen::VectorXd::Zero(nn);
  r_acc_ = acc_cost_ ? p.r_acc : Eigen::VectorXd::Zero(nn);
  r_jerk_ = p.r_jerk.size() == 0 ? Eigen::VectorXd::Ones(nn) : p.r_jerk;
  r_jerk_stop_ = p.r_jerk_stop.size() == 0 ? r_jerk_ : p.r_jerk_stop;
  w_q_nom_ = posture_cost_ ? p.w_q_nom : Eigen::VectorXd::Zero(nn);
  q_nom_ = posture_cost_ ? p.q_nom : Eigen::VectorXd::Zero(nn);
  manip_d_q_ = p.manip_d_q.size() == 0 ? Eigen::VectorXd::Ones(nn) : p.manip_d_q;
  r_tau2_ = (2.0 * p.dt_pre) * r_tau_;

  // Grid. Node instants are k·Δ within each part, never a running sum.
  for (int k = 0; k <= n_nodes_; ++k) {
    const auto i = static_cast<std::size_t>(k);
    t_node_[i] = k <= n_pre_ ? static_cast<double>(k) * p.dt_pre
                             : static_cast<double>(n_pre_) * p.dt_pre +
                                   static_cast<double>(k - n_pre_) * p.dt_stop;
    if (k < n_nodes_) {
      dt_node_[i] = k < n_pre_ ? p.dt_pre : p.dt_stop;
    }
  }
  {
    int k = 0;
    for (int b = 0; b < n_blocks_; ++b) {
      const int size = p.block_sizes[static_cast<std::size_t>(b)];
      block_first_[static_cast<std::size_t>(b)] = k;
      // A block lies on one side of the catch node, so Σ_{k∈b} Δ_k is its size
      // times that side's spacing.
      block_span_[static_cast<std::size_t>(b)] =
          static_cast<double>(size) * (k < n_pre_ ? p.dt_pre : p.dt_stop);
      k += size;
    }
  }
  FillBlockOfInterval(n_blocks_, p.block_sizes, block_of_node_);
  const double t_c = t_node_[static_cast<std::size_t>(n_pre_)];
  n_app_ = 0;
  for (int k = 1; k < n_pre_; ++k) {
    if (t_c - t_node_[static_cast<std::size_t>(k)] <= p.approach_window + kWindowSlack) {
      app_node_[static_cast<std::size_t>(n_app_++)] = k;
    }
  }
  for (int k = 0; k <= n_nodes_; ++k) {
    const double dt = t_node_[static_cast<std::size_t>(k)] - t_c;
    near_weight_[static_cast<std::size_t>(k)] = std::exp(-0.5 * dt * dt / (p.sigma_T * p.sigma_T));
  }
  for (int j = 0; j < p.speed_faces; ++j) {
    const double angle = 2.0 * std::numbers::pi * static_cast<double>(j) / p.speed_faces;
    speed_face_[static_cast<std::size_t>(j)] = Eigen::Vector2d(std::cos(angle), std::sin(angle));
  }
  speed_face_bound_ = p.v_perp_max * std::cos(std::numbers::pi / p.speed_faces);

  // Stage gains ĝ_{m,k}[b]: the scalar triple integrator's response at node k
  // to jerk u_scale held over block b (all others zero), from rest — the one
  // recursion MpcSegmentCore uses too (mpc_block_grid.hpp).
  const Eigen::Index N1 = n_nodes_ + 1;
  const Eigen::Index nb = n_blocks_;
  const auto n_intervals = static_cast<std::size_t>(n_nodes_);
  ComputeBlockStageGains(std::span<const double>(dt_node_).first(n_intervals),
                         std::span<const int>(block_of_node_).first(n_intervals), n_blocks_,
                         p.u_scale, gq_, gv_, ga_);

  // ── Dimensions ──
  const int nN = n * n_nodes_;
  nu_ = n * n_blocks_;
  n_elastic_ = n_nodes_ + n_app_ + 5;
  o_sc_ = nu_;
  o_sv_ = o_sc_ + n_app_;
  o_e_ = o_sv_ + n_app_;
  nx_ = o_e_ + n_elastic_;
  e_torque_ = 0;
  e_gap_ = n_nodes_;
  e_ent_ = e_gap_ + n_app_;
  e_lat_ = e_ent_ + 1;
  e_tim_ = e_lat_ + 1;
  e_vel_ = e_tim_ + 1;
  e_imp_ = e_vel_ + 1;
  n_eq_ = 2 * n;
  row_q_ = 0;
  row_v_ = row_q_ + nN;
  row_a_ = row_v_ + nN;
  row_u_ = row_a_ + (p.accel_box ? nN : 0);
  row_tau_ = row_u_ + (p.jerk_box ? nu_ : 0);
  row_e_ = row_tau_ + 2 * nN;
  row_s_ = row_e_ + n_elastic_;
  row_app_ = row_s_ + 2 * n_app_;
  row_ent_ = row_app_ + 3 * n_app_;
  row_lat_ = row_ent_ + 2;
  row_tim_ = row_lat_ + p.n_faces;
  row_vel_ = row_tim_ + 1;
  row_imp_ = row_vel_ + 2 + p.speed_faces;
  n_in_ = row_imp_ + 3;
  n_in_init_ = row_tau_;

  // ── Constant Hessian of the jerk variables (no ½ in J ⇒ H = 2·…) ──
  jerk_diag_.setZero(nu_);
  for (Eigen::Index b = 0; b < nb; ++b) {
    const auto bi = static_cast<std::size_t>(b);
    const Eigen::VectorXd& r = block_first_[bi] < n_pre_ ? r_jerk_ : r_jerk_stop_;
    for (Eigen::Index j = 0; j < nn; ++j) {
      jerk_diag_[b * nn + j] = 2.0 * block_span_[bi] * r[j];
    }
  }
  h_const_.setZero(nu_, nu_);
  h_const_.diagonal() = jerk_diag_;
  h_init_all_ = h_const_;
  h_init_catch_ = h_const_;
  const auto add_gain_outer = [nn, nb](Eigen::MatrixXd& h, const Eigen::MatrixXd& g, Eigen::Index k,
                                       const Eigen::VectorXd& w, double scale) {
    for (Eigen::Index b = 0; b < nb; ++b) {
      for (Eigen::Index c = 0; c < nb; ++c) {
        const double gg = scale * (g(k, b) * g(k, c));
        if (gg == 0.0) {
          continue;
        }
        for (Eigen::Index j = 0; j < nn; ++j) {
          h(b * nn + j, c * nn + j) += gg * w[j];
        }
      }
    }
  };
  const Eigen::VectorXd ones = Eigen::VectorXd::Ones(nn);
  for (Eigen::Index k = 1; k < n_pre_; ++k) {
    if (acc_cost_) {
      add_gain_outer(h_const_, ga_, k, r_acc_, 2.0 * p.dt_pre);
    }
    if (posture_cost_) {
      add_gain_outer(h_const_, gq_, k, w_q_nom_, 2.0 * p.dt_pre);
    }
  }
  for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
    add_gain_outer(h_init_all_, gq_, k, ones, 2.0 * p.init_w_q);
    add_gain_outer(h_init_all_, gv_, k, ones, 2.0 * p.init_w_v);
  }
  add_gain_outer(h_init_catch_, gq_, n_pre_, ones, 2.0 * p.init_w_q);
  add_gain_outer(h_init_catch_, gv_, n_pre_, ones, 2.0 * p.init_w_v);

  // ── QP storage and constant parts ──
  qp_.Init(nx_, n_eq_, n_in_);
  qp_.n_vars = nx_;
  qp_.n_eq = n_eq_;
  qp_.n_ineq = n_in_;
  qp_init_.Init(nu_, n_eq_, n_in_init_);
  qp_init_.n_vars = nu_;
  qp_init_.n_eq = n_eq_;
  qp_init_.n_ineq = n_in_init_;

  const Eigen::Index N = n_nodes_;
  for (Eigen::Index b = 0; b < nb; ++b) {
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp_.A(j, b * nn + j) = gv_(N, b);
      qp_.A(nn + j, b * nn + j) = ga_(N, b);
    }
  }
  for (Eigen::Index k = 1; k <= N; ++k) {
    const Eigen::Index o = (k - 1) * nn;
    for (Eigen::Index b = 0; b < nb; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        qp_.C(row_q_ + o + j, b * nn + j) = gq_(k, b);
        qp_.C(row_v_ + o + j, b * nn + j) = gv_(k, b);
        if (p.accel_box) {
          qp_.C(row_a_ + o + j, b * nn + j) = ga_(k, b);
        }
      }
    }
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp_.C(row_tau_ + o + j, o_e_ + e_torque_ + (k - 1)) = -1.0;      // upper: … − e ≤ hi
      qp_.C(row_tau_ + nN + o + j, o_e_ + e_torque_ + (k - 1)) = 1.0;  // lower: … + e ≥ lo
    }
  }
  if (p.jerk_box) {
    for (Eigen::Index i = 0; i < nu_; ++i) {
      qp_.C(row_u_ + i, i) = 1.0;
    }
  }
  for (Eigen::Index i = 0; i < n_elastic_; ++i) {
    qp_.C(row_e_ + i, o_e_ + i) = 1.0;
  }
  for (Eigen::Index i = 0; i < n_app_; ++i) {
    qp_.C(row_s_ + i, o_sc_ + i) = 1.0;
    qp_.C(row_s_ + n_app_ + i, o_sv_ + i) = 1.0;
    qp_.C(row_app_ + 3 * i, o_e_ + e_gap_ + i) = 1.0;  // ∂ℓ d + e ≥ −ℓ̄
    qp_.C(row_app_ + 3 * i + 2, o_sv_ + i) = -1.0;     // envelope: … − s_v ≤ …
    qp_.H(o_sc_ + i, o_sc_ + i) = 2.0 * p.dt_pre * p.lambda2_c;
    qp_.H(o_sv_ + i, o_sv_ + i) = 2.0 * p.dt_pre * p.lambda2_v;
  }
  qp_.C(row_ent_, o_e_ + e_ent_) = -1.0;
  qp_.C(row_ent_ + 1, o_e_ + e_ent_) = 1.0;
  for (Eigen::Index i = 0; i < p.n_faces; ++i) {
    qp_.C(row_lat_ + i, o_e_ + e_lat_) = -1.0;
  }
  qp_.C(row_tim_, o_e_ + e_tim_) = 1.0;
  qp_.C(row_vel_, o_e_ + e_vel_) = 1.0;
  for (Eigen::Index i = 1; i < 2 + p.speed_faces; ++i) {
    qp_.C(row_vel_ + i, o_e_ + e_vel_) = -1.0;
  }
  for (Eigen::Index i = 0; i < 3; ++i) {
    qp_.C(row_imp_ + i, o_e_ + e_imp_) = -1.0;
  }
  // Which penalty each inequality row answers to (−1: none — the linear rows,
  // the sign rows of the slacks and elastics, the two slack rows).
  row_group_.assign(static_cast<std::size_t>(n_in_), -1);
  const auto tag_rows = [this](int first, int count, DockingRowGroup g) {
    for (int i = first; i < first + count; ++i) {
      row_group_[static_cast<std::size_t>(i)] = static_cast<int>(g);
    }
  };
  tag_rows(row_tau_, 2 * nN, DockingRowGroup::kTorque);
  for (int i = 0; i < n_app_; ++i) {
    tag_rows(row_app_ + 3 * i, 1, DockingRowGroup::kGap);
  }
  tag_rows(row_ent_, 2, DockingRowGroup::kEntrance);
  tag_rows(row_lat_, p.n_faces, DockingRowGroup::kLateral);
  tag_rows(row_tim_, 1, DockingRowGroup::kTiming);
  tag_rows(row_vel_, 2 + p.speed_faces, DockingRowGroup::kVelocitySet);
  tag_rows(row_imp_, 3, DockingRowGroup::kImpact);
  // Everything open until a solve sets its bounds.
  qp_.l.setConstant(-kInf);
  qp_.u.setConstant(kInf);
  qp_.H.topLeftCorner(nu_, nu_) = h_const_;

  qp_init_.A = qp_.A.leftCols(nu_);
  qp_init_.C = qp_.C.topLeftCorner(n_in_init_, nu_);
  qp_init_.H = h_init_all_;

  // Rank self-check of the assembled terminal equality (2n × nB).
  {
    const Eigen::MatrixXd a = qp_.A.leftCols(nu_);
    const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
    terminal_rank_ = static_cast<int>(qr.rank());
  }
  if (terminal_rank_ != n_eq_) {
    return MpcDockingReason::kTerminalRankDeficient;
  }

  solver_.Init(nx_, n_eq_, n_in_, p.solver);
  init_solver_.Init(nu_, n_eq_, n_in_init_, p.solver);

  // ── Workspace ──
  x_qp_.setZero(nx_);
  y_qp_.setZero(n_eq_);
  z_qp_.setZero(n_in_);
  x_keep_.setZero(nx_);
  y_keep_.setZero(n_eq_);
  z_keep_.setZero(n_in_);
  cx_.setZero(n_in_);
  kkt_.setZero(nx_);
  q0_.setZero(nn);
  v0_.setZero(nn);
  a0_.setZero(nn);
  qf_.setZero(nn, N1);
  vf_.setZero(nn, N1);
  af_.setZero(nn, N1);
  z_.setZero(nu_);
  z_trial_.setZero(nu_);
  d_.setZero(nu_);
  q_.setZero(nn, N1);
  v_.setZero(nn, N1);
  a_.setZero(nn, N1);
  kin_.Resize(nn);
  for (DockingRelativeState& rel : rel_) {
    rel.Resize(nn);
  }
  tau_.setZero(nn, N1);
  d_tau_.setZero(nn, 3 * nn * N1);
  for (std::size_t i = 0; i < corridor_.size(); ++i) {
    corridor_[i].Resize(nn);
    envelope_[i].Resize(nn);
  }
  for (DockingScalar& s : lateral_) {
    s.Resize(nn);
  }
  for (DockingScalar& s : speed_) {
    s.Resize(nn);
  }
  timing_.Resize(nn);
  axial_lo_.Resize(nn);
  axial_hi_.Resize(nn);
  impact_.Resize(nn);
  impact_work_.Init(model_);
  manip_work_.Init(model_);
  manip_grad_.setZero(nn, N1);
  perp_jac_.setZero(3, nn * N1);
  perp_res_.setZero(3, N1);
  t_node_jac_.setZero(nn, nu_);
  wt_.setZero(nn, nu_);
  row_.setZero(nu_);
  aq_.setZero(nn);
  av_.setZero(nn);
  work_n_.setZero(nn);
  zero_n_.setZero(nn);
  j6_.setZero(6, nn);
  mu_ = p.mu_init;

  initialized_ = true;
  return MpcDockingReason::kNone;
}

void MpcDockingSegmentCore::ResizeResult(MpcDockingSegmentCoreResult& result) const {
  const Eigen::Index nn = n_;
  result.q.setZero(nn, n_nodes_ + 1);
  result.qd.setZero(nn, n_nodes_ + 1);
  result.qdd.setZero(nn, n_nodes_ + 1);
  result.u.setZero(nn, n_nodes_);
  result.feasible = false;
  result.converged = false;
  result.reason = MpcDockingReason::kNotInitialized;
}

void MpcDockingSegmentCore::ResizeInput(MpcDockingSegmentCoreInput& input) const {
  const Eigen::Index nn = n_;
  input.q0.setZero(nn);
  input.qd0.setZero(nn);
  input.qdd0.setZero(nn);
  input.q_init.setZero(nn, n_nodes_ + 1);
  input.qd_init.setZero(nn, n_nodes_ + 1);
  input.qdd_init.setZero(nn, n_nodes_ + 1);
  input.q_catch_target.setZero(nn);
}

double MpcDockingSegmentCore::NodeTime(int k) const noexcept {
  if (!initialized_ || k < 0 || k > n_nodes_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return t_node_[static_cast<std::size_t>(k)];
}

double MpcDockingSegmentCore::StageGain(int m, int k, int b) const noexcept {
  if (!initialized_ || k < 0 || k > n_nodes_ || b < 0 || b >= n_blocks_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  switch (m) {
    case 0:
      return gq_(k, b);
    case 1:
      return gv_(k, b);
    case 2:
      return ga_(k, b);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

int MpcDockingSegmentCore::ApproachNode(int i) const noexcept {
  if (!initialized_ || i < 0 || i >= n_app_) {
    return -1;
  }
  return app_node_[static_cast<std::size_t>(i)];
}

double MpcDockingSegmentCore::FaceKappa(int i) const noexcept {
  if (!initialized_ || i < 0 || i >= params_.n_faces) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return kappa_face_[static_cast<std::size_t>(i)];
}

int MpcDockingSegmentCore::GroupRowBegin(DockingRowGroup group) const noexcept {
  switch (group) {
    case DockingRowGroup::kTorque:
      return row_tau_;
    case DockingRowGroup::kGap:
      return row_app_;
    case DockingRowGroup::kEntrance:
      return row_ent_;
    case DockingRowGroup::kLateral:
      return row_lat_;
    case DockingRowGroup::kTiming:
      return row_tim_;
    case DockingRowGroup::kVelocitySet:
      return row_vel_;
    case DockingRowGroup::kImpact:
      return row_imp_;
    case DockingRowGroup::kBox:
      return row_q_;
    case DockingRowGroup::kTerminal:
      return 0;
  }
  return -1;
}

int MpcDockingSegmentCore::GroupRowCount(DockingRowGroup group) const noexcept {
  switch (group) {
    case DockingRowGroup::kTorque:
      return 2 * n_ * n_nodes_;
    case DockingRowGroup::kGap:
      return 3 * n_app_;  // gap, corridor, envelope per approach node
    case DockingRowGroup::kEntrance:
      return 2;
    case DockingRowGroup::kLateral:
      return params_.n_faces;
    case DockingRowGroup::kTiming:
      return 1;
    case DockingRowGroup::kVelocitySet:
      return 2 + params_.speed_faces;
    case DockingRowGroup::kImpact:
      return 3;
    case DockingRowGroup::kBox:
      return row_tau_;
    case DockingRowGroup::kTerminal:
      return n_eq_;
  }
  return 0;
}

// ── Trajectory from the iterate ──────────────────────────────────────────────

void MpcDockingSegmentCore::FreeResponse() noexcept {
  const Eigen::Index nn = n_;
  for (Eigen::Index k = 0; k <= n_nodes_; ++k) {
    const double t = t_node_[static_cast<std::size_t>(k)];
    for (Eigen::Index j = 0; j < nn; ++j) {
      qf_(j, k) = q0_[j] + t * v0_[j] + 0.5 * t * t * a0_[j];
      vf_(j, k) = v0_[j] + t * a0_[j];
      af_(j, k) = a0_[j];
    }
  }
}

void MpcDockingSegmentCore::TrajectoryFromZ(const Eigen::VectorXd& z) noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index nb = n_blocks_;
  for (Eigen::Index k = 0; k <= n_nodes_; ++k) {
    for (Eigen::Index j = 0; j < nn; ++j) {
      double q = qf_(j, k);
      double v = vf_(j, k);
      double a = af_(j, k);
      for (Eigen::Index b = 0; b < nb; ++b) {
        const double zb = z[b * nn + j];
        q += gq_(k, b) * zb;
        v += gv_(k, b) * zb;
        a += ga_(k, b) * zb;
      }
      q_(j, k) = q;
      v_(j, k) = v;
      a_(j, k) = a;
    }
  }
}

void MpcDockingSegmentCore::ProjectInitial(const MpcDockingSegmentCoreInput& in) noexcept {
  // The block jerk nearest (in the mean) to the caller's nodes: per block, the
  // average of the node-acceleration differences over its intervals. A
  // trajectory this core produced on the same grid is reproduced exactly.
  const Eigen::Index nn = n_;
  for (Eigen::Index b = 0; b < n_blocks_; ++b) {
    const auto bi = static_cast<std::size_t>(b);
    const int first = block_first_[bi];
    const int size = params_.block_sizes[bi];
    for (Eigen::Index j = 0; j < nn; ++j) {
      double sum = 0.0;
      for (int k = first; k < first + size; ++k) {
        sum += (in.qdd_init(j, k + 1) - in.qdd_init(j, k)) / dt_node_[static_cast<std::size_t>(k)];
      }
      z_[b * nn + j] = sum / (static_cast<double>(size) * params_.u_scale);
    }
  }
}

bool MpcDockingSegmentCore::LinearRowsHold() const noexcept {
  const Eigen::Index nn = n_;
  const double tol = params_.tol_linear;
  for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
    for (Eigen::Index j = 0; j < nn; ++j) {
      if (!(q_(j, k) >= q_lo_[j] - tol) || !(q_(j, k) <= q_hi_[j] + tol) ||
          !(std::abs(v_(j, k)) <= v_hi_[j] + tol) ||
          (params_.accel_box && !(std::abs(a_(j, k)) <= a_hi_[j] + tol))) {
        return false;
      }
    }
  }
  if (params_.jerk_box) {
    for (Eigen::Index b = 0; b < n_blocks_; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        if (!(std::abs(params_.u_scale * z_[b * nn + j]) <= u_hi_[j] + tol * params_.u_scale)) {
          return false;
        }
      }
    }
  }
  for (Eigen::Index j = 0; j < nn; ++j) {
    if (!(std::abs(v_(j, n_nodes_)) <= tol) || !(std::abs(a_(j, n_nodes_)) <= tol)) {
      return false;
    }
  }
  return true;
}

MpcDockingReason MpcDockingSegmentCore::RunInitQp(const MpcDockingSegmentCoreInput& in,
                                                  MpcDockingSegmentCoreResult& out) noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index nb = n_blocks_;
  const Eigen::Index N = n_nodes_;
  const double us = params_.u_scale;
  StageBegin(MpcDockingStage::kStart);
  qp_init_.g.setZero();
  if (in.initial_valid) {
    // Nearest to the caller's whole trajectory.
    qp_init_.H = h_init_all_;
    for (Eigen::Index k = 1; k <= N; ++k) {
      for (Eigen::Index b = 0; b <= block_of_node_[static_cast<std::size_t>(k - 1)]; ++b) {
        for (Eigen::Index j = 0; j < nn; ++j) {
          qp_init_.g[b * nn + j] -=
              2.0 * (params_.init_w_q * gq_(k, b) * (in.q_init(j, k) - qf_(j, k)) +
                     params_.init_w_v * gv_(k, b) * (in.qd_init(j, k) - vf_(j, k)));
        }
      }
    }
  } else {
    // Nearest to the catch-node pose with the joint velocity that gives ν_ref
    // there: v_h = v_b − R ν_ref, q̇ = J_pᵀ (J_p J_pᵀ + λ²I)⁻¹ v_h (NUM-1). The
    // target is NOT clipped to the velocity limits — the QP's rows do that.
    qp_init_.H = h_init_catch_;
    const Eigen::Index kc = n_pre_;
    if (!ComputeDockingFrameKinematics(model_, data_, frame_, in.q_catch_target, zero_n_, kin_)) {
      StageEnd(MpcDockingStage::kStart);
      return MpcDockingReason::kDimMismatch;
    }
    const Eigen::Vector3d v_hand = ball_[static_cast<std::size_t>(kc)].v - kin_.R * params_.nu_ref;
    Eigen::Matrix3d jjt =
        (params_.init_pinv_damping * params_.init_pinv_damping) * Eigen::Matrix3d::Identity();
    jjt.noalias() += kin_.j_p * kin_.j_p.transpose();
    const Eigen::LLT<Eigen::Matrix3d> llt(jjt);
    const Eigen::Vector3d lambda = llt.solve(v_hand);
    work_n_.noalias() = kin_.j_p.transpose() * lambda;  // q̇ target
    if (llt.info() != Eigen::Success || !work_n_.allFinite()) {
      StageEnd(MpcDockingStage::kStart);
      return MpcDockingReason::kSolutionNonFinite;
    }
    for (Eigen::Index b = 0; b <= block_of_node_[static_cast<std::size_t>(kc - 1)]; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        qp_init_.g[b * nn + j] -=
            2.0 * (params_.init_w_q * gq_(kc, b) * (in.q_catch_target[j] - qf_(j, kc)) +
                   params_.init_w_v * gv_(kc, b) * (work_n_[j] - vf_(j, kc)));
      }
    }
  }
  for (Eigen::Index j = 0; j < nn; ++j) {
    qp_init_.b[j] = -vf_(j, N);
    qp_init_.b[nn + j] = -af_(j, N);
  }
  for (Eigen::Index k = 1; k <= N; ++k) {
    const Eigen::Index o = (k - 1) * nn;
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp_init_.l[row_q_ + o + j] = q_lo_[j] - qf_(j, k);
      qp_init_.u[row_q_ + o + j] = q_hi_[j] - qf_(j, k);
      qp_init_.l[row_v_ + o + j] = -v_hi_[j] - vf_(j, k);
      qp_init_.u[row_v_ + o + j] = v_hi_[j] - vf_(j, k);
      if (params_.accel_box) {
        qp_init_.l[row_a_ + o + j] = -a_hi_[j] - af_(j, k);
        qp_init_.u[row_a_ + o + j] = a_hi_[j] - af_(j, k);
      }
    }
  }
  if (params_.jerk_box) {
    for (Eigen::Index b = 0; b < nb; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        qp_init_.l[row_u_ + b * nn + j] = -u_hi_[j] / us;
        qp_init_.u[row_u_ + b * nn + j] = u_hi_[j] / us;
      }
    }
  }
  StageEnd(MpcDockingStage::kStart);

  // A new start is a new problem: cold, so the answer does not depend on what
  // this core solved before.
  init_solver_.ResetWarmStart();
  const auto t0 = std::chrono::steady_clock::now();
  const tsid::SolveResult& res = init_solver_.Solve(qp_init_);
  out.qp_us += MicrosSince(t0);
  ++out.qp_solves;
  out.qp_iterations += res.iterations;
  out.qp_status = res.status;
  out.init_qp_used = true;
  if (!res.converged) {
    return res.non_finite ? MpcDockingReason::kSolutionNonFinite
                          : MpcDockingReason::kLinearInfeasible;
  }
  StageBegin(MpcDockingStage::kStart);
  z_ = res.x_opt.head(nu_);
  StageEnd(MpcDockingStage::kStart);
  return MpcDockingReason::kNone;
}

// ── Nonlinear evaluation ─────────────────────────────────────────────────────
// Everything Solve() knows about the nonlinear problem at the trajectory in
// (q_, v_, a_): the cost by term, the violation of every hard row in its
// group's unit, the smallest feasible corridor slacks — and, with
// `with_jacobians`, the derivatives AssembleQp reads. One function for both
// uses, so the merit a step is judged by and the model it was computed from
// cannot drift apart.

bool MpcDockingSegmentCore::EvaluateTrajectory(bool with_jacobians, Evaluation& ev) noexcept {
  const MpcDockingSegmentCoreParams& p = params_;
  const Eigen::Index nn = n_;
  const Eigen::Index N = n_nodes_;
  const Eigen::Index kc = n_pre_;
  const double da = p.dt_pre;
  const double ds = p.dt_stop;
  const double us = p.u_scale;
  ev = Evaluation{};
  const auto note = [&ev](DockingRowGroup g, double viol) noexcept {
    const auto i = static_cast<std::size_t>(g);
    ev.viol_max[i] = std::max(ev.viol_max[i], viol);
  };
  const auto add = [&ev, &note](DockingRowGroup g, double node_viol) noexcept {
    note(g, node_viol);
    ev.viol_sum[static_cast<std::size_t>(g)] += node_viol;
  };
  // std::max and Positive() drop a NaN operand, so a non-finite row value
  // would vanish from the maxima below and read as "no violation" (NUM-7).
  // Every row value is therefore seen here BEFORE it enters one.
  bool rows_finite = q_.allFinite() && v_.allFinite() && a_.allFinite();
  const auto seen = [&rows_finite](double x) noexcept {
    rows_finite = rows_finite && std::isfinite(x);
    return x;
  };

  // ── Torque (rows on nodes 1..N; cost on stages 0..k_c−1) ──
  for (Eigen::Index k = 0; k <= N; ++k) {
    if (k == 0 && !torque_cost_) {
      continue;
    }
    if (with_jacobians && k >= 1) {
      if (!LinearizeTorqueAt(model_, data_, q_.col(k), v_.col(k), a_.col(k), tau_.col(k),
                             d_tau_.middleCols(3 * nn * k, 3 * nn))) {
        return false;
      }
    } else {
      pinocchio::rnea(model_, data_, q_.col(k), v_.col(k), a_.col(k));
      tau_.col(k) = data_.tau;
    }
    if (k >= 1) {
      double node = 0.0;
      for (Eigen::Index j = 0; j < nn; ++j) {
        const double t = seen(tau_(j, k) * inv_tau_[j]);
        node = std::max(node, t - tau_hi_[j] * inv_tau_[j]);
        node = std::max(node, tau_lo_[j] * inv_tau_[j] - t);
        ev.tau_ratio_max = std::max(ev.tau_ratio_max, std::abs(t));
      }
      add(DockingRowGroup::kTorque, node);
    }
    if (k < kc && torque_cost_) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        ev.cost.tau += da * r_tau_[j] * tau_(j, k) * tau_(j, k);
      }
    }
  }

  // ── Joint-space terms and the linear rows ──
  for (Eigen::Index k = 0; k < N; ++k) {
    const bool pre = k < kc;
    const double dt = pre ? da : ds;
    const Eigen::VectorXd& rj = pre ? r_jerk_ : r_jerk_stop_;
    double jerk = 0.0;
    for (Eigen::Index j = 0; j < nn; ++j) {
      const double u = (a_(j, k + 1) - a_(j, k)) / dt;
      const double un = u / us;
      jerk += rj[j] * un * un;
      if (p.jerk_box) {
        // In units of u_s, the scale the QP's jerk rows live in — the solver
        // holds them to its tolerance THERE, which is u_s times larger in u.
        note(DockingRowGroup::kBox, Positive(std::abs(u) - u_hi_[j]) / us);
      }
      if (pre) {
        ev.cost.acc += da * r_acc_[j] * a_(j, k) * a_(j, k);
        const double dq = q_(j, k) - q_nom_[j];
        ev.cost.posture += da * w_q_nom_[j] * dq * dq;
      }
    }
    (pre ? ev.cost.jerk : ev.cost.stop_jerk) += dt * jerk;
  }
  for (Eigen::Index k = 1; k <= N; ++k) {
    for (Eigen::Index j = 0; j < nn; ++j) {
      note(DockingRowGroup::kBox, Positive(q_lo_[j] - q_(j, k)));
      note(DockingRowGroup::kBox, Positive(q_(j, k) - q_hi_[j]));
      note(DockingRowGroup::kBox, Positive(std::abs(v_(j, k)) - v_hi_[j]));
      if (p.accel_box) {
        note(DockingRowGroup::kBox, Positive(std::abs(a_(j, k)) - a_hi_[j]));
      }
    }
  }
  for (Eigen::Index j = 0; j < nn; ++j) {
    note(DockingRowGroup::kTerminal, std::abs(v_(j, N)));
    note(DockingRowGroup::kTerminal, std::abs(a_(j, N)));
  }

  // ── The ball in the capture frame: nodes 0..k_c ──
  const Eigen::Vector3d r_ref(p.rho_ref.x(), p.rho_ref.y(), p.s_ent);
  const double t_c = t_node_[static_cast<std::size_t>(kc)];
  int app = 0;
  for (Eigen::Index k = 0; k <= kc; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    const bool is_app = app < n_app_ && app_node_[static_cast<std::size_t>(app)] == k;
    if (k < kc && !near_cost_ && !is_app) {
      continue;
    }
    DockingRelativeState& rel = rel_[ki];
    if (!ComputeDockingFrameKinematics(model_, data_, frame_, q_.col(k), v_.col(k), kin_) ||
        !ComputeDockingRelativeState(kin_, ball_[ki].p, ball_[ki].v, rel)) {
      return false;
    }
    if (k < kc && near_cost_) {
      // r_ref,k = r_ref + (t_c − t_k)(−ν_ref): the constant-velocity approach
      // line through the terminal reference.
      const Eigen::Vector3d r_ref_k = r_ref - (t_c - t_node_[ki]) * p.nu_ref;
      const Eigen::Vector3d er = rel.r_h - r_ref_k;
      const Eigen::Vector3d ev_nu = rel.nu_h - p.nu_ref;
      ev.cost.near +=
          da * near_weight_[ki] * (p.q_p.dot(er.cwiseAbs2()) + p.q_v.dot(ev_nu.cwiseAbs2()));
    }
    if (is_app) {
      const auto ai = static_cast<std::size_t>(app);
      add(DockingRowGroup::kGap, Positive(-(seen(rel.s) - p.s_ent)));
      (void)seen(rel.c);
      slack_c_[ai] = DockingCorridorMinSlack(rel, p.s_ent, p.r_ent, p.tan_theta);
      slack_v_[ai] = DockingEnvelopeMinSlack(rel, p.s_ent, p.c_ent_max, p.a_brake);
      DockingCorridorRow(rel, p.s_ent, p.r_ent, p.tan_theta, slack_c_[ai], corridor_[ai],
                         corridor_ds_[ai]);
      DockingEnvelopeRow(rel, p.s_ent, p.c_ent_max, p.a_brake, slack_v_[ai], envelope_[ai]);
      ev.cost.slack +=
          da * (p.lambda1_c * slack_c_[ai] + p.lambda2_c * slack_c_[ai] * slack_c_[ai] +
                p.lambda1_v * slack_v_[ai] + p.lambda2_v * slack_v_[ai] * slack_v_[ai]);
      ++app;
    }
    if (k == kc) {
      add(DockingRowGroup::kEntrance, std::abs(seen(rel.s) - p.s_ent));
      (void)seen(rel.c);
      const Eigen::Vector2d e_rho = rel.Rho() - p.rho_ref;
      const Eigen::Vector3d e_nu = rel.nu_h - p.nu_ref;
      ev.cost.terminal = p.q_rho_f.dot(e_rho.cwiseAbs2()) + p.q_nu_f.dot(e_nu.cwiseAbs2());

      double lateral = 0.0;
      for (int i = 0; i < p.n_faces; ++i) {
        const auto fi = static_cast<std::size_t>(i);
        DockingLateralChanceRow(kin_, rel, sigma_p_, p.face_a[fi], kappa_face_[fi], p.c_min,
                                p.eps_sigma, lateral_[fi]);
        lateral = std::max(lateral, seen(lateral_[fi].value) - p.face_b[fi]);
      }
      add(DockingRowGroup::kLateral, Positive(lateral));
      if (timing_on_) {
        DockingTimingRow(kin_, rel, sigma_p_, k_timing_, p.eps_sigma, timing_);
        add(DockingRowGroup::kTiming, Positive(-seen(timing_.value)));
      }
      DockingAxialSpeedRow(kin_, rel, sigma_b_, -kappa_nu_, p.eps_sigma, axial_lo_);
      DockingAxialSpeedRow(kin_, rel, sigma_b_, kappa_nu_, p.eps_sigma, axial_hi_);
      double vel = std::max(p.c_min - seen(axial_lo_.value), seen(axial_hi_.value) - p.c_cap_max);
      for (int i = 0; i < p.speed_faces; ++i) {
        const auto fi = static_cast<std::size_t>(i);
        DockingLateralSpeedRow(kin_, rel, sigma_b_, speed_face_[fi], kappa_nu_, p.eps_sigma,
                               speed_[fi]);
        vel = std::max(vel, seen(speed_[fi].value) - speed_face_bound_);
      }
      add(DockingRowGroup::kVelocitySet, Positive(vel));
      if (impact_on_) {
        if (!ComputeDockingImpact(model_, frame_, q_.col(k), v_.col(k), kin_, ball_[ki].v,
                                  p.contact_point_hand, p.m_ball, p.restitution, impact_work_,
                                  impact_)) {
          return false;
        }
        ev.cost.impact = p.w_impact * impact_.energy.value / p.e_ref;
        if (impact_rows_) {
          double imp = seen(impact_.g_n.value);
          if (std::isfinite(p.e_max)) {
            imp = std::max(imp, seen(impact_.energy.value) / p.e_max - 1.0);
          }
          if (std::isfinite(p.p_max)) {
            imp = std::max(imp, seen(impact_.impulse.value) / p.p_max - 1.0);
          }
          add(DockingRowGroup::kImpact, Positive(imp));
        }
      }
    }
  }

  // ── Manipulability: stages 0..k_c−1 ──
  if (manip_on_) {
    for (Eigen::Index k = 0; k < kc; ++k) {
      double psi = 0.0;
      if (!ComputeDockingManipulability(model_, frame_, q_.col(k), p.manip_d_lin, p.manip_d_ang,
                                        manip_d_q_, p.manip_delta, manip_work_, psi,
                                        manip_grad_.col(k))) {
        return false;
      }
      ev.cost.manip += da * p.w_manip * psi;
    }
  }

  // ── Stop line: nodes k_c..N ──
  if (perp_on_) {
    for (Eigen::Index k = kc; k <= N; ++k) {
      j6_.setZero();
      pinocchio::computeFrameJacobian(model_, data_, q_.col(k), frame_,
                                      pinocchio::LOCAL_WORLD_ALIGNED, j6_);
      const Eigen::Vector3d off = data_.oMf[frame_].translation() - p_line_;
      const Eigen::Vector3d res = p_perp_ * off;
      perp_res_.col(k) = res;
      if (with_jacobians) {
        perp_jac_.middleCols(nn * k, nn).noalias() = p_perp_ * j6_.topRows<3>();
      }
      ev.cost.stop_line += ds * p.w_perp * res.squaredNorm();
    }
  }

  ev.cost.reference = ev.cost.tau + ev.cost.acc + ev.cost.jerk + ev.cost.posture + ev.cost.manip +
                      ev.cost.near + ev.cost.terminal + ev.cost.impact + ev.cost.slack;
  ev.cost.stop = ev.cost.stop_jerk + ev.cost.stop_line;
  ev.cost.total = ev.cost.reference + ev.cost.stop;
  bool finite = rows_finite && std::isfinite(ev.cost.total) && std::isfinite(ev.tau_ratio_max);
  for (const double v : ev.viol_max) {
    finite = finite && std::isfinite(v);
  }
  for (const double v : ev.viol_sum) {
    finite = finite && std::isfinite(v);
  }
  ev.ok = finite;
  return finite;
}

// ── QP assembly ──────────────────────────────────────────────────────────────
// Variables x = [d | s_c | s_v | e]: d the step of ũ/u_s, the slacks and
// elastics as absolute values. A row a_qᵀδq_k + a_vᵀδq̇_k of node k becomes a
// row over d through the stage gains (MapRow).

void MpcDockingSegmentCore::MapRow(Eigen::Index k, const Eigen::VectorXd* a_q,
                                   const Eigen::VectorXd* a_v) noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index last = block_of_node_[static_cast<std::size_t>(k - 1)];
  row_len_ = (last + 1) * nn;
  row_.setZero();
  for (Eigen::Index b = 0; b <= last; ++b) {
    if (a_q != nullptr) {
      row_.segment(b * nn, nn) += gq_(k, b) * (*a_q);
    }
    if (a_v != nullptr) {
      row_.segment(b * nn, nn) += gv_(k, b) * (*a_v);
    }
  }
}

void MpcDockingSegmentCore::AddScalarResidual(double weight, double residual) noexcept {
  // The term weight/2 · (residual + row_ᵀd)², i.e. W·res² with weight = 2W.
  // Written entry-wise as weight·(row_c·row_r): the product commutes exactly,
  // so H stays exactly symmetric.
  if (!(weight > 0.0)) {
    return;
  }
  const Eigen::Index len = row_len_;
  for (Eigen::Index c = 0; c < len; ++c) {
    const double rc = row_[c];
    if (rc == 0.0) {
      continue;
    }
    qp_.H.col(c).head(len) += weight * (rc * row_.head(len));
  }
  qp_.g.head(len) += (weight * residual) * row_.head(len);
}

void MpcDockingSegmentCore::SetRow(Eigen::Index row, double lower, double upper) noexcept {
  qp_.C.row(row).head(nu_) = row_.transpose();
  qp_.l[row] = lower;
  qp_.u[row] = upper;
}

void MpcDockingSegmentCore::AssembleQp() noexcept {
  const MpcDockingSegmentCoreParams& p = params_;
  const Eigen::Index nn = n_;
  const Eigen::Index N = n_nodes_;
  const Eigen::Index nN = nn * N;
  const Eigen::Index kc = n_pre_;
  const Eigen::Index nb = n_blocks_;
  const double da = p.dt_pre;
  const double ds = p.dt_stop;
  const double us = p.u_scale;
  const double tr = p.delta_tr;

  qp_.H.topLeftCorner(nu_, nu_) = h_const_;
  qp_.g.setZero();

  // ── Gradient of the constant-Hessian terms at the iterate ──
  qp_.g.head(nu_) = jerk_diag_.cwiseProduct(z_);
  for (Eigen::Index k = 1; k < kc; ++k) {
    if (!acc_cost_ && !posture_cost_) {
      break;
    }
    for (Eigen::Index b = 0; b <= block_of_node_[static_cast<std::size_t>(k - 1)]; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        qp_.g[b * nn + j] +=
            2.0 * da *
            (r_acc_[j] * ga_(k, b) * a_(j, k) + w_q_nom_[j] * gq_(k, b) * (q_(j, k) - q_nom_[j]));
      }
    }
  }

  // ── Torque: cost on nodes 1..k_c−1, rows on nodes 1..N ──
  for (Eigen::Index k = 1; k <= N; ++k) {
    const Eigen::Index col = 3 * nn * k;
    const Eigen::Index o = (k - 1) * nn;
    const Eigen::Index last = block_of_node_[static_cast<std::size_t>(k - 1)];
    t_node_jac_.setZero();
    for (Eigen::Index b = 0; b <= last; ++b) {
      t_node_jac_.block(0, b * nn, nn, nn) = gq_(k, b) * d_tau_.block(0, col, nn, nn) +
                                             gv_(k, b) * d_tau_.block(0, col + nn, nn, nn) +
                                             ga_(k, b) * d_tau_.block(0, col + 2 * nn, nn, nn);
    }
    if (torque_cost_ && k < kc) {
      wt_ = r_tau2_.asDiagonal() * t_node_jac_;
      qp_.H.topLeftCorner(nu_, nu_).noalias() += t_node_jac_.transpose() * wt_;
      work_n_ = tau_.col(k);
      qp_.g.head(nu_).noalias() += wt_.transpose() * work_n_;
    }
    qp_.C.block(row_tau_ + o, 0, nn, nu_) = inv_tau_.asDiagonal() * t_node_jac_;
    qp_.C.block(row_tau_ + nN + o, 0, nn, nu_) = qp_.C.block(row_tau_ + o, 0, nn, nu_);
    for (Eigen::Index j = 0; j < nn; ++j) {
      const double t = tau_(j, k) * inv_tau_[j];
      qp_.l[row_tau_ + o + j] = -kInf;
      qp_.u[row_tau_ + o + j] = tau_hi_[j] * inv_tau_[j] - t;
      qp_.l[row_tau_ + nN + o + j] = tau_lo_[j] * inv_tau_[j] - t;
      qp_.u[row_tau_ + nN + o + j] = kInf;
    }
  }
  if (torque_cost_) {
    // The matrix products above are symmetric only to rounding.
    for (Eigen::Index c = 0; c < nu_; ++c) {
      for (Eigen::Index r = c + 1; r < nu_; ++r) {
        const double m = 0.5 * (qp_.H(r, c) + qp_.H(c, r));
        qp_.H(r, c) = m;
        qp_.H(c, r) = m;
      }
    }
  }

  // ── Catch-vicinity cost: nodes 1..k_c−1 ──
  const Eigen::Vector3d r_ref(p.rho_ref.x(), p.rho_ref.y(), p.s_ent);
  const double t_c = t_node_[static_cast<std::size_t>(kc)];
  if (near_cost_) {
    for (Eigen::Index k = 1; k < kc; ++k) {
      const auto ki = static_cast<std::size_t>(k);
      const DockingRelativeState& rel = rel_[ki];
      const double w = 2.0 * da * near_weight_[ki];
      const Eigen::Vector3d r_ref_k = r_ref - (t_c - t_node_[ki]) * p.nu_ref;
      for (Eigen::Index i = 0; i < 3; ++i) {
        if (p.q_p[i] > 0.0) {
          aq_ = rel.dr_dq.row(i).transpose();
          MapRow(k, &aq_, nullptr);
          AddScalarResidual(w * p.q_p[i], rel.r_h[i] - r_ref_k[i]);
        }
        if (p.q_v[i] > 0.0) {
          aq_ = rel.dnu_dq.row(i).transpose();
          av_ = rel.dr_dq.row(i).transpose();
          MapRow(k, &aq_, &av_);
          AddScalarResidual(w * p.q_v[i], rel.nu_h[i] - p.nu_ref[i]);
        }
      }
    }
  }

  // ── Terminal cost at the catch node ──
  {
    const DockingRelativeState& rel = rel_[static_cast<std::size_t>(kc)];
    for (Eigen::Index i = 0; i < 2; ++i) {
      if (p.q_rho_f[i] > 0.0) {
        aq_ = rel.dr_dq.row(i).transpose();
        MapRow(kc, &aq_, nullptr);
        AddScalarResidual(2.0 * p.q_rho_f[i], rel.r_h[i] - p.rho_ref[i]);
      }
    }
    for (Eigen::Index i = 0; i < 3; ++i) {
      if (p.q_nu_f[i] > 0.0) {
        aq_ = rel.dnu_dq.row(i).transpose();
        av_ = rel.dr_dq.row(i).transpose();
        MapRow(kc, &aq_, &av_);
        AddScalarResidual(2.0 * p.q_nu_f[i], rel.nu_h[i] - p.nu_ref[i]);
      }
    }
    if (impact_cost_) {
      // w_E E / E_ref = (w_E / E_ref) · (√E)².
      MapRow(kc, &impact_.root_energy.dq, &impact_.root_energy.dv);
      AddScalarResidual(2.0 * p.w_impact / p.e_ref, impact_.root_energy.value);
    }
  }

  // ── Manipulability: gradient only ──
  if (manip_on_) {
    for (Eigen::Index k = 1; k < kc; ++k) {
      aq_ = manip_grad_.col(k);
      MapRow(k, &aq_, nullptr);
      qp_.g.head(row_len_) += (da * p.w_manip) * row_.head(row_len_);
    }
  }

  // ── Stop line: nodes k_c..N ──
  if (perp_on_) {
    for (Eigen::Index k = kc; k <= N; ++k) {
      for (Eigen::Index i = 0; i < 3; ++i) {
        aq_ = perp_jac_.block(i, nn * k, 1, nn).transpose();
        MapRow(k, &aq_, nullptr);
        AddScalarResidual(2.0 * ds * p.w_perp, perp_res_(i, k));
      }
    }
  }

  // ── Slack and elastic costs ──
  for (Eigen::Index i = 0; i < n_app_; ++i) {
    qp_.g[o_sc_ + i] = da * p.lambda1_c;
    qp_.g[o_sv_ + i] = da * p.lambda1_v;
  }
  SetPenaltyGradient();

  // ── Linear rows ──
  for (Eigen::Index j = 0; j < nn; ++j) {
    qp_.b[j] = -v_(j, N);
    qp_.b[nn + j] = -a_(j, N);
  }
  for (Eigen::Index k = 1; k <= N; ++k) {
    const Eigen::Index o = (k - 1) * nn;
    for (Eigen::Index j = 0; j < nn; ++j) {
      // Every operand is finite (the evaluation was), so max/min cannot
      // launder a NaN into a vanished trust region (NUM-7). The iterate
      // satisfies the box only to the QP's tolerance: when it sits outside by
      // more than a (tiny) trust region, the two would cross — the step back
      // to the box then takes precedence over the trust region.
      const double box_lo = q_lo_[j] - q_(j, k);
      const double box_hi = q_hi_[j] - q_(j, k);
      double lower = std::max(box_lo, -tr);
      double upper = std::min(box_hi, tr);
      if (lower > upper) {
        if (box_lo > tr) {
          upper = lower;
        } else {
          lower = upper;
        }
      }
      qp_.l[row_q_ + o + j] = lower;
      qp_.u[row_q_ + o + j] = upper;
      qp_.l[row_v_ + o + j] = -v_hi_[j] - v_(j, k);
      qp_.u[row_v_ + o + j] = v_hi_[j] - v_(j, k);
      if (p.accel_box) {
        qp_.l[row_a_ + o + j] = -a_hi_[j] - a_(j, k);
        qp_.u[row_a_ + o + j] = a_hi_[j] - a_(j, k);
      }
    }
  }
  if (p.jerk_box) {
    for (Eigen::Index b = 0; b < nb; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        qp_.l[row_u_ + b * nn + j] = -u_hi_[j] / us - z_[b * nn + j];
        qp_.u[row_u_ + b * nn + j] = u_hi_[j] / us - z_[b * nn + j];
      }
    }
  }
  for (Eigen::Index i = 0; i < n_elastic_; ++i) {
    qp_.l[row_e_ + i] = 0.0;
    qp_.u[row_e_ + i] = kInf;
  }
  // A group whose rows are not built keeps its elastic at 0 (it would
  // otherwise be a cost-only variable the solver has to push down).
  if (!timing_on_) {
    qp_.u[row_e_ + e_tim_] = 0.0;
  }
  if (!impact_rows_) {
    qp_.u[row_e_ + e_imp_] = 0.0;
  }
  for (Eigen::Index i = 0; i < 2 * n_app_; ++i) {
    qp_.l[row_s_ + i] = 0.0;
    qp_.u[row_s_ + i] = kInf;
  }

  // ── Approach rows ──
  for (Eigen::Index i = 0; i < n_app_; ++i) {
    const auto ai = static_cast<std::size_t>(i);
    const Eigen::Index k = app_node_[ai];
    const DockingRelativeState& rel = rel_[static_cast<std::size_t>(k)];
    const Eigen::Index row = row_app_ + 3 * i;
    aq_ = rel.dr_dq.row(2).transpose();
    MapRow(k, &aq_, nullptr);
    SetRow(row, -(rel.s - p.s_ent), kInf);
    // g + ∂g d + ∂g/∂s_c (s_c − s̄_c) ≤ 0.
    MapRow(k, &corridor_[ai].dq, nullptr);
    SetRow(row + 1, -kInf, -corridor_[ai].value + corridor_ds_[ai] * slack_c_[ai]);
    qp_.C(row + 1, o_sc_ + i) = corridor_ds_[ai];
    // g(s̄_v) + ∂g d − (s_v − s̄_v) ≤ 0.
    MapRow(k, &envelope_[ai].dq, &envelope_[ai].dv);
    SetRow(row + 2, -kInf, -envelope_[ai].value - slack_v_[ai]);
  }

  // ── Catch-node rows ──
  {
    const DockingRelativeState& rel = rel_[static_cast<std::size_t>(kc)];
    const double gap = rel.s - p.s_ent;
    aq_ = rel.dr_dq.row(2).transpose();
    MapRow(kc, &aq_, nullptr);
    SetRow(row_ent_, -kInf, -gap);
    SetRow(row_ent_ + 1, -gap, kInf);
    for (Eigen::Index i = 0; i < p.n_faces; ++i) {
      const auto fi = static_cast<std::size_t>(i);
      MapRow(kc, &lateral_[fi].dq, &lateral_[fi].dv);
      SetRow(row_lat_ + i, -kInf, p.face_b[fi] - lateral_[fi].value);
    }
    if (timing_on_) {
      MapRow(kc, &timing_.dq, &timing_.dv);
      SetRow(row_tim_, -timing_.value, kInf);
    }
    MapRow(kc, &axial_lo_.dq, &axial_lo_.dv);
    SetRow(row_vel_, p.c_min - axial_lo_.value, kInf);
    MapRow(kc, &axial_hi_.dq, &axial_hi_.dv);
    SetRow(row_vel_ + 1, -kInf, p.c_cap_max - axial_hi_.value);
    for (Eigen::Index i = 0; i < p.speed_faces; ++i) {
      const auto fi = static_cast<std::size_t>(i);
      MapRow(kc, &speed_[fi].dq, &speed_[fi].dv);
      SetRow(row_vel_ + 2 + i, -kInf, speed_face_bound_ - speed_[fi].value);
    }
    if (impact_rows_) {
      MapRow(kc, &impact_.g_n.dq, &impact_.g_n.dv);
      SetRow(row_imp_, -kInf, -impact_.g_n.value);
      if (std::isfinite(p.e_max)) {
        aq_ = impact_.energy.dq / p.e_max;
        av_ = impact_.energy.dv / p.e_max;
        MapRow(kc, &aq_, &av_);
        SetRow(row_imp_ + 1, -kInf, 1.0 - impact_.energy.value / p.e_max);
      }
      if (std::isfinite(p.p_max)) {
        aq_ = impact_.impulse.dq / p.p_max;
        av_ = impact_.impulse.dv / p.p_max;
        MapRow(kc, &aq_, &av_);
        SetRow(row_imp_ + 2, -kInf, 1.0 - impact_.impulse.value / p.p_max);
      }
    }
  }
}

void MpcDockingSegmentCore::SetPenaltyGradient() noexcept {
  const auto mu = [this](DockingRowGroup g) noexcept { return mu_[static_cast<std::size_t>(g)]; };
  for (Eigen::Index k = 0; k < n_nodes_; ++k) {
    qp_.g[o_e_ + e_torque_ + k] = mu(DockingRowGroup::kTorque);
  }
  for (Eigen::Index i = 0; i < n_app_; ++i) {
    qp_.g[o_e_ + e_gap_ + i] = mu(DockingRowGroup::kGap);
  }
  qp_.g[o_e_ + e_ent_] = mu(DockingRowGroup::kEntrance);
  qp_.g[o_e_ + e_lat_] = mu(DockingRowGroup::kLateral);
  qp_.g[o_e_ + e_tim_] = mu(DockingRowGroup::kTiming);
  qp_.g[o_e_ + e_vel_] = mu(DockingRowGroup::kVelocitySet);
  qp_.g[o_e_ + e_imp_] = mu(DockingRowGroup::kImpact);
}

void MpcDockingSegmentCore::ElasticByGroup(
    std::array<double, kNumDockingElasticGroups>& max_out,
    std::array<double, kNumDockingElasticGroups>& sum_out) const noexcept {
  max_out.fill(0.0);
  sum_out.fill(0.0);
  const auto take = [&](DockingRowGroup g, Eigen::Index first, Eigen::Index count) noexcept {
    const auto gi = static_cast<std::size_t>(g);
    for (Eigen::Index i = 0; i < count; ++i) {
      const double e = Positive(x_qp_[o_e_ + first + i]);
      max_out[gi] = std::max(max_out[gi], e);
      sum_out[gi] += e;
    }
  };
  take(DockingRowGroup::kTorque, e_torque_, n_nodes_);
  take(DockingRowGroup::kGap, e_gap_, n_app_);
  take(DockingRowGroup::kEntrance, e_ent_, 1);
  take(DockingRowGroup::kLateral, e_lat_, 1);
  take(DockingRowGroup::kTiming, e_tim_, 1);
  take(DockingRowGroup::kVelocitySet, e_vel_, 1);
  take(DockingRowGroup::kImpact, e_imp_, 1);
}

MpcDockingReason MpcDockingSegmentCore::RunQp(MpcDockingSegmentCoreResult& out) noexcept {
  const auto t0 = std::chrono::steady_clock::now();
  const tsid::SolveResult* res = &solver_.Solve(qp_);
  ++out.qp_solves;
  out.qp_iterations += res->iterations;
  if (!res->converged && solver_warm_) {
    // Iterates left by the previous linearisation can make ProxQP call a
    // feasible QP infeasible; one more run from zero settles it.
    solver_.ResetWarmStart();
    res = &solver_.Solve(qp_);
    ++out.qp_solves;
    out.qp_iterations += res->iterations;
  }
  out.qp_us += MicrosSince(t0);
  out.qp_status = res->status;
  if (!res->converged) {
    solver_.ResetWarmStart();
    solver_warm_ = false;
    return res->non_finite ? MpcDockingReason::kSolutionNonFinite : MpcDockingReason::kQpFailed;
  }
  solver_warm_ = true;
  x_qp_ = res->x_opt.head(nx_);
  y_qp_ = solver_.EqualityDual();
  z_qp_ = solver_.InequalityDual();
  d_ = x_qp_.head(nu_);
  return MpcDockingReason::kNone;
}

bool MpcDockingSegmentCore::GrowPenalties() noexcept {
  // When the last QP left an elastic positive, raise EVERY penalty by the same
  // factor (until the largest reaches the cap), keeping what is needed to
  // undo it. All of them, not only the groups with an elastic: the ratio
  // between the groups is the caller's (mu_init) and it decides in which
  // group an infeasible problem leaves its residual — growing some groups
  // alone would move the diagnosis with the iteration count.
  std::array<double, kNumDockingElasticGroups> e_max{};
  ElasticByGroup(e_max, elastic_sum_);
  bool left = false;
  double mu_top = 0.0;
  for (std::size_t g = 0; g < mu_.size(); ++g) {
    left = left || e_max[g] > params_.tol_violation;
    mu_top = std::max(mu_top, mu_[g]);
  }
  const double factor = std::min(params_.mu_growth, params_.mu_max / mu_top);
  if (!left || !(factor > 1.0)) {
    return false;
  }
  mu_keep_ = mu_;
  for (double& mu : mu_) {
    mu *= factor;
  }
  x_keep_ = x_qp_;
  y_keep_ = y_qp_;
  z_keep_ = z_qp_;
  SetPenaltyGradient();
  return true;
}

void MpcDockingSegmentCore::RestorePenalties() noexcept {
  mu_ = mu_keep_;
  SetPenaltyGradient();
  x_qp_ = x_keep_;
  y_qp_ = y_keep_;
  z_qp_ = z_keep_;
  d_ = x_qp_.head(nu_);
}

void MpcDockingSegmentCore::PostQp(MpcDockingSegmentCoreResult& out) noexcept {
  // ∇L of the NLP at the iterate with the QP's multipliers, ProxQP's signs:
  // g + Cᵀz + Aᵀy. The QP's own stationarity adds H x to this, so on the jerk
  // variables it equals −H d up to the solver's tolerance — zero exactly when
  // the iterate is a KKT point of the (penalised) problem. (On the slack and
  // elastic variables the vector is −H x, which says nothing; only the jerk
  // part is reported.)
  kkt_ = qp_.g;
  kkt_.noalias() += qp_.C.transpose() * z_qp_;
  kkt_.noalias() += qp_.A.transpose() * y_qp_;
  cx_.noalias() = qp_.C * x_qp_;
  out.kkt_residual = kkt_.head(nu_).cwiseAbs().maxCoeff();
  out.grad_norm = qp_.g.head(nu_).cwiseAbs().maxCoeff();
  double comp = 0.0;
  for (Eigen::Index i = 0; i < n_in_; ++i) {
    const double lam = z_qp_[i];
    if (lam > 0.0 && std::isfinite(qp_.u[i])) {
      comp = std::max(comp, std::abs(lam * (qp_.u[i] - cx_[i])));
    } else if (lam < 0.0 && std::isfinite(qp_.l[i])) {
      comp = std::max(comp, std::abs(lam * (cx_[i] - qp_.l[i])));
    }
  }
  out.complementarity = comp;
  ElasticByGroup(out.elastic, elastic_sum_);
  // What the solution leaves of the elastic groups' rows, in penalty units:
  // Σ μ_G · (row residual)⁺. Zero for an exact QP solution.
  double noise = 0.0;
  for (Eigen::Index i = row_tau_; i < n_in_; ++i) {
    const int g = row_group_[static_cast<std::size_t>(i)];
    if (g < 0) {
      continue;
    }
    const double over = std::max(cx_[i] - qp_.u[i], qp_.l[i] - cx_[i]);
    if (over > 0.0) {
      noise += mu_[static_cast<std::size_t>(g)] * over;
    }
  }
  qp_noise_ = noise;
}

double MpcDockingSegmentCore::Merit(const Evaluation& ev) const noexcept {
  double phi = ev.cost.total;
  for (std::size_t g = 0; g < mu_.size(); ++g) {
    phi += mu_[g] * ev.viol_sum[g];
  }
  return phi;
}

double MpcDockingSegmentCore::LinearizedDecrease(const Evaluation& ev) const noexcept {
  // The QP's own model of φ along the step, without the curvature term:
  // ∇Jᵀd + (slack cost at the QP's slacks − at the iterate's) +
  // Σ_G μ_G (Σ e − Σ viol). d = 0 with the iterate's slacks and e = viol is
  // feasible for the QP, so this is ≤ −½ dᵀHd ≤ 0.
  const MpcDockingSegmentCoreParams& p = params_;
  double dec = qp_.g.head(nu_).dot(d_);
  for (Eigen::Index i = 0; i < n_app_; ++i) {
    const auto ai = static_cast<std::size_t>(i);
    const double sc = x_qp_[o_sc_ + i];
    const double sv = x_qp_[o_sv_ + i];
    dec += p.dt_pre * (p.lambda1_c * (sc - slack_c_cur_[ai]) +
                       p.lambda2_c * (sc * sc - slack_c_cur_[ai] * slack_c_cur_[ai]) +
                       p.lambda1_v * (sv - slack_v_cur_[ai]) +
                       p.lambda2_v * (sv * sv - slack_v_cur_[ai] * slack_v_cur_[ai]));
  }
  for (std::size_t g = 0; g < mu_.size(); ++g) {
    dec += mu_[g] * (elastic_sum_[g] - ev.viol_sum[g]);
  }
  return dec;
}

// ── Result ───────────────────────────────────────────────────────────────────

void MpcDockingSegmentCore::Finish(const Evaluation& ev, MpcDockingReason reason,
                                   MpcDockingSegmentCoreResult& out) noexcept {
  const MpcDockingSegmentCoreParams& p = params_;
  const Eigen::Index nn = n_;
  const Eigen::Index N = n_nodes_;
  const Eigen::Index kc = n_pre_;
  out.q = q_;
  out.qd = v_;
  out.qdd = a_;
  for (Eigen::Index k = 0; k < N; ++k) {
    const Eigen::Index b = block_of_node_[static_cast<std::size_t>(k)];
    for (Eigen::Index j = 0; j < nn; ++j) {
      out.u(j, k) = p.u_scale * z_[b * nn + j];
    }
  }
  out.cost = ev.cost;
  out.violation = ev.viol_max;
  out.mu = mu_;
  out.slack_c.fill(0.0);
  out.slack_v.fill(0.0);
  for (int i = 0; i < n_app_; ++i) {
    out.slack_c[static_cast<std::size_t>(i)] = slack_c_[static_cast<std::size_t>(i)];
    out.slack_v[static_cast<std::size_t>(i)] = slack_v_[static_cast<std::size_t>(i)];
  }
  out.tau_ratio_max = ev.tau_ratio_max;
  out.approach_nodes = n_app_;

  // Catch-node diagnostics. kin_ still holds node k_c only if nothing after it
  // used the workspace, so it is recomputed here.
  const auto kci = static_cast<std::size_t>(kc);
  const DockingRelativeState& rel = rel_[kci];
  out.c_catch = rel.c;
  out.c_guarded = !(rel.c > p.c_min);
  out.sigma_s = 0.0;
  out.sigma_t = 0.0;
  out.linearization_ratio = 0.0;
  out.linearization_ratio_defined = false;
  if (p.chance && ev.ok &&
      ComputeDockingFrameKinematics(model_, data_, frame_, q_.col(kc), v_.col(kc), kin_)) {
    DockingCrossing cross;
    ComputeDockingCrossing(kin_, rel, sigma_p_, p.c_min, p.eps_sigma, cross);
    out.sigma_s = cross.sigma_s;
    out.sigma_t = cross.sigma_t;
    Eigen::Vector3d a_rel = Eigen::Vector3d::Zero();
    double margin = kInf;
    double kappa = 0.0;
    for (int i = 0; i < p.n_faces; ++i) {
      const auto fi = static_cast<std::size_t>(i);
      margin = std::min(margin, (p.face_b[fi] - p.face_a[fi].dot(rel.Rho())) / p.face_a[fi].norm());
      kappa = std::max(kappa, kappa_face_[fi]);
    }
    if (p.n_faces > 0 && margin > 0.0 && !cross.c_guarded &&
        DockingRelativeAcceleration(model_, data_, frame_, q_.col(kc), v_.col(kc), a_.col(kc),
                                    ball_[kci].p, ball_[kci].v, ball_[kci].a, a_rel)) {
      const double reach = kappa * cross.sigma_t;
      const double ratio = 0.5 * a_rel.head<2>().norm() * reach * reach / margin;
      if (std::isfinite(ratio)) {
        out.linearization_ratio = ratio;
        out.linearization_ratio_defined = true;
      }
    }
  }

  // Feasible: every hard row holds on the nonlinear model at THIS iterate.
  bool feasible = ev.ok;
  double worst = -1.0;
  out.infeasible_group = DockingRowGroup::kTorque;
  for (std::size_t g = 0; g < ev.viol_max.size(); ++g) {
    if (!(ev.viol_max[g] <= p.tol_violation)) {
      feasible = false;
    }
    if (g < static_cast<std::size_t>(kNumDockingElasticGroups) && ev.viol_max[g] > worst) {
      worst = ev.viol_max[g];
      out.infeasible_group = static_cast<DockingRowGroup>(g);
    }
  }
  out.feasible = feasible;
  out.converged = reason == MpcDockingReason::kConverged && feasible;
  out.reason = reason;
}

// ── Solve ────────────────────────────────────────────────────────────────────

MpcDockingReason MpcDockingSegmentCore::Prepare(const MpcDockingSegmentCoreInput& in,
                                                const MpcDockingSegmentCoreResult& out,
                                                bool need_initial) noexcept {
  if (!initialized_) {
    return MpcDockingReason::kNotInitialized;
  }
  const MpcDockingSegmentCoreParams& p = params_;
  const Eigen::Index nn = n_;
  const Eigen::Index N = n_nodes_;
  const Eigen::Index N1 = N + 1;
  const Eigen::Index kc = n_pre_;
  if (out.q.rows() != nn || out.q.cols() != N1 || out.qd.rows() != nn || out.qd.cols() != N1 ||
      out.qdd.rows() != nn || out.qdd.cols() != N1 || out.u.rows() != nn || out.u.cols() != N ||
      in.q0.size() != nn || in.qd0.size() != nn || in.qdd0.size() != nn) {
    return MpcDockingReason::kDimMismatch;
  }
  if (in.initial_valid &&
      (in.q_init.rows() != nn || in.q_init.cols() != N1 || in.qd_init.rows() != nn ||
       in.qd_init.cols() != N1 || in.qdd_init.rows() != nn || in.qdd_init.cols() != N1)) {
    return MpcDockingReason::kDimMismatch;
  }
  const bool use_target = !in.initial_valid && in.catch_target_valid && !need_initial;
  if (use_target && in.q_catch_target.size() != nn) {
    return MpcDockingReason::kDimMismatch;
  }
  // Finiteness BEFORE any bound or comparison is formed (NUM-7).
  if (!in.q0.allFinite() || !in.qd0.allFinite() || !in.qdd0.allFinite()) {
    return MpcDockingReason::kNonFinite;
  }
  if (in.initial_valid &&
      (!in.q_init.allFinite() || !in.qd_init.allFinite() || !in.qdd_init.allFinite())) {
    return MpcDockingReason::kNonFinite;
  }
  if (use_target && !in.q_catch_target.allFinite()) {
    return MpcDockingReason::kNonFinite;
  }
  if (!in.initial_valid && (need_initial || !in.catch_target_valid)) {
    return MpcDockingReason::kTargetRequired;
  }
  for (Eigen::Index k = 0; k <= kc; ++k) {
    const BallNodeSample& b = in.ball[static_cast<std::size_t>(k)];
    if (!b.valid || !b.p.allFinite() || !b.v.allFinite() || !b.a.allFinite()) {
      return MpcDockingReason::kBallInvalid;
    }
  }
  if (p.chance) {
    const BallNodeSample& b = in.ball[static_cast<std::size_t>(kc)];
    if (!b.cov_valid || !b.cov.allFinite()) {
      return MpcDockingReason::kCovarianceInvalid;
    }
    // PSD to rounding: every standard deviation is the square root of a
    // quadratic form of this matrix.
    const DockingCovariance6 sym = 0.5 * (b.cov + b.cov.transpose());
    Eigen::SelfAdjointEigenSolver<DockingCovariance6> es;  // fixed size: no heap
    es.compute(sym, Eigen::EigenvaluesOnly);
    if (es.info() != Eigen::Success || !es.eigenvalues().allFinite() ||
        !(es.eigenvalues().minCoeff() >= -kPsdRelTolerance * std::max(sym.trace(), 0.0))) {
      return MpcDockingReason::kCovarianceInvalid;
    }
    sigma_b_ = sym;
    sigma_p_ = sym.topLeftCorner<3, 3>();
  } else {
    sigma_b_.setZero();
    sigma_p_.setZero();
  }
  if (perp_on_) {
    if (!in.p_line.allFinite() || !in.d_line.allFinite()) {
      return MpcDockingReason::kNonFinite;
    }
    if (!(std::abs(in.d_line.norm() - 1.0) <= kUnitTolerance)) {
      return MpcDockingReason::kInputOutOfRange;
    }
    p_line_ = in.p_line;
    p_perp_ = Eigen::Matrix3d::Identity() - in.d_line * in.d_line.transpose();
  }
  for (Eigen::Index j = 0; j < nn; ++j) {
    if (in.q0[j] < q_lo_[j] - kBoxRoundingSlack || in.q0[j] > q_hi_[j] + kBoxRoundingSlack ||
        std::abs(in.qd0[j]) > v_hi_[j]) {
      return MpcDockingReason::kInitialStateOutsideBox;
    }
  }
  q0_ = in.q0;
  v0_ = in.qd0;
  a0_ = in.qdd0;
  for (Eigen::Index k = 0; k <= kc; ++k) {
    ball_[static_cast<std::size_t>(k)] = in.ball[static_cast<std::size_t>(k)];
  }
  FreeResponse();
  return MpcDockingReason::kNone;
}

void MpcDockingSegmentCore::ResetRecord(MpcDockingSegmentCoreResult& out) noexcept {
  out.feasible = false;
  out.converged = false;
  out.iterations = 0;
  out.qp_solves = 0;
  out.qp_iterations = 0;
  out.backtracks = 0;
  out.mu_updates = 0;
  out.init_qp_used = false;
  out.qp_status = -1;
  out.start_us = 0.0;
  out.linearize_us = 0.0;
  out.assemble_us = 0.0;
  out.qp_us = 0.0;
  out.merit_us = 0.0;
  out.total_us = 0.0;
  out.kkt_residual = 0.0;
  out.grad_norm = 0.0;
  out.complementarity = 0.0;
  out.elastic.fill(0.0);
}

bool MpcDockingSegmentCore::Evaluate(const MpcDockingSegmentCoreInput& in,
                                     MpcDockingSegmentCoreResult& out) noexcept {
  const auto t_start = std::chrono::steady_clock::now();
  ResetRecord(out);
  StageBegin(MpcDockingStage::kStart);
  const MpcDockingReason why = Prepare(in, out, true);
  if (why != MpcDockingReason::kNone) {
    StageEnd(MpcDockingStage::kStart);
    out.reason = why;
    return false;
  }
  ProjectInitial(in);
  StageEnd(MpcDockingStage::kStart);
  mu_ = params_.mu_init;
  StageBegin(MpcDockingStage::kFinish);
  TrajectoryFromZ(z_);
  Evaluation ev;
  (void)EvaluateTrajectory(false, ev);
  Finish(ev, ev.ok ? MpcDockingReason::kNone : MpcDockingReason::kSolutionNonFinite, out);
  StageEnd(MpcDockingStage::kFinish);
  out.total_us = MicrosSince(t_start);
  return true;
}

bool MpcDockingSegmentCore::Solve(const MpcDockingSegmentCoreInput& in,
                                  MpcDockingSegmentCoreResult& out) noexcept {
  using Clock = std::chrono::steady_clock;
  const auto t_solve = Clock::now();
  ResetRecord(out);
  const auto reject = [&](MpcDockingReason r) noexcept {
    out.reason = r;
    out.total_us = MicrosSince(t_solve);
    return false;
  };
  const MpcDockingSegmentCoreParams& p = params_;

  // ── Start point ──
  {
    const auto t0 = Clock::now();
    StageBegin(MpcDockingStage::kStart);
    const MpcDockingReason why = Prepare(in, out, false);
    if (why != MpcDockingReason::kNone) {
      StageEnd(MpcDockingStage::kStart);
      return reject(why);
    }
    bool need_init = true;
    if (in.initial_valid) {
      ProjectInitial(in);
      TrajectoryFromZ(z_);
      need_init = !LinearRowsHold();
    }
    StageEnd(MpcDockingStage::kStart);
    if (need_init) {
      const MpcDockingReason init_why = RunInitQp(in, out);
      if (init_why != MpcDockingReason::kNone) {
        return reject(init_why);
      }
    }
    out.start_us = MicrosSince(t0) - out.qp_us;
  }

  // A new Solve is a new problem: the same input gives the same answer
  // whatever was solved before.
  mu_ = p.mu_init;
  solver_.ResetWarmStart();
  solver_warm_ = false;

  MpcDockingReason reason = MpcDockingReason::kIterationLimit;
  for (int it = 0; it < p.max_iterations; ++it) {
    if (it > 0 && clock_ != nullptr && in.deadline_ns > 0 && clock_() >= in.deadline_ns) {
      reason = MpcDockingReason::kDeadline;
      break;
    }
    {
      const auto t0 = Clock::now();
      StageBegin(MpcDockingStage::kLinearize);
      TrajectoryFromZ(z_);
      const bool ok = EvaluateTrajectory(true, ev_cur_);
      slack_c_cur_ = slack_c_;
      slack_v_cur_ = slack_v_;
      StageEnd(MpcDockingStage::kLinearize);
      out.linearize_us += MicrosSince(t0);
      if (!ok) {
        reason = MpcDockingReason::kSolutionNonFinite;
        break;
      }
    }
    {
      const auto t0 = Clock::now();
      StageBegin(MpcDockingStage::kAssemble);
      AssembleQp();
      StageEnd(MpcDockingStage::kAssemble);
      out.assemble_us += MicrosSince(t0);
    }
    MpcDockingReason qp_why = RunQp(out);
    if (qp_why != MpcDockingReason::kNone && qp_why != MpcDockingReason::kSolutionNonFinite) {
      // A penalty raised by an earlier probe can be what the solver chokes on
      // at this linearisation: fall back to the initial penalties once.
      bool raised = false;
      for (std::size_t g = 0; g < mu_.size(); ++g) {
        raised = raised || mu_[g] > p.mu_init[g];
      }
      if (raised) {
        StageBegin(MpcDockingStage::kPostQp);
        mu_ = p.mu_init;
        SetPenaltyGradient();
        StageEnd(MpcDockingStage::kPostQp);
        qp_why = RunQp(out);
      }
    }
    if (qp_why != MpcDockingReason::kNone) {
      reason = qp_why;
      break;
    }
    // Penalty update. An elastic the QP left positive means either that the
    // linearised rows cannot be met here, or that μ is below the exactness
    // threshold; the multipliers cannot tell the two apart (they sit at μ),
    // but the QP's RESPONSE to a larger μ can. So each geometric step is a
    // probe: it is kept only if the elastic total falls by min_gain, and
    // undone otherwise — raising μ on rows that cannot be met buys nothing
    // and ruins the conditioning of every QP after it.
    for (;;) {
      StageBegin(MpcDockingStage::kPostQp);
      double before = 0.0;
      const bool grew = GrowPenalties();
      for (const double e : elastic_sum_) {
        before += e;
      }
      StageEnd(MpcDockingStage::kPostQp);
      if (!grew) {
        break;
      }
      const MpcDockingReason probe = RunQp(out);
      StageBegin(MpcDockingStage::kPostQp);
      bool keep = probe == MpcDockingReason::kNone;
      if (keep) {
        std::array<double, kNumDockingElasticGroups> e_max{};
        std::array<double, kNumDockingElasticGroups> e_sum{};
        ElasticByGroup(e_max, e_sum);
        double after = 0.0;
        for (const double e : e_sum) {
          after += e;
        }
        // Kept when the larger penalty buys feasibility of the LINEARISED
        // rows: the elastic total falls by min_gain, or — when a trust region
        // caps what one step can remove, so that the total barely moves — the
        // amount this step removes grows by min_gain.
        double violation = 0.0;
        for (const double v : ev_cur_.viol_sum) {
          violation += v;
        }
        const double removed_before = violation - before;
        const double removed_after = violation - after;
        keep = after <= (1.0 - p.mu_min_gain) * before ||
               (removed_after - removed_before > p.tol_violation &&
                removed_after >= (1.0 + p.mu_min_gain) * removed_before);
      }
      if (!keep) {
        RestorePenalties();
      }
      StageEnd(MpcDockingStage::kPostQp);
      if (!keep) {
        break;
      }
      ++out.mu_updates;
    }
    StageBegin(MpcDockingStage::kPostQp);
    PostQp(out);
    StageEnd(MpcDockingStage::kPostQp);
    ++out.iterations;

    bool feasible_now = true;
    for (const double v : ev_cur_.viol_max) {
      feasible_now = feasible_now && v <= p.tol_violation;
    }
    bool elastic_left = false;
    for (const double e : out.elastic) {
      elastic_left = elastic_left || e > p.tol_violation;
    }
    const bool stationary = out.kkt_residual <= p.tol_kkt * std::max(1.0, out.grad_norm);
    if (stationary && feasible_now && out.complementarity <= p.tol_complementarity) {
      reason = MpcDockingReason::kConverged;
      break;
    }
    // The hard rows' total violation at this iterate, weighted by the INITIAL
    // penalties (the current ones move during a solve).
    double violation_now = 0.0;
    for (std::size_t g = 0; g < mu_.size(); ++g) {
      violation_now += p.mu_init[g] * ev_cur_.viol_sum[g];
    }
    // Read the old entry BEFORE this iteration's overwrites it: with the
    // longest window the two are the same slot of the ring.
    const double violation_then = it >= p.stall_window
                                      ? violation_hist_[static_cast<std::size_t>(
                                            (it - p.stall_window) % kDockingStallHistory)]
                                      : kInf;
    violation_hist_[static_cast<std::size_t>(it % kDockingStallHistory)] = violation_now;
    const bool stalled = violation_now >= (1.0 - p.stall_reduction) * violation_then;
    if (elastic_left && !feasible_now && (stationary || stalled)) {
      // The linearised rows cannot be met here, a larger penalty does not
      // help (the probe above), and either the iterate is a stationary point
      // of the penalised problem or the violation has stopped falling: over
      // the last stall_window iterations it fell by less than stall_reduction.
      // The second test is what ends an out-of-reach problem — there the
      // first-order model keeps promising a gain the arm's geometry does not
      // deliver, and the cost alone would be polished for many iterations.
      // (A violation with NO elastic left is not this case: the step that
      // removes it is just small. Nor is an elastic at a FEASIBLE iterate:
      // the QP is then stepping into a violation, which the merit judges.)
      reason = MpcDockingReason::kInfeasible;
      break;
    }
    if (p.max_iterations == 1) {
      // One real-time iteration: the full step, unjudged.
      StageBegin(MpcDockingStage::kMerit);
      z_ += d_;
      StageEnd(MpcDockingStage::kMerit);
      reason = MpcDockingReason::kIterationLimit;
      break;
    }

    const auto t0 = Clock::now();
    StageBegin(MpcDockingStage::kMerit);
    const double phi0 = Merit(ev_cur_);
    const double decrease = LinearizedDecrease(ev_cur_);
    // Armijo on the ℓ₁ merit, with the QP's own inexactness allowed for. An
    // exact QP solution gives decrease ≤ 0 and leaves no row residual. A
    // solution to tolerance can leave both an elastic that should be zero
    // (decrease > 0: the model as solved predicts a rise of that much) and
    // row residuals (qp_noise_: their penalty value, measured on this
    // solution). Close to the solution those exceed the decrease still to be
    // had, and a test that ignored them would reject every step length —
    // demanding a decrease smaller than the noise of the step it judges.
    const double allowed = (decrease < 0.0 ? p.armijo_eta * decrease : decrease) + 2.0 * qp_noise_;
    double alpha = 1.0;
    bool accepted = false;
    for (int bt = 0; bt <= p.max_backtracks; ++bt) {
      z_trial_ = z_ + alpha * d_;
      TrajectoryFromZ(z_trial_);
      Evaluation trial;
      if (EvaluateTrajectory(false, trial) && Merit(trial) <= phi0 + alpha * allowed) {
        accepted = true;
        break;
      }
      alpha *= p.backtrack_beta;
      ++out.backtracks;
    }
    if (accepted) {
      z_ = z_trial_;
    }
    StageEnd(MpcDockingStage::kMerit);
    out.merit_us += MicrosSince(t0);
    if (!accepted) {
      reason = feasible_now ? MpcDockingReason::kLineSearchFailed : MpcDockingReason::kInfeasible;
      break;
    }
  }

  StageBegin(MpcDockingStage::kFinish);
  TrajectoryFromZ(z_);
  Evaluation final_ev;
  (void)EvaluateTrajectory(false, final_ev);
  if (!final_ev.ok && reason != MpcDockingReason::kQpFailed) {
    reason = MpcDockingReason::kSolutionNonFinite;
  }
  Finish(final_ev, reason, out);
  StageEnd(MpcDockingStage::kFinish);
  out.total_us = MicrosSince(t_solve);
  return true;
}

}  // namespace rtc::catching
