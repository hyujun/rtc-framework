#include "rtc_controllers/catching/decel_mpc.hpp"

#include "rtc_controllers/catching/decel_mpc_catch.hpp"
#include "rtc_controllers/catching/decel_mpc_torque.hpp"
#include "rtc_controllers/gain_floor.hpp"
#include "rtc_math/se3/axis_align.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>

namespace rtc::catching {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// ‖d̂‖ must be 1 to this tolerance; P_⊥ = I − d̂d̂ᵀ is a projector only then.
constexpr double kUnitTolerance = 1e-6;
// Entry-box comparison slack [rad] — rounding only, far below any margin.
constexpr double kBoxRoundingSlack = 1e-12;
// W_p symmetry and PSD tolerance, relative to its largest entry / its trace: a
// weight built as V·diag·Vᵀ is symmetric and PSD only to rounding.
constexpr double kWeightRelTolerance = 1e-9;
// Ball speed [m/s] below which its direction of travel is undefined: the
// velocity weight is then isotropic (w_⊥) and γ is reported as 0.
constexpr double kMinBallSpeed = 1e-6;

[[nodiscard]] bool FinitePositive(double x) noexcept {
  return std::isfinite(x) && x > 0.0;
}

[[nodiscard]] bool InUnitInterval(double x) noexcept {
  return std::isfinite(x) && x > 0.0 && x <= 1.0;
}

[[nodiscard]] double MicrosSince(std::chrono::steady_clock::time_point t0) noexcept {
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

}  // namespace

const char* DecelMpcReasonName(DecelMpcReason reason) noexcept {
  switch (reason) {
    case DecelMpcReason::kNone:
      return "none";
    case DecelMpcReason::kNotInitialized:
      return "not_initialized";
    case DecelMpcReason::kParamsInvalid:
      return "params_invalid";
    case DecelMpcReason::kBlocksTooFew:
      return "blocks_too_few";
    case DecelMpcReason::kLimitsInvalid:
      return "limits_invalid";
    case DecelMpcReason::kModelUnsupported:
      return "model_unsupported";
    case DecelMpcReason::kFrameUnknown:
      return "frame_unknown";
    case DecelMpcReason::kTerminalRankDeficient:
      return "terminal_rank_deficient";
    case DecelMpcReason::kDimMismatch:
      return "dim_mismatch";
    case DecelMpcReason::kNonFinite:
      return "non_finite";
    case DecelMpcReason::kDirectionNotUnit:
      return "direction_not_unit";
    case DecelMpcReason::kInitialStateOutsideBox:
      return "initial_state_outside_box";
    case DecelMpcReason::kReferenceNotAtRest:
      return "reference_not_at_rest";
    case DecelMpcReason::kTrustRegionConflict:
      return "trust_region_conflict";
    case DecelMpcReason::kPresolveFailed:
      return "presolve_failed";
    case DecelMpcReason::kQpFailed:
      return "qp_failed";
    case DecelMpcReason::kSolutionNonFinite:
      return "solution_non_finite";
    case DecelMpcReason::kBlocksAcrossCatch:
      return "blocks_across_catch";
    case DecelMpcReason::kInputOutOfRange:
      return "input_out_of_range";
    case DecelMpcReason::kReferenceRequired:
      return "reference_required";
    case DecelMpcReason::kCatchAxisOutOfRange:
      return "catch_axis_out_of_range";
  }
  return "unknown";
}

// ── Torque linearisation seam ────────────────────────────────────────────────

bool LinearizeTorqueAt(const pinocchio::Model& model, pinocchio::Data& data,
                       const Eigen::Ref<const Eigen::VectorXd>& q,
                       const Eigen::Ref<const Eigen::VectorXd>& v,
                       const Eigen::Ref<const Eigen::VectorXd>& a, Eigen::Ref<Eigen::VectorXd> tau,
                       Eigen::Ref<Eigen::MatrixXd> d) noexcept {
  const Eigen::Index n = model.nv;
  if (model.nq != model.nv || q.size() != n || v.size() != n || a.size() != n || tau.size() != n ||
      d.rows() != n || d.cols() != 3 * n) {
    return false;
  }
  // Pinocchio accumulates into the partials (the armature is ADDED to the
  // diagonal of the ∂τ/∂q̈ output) — clear them every call.
  d.setZero();
  pinocchio::computeRNEADerivatives(model, data, q, v, a, d.leftCols(n), d.middleCols(n, n),
                                    d.rightCols(n));
  // ∂τ/∂q̈ = M comes back upper-triangular only.
  for (Eigen::Index c = 0; c < n; ++c) {
    for (Eigen::Index r = c + 1; r < n; ++r) {
      d(r, 2 * n + c) = d(c, 2 * n + r);
    }
  }
  tau = data.tau;
  return true;
}

// ── Init (non-RT) ────────────────────────────────────────────────────────────

DecelMpcReason DecelMpc::Init(const pinocchio::Model& arm, pinocchio::FrameIndex catch_frame,
                              const DecelMpcParams& params, const DecelMpcLimits& limits) {
  initialized_ = false;
  solver_warm_ = false;

  if (arm.nv < 1 || arm.nv > kMaxPlanNv || arm.nq != arm.nv) {
    return DecelMpcReason::kModelUnsupported;
  }
  if (catch_frame >= arm.frames.size()) {
    return DecelMpcReason::kFrameUnknown;
  }
  const int n = arm.nv;
  const auto nn = static_cast<Eigen::Index>(n);

  // Parameters. n_nodes is the STOP segment; the horizon is n_pre + n_nodes.
  const int n_stop = params.n_nodes;
  if (n_stop < 1 || n_stop > kMaxDecelNodes || !FinitePositive(params.dt)) {
    return DecelMpcReason::kParamsInvalid;
  }
  const int n_pre = params.n_pre;
  // dt_pre enters the node times whatever n_pre is, so it is checked finite
  // even when no pre-catch node uses it.
  if (n_pre < 0 || n_pre > kMaxMpcNodes - n_stop || !rtc::IsFiniteNonNegative(params.dt_pre) ||
      (n_pre > 0 && !FinitePositive(params.dt_pre))) {
    return DecelMpcReason::kParamsInvalid;
  }
  const int n_nodes = n_pre + n_stop;
  if (params.n_blocks < 3) {
    return DecelMpcReason::kBlocksTooFew;
  }
  // block_sizes holds kMaxDecelNodes entries whatever N is.
  if (params.n_blocks > n_nodes || params.n_blocks > kMaxDecelNodes) {
    return DecelMpcReason::kParamsInvalid;
  }
  int block_sum = 0;
  for (int b = 0; b < params.n_blocks; ++b) {
    const int size = params.block_sizes[static_cast<std::size_t>(b)];
    if (size < 1) {
      return DecelMpcReason::kParamsInvalid;
    }
    block_sum += size;
  }
  if (block_sum != n_nodes) {
    return DecelMpcReason::kParamsInvalid;
  }
  // The catch node splits the blocks: none may span it, and the terminal
  // equality needs three after it. The rank self-check below cannot see the
  // second rule — pre-catch blocks supply rank too.
  {
    int start = 0;
    int after_catch = 0;
    for (int b = 0; b < params.n_blocks; ++b) {
      const int end = start + params.block_sizes[static_cast<std::size_t>(b)];
      if (start < n_pre && end > n_pre) {
        return DecelMpcReason::kBlocksAcrossCatch;
      }
      if (start >= n_pre) {
        ++after_catch;
      }
      start = end;
    }
    if (after_catch < 3) {
      return DecelMpcReason::kBlocksTooFew;
    }
  }
  if (params.jerk_weight.size() != 0) {
    if (params.jerk_weight.size() != nn) {
      return DecelMpcReason::kParamsInvalid;
    }
    for (Eigen::Index j = 0; j < nn; ++j) {
      if (!FinitePositive(params.jerk_weight[j])) {
        return DecelMpcReason::kParamsInvalid;
      }
    }
  }
  if (!FinitePositive(params.u_scale) || !rtc::IsFiniteNonNegative(params.w_delta) ||
      !rtc::IsFiniteNonNegative(params.w_perp) || !rtc::IsFiniteNonNegative(params.rho_tau) ||

      !InUnitInterval(params.eta_v) || !InUnitInterval(params.eta_tau) ||
      !rtc::IsFiniteNonNegative(params.m_q) || std::isnan(params.delta_tr) ||
      !(params.delta_tr > 0.0) || !FinitePositive(params.reference_rest_tol) ||
      !FinitePositive(params.solver.eps_abs) ||
      // A shifted previous solution meets the terminal equality only to
      // eps_abs; a rest tolerance at or below it rejects every warm cycle.
      !(params.reference_rest_tol > params.solver.eps_abs) ||
      !rtc::IsFiniteNonNegative(params.solver.eps_rel) || params.solver.max_iter < 1 ||
      params.solver.max_iter_in < 1) {
    return DecelMpcReason::kParamsInvalid;
  }
  if (!rtc::IsFiniteNonNegative(params.w_axis) || !rtc::IsFiniteNonNegative(params.w_v_par) ||
      !rtc::IsFiniteNonNegative(params.w_v_perp) || !rtc::IsFiniteNonNegative(params.rho_v) ||
      !rtc::IsFiniteNonNegative(params.v_rel_allow) || !(params.axis_theta_max > 0.0) ||
      !(params.axis_theta_max < std::numbers::pi)) {
    return DecelMpcReason::kParamsInvalid;
  }
  if (params.catch_terms && (n_pre < 1 || (params.rho_v > 0.0 && !(params.v_rel_allow > 0.0)))) {
    return DecelMpcReason::kParamsInvalid;
  }

  // Limits (finite first — max/min below would launder a NaN, NUM-7).
  const auto sized = [nn](const Eigen::VectorXd& x) { return x.size() == nn && x.allFinite(); };
  if (!sized(limits.q_min) || !sized(limits.q_max) || !sized(limits.qd_max) ||
      !sized(limits.tau_max) || !sized(limits.armature)) {
    return DecelMpcReason::kLimitsInvalid;
  }
  for (Eigen::Index j = 0; j < nn; ++j) {
    if (!(limits.q_min[j] <= limits.q_max[j]) || !(limits.qd_max[j] > 0.0) ||
        !(limits.tau_max[j] > 0.0) || !(limits.armature[j] >= 0.0)) {
      return DecelMpcReason::kLimitsInvalid;
    }
  }

  params_ = params;
  n_ = n;
  n_nodes_ = n_nodes;
  n_pre_ = n_pre;
  n_blocks_ = params.n_blocks;
  frame_ = catch_frame;
  w_perp_on_ = params.w_perp > 0.0;
  torque_on_ = params.rho_tau > 0.0;
  catch_on_ = params.catch_terms;
  vel_on_ = catch_on_ && (params.w_v_par > 0.0 || params.w_v_perp > 0.0);
  slack_v_on_ = catch_on_ && params.rho_v > 0.0;
  pos_on_ = false;
  h_modified_ = false;
  w_delta_scale_ = 1.0;
  nu_ = n * n_blocks_;
  nz_ = n * (n_blocks_ + n_nodes_) + (slack_v_on_ ? 1 : 0);
  n_eq_ = 2 * n;
  n_in_ = 5 * n * n_nodes_ + (slack_v_on_ ? 7 : 0);

  model_ = arm;
  // ADDED to whatever the model already carries (pinocchio's URDF parser
  // leaves it zero; a caller may have set it) — the limits field is the
  // motor-side term the model lacks, not a replacement.
  model_.armature = arm.armature + limits.armature;
  data_ = pinocchio::Data(model_);

  // Effective box: the margin never inverts a narrow (or locked) joint's range.
  q_lo_.resize(nn);
  q_hi_.resize(nn);
  v_hi_.resize(nn);
  inv_tau_.resize(nn);
  jerk_w_ = params.jerk_weight.size() == 0 ? Eigen::VectorXd::Ones(nn) : params.jerk_weight;
  for (Eigen::Index j = 0; j < nn; ++j) {
    const double m = std::min(params.m_q, 0.5 * (limits.q_max[j] - limits.q_min[j]));
    q_lo_[j] = limits.q_min[j] + m;
    q_hi_[j] = limits.q_max[j] - m;
    v_hi_[j] = params.eta_v * limits.qd_max[j];
    inv_tau_[j] = 1.0 / limits.tau_max[j];
  }

  // Grid. Node instants are k·Δ within each part, never a running sum, so the
  // uniform grid (n_pre = 0) keeps the exact values k·Δ_s it always had.
  for (int k = 0; k <= n_nodes_; ++k) {
    const auto i = static_cast<std::size_t>(k);
    t_node_[i] = k <= n_pre_ ? static_cast<double>(k) * params.dt_pre
                             : static_cast<double>(n_pre_) * params.dt_pre +
                                   static_cast<double>(k - n_pre_) * params.dt;
    if (k < n_nodes_) {
      dt_node_[i] = k < n_pre_ ? params.dt_pre : params.dt;
    }
  }
  {
    int k = 0;
    for (int b = 0; b < n_blocks_; ++b) {
      const int size = params.block_sizes[static_cast<std::size_t>(b)];
      // Σ_{k∈b} Δ_k/Δ_s — a block lies on one side of the catch node, so this
      // is its node count, times Δ_a/Δ_s before the catch.
      block_weight_[static_cast<std::size_t>(b)] =
          k < n_pre_ ? static_cast<double>(size) * (params.dt_pre / params.dt)
                     : static_cast<double>(size);
      for (int i = 0; i < size; ++i) {
        block_of_node_[static_cast<std::size_t>(k++)] = b;
      }
    }
  }

  // Stage gains ĝ_{m,k}[b]: the scalar triple integrator's response at node k
  // to jerk u_scale held over block b (all others zero), from rest.
  const Eigen::Index N1 = n_nodes_ + 1;
  const Eigen::Index nb = n_blocks_;
  gq_.setZero(N1, nb);
  gv_.setZero(N1, nb);
  ga_.setZero(N1, nb);
  for (Eigen::Index b = 0; b < nb; ++b) {
    double q = 0.0;
    double v = 0.0;
    double a = 0.0;
    for (int k = 0; k < n_nodes_; ++k) {
      const double dt = dt_node_[static_cast<std::size_t>(k)];
      const double u =
          block_of_node_[static_cast<std::size_t>(k)] == static_cast<int>(b) ? params.u_scale : 0.0;
      q += dt * v + 0.5 * dt * dt * a + dt * dt * dt * u / 6.0;
      v += dt * a + 0.5 * dt * dt * u;
      a += dt * u;
      gq_(k + 1, b) = q;
      gv_(k + 1, b) = v;
      ga_(k + 1, b) = a;
    }
  }

  // Dense stage matrices (reference assembly and the rank self-check).
  g_dense_.setZero(3 * nn * N1, nu_);
  for (Eigen::Index k = 0; k < N1; ++k) {
    for (Eigen::Index b = 0; b < nb; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        g_dense_(3 * nn * k + j, b * nn + j) = gq_(k, b);
        g_dense_(3 * nn * k + nn + j, b * nn + j) = gv_(k, b);
        g_dense_(3 * nn * k + 2 * nn + j, b * nn + j) = ga_(k, b);
      }
    }
  }

  // Constant Hessians.
  h_kin_.setZero(nz_, nz_);
  for (Eigen::Index b = 0; b < nb; ++b) {
    const double n_b = block_weight_[static_cast<std::size_t>(b)];
    for (Eigen::Index j = 0; j < nn; ++j) {
      h_kin_(b * nn + j, b * nn + j) = jerk_w_[j] * n_b;
    }
  }
  h_main_ = h_kin_;
  h_delta_.setZero(nz_, nz_);
  if (params.w_delta > 0.0) {
    for (Eigen::Index k = 1; k < N1; ++k) {
      for (Eigen::Index b = 0; b < nb; ++b) {
        for (Eigen::Index c = 0; c < nb; ++c) {
          const double w = params.w_delta * gq_(k, b) * gq_(k, c);
          for (Eigen::Index j = 0; j < nn; ++j) {
            h_main_(b * nn + j, c * nn + j) += w;
            h_delta_(b * nn + j, c * nn + j) += w;
          }
        }
      }
    }
  }

  // QP storage: both problems keep their constant parts from here on.
  qp_pre_.Init(nz_, n_eq_, n_in_);
  qp_main_.Init(nz_, n_eq_, n_in_);
  for (tsid::QPData* qp : {&qp_pre_, &qp_main_}) {
    qp->n_vars = nz_;
    qp->n_eq = n_eq_;
    qp->n_ineq = n_in_;
  }
  AssembleConstant(qp_pre_, false);
  AssembleConstant(qp_main_, true);

  // Rank self-check of the assembled terminal equality (2n × nB).
  {
    const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(qp_main_.A);
    terminal_rank_ = static_cast<int>(qr.rank());
  }
  if (terminal_rank_ != n_eq_) {
    return DecelMpcReason::kTerminalRankDeficient;
  }

  solver_.Init(nz_, n_eq_, n_in_, params.solver);

  // Workspace.
  q0_.setZero(nn);
  v0_.setZero(nn);
  a0_.setZero(nn);
  qf_.setZero(nn, N1);
  vf_.setZero(nn, N1);
  af_.setZero(nn, N1);
  qr_.setZero(nn, N1);
  vr_.setZero(nn, N1);
  ar_.setZero(nn, N1);
  tau_bar_.setZero(nn, N1);
  d_scaled_.setZero(nn, 3 * nn * N1);
  c_tilde_.setZero(nn * n_nodes_);
  j6_.setZero(6, nn);
  p_perp_.setIdentity();
  l_perp_.setZero(3, nn * N1);
  r_perp_.setZero(3 * N1);
  work_n_.setZero(nn);
  work_n2_.setZero(nn);
  work_n3_.setZero(nn);
  work_row_.setZero(3);
  work_dg_.setZero(nn, nu_);
  z_.setZero(nz_);
  q_out_.setZero(nn, N1);
  v_out_.setZero(nn, N1);
  a_out_.setZero(nn, N1);
  tau_lin_.setZero(nn * n_nodes_);
  jv_c_.setZero(3, nn);
  jw_c_.setZero(3, nn);
  hv_c_.setZero(3, nn);
  la_c_.setZero(3, nn);
  dv_c_.setZero(3, nn);
  wa_q_.setZero(3, nn);
  wa_v_.setZero(3, nn);
  m_qq_.setZero(nn, nn);
  m_qv_.setZero(nn, nn);
  m_vv_.setZero(nn, nn);
  blk_.setZero(nn, nn);
  c_q_.setZero(nn);
  c_v_.setZero(nn);
  l_dense_.setZero(3, nu_);
  wl_dense_.setZero(3, nu_);
  perp_l_.setZero(3, nn);

  initialized_ = true;
  return DecelMpcReason::kNone;
}

void DecelMpc::ResizeResult(DecelMpcResult& result) const {
  const Eigen::Index nn = n_;
  result.q.setZero(nn, n_nodes_ + 1);
  result.qd.setZero(nn, n_nodes_ + 1);
  result.qdd.setZero(nn, n_nodes_ + 1);
  result.u.setZero(nn, n_nodes_);
  result.slack.setZero(nn, n_nodes_);
  result.tau_ratio.setZero(nn, n_nodes_);
  result.valid = false;
  result.reason = initialized_ ? DecelMpcReason::kNone : DecelMpcReason::kNotInitialized;
}

double DecelMpc::NodeTime(int k) const noexcept {
  if (!initialized_ || k < 0 || k > n_nodes_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return t_node_[static_cast<std::size_t>(k)];
}

double DecelMpc::StageGain(int m, int k, int b) const noexcept {
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

// ── Assembly ─────────────────────────────────────────────────────────────────
// Row layout of C (r = (k−1)·n + j, k = 1..N):
//   [0,   nN)  position ∪ trust region      G_{q,k}
//   [nN, 2nN)  velocity                      G_{q̇,k}
//   [2nN,3nN)  torque upper  D̃_k G_k z − s ≤ η − c̃
//   [3nN,4nN)  torque lower  D̃_k G_k z + s ≥ −η − c̃
//   [4nN,5nN)  slack         s ≥ 0
// and, with the velocity slack (the last variable s_v, rows scaled by 1/v_allow):
//   [5nN,   +3)  −L̃ z − s_v ≤ 1 − d̃      L̃ = (ĝ_q⊗H_v + ĝ_v⊗J_v)/v_allow at k_c
//   [5nN+3, +3)  −L̃ z + s_v ≥ −1 − d̃     d̃ = (v̂_b − v_C(z = 0))/v_allow
//   [5nN+6]      s_v ≥ 0

void DecelMpc::AssembleConstant(tsid::QPData& qp, bool main) noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index nN = nn * n_nodes_;
  const Eigen::Index nb = n_blocks_;
  const Eigen::Index N = n_nodes_;

  qp.H = main ? h_main_ : h_kin_;
  qp.g.setZero();
  qp.A.setZero();
  qp.C.setZero();
  for (Eigen::Index b = 0; b < nb; ++b) {
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp.A(j, b * nn + j) = gv_(N, b);
      qp.A(nn + j, b * nn + j) = ga_(N, b);
    }
  }
  for (Eigen::Index k = 1; k <= N; ++k) {
    const Eigen::Index o = (k - 1) * nn;
    for (Eigen::Index b = 0; b < nb; ++b) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        qp.C(o + j, b * nn + j) = gq_(k, b);
        qp.C(nN + o + j, b * nn + j) = gv_(k, b);
      }
    }
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp.C(2 * nN + o + j, nu_ + o + j) = -1.0;
      qp.C(3 * nN + o + j, nu_ + o + j) = 1.0;
      qp.C(4 * nN + o + j, nu_ + o + j) = 1.0;
    }
  }
  AssembleSlackVConstant(qp);
}

void DecelMpc::AssembleSlackVConstant(tsid::QPData& qp) const noexcept {
  if (!slack_v_on_) {
    return;
  }
  const Eigen::Index base = 5 * static_cast<Eigen::Index>(n_) * n_nodes_;
  const Eigen::Index sv = nz_ - 1;
  for (Eigen::Index i = 0; i < 3; ++i) {
    qp.C(base + i, sv) = -1.0;
    qp.C(base + 3 + i, sv) = 1.0;
  }
  qp.C(base + 6, sv) = 1.0;
}

void DecelMpc::AssembleConstantDense(tsid::QPData& qp, bool main) noexcept {
  // The "before" of O-1/O-2: every matrix rebuilt from the dense Γ_k·E.
  const Eigen::Index nn = n_;
  const Eigen::Index nN = nn * n_nodes_;
  const Eigen::Index N = n_nodes_;
  const Eigen::Index nb = n_blocks_;

  qp.H.setZero();
  for (Eigen::Index b = 0; b < nb; ++b) {
    const double n_b = block_weight_[static_cast<std::size_t>(b)];
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp.H(b * nn + j, b * nn + j) = jerk_w_[j] * n_b;
    }
  }
  if (main && params_.w_delta > 0.0) {
    const double sw = std::sqrt(params_.w_delta * w_delta_scale_);
    for (Eigen::Index k = 1; k <= N; ++k) {
      work_dg_ = g_dense_.middleRows(3 * nn * k, nn);
      work_dg_ *= sw;
      qp.H.topLeftCorner(nu_, nu_).noalias() += work_dg_.transpose() * work_dg_;
    }
  }
  qp.g.setZero();
  qp.A.setZero();
  qp.A.topLeftCorner(nn, nu_) = g_dense_.middleRows(3 * nn * N + nn, nn);
  qp.A.block(nn, 0, nn, nu_) = g_dense_.middleRows(3 * nn * N + 2 * nn, nn);
  qp.C.setZero();
  for (Eigen::Index k = 1; k <= N; ++k) {
    const Eigen::Index o = (k - 1) * nn;
    qp.C.block(o, 0, nn, nu_) = g_dense_.middleRows(3 * nn * k, nn);
    qp.C.block(nN + o, 0, nn, nu_) = g_dense_.middleRows(3 * nn * k + nn, nn);
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp.C(2 * nN + o + j, nu_ + o + j) = -1.0;
      qp.C(3 * nN + o + j, nu_ + o + j) = 1.0;
      qp.C(4 * nN + o + j, nu_ + o + j) = 1.0;
    }
  }
  AssembleSlackVConstant(qp);
}

void DecelMpc::FreeResponse() noexcept {
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

bool DecelMpc::Linearize() noexcept {
  const Eigen::Index nn = n_;
  for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
    const Eigen::Index col = 3 * nn * k;
    if (torque_on_) {
      if (!LinearizeTorqueAt(model_, data_, qr_.col(k), vr_.col(k), ar_.col(k), tau_bar_.col(k),
                             d_scaled_.middleCols(col, 3 * nn))) {
        return false;
      }
      for (Eigen::Index i = 0; i < nn; ++i) {
        d_scaled_.block(i, col, 1, 3 * nn) *= inv_tau_[i];
      }
      // c̃_k = τ̄_k/τ_max + D̃_k (Φ_k x_0 − x̄_k)
      const Eigen::Index o = (k - 1) * nn;
      work_n_ = qf_.col(k) - qr_.col(k);
      work_n2_ = vf_.col(k) - vr_.col(k);
      work_n3_ = af_.col(k) - ar_.col(k);
      c_tilde_.segment(o, nn) = tau_bar_.col(k).cwiseProduct(inv_tau_);
      c_tilde_.segment(o, nn).noalias() += d_scaled_.block(0, col, nn, nn) * work_n_;
      c_tilde_.segment(o, nn).noalias() += d_scaled_.block(0, col + nn, nn, nn) * work_n2_;
      c_tilde_.segment(o, nn).noalias() += d_scaled_.block(0, col + 2 * nn, nn, nn) * work_n3_;
    }
    // The stop path starts at the catch node: w_⊥ skips the pre-catch nodes.
    if (w_perp_on_ && k >= n_pre_) {
      j6_.setZero();
      pinocchio::computeFrameJacobian(model_, data_, qr_.col(k), frame_,
                                      pinocchio::LOCAL_WORLD_ALIGNED, j6_);
      // L_k = P_⊥ J_v,  r_k = P_⊥ (p(q̄_k) + J_v (Φ_k x_0 − q̄_k)_q − p_c)
      l_perp_.middleCols(nn * k, nn).noalias() = p_perp_ * j6_.topRows(3);
      work_n_ = qf_.col(k) - qr_.col(k);
      work_row_ = data_.oMf[frame_].translation() - p_c_;
      work_row_.noalias() += j6_.topRows(3) * work_n_;
      r_perp_.segment(3 * k, 3).noalias() = p_perp_ * work_row_;
    }
  }
  return true;
}

void DecelMpc::AssembleTorqueRows() noexcept {
  // O-1: the torque block for (node k, block b) is Σ_m ĝ_{m,k}[b]·D̃^m_k —
  // three n×n matrices combined, never the n×3n by 3n×nB product. Blocks that
  // start at or after node k have ĝ ≡ 0 (causality) and stay the zeros Init
  // wrote.
  const Eigen::Index nn = n_;
  const Eigen::Index nN = nn * n_nodes_;
  for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
    const Eigen::Index col = 3 * nn * k;
    const Eigen::Index o = (k - 1) * nn;
    const Eigen::Index last_block = block_of_node_[static_cast<std::size_t>(k - 1)];
    for (Eigen::Index b = 0; b <= last_block; ++b) {
      qp_main_.C.block(2 * nN + o, b * nn, nn, nn) =
          gq_(k, b) * d_scaled_.block(0, col, nn, nn) +
          gv_(k, b) * d_scaled_.block(0, col + nn, nn, nn) +
          ga_(k, b) * d_scaled_.block(0, col + 2 * nn, nn, nn);
      qp_main_.C.block(3 * nN + o, b * nn, nn, nn) = qp_main_.C.block(2 * nN + o, b * nn, nn, nn);
    }
  }
}

void DecelMpc::AssembleTorqueRowsDense() noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index nN = nn * n_nodes_;
  for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
    const Eigen::Index o = (k - 1) * nn;
    work_dg_.noalias() =
        d_scaled_.middleCols(3 * nn * k, 3 * nn) * g_dense_.middleRows(3 * nn * k, 3 * nn);
    qp_main_.C.block(2 * nN + o, 0, nn, nu_) = work_dg_;
    qp_main_.C.block(3 * nN + o, 0, nn, nu_) = work_dg_;
  }
}

void DecelMpc::AssemblePerp() noexcept {
  // ½ w_⊥ Σ_k ‖r_k + L_k (ĝ_{q,k}ᵀ ⊗ I) z‖² — a position-only node term at
  // every node of the STOP segment (k ≥ k_c; all of 1..N when n_pre = 0),
  // through the same routine the catch terms use. The line through p_c is
  // where the hand stops, not how it gets there: on the pre-catch nodes the
  // term would pull the whole approach onto that line.
  const Eigen::Index nn = n_;
  const Eigen::Matrix3d w = params_.w_perp * Eigen::Matrix3d::Identity();
  for (Eigen::Index k = std::max<Eigen::Index>(1, n_pre_); k <= n_nodes_; ++k) {
    perp_l_ = l_perp_.middleCols(nn * k, nn);
    const Eigen::Vector3d r = r_perp_.segment<3>(3 * k);
    if (params_.reference_assembly) {
      AccumulateNodeTermDense(k, perp_l_, nullptr, w, r);
    } else {
      AccumulateNodeTerm(k, perp_l_, nullptr, w, r);
    }
  }
}

// ── Catch terms (E1-F07) ─────────────────────────────────────────────────────

DecelMpcReason LinearizeCatchAt(
    const pinocchio::Model& model, pinocchio::Data& data, pinocchio::FrameIndex frame,
    const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Vector3d& a_d, double axis_theta_max, bool with_axis, bool with_velocity,
    Eigen::MatrixXd& j6_work, Eigen::Matrix<double, 3, Eigen::Dynamic>& j_v,
    Eigen::Matrix<double, 3, Eigen::Dynamic>& j_w, Eigen::Matrix<double, 3, Eigen::Dynamic>& l_a,
    Eigen::Matrix<double, 3, Eigen::Dynamic>& h_v,
    Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_work, CatchLinearization& out) noexcept {
  const Eigen::Index n = model.nv;
  if (model.nq != model.nv || frame >= model.frames.size() || q.size() != n || v.size() != n ||
      j6_work.rows() != 6 || j6_work.cols() != n || j_v.cols() != n || j_w.cols() != n ||
      l_a.cols() != n || h_v.cols() != n || dv_work.cols() != n) {
    return DecelMpcReason::kDimMismatch;
  }
  // The velocity derivative runs its own forward pass, so it goes before the
  // frame Jacobian (which leaves oMf for the pose below).
  if (with_velocity) {
    pinocchio::computeForwardKinematicsDerivatives(model, data, q, v);
    const pinocchio::Frame& f = model.frames[frame];
    h_v.setZero();
    dv_work.setZero();
    pinocchio::getPointVelocityDerivatives(model, data, f.parentJoint, f.placement,
                                           pinocchio::LOCAL_WORLD_ALIGNED, h_v, dv_work);
  }
  j6_work.setZero();
  pinocchio::computeFrameJacobian(model, data, q, frame, pinocchio::LOCAL_WORLD_ALIGNED, j6_work);
  j_v = j6_work.topRows(3);
  j_w = j6_work.bottomRows(3);
  out.p = data.oMf[frame].translation();
  out.z = data.oMf[frame].rotation().col(2);
  if (with_axis) {
    const rtc::math::se3::AxisAlignErrorResult e = rtc::math::se3::AxisAlignError(out.z, a_d);
    rtc::math::se3::AxisAlignJacobianResult j = rtc::math::se3::AxisAlignJacobian(out.z, a_d);
    if (!e.IsValid() || !j.IsValid() ||
        e.region == rtc::math::se3::AxisAlignRegion::kAntiparallelDeadband ||
        j.region == rtc::math::se3::AxisAlignRegion::kJacobianCapped ||
        !(e.error.norm() <= axis_theta_max)) {
      return DecelMpcReason::kCatchAxisOutOfRange;
    }
    if (j.region == rtc::math::se3::AxisAlignRegion::kAlignedDeadband) {
      j.jacobian.noalias() = rtc::math::se3::hat(a_d) * rtc::math::se3::hat(out.z);
    }
    l_a.noalias() = j.jacobian * j_w;
    out.e_a = e.error;
  }
  return DecelMpcReason::kNone;
}

DecelMpcReason DecelMpc::LinearizeCatch() noexcept {
  const Eigen::Index kc = n_pre_;
  const bool with_axis = params_.w_axis > 0.0;
  const bool with_velocity = vel_on_ || slack_v_on_;
  CatchLinearization lin;
  const DecelMpcReason why = LinearizeCatchAt(model_, data_, frame_, qr_.col(kc), vr_.col(kc), a_d_,
                                              params_.axis_theta_max, with_axis, with_velocity, j6_,
                                              jv_c_, jw_c_, la_c_, hv_c_, dv_c_, lin);
  if (why != DecelMpcReason::kNone) {
    return why;
  }
  work_n_ = qf_.col(kc) - qr_.col(kc);  // free response − reference, position part

  // Position: r = p(q̄) + J_v (q_f − q̄) − p̂_b.
  r_pos_ = lin.p - p_b_;
  r_pos_.noalias() += jv_c_ * work_n_;
  // Approach axis: r = e_a(q̄) + L_a (q_f − q̄).
  if (with_axis) {
    r_axis_ = lin.e_a;
    r_axis_.noalias() += la_c_ * work_n_;
  }
  // Velocity: v_C(z = 0) = J_v v_f + H_v (q_f − q̄); weight by the ball's travel.
  if (with_velocity) {
    v_c0_.noalias() = jv_c_ * vf_.col(kc);
    v_c0_.noalias() += hv_c_ * work_n_;
    const double speed = v_b_.norm();
    if (speed > kMinBallSpeed) {
      const Eigen::Vector3d d = v_b_ / speed;
      const Eigen::Matrix3d dd = d * d.transpose();
      w_v_ = params_.w_v_par * dd + params_.w_v_perp * (Eigen::Matrix3d::Identity() - dd);
    } else {
      w_v_ = params_.w_v_perp * Eigen::Matrix3d::Identity();
    }
  }
  return DecelMpcReason::kNone;
}

void DecelMpc::AccumulateNodeTerm(Eigen::Index k,
                                  const Eigen::Matrix<double, 3, Eigen::Dynamic>& a_q,
                                  const Eigen::Matrix<double, 3, Eigen::Dynamic>* a_v,
                                  const Eigen::Matrix3d& w, const Eigen::Vector3d& r) noexcept {
  // ½ ‖r + (ĝ_{q,k}ᵀ ⊗ A_q + ĝ_{v,k}ᵀ ⊗ A_v) z‖²_W. With M_b = ĝ_q[b] A_q + ĝ_v[b] A_v:
  //   H(b,c) += M_bᵀ W M_c,   g(b) += M_bᵀ W r.
  // The cross product A_qᵀ W A_v is NOT symmetric, so an off-diagonal block is
  // not either: H is kept exactly symmetric by writing H(c,b) = H(b,c)ᵀ, and
  // only the diagonal blocks (symmetric by construction) are mirrored in place.
  const Eigen::Index nn = n_;
  const auto mirror = [nn](Eigen::MatrixXd& m) {
    for (Eigen::Index c = 0; c < nn; ++c) {
      for (Eigen::Index row = c + 1; row < nn; ++row) {
        m(c, row) = m(row, c);
      }
    }
  };
  wa_q_.noalias() = w * a_q;
  m_qq_.noalias() = a_q.transpose() * wa_q_;
  mirror(m_qq_);
  c_q_.noalias() = wa_q_.transpose() * r;  // W symmetric: (W A)ᵀ r = Aᵀ W r
  const bool with_v = a_v != nullptr;
  if (with_v) {
    wa_v_.noalias() = w * (*a_v);
    m_vv_.noalias() = a_v->transpose() * wa_v_;
    mirror(m_vv_);
    m_qv_.noalias() = a_q.transpose() * wa_v_;
    c_v_.noalias() = wa_v_.transpose() * r;
  }
  const Eigen::Index last_block = block_of_node_[static_cast<std::size_t>(k - 1)];
  for (Eigen::Index b = 0; b <= last_block; ++b) {
    const double gqb = gq_(k, b);
    const double gvb = gv_(k, b);
    for (Eigen::Index c = b; c <= last_block; ++c) {
      const double gqc = gq_(k, c);
      const double gvc = gv_(k, c);
      blk_ = (gqb * gqc) * m_qq_;
      if (with_v) {
        blk_ += (gqb * gvc) * m_qv_;
        blk_ += (gvb * gqc) * m_qv_.transpose();
        blk_ += (gvb * gvc) * m_vv_;
      }
      if (c == b) {
        mirror(blk_);
        qp_main_.H.block(b * nn, b * nn, nn, nn) += blk_;
      } else {
        qp_main_.H.block(b * nn, c * nn, nn, nn) += blk_;
        qp_main_.H.block(c * nn, b * nn, nn, nn) += blk_.transpose();
      }
    }
    qp_main_.g.segment(b * nn, nn) += gqb * c_q_;
    if (with_v) {
      qp_main_.g.segment(b * nn, nn) += gvb * c_v_;
    }
  }
}

void DecelMpc::AccumulateNodeTermDense(Eigen::Index k,
                                       const Eigen::Matrix<double, 3, Eigen::Dynamic>& a_q,
                                       const Eigen::Matrix<double, 3, Eigen::Dynamic>* a_v,
                                       const Eigen::Matrix3d& w,
                                       const Eigen::Vector3d& r) noexcept {
  // The oracle: L from the dense stage matrices, then LᵀWL — no block structure.
  const Eigen::Index nn = n_;
  l_dense_.noalias() = a_q * g_dense_.middleRows(3 * nn * k, nn);
  if (a_v != nullptr) {
    l_dense_.noalias() += (*a_v) * g_dense_.middleRows(3 * nn * k + nn, nn);
  }
  wl_dense_.noalias() = w * l_dense_;
  qp_main_.H.topLeftCorner(nu_, nu_).noalias() += l_dense_.transpose() * wl_dense_;
  qp_main_.g.head(nu_).noalias() += wl_dense_.transpose() * r;
}

void DecelMpc::AssembleCatch() noexcept {
  const Eigen::Index kc = n_pre_;
  const bool dense = params_.reference_assembly;
  const auto add = [this, kc, dense](const Eigen::Matrix<double, 3, Eigen::Dynamic>& a_q,
                                     const Eigen::Matrix<double, 3, Eigen::Dynamic>* a_v,
                                     const Eigen::Matrix3d& w, const Eigen::Vector3d& r) noexcept {
    if (dense) {
      AccumulateNodeTermDense(kc, a_q, a_v, w, r);
    } else {
      AccumulateNodeTerm(kc, a_q, a_v, w, r);
    }
  };
  if (pos_on_) {
    add(jv_c_, nullptr, w_p_, r_pos_);
  }
  if (params_.w_axis > 0.0) {
    const Eigen::Matrix3d w = params_.w_axis * Eigen::Matrix3d::Identity();
    add(la_c_, nullptr, w, r_axis_);
  }
  if (vel_on_) {
    // v_C − γ_ref v̂_b, with v_C = v_c0 + (ĝ_q ⊗ H_v + ĝ_v ⊗ J_v) z.
    const Eigen::Vector3d r = v_c0_ - gamma_ref_ * v_b_;
    add(hv_c_, &jv_c_, w_v_, r);
  }
}

void DecelMpc::AssembleSlackVRows() noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index kc = n_pre_;
  const Eigen::Index base = 5 * nn * n_nodes_;
  const double inv = 1.0 / params_.v_rel_allow;
  const Eigen::Index last_block = block_of_node_[static_cast<std::size_t>(kc - 1)];
  for (Eigen::Index b = 0; b <= last_block; ++b) {
    qp_main_.C.block(base, b * nn, 3, nn) =
        (-inv * gq_(kc, b)) * hv_c_ + (-inv * gv_(kc, b)) * jv_c_;
    qp_main_.C.block(base + 3, b * nn, 3, nn) = qp_main_.C.block(base, b * nn, 3, nn);
  }
}

void DecelMpc::EvaluateCatch(DecelMpcResult& out) noexcept {
  const Eigen::Index kc = n_pre_;
  j6_.setZero();
  pinocchio::computeFrameJacobian(model_, data_, q_out_.col(kc), frame_,
                                  pinocchio::LOCAL_WORLD_ALIGNED, j6_);
  const Eigen::Vector3d z = data_.oMf[frame_].rotation().col(2);
  Eigen::Vector3d v_c;
  v_c.noalias() = j6_.topRows(3) * v_out_.col(kc);
  out.catch_pos_err = data_.oMf[frame_].translation() - p_b_;
  out.catch_axis_err = std::atan2(z.cross(a_d_).norm(), z.dot(a_d_));
  out.catch_v_rel = v_b_ - v_c;
  const double speed = v_b_.norm();
  out.catch_gamma = speed > kMinBallSpeed ? v_b_.dot(v_c) / (speed * speed) : 0.0;
  out.slack_v = slack_v_on_ ? z_[nz_ - 1] : 0.0;
  out.catch_evaluated = true;
}

bool DecelMpc::AssembleBounds(tsid::QPData& qp, bool main) noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index nN = nn * n_nodes_;
  const Eigen::Index N = n_nodes_;
  const bool torque_rows = main && torque_on_;
  const double eta = params_.eta_tau;
  const double tr = params_.delta_tr;
  for (Eigen::Index j = 0; j < nn; ++j) {
    qp.b[j] = -vf_(j, N);
    qp.b[nn + j] = -af_(j, N);
  }
  for (Eigen::Index k = 1; k <= N; ++k) {
    const Eigen::Index o = (k - 1) * nn;
    for (Eigen::Index j = 0; j < nn; ++j) {
      const Eigen::Index r = o + j;
      double lo = q_lo_[j];
      double hi = q_hi_[j];
      if (main) {
        // Every operand was checked finite before this point, so max/min
        // cannot launder a NaN into a vanished trust region (NUM-7).
        lo = std::max(lo, qr_(j, k) - tr);
        hi = std::min(hi, qr_(j, k) + tr);
        if (lo > hi) {
          return false;
        }
      }
      qp.l[r] = lo - qf_(j, k);
      qp.u[r] = hi - qf_(j, k);
      qp.l[nN + r] = -v_hi_[j] - vf_(j, k);
      qp.u[nN + r] = v_hi_[j] - vf_(j, k);
      if (torque_rows) {
        qp.l[2 * nN + r] = -kInf;
        qp.u[2 * nN + r] = eta - c_tilde_[r];
        qp.l[3 * nN + r] = -eta - c_tilde_[r];
        qp.u[3 * nN + r] = kInf;
        qp.l[4 * nN + r] = 0.0;
        qp.u[4 * nN + r] = kInf;
      } else {
        qp.l[2 * nN + r] = -kInf;
        qp.u[2 * nN + r] = kInf;
        qp.l[3 * nN + r] = -kInf;
        qp.u[3 * nN + r] = kInf;
        // Rows off: pin the slack at 0 (it would otherwise be a free,
        // cost-less direction).
        qp.l[4 * nN + r] = 0.0;
        qp.u[4 * nN + r] = 0.0;
      }
    }
  }
  if (slack_v_on_) {
    const Eigen::Index base = 5 * nN;
    if (main) {
      const double inv = 1.0 / params_.v_rel_allow;
      for (Eigen::Index i = 0; i < 3; ++i) {
        const double d = (v_b_[i] - v_c0_[i]) * inv;
        qp.l[base + i] = -kInf;
        qp.u[base + i] = 1.0 - d;
        qp.l[base + 3 + i] = -1.0 - d;
        qp.u[base + 3 + i] = kInf;
      }
      qp.l[base + 6] = 0.0;
      qp.u[base + 6] = kInf;
    } else {
      // The pre-solve has no catch node: rows off, slack pinned.
      for (Eigen::Index i = 0; i < 6; ++i) {
        qp.l[base + i] = -kInf;
        qp.u[base + i] = kInf;
      }
      qp.l[base + 6] = 0.0;
      qp.u[base + 6] = 0.0;
    }
  }
  return true;
}

void DecelMpc::AssembleGradient(tsid::QPData& qp, bool main) noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index nb = n_blocks_;
  qp.g.setZero();
  if (!main) {
    return;
  }
  if (params_.w_delta > 0.0) {
    const double w_delta = params_.w_delta * w_delta_scale_;
    for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
      const Eigen::Index last_block = block_of_node_[static_cast<std::size_t>(k - 1)];
      for (Eigen::Index b = 0; b <= last_block && b < nb; ++b) {
        const double w = w_delta * gq_(k, b);
        for (Eigen::Index j = 0; j < nn; ++j) {
          qp.g[b * nn + j] += w * (qf_(j, k) - qr_(j, k));
        }
      }
    }
  }
  if (torque_on_) {
    qp.g.segment(nu_, nn * n_nodes_).setConstant(params_.rho_tau);
  }
  if (slack_v_on_) {
    qp.g[nz_ - 1] = params_.rho_v;
  }
}

DecelMpcReason DecelMpc::RunQp(tsid::QPData& qp, int& status, int& iterations) noexcept {
  const tsid::SolveResult& res = solver_.Solve(qp);
  status = res.status;
  iterations = res.iterations;
  if (!res.converged) {
    // The wrapper reports non-finite iterates as not converged; kept as its
    // own reason because a pre-solve answer becomes x̄ and feeds the max/min
    // of the trust-region bounds, where a NaN would drop the row (NUM-7).
    ResetSolver();
    return res.non_finite ? DecelMpcReason::kSolutionNonFinite : DecelMpcReason::kQpFailed;
  }
  solver_warm_ = true;
  z_ = res.x_opt.head(nz_);
  return DecelMpcReason::kNone;
}

void DecelMpc::ResetSolver() noexcept {
  solver_.ResetWarmStart();
  solver_warm_ = false;
}

void DecelMpc::TrajectoryFromZ() noexcept {
  const Eigen::Index nn = n_;
  const Eigen::Index nb = n_blocks_;
  for (Eigen::Index k = 0; k <= n_nodes_; ++k) {
    for (Eigen::Index j = 0; j < nn; ++j) {
      double q = qf_(j, k);
      double v = vf_(j, k);
      double a = af_(j, k);
      for (Eigen::Index b = 0; b < nb; ++b) {
        const double zb = z_[b * nn + j];
        q += gq_(k, b) * zb;
        v += gv_(k, b) * zb;
        a += ga_(k, b) * zb;
      }
      q_out_(j, k) = q;
      v_out_(j, k) = v;
      a_out_(j, k) = a;
    }
  }
}

// ── Solve (RT) ───────────────────────────────────────────────────────────────

bool DecelMpc::Solve(const DecelMpcInput& in, DecelMpcResult& out) noexcept {
  using Clock = std::chrono::steady_clock;
  out.valid = false;
  out.presolved = false;
  out.presolve_us = 0.0;
  out.linearize_us = 0.0;
  out.condense_us = 0.0;
  out.solve_us = 0.0;
  out.iterations = 0;
  out.presolve_iterations = 0;
  out.qp_status = -1;
  out.cold_retried = false;
  const auto fail = [&out](DecelMpcReason r) noexcept {
    out.reason = r;
    out.valid = false;
    return false;
  };

  if (!initialized_) {
    return fail(DecelMpcReason::kNotInitialized);
  }
  const Eigen::Index nn = n_;
  const Eigen::Index N = n_nodes_;
  const Eigen::Index N1 = N + 1;

  // Shapes.
  if (out.q.rows() != nn || out.q.cols() != N1 || out.qd.rows() != nn || out.qd.cols() != N1 ||
      out.qdd.rows() != nn || out.qdd.cols() != N1 || out.u.rows() != nn || out.u.cols() != N ||
      out.slack.rows() != nn || out.slack.cols() != N || out.tau_ratio.rows() != nn ||
      out.tau_ratio.cols() != N) {
    return fail(DecelMpcReason::kDimMismatch);
  }
  if (in.q0.size() != nn || in.qd0.size() != nn || in.qdd0.size() != nn) {
    return fail(DecelMpcReason::kDimMismatch);
  }
  if (in.reference_valid &&
      (in.q_ref.rows() != nn || in.q_ref.cols() != N1 || in.qd_ref.rows() != nn ||
       in.qd_ref.cols() != N1 || in.qdd_ref.rows() != nn || in.qdd_ref.cols() != N1)) {
    return fail(DecelMpcReason::kDimMismatch);
  }

  // Finiteness BEFORE any bound is formed (NUM-7).
  if (!in.q0.allFinite() || !in.qd0.allFinite() || !in.qdd0.allFinite()) {
    return fail(DecelMpcReason::kNonFinite);
  }
  if (in.reference_valid &&
      (!in.q_ref.allFinite() || !in.qd_ref.allFinite() || !in.qdd_ref.allFinite())) {
    return fail(DecelMpcReason::kNonFinite);
  }
  if (w_perp_on_) {
    if (!in.p_c.allFinite() || !in.d_hat.allFinite()) {
      return fail(DecelMpcReason::kNonFinite);
    }
    if (!(std::abs(in.d_hat.norm() - 1.0) <= kUnitTolerance)) {
      return fail(DecelMpcReason::kDirectionNotUnit);
    }
  }

  if (!std::isfinite(in.w_delta_scale)) {
    return fail(DecelMpcReason::kNonFinite);
  }
  if (!(in.w_delta_scale >= 0.0 && in.w_delta_scale <= 1.0)) {
    return fail(DecelMpcReason::kInputOutOfRange);
  }
  Eigen::Matrix3d w_p_sym = Eigen::Matrix3d::Zero();
  if (catch_on_) {
    if (!in.p_b.allFinite() || !in.w_p.allFinite() || !in.a_d.allFinite() || !in.v_b.allFinite() ||
        !std::isfinite(in.gamma_ref)) {
      return fail(DecelMpcReason::kNonFinite);
    }
    if (!(std::abs(in.a_d.norm() - 1.0) <= kUnitTolerance)) {
      return fail(DecelMpcReason::kDirectionNotUnit);
    }
    if (!(in.gamma_ref > 0.0 && in.gamma_ref <= 1.0)) {
      return fail(DecelMpcReason::kInputOutOfRange);
    }
    // W_p: symmetric and PSD to rounding (a weight assembled as V·diag·Vᵀ is
    // neither exactly). The symmetrised matrix is what the term uses.
    const double w_scale = in.w_p.cwiseAbs().maxCoeff();
    if ((in.w_p - in.w_p.transpose()).cwiseAbs().maxCoeff() > kWeightRelTolerance * w_scale) {
      return fail(DecelMpcReason::kInputOutOfRange);
    }
    w_p_sym = 0.5 * (in.w_p + in.w_p.transpose());
    if (w_scale > 0.0) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;  // fixed size: no heap
      es.compute(w_p_sym, Eigen::EigenvaluesOnly);
      if (es.info() != Eigen::Success ||
          !(es.eigenvalues().minCoeff() >= -kWeightRelTolerance * w_p_sym.trace())) {
        return fail(DecelMpcReason::kInputOutOfRange);
      }
    }
    // The pre-solve is a stop problem with no catch point (header note).
    if (!in.reference_valid) {
      return fail(DecelMpcReason::kReferenceRequired);
    }
  }

  // Entry state inside the core's own box (header note).
  for (Eigen::Index j = 0; j < nn; ++j) {
    // A rounding-level slack so a locked joint (q_lo == q_hi) accepts its
    // own value.
    if (in.q0[j] < q_lo_[j] - kBoxRoundingSlack || in.q0[j] > q_hi_[j] + kBoxRoundingSlack ||
        std::abs(in.qd0[j]) > v_hi_[j]) {
      return fail(DecelMpcReason::kInitialStateOutsideBox);
    }
  }
  if (in.reference_valid) {
    for (Eigen::Index j = 0; j < nn; ++j) {
      if (std::abs(in.qd_ref(j, N)) > params_.reference_rest_tol ||
          std::abs(in.qdd_ref(j, N)) > params_.reference_rest_tol) {
        return fail(DecelMpcReason::kReferenceNotAtRest);
      }
    }
    // x_0 drifted more than δ from the reference's node 0: node 1 cannot sit
    // inside its trust window, and handing that to ProxQP only buys a slow
    // infeasibility verdict. The caller re-anchors (reference_valid = false).
    for (Eigen::Index j = 0; j < nn; ++j) {
      if (std::abs(in.q0[j] - in.q_ref(j, 0)) > params_.delta_tr) {
        return fail(DecelMpcReason::kTrustRegionConflict);
      }
    }
  }

  q0_ = in.q0;
  v0_ = in.qd0;
  a0_ = in.qdd0;
  if (w_perp_on_) {
    p_c_ = in.p_c;
    p_perp_ = Eigen::Matrix3d::Identity() - in.d_hat * in.d_hat.transpose();  // fixed size
  }
  w_delta_scale_ = in.w_delta_scale;
  if (catch_on_) {
    p_b_ = in.p_b;
    a_d_ = in.a_d;
    v_b_ = in.v_b;
    w_p_ = w_p_sym;
    gamma_ref_ = in.gamma_ref;
    pos_on_ = w_p_sym.cwiseAbs().maxCoeff() > 0.0;
  }
  FreeResponse();

  // Reference: supplied, or the kinematic pre-solve.
  if (!in.reference_valid) {
    const auto t0 = Clock::now();
    if (params_.reference_assembly) {
      AssembleConstantDense(qp_pre_, false);
    }
    (void)AssembleBounds(qp_pre_, false);  // cannot fail without the trust region
    AssembleGradient(qp_pre_, false);
    // A pre-solve starts a NEW stop problem: warm-starting it from the last
    // cycle's iterates (a different problem, with other rows active) made
    // ProxQP report PRIMAL_INFEASIBLE on feasible kinematic stops (~7 % of
    // random entry states in the timing fixture). Cold here; the full solve
    // that follows warm-starts from this one.
    ResetSolver();
    int status = -1;
    int iters = 0;
    const DecelMpcReason why = RunQp(qp_pre_, status, iters);
    out.presolve_iterations = iters;
    out.presolved = true;
    out.presolve_us = MicrosSince(t0);
    if (why != DecelMpcReason::kNone) {
      out.qp_status = status;
      return fail(why == DecelMpcReason::kQpFailed ? DecelMpcReason::kPresolveFailed : why);
    }
    TrajectoryFromZ();
    qr_ = q_out_;
    vr_ = v_out_;
    ar_ = a_out_;
  } else {
    qr_ = in.q_ref;
    vr_ = in.qd_ref;
    ar_ = in.qdd_ref;
  }

  // Linearise at x̄ (torque derivatives, and FK for w_⊥).
  {
    const auto t0 = Clock::now();
    const bool ok = (torque_on_ || w_perp_on_) ? Linearize() : true;
    const DecelMpcReason catch_why = (ok && catch_on_) ? LinearizeCatch() : DecelMpcReason::kNone;
    out.linearize_us = MicrosSince(t0);
    if (!ok) {
      // Internal-invariant guard: Init sized every operand, so the seam's
      // size check cannot fail here. Kept rather than assumed.
      return fail(DecelMpcReason::kDimMismatch);
    }
    if (catch_why != DecelMpcReason::kNone) {
      return fail(catch_why);
    }
  }

  // Condense.
  {
    const auto t0 = Clock::now();
    if (params_.reference_assembly) {
      AssembleConstantDense(qp_main_, true);
      if (torque_on_) {
        AssembleTorqueRowsDense();
      }
    } else {
      // H is h_main_ from Init until a term or a scale touches it; rebuild it
      // only then. At scale 1 it is the SAME matrix as before E1-F07.
      const bool scaled = w_delta_scale_ != 1.0;
      if (h_modified_ || scaled) {
        if (scaled) {
          qp_main_.H = h_kin_;
          qp_main_.H += w_delta_scale_ * h_delta_;
        } else {
          qp_main_.H = h_main_;
        }
      }
      h_modified_ = w_perp_on_ || catch_on_ || scaled;
      if (torque_on_) {
        AssembleTorqueRows();
      }
    }
    if (slack_v_on_) {
      AssembleSlackVRows();
    }
    AssembleGradient(qp_main_, true);
    if (w_perp_on_) {
      AssemblePerp();
    }
    if (catch_on_) {
      AssembleCatch();
    }
    // Last, so that a trust-region rejection has run the WHOLE condensing — the
    // path the allocation gate measures up to the QP. The price is one
    // condensing on that rejection; most conflicts are caught earlier, by the
    // x_0 check before the free response.
    if (!AssembleBounds(qp_main_, true)) {
      out.condense_us = MicrosSince(t0);
      return fail(DecelMpcReason::kTrustRegionConflict);
    }
    out.condense_us = MicrosSince(t0);
  }

  // Solve.
  {
    const auto t0 = Clock::now();
    int status = -1;
    int iters = 0;
    if (in.cold_start) {
      // Only here: every rejection above leaves the solver as it was.
      ResetSolver();
    }
    const bool warm = solver_warm_;
    DecelMpcReason why = RunQp(qp_main_, status, iters);
    if (why != DecelMpcReason::kNone && warm) {
      // Iterates left by ANOTHER problem (the caller did not set cold_start on
      // a new plan) make ProxQP report a feasible QP infeasible — about a
      // quarter of the solves in the timing fixture. The failed run reset the
      // solver, so this second run starts from zero; a problem that is really
      // infeasible fails again and that verdict is returned.
      out.cold_retried = true;
      why = RunQp(qp_main_, status, iters);
    }
    out.solve_us = MicrosSince(t0);
    out.qp_status = status;
    out.iterations = iters;
    if (why != DecelMpcReason::kNone) {
      return fail(why);
    }
  }

  TrajectoryFromZ();
  const Eigen::Index nN = nn * N;
  const double us = params_.u_scale;
  for (Eigen::Index k = 0; k < N; ++k) {
    const Eigen::Index b = block_of_node_[static_cast<std::size_t>(k)];
    for (Eigen::Index j = 0; j < nn; ++j) {
      out.u(j, k) = us * z_[b * nn + j];
      out.slack(j, k) = z_[nu_ + k * nn + j];
    }
  }
  out.q = q_out_;
  out.qd = v_out_;
  out.qdd = a_out_;
  out.slack_max = out.slack.maxCoeff();
  out.slack_terminal_max = out.slack.col(N - 1).maxCoeff();
  out.torque_evaluated = torque_on_;
  if (torque_on_) {
    tau_lin_ = c_tilde_;
    tau_lin_.noalias() += qp_main_.C.block(2 * nN, 0, nN, nu_) * z_.head(nu_);
    out.tau_ratio_max = tau_lin_.cwiseAbs().maxCoeff();
    for (Eigen::Index k = 0; k < N; ++k) {
      for (Eigen::Index j = 0; j < nn; ++j) {
        out.tau_ratio(j, k) = tau_lin_[k * nn + j];
      }
    }
  } else {
    out.tau_ratio_max = 0.0;
  }
  out.catch_evaluated = false;
  if (catch_on_) {
    EvaluateCatch(out);
  }
  out.reason = DecelMpcReason::kNone;
  out.valid = true;
  return true;
}

// ── Position weight from a covariance ────────────────────────────────────────

bool CatchPositionWeight(const Eigen::Matrix3d& sigma_p, double kappa, double sigma_floor,
                         double w_max, Eigen::Matrix3d& w_p) noexcept {
  // Finite and positive BEFORE any min/max: std::min/max launder a NaN (NUM-7).
  if (!sigma_p.allFinite() || !FinitePositive(kappa) || !FinitePositive(sigma_floor) ||
      !FinitePositive(w_max)) {
    return false;
  }
  const Eigen::Matrix3d sym = 0.5 * (sigma_p + sigma_p.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;  // fixed size: no heap
  es.compute(sym);
  if (es.info() != Eigen::Success || !es.eigenvalues().allFinite() ||
      !es.eigenvectors().allFinite()) {
    return false;
  }
  const double floor_sq = sigma_floor * sigma_floor;
  Eigen::Vector3d w;
  for (Eigen::Index i = 0; i < 3; ++i) {
    // A negative eigenvalue (a covariance that is not PSD) counts as zero, so
    // the denominator is never below σ_floor².
    const double mu = std::max(es.eigenvalues()[i], 0.0) + floor_sq;
    w[i] = std::min(kappa / mu, w_max);
  }
  const Eigen::Matrix3d m = es.eigenvectors() * w.asDiagonal() * es.eigenvectors().transpose();
  w_p = 0.5 * (m + m.transpose());
  return true;
}

}  // namespace rtc::catching
