#include "rtc_controllers/catching/decel_mpc.hpp"

#include "rtc_controllers/catching/decel_mpc_torque.hpp"

#include <Eigen/QR>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rtc::catching {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// ‖d̂‖ must be 1 to this tolerance; P_⊥ = I − d̂d̂ᵀ is a projector only then.
constexpr double kUnitTolerance = 1e-6;
// Entry-box comparison slack [rad] — rounding only, far below any margin.
constexpr double kBoxRoundingSlack = 1e-12;

[[nodiscard]] bool FinitePositive(double x) noexcept {
  return std::isfinite(x) && x > 0.0;
}

[[nodiscard]] bool FiniteNonNegative(double x) noexcept {
  return std::isfinite(x) && x >= 0.0;
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

  if (arm.nv < 1 || arm.nv > kMaxPlanNv || arm.nq != arm.nv) {
    return DecelMpcReason::kModelUnsupported;
  }
  if (catch_frame >= arm.frames.size()) {
    return DecelMpcReason::kFrameUnknown;
  }
  const int n = arm.nv;
  const auto nn = static_cast<Eigen::Index>(n);

  // Parameters.
  const int n_nodes = params.n_nodes;
  if (n_nodes < 1 || n_nodes > kMaxDecelNodes || !FinitePositive(params.dt)) {
    return DecelMpcReason::kParamsInvalid;
  }
  if (params.n_blocks < 3) {
    return DecelMpcReason::kBlocksTooFew;
  }
  if (params.n_blocks > n_nodes) {
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
  if (!FinitePositive(params.u_scale) || !FiniteNonNegative(params.w_delta) ||
      !FiniteNonNegative(params.w_perp) || !FiniteNonNegative(params.rho_tau) ||

      !InUnitInterval(params.eta_v) || !InUnitInterval(params.eta_tau) ||
      !FiniteNonNegative(params.m_q) || std::isnan(params.delta_tr) || !(params.delta_tr > 0.0) ||
      !FinitePositive(params.reference_rest_tol) || !FinitePositive(params.solver.eps_abs) ||
      !FiniteNonNegative(params.solver.eps_rel) || params.solver.max_iter < 1 ||
      params.solver.max_iter_in < 1) {
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
  n_blocks_ = params.n_blocks;
  nu_ = n * n_blocks_;
  nz_ = n * (n_blocks_ + n_nodes_);
  n_eq_ = 2 * n;
  n_in_ = 5 * n * n_nodes_;
  frame_ = catch_frame;
  w_perp_on_ = params.w_perp > 0.0;
  torque_on_ = params.rho_tau > 0.0;

  model_ = arm;
  model_.armature = limits.armature;
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

  {
    int k = 0;
    for (int b = 0; b < n_blocks_; ++b) {
      for (int i = 0; i < params.block_sizes[static_cast<std::size_t>(b)]; ++i) {
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
  const double dt = params.dt;
  for (Eigen::Index b = 0; b < nb; ++b) {
    double q = 0.0;
    double v = 0.0;
    double a = 0.0;
    for (int k = 0; k < n_nodes_; ++k) {
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
    const double n_b = params.block_sizes[static_cast<std::size_t>(b)];
    for (Eigen::Index j = 0; j < nn; ++j) {
      h_kin_(b * nn + j, b * nn + j) = jerk_w_[j] * n_b;
    }
  }
  h_main_ = h_kin_;
  if (params.w_delta > 0.0) {
    for (Eigen::Index k = 1; k < N1; ++k) {
      for (Eigen::Index b = 0; b < nb; ++b) {
        for (Eigen::Index c = 0; c < nb; ++c) {
          const double w = params.w_delta * gq_(k, b) * gq_(k, c);
          for (Eigen::Index j = 0; j < nn; ++j) {
            h_main_(b * nn + j, c * nn + j) += w;
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
  ltl_.setZero(nn, nn);
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
}

void DecelMpc::AssembleConstantDense(tsid::QPData& qp, bool main) noexcept {
  // The "before" of O-1/O-2: every matrix rebuilt from the dense Γ_k·E.
  const Eigen::Index nn = n_;
  const Eigen::Index nN = nn * n_nodes_;
  const Eigen::Index N = n_nodes_;
  const Eigen::Index nb = n_blocks_;

  qp.H.setZero();
  for (Eigen::Index b = 0; b < nb; ++b) {
    const double n_b = params_.block_sizes[static_cast<std::size_t>(b)];
    for (Eigen::Index j = 0; j < nn; ++j) {
      qp.H(b * nn + j, b * nn + j) = jerk_w_[j] * n_b;
    }
  }
  if (main && params_.w_delta > 0.0) {
    const double sw = std::sqrt(params_.w_delta);
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
}

void DecelMpc::FreeResponse() noexcept {
  const Eigen::Index nn = n_;
  for (Eigen::Index k = 0; k <= n_nodes_; ++k) {
    const double t = static_cast<double>(k) * params_.dt;
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
    if (w_perp_on_) {
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
  // ½ w_⊥ Σ_k ‖r_k + L_k (ĝ_{q,k}ᵀ ⊗ I) z‖²
  const Eigen::Index nn = n_;
  const double w = params_.w_perp;
  for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
    ltl_.noalias() = l_perp_.middleCols(nn * k, nn).transpose() * l_perp_.middleCols(nn * k, nn);
    // Exactly symmetric (GEMM rounding differs across the two triangles).
    for (Eigen::Index c = 0; c < nn; ++c) {
      for (Eigen::Index r = c + 1; r < nn; ++r) {
        ltl_(c, r) = ltl_(r, c);
      }
    }
    work_n_.noalias() = l_perp_.middleCols(nn * k, nn).transpose() * r_perp_.segment(3 * k, 3);
    const Eigen::Index last_block = block_of_node_[static_cast<std::size_t>(k - 1)];
    for (Eigen::Index b = 0; b <= last_block; ++b) {
      for (Eigen::Index c = 0; c <= last_block; ++c) {
        qp_main_.H.block(b * nn, c * nn, nn, nn) += (w * gq_(k, b) * gq_(k, c)) * ltl_;
      }
      qp_main_.g.segment(b * nn, nn) += (w * gq_(k, b)) * work_n_;
    }
  }
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
    for (Eigen::Index k = 1; k <= n_nodes_; ++k) {
      const Eigen::Index last_block = block_of_node_[static_cast<std::size_t>(k - 1)];
      for (Eigen::Index b = 0; b <= last_block && b < nb; ++b) {
        const double w = params_.w_delta * gq_(k, b);
        for (Eigen::Index j = 0; j < nn; ++j) {
          qp.g[b * nn + j] += w * (qf_(j, k) - qr_(j, k));
        }
      }
    }
  }
  if (torque_on_) {
    qp.g.segment(nu_, nn * n_nodes_).setConstant(params_.rho_tau);
  }
}

bool DecelMpc::RunQp(tsid::QPData& qp, int& status, int& iterations) noexcept {
  const tsid::SolveResult& res = solver_.Solve(qp);
  status = res.status;
  iterations = res.iterations;
  if (!res.converged) {
    return false;
  }
  z_ = res.x_opt.head(nz_);
  // The wrapper already refuses non-finite iterates; checked again here
  // because the pre-solve's answer becomes x̄ and feeds the max/min of the
  // trust-region bounds, where a NaN would silently drop the row (NUM-7).
  return z_.allFinite();
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
    solver_.ResetWarmStart();
    int status = -1;
    int iters = 0;
    const bool ok = RunQp(qp_pre_, status, iters);
    out.presolve_iterations = iters;
    out.presolved = true;
    out.presolve_us = MicrosSince(t0);
    if (!ok) {
      out.qp_status = status;
      solver_.ResetWarmStart();
      return fail(DecelMpcReason::kPresolveFailed);
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
    out.linearize_us = MicrosSince(t0);
    if (!ok) {
      return fail(DecelMpcReason::kDimMismatch);
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
      if (w_perp_on_) {
        qp_main_.H = h_main_;
      }
      if (torque_on_) {
        AssembleTorqueRows();
      }
    }
    const bool bounds_ok = AssembleBounds(qp_main_, true);
    AssembleGradient(qp_main_, true);
    if (w_perp_on_) {
      AssemblePerp();
    }
    out.condense_us = MicrosSince(t0);
    if (!bounds_ok) {
      return fail(DecelMpcReason::kTrustRegionConflict);
    }
  }

  // Solve.
  {
    const auto t0 = Clock::now();
    int status = -1;
    int iters = 0;
    const bool ok = RunQp(qp_main_, status, iters);
    out.solve_us = MicrosSince(t0);
    out.qp_status = status;
    out.iterations = iters;
    if (!ok) {
      solver_.ResetWarmStart();
      return fail(DecelMpcReason::kQpFailed);
    }
  }
  if (!z_.allFinite()) {
    solver_.ResetWarmStart();
    return fail(DecelMpcReason::kSolutionNonFinite);
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
  out.reason = DecelMpcReason::kNone;
  out.valid = true;
  return true;
}

}  // namespace rtc::catching
