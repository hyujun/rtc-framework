#include "rtc_controllers/catching/catch_pose_ik.hpp"

#include "rtc_math/se3/axis_align.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rtc::catching {

namespace {

/// Every option this function divides by, compares against or feeds to a
/// decomposition, checked once up front. A rejected option set is never
/// repaired into a working one (NUM-7) — silently substituting a default would
/// make the offline map and the runtime planner disagree about what they ran,
/// which is the one failure mode plan §11 exists to prevent.
[[nodiscard]] bool OptionsUsable(const CatchPoseIkOptions& o) noexcept {
  return o.max_iter > 0 &&                                              //
         std::isfinite(o.eps_pos) && o.eps_pos > 0.0 &&                 //
         std::isfinite(o.alpha_max) && o.alpha_max >= 0.0 &&            //
         std::isfinite(o.rho) && o.rho > 0.0 &&                         //
         std::isfinite(o.sigma0) && o.sigma0 >= 0.0 &&                  //
         std::isfinite(o.lambda_max) && o.lambda_max >= 0.0 &&          //
         std::isfinite(o.dq_step_max) && o.dq_step_max > 0.0 &&         //
         std::isfinite(o.mu) && o.mu > 0.0 &&                           //
         std::isfinite(o.qp_eps_abs) && o.qp_eps_abs > 0.0 &&           //
         o.qp_max_iter > 0 &&                                           //
         std::isfinite(o.k_null) && std::isfinite(o.k_manip) &&         //
         std::isfinite(o.manip_grad_tol) && o.manip_grad_tol >= 0.0 &&  //
         std::isfinite(o.fd_step) && o.fd_step > 0.0 &&                 //
         std::isfinite(o.v_eps) && o.v_eps > 0.0 &&                     //
         std::isfinite(o.manipulability_min) && o.manipulability_min >= 0.0;
}

/// ½·log det(A) from a fixed-size LDLT, or invalid.
///
/// det(A) = ∏ dᵢ regardless of the pivoting permutation (L is unit triangular
/// and det(P)² = 1), so the diagonal is the whole determinant.
///
/// A pivot that is zero, negative or non-finite means the pose is rank
/// deficient or the factorisation broke down, and BOTH are rejections. This is
/// the fail-closed rule L3 §4.2 states: not `det > 0` on a materialised
/// determinant, which rounds to a small positive number about as often as to a
/// negative one near a singularity, and which launders a NaN into a false
/// comparison. Working in logs also keeps a legitimately small w from
/// underflowing the product of five pivots to zero.
template <int N>
[[nodiscard]] detail::LogManip LogDetHalf(Eigen::LDLT<Eigen::Matrix<double, N, N>>& ldlt,
                                          const Eigen::Matrix<double, N, N>& a) noexcept {
  detail::LogManip out;
  if (!a.allFinite())
    return out;
  ldlt.compute(a);
  if (ldlt.info() != Eigen::Success)
    return out;
  double sum = 0.0;
  for (int i = 0; i < N; ++i) {
    const double d = ldlt.vectorD()(i);
    if (!std::isfinite(d) || d <= 0.0)
      return out;
    sum += std::log(d);
  }
  out.log_w = 0.5 * sum;
  out.valid = true;
  return out;
}

}  // namespace

void CatchPoseIk::Resize(int nv) {
  // Off-RT, and the one place that allocates. An out-of-range nv leaves the
  // object unsized rather than throwing (RT-2 forbids exceptions on this path
  // and a half-sized object is worse than a rejected Solve): nv_ = 0 makes
  // every subsequent Solve() return kModelInvalid.
  if (nv <= 0 || nv > rtc::kMaxRobotDOF) {
    nv_ = 0;
    return;
  }
  nv_ = nv;
  J6_ = Eigen::MatrixXd::Zero(6, nv);
  Jlocal_ = Eigen::MatrixXd::Zero(6, nv);
  J5w_ = Eigen::MatrixXd::Zero(5, nv);
  e5w_ = Eigen::VectorXd::Zero(5);
  q_ = Eigen::VectorXd::Zero(nv);
  q_try_ = Eigen::VectorXd::Zero(nv);
  q_acc_ = Eigen::VectorXd::Zero(nv);
  grad_ = Eigen::VectorXd::Zero(nv);
  qdot_sec_ = Eigen::VectorXd::Zero(nv);
  qdot_clik_ = Eigen::VectorXd::Zero(nv);
  qdot_n_ = Eigen::VectorXd::Zero(nv);
  qdot_d_ = Eigen::VectorXd::Zero(nv);
  dls_.Resize(nv, 5);

  // The task QP: nv variables (q̇), no equalities, nv box rows. Init() is the
  // only allocation — Solve() updates the same workspace.
  JtJ_ = Eigen::MatrixXd::Zero(nv, nv);
  qp_data_.Init(nv, 0, nv);
  qp_data_.n_vars = nv;
  qp_data_.n_eq = 0;
  qp_data_.n_ineq = nv;
  qp_data_.C = Eigen::MatrixXd::Identity(nv, nv);
  qp_.Init(nv, 0, nv);
}

bool CatchPoseIk::StackJacobian(rtc_urdf_bridge::RtModelHandle& model,
                                pinocchio::FrameIndex catch_frame,
                                const Eigen::Ref<const Eigen::VectorXd>& q) noexcept {
  const std::span<const double> q_span(q.data(), static_cast<std::size_t>(q.size()));
  model.ComputeJacobians(q_span);
  // Two extractions from ONE kinematics pass. The reference frame changes what
  // the rows mean, not what was computed: LOCAL_WORLD_ALIGNED gives the frame
  // origin's linear velocity in world axes (rows 0-2 — the block the position
  // task lives in), LOCAL gives the angular velocity in the frame's own axes
  // (rows 3-5 — the block e_a^C lives in, since AxisAlignError is evaluated
  // against the frame's own ê_z). Mixing the two up is the trap S2 hit: the
  // result stays finite and smooth, so no norm or NaN sensor catches it.
  model.GetFrameJacobian(catch_frame, pinocchio::LOCAL_WORLD_ALIGNED, J6_);
  model.GetFrameJacobian(catch_frame, pinocchio::LOCAL, Jlocal_);
  J6_.bottomRows(3) = Jlocal_.bottomRows(3);
  return J6_.allFinite();
}

detail::LogManip CatchPoseIk::LogW5() noexcept {
  const auto j5 = J6_.topRows<5>();
  A5_.noalias() = j5 * j5.transpose();
  return LogDetHalf<5>(ldlt5_, A5_);
}

detail::LogManip CatchPoseIk::LogW6() noexcept {
  A6_.noalias() = J6_ * J6_.transpose();
  return LogDetHalf<6>(ldlt6_, A6_);
}

bool CatchPoseIk::ManipGradient(rtc_urdf_bridge::RtModelHandle& model,
                                pinocchio::FrameIndex catch_frame,
                                const Eigen::Ref<const Eigen::VectorXd>& q, double h) noexcept {
  // Central difference on log w₅ directly, rather than assembling
  // ∂(J₅J₅ᵀ)/∂qᵢ and taking ½·tr(A⁻¹·∂A/∂qᵢ). Both are 2·nv Jacobian
  // evaluations here and neither allocates; the difference is that this form
  // has no second decomposition to go wrong near a singularity, and it is
  // exactly the quantity the ascent step uses, so an oracle comparing h with
  // h/2 tests the thing that is used rather than a proxy for it. The analytic
  // route (pinocchio computeJointKinematicHessians) is the fallback if the
  // 2·nv evaluations turn out not to fit the planner's budget.
  //
  // The probes deliberately ignore joint limits: forward kinematics is defined
  // outside them, h is ~1e-5 rad, and a one-sided difference at a bound would
  // bias the gradient exactly where the step is most constrained.
  q_try_ = q;
  for (int i = 0; i < nv_; ++i) {
    const double qi = q(i);

    q_try_(i) = qi + h;
    const bool up_ok = StackJacobian(model, catch_frame, q_try_);
    const detail::LogManip up = up_ok ? LogW5() : detail::LogManip{};

    q_try_(i) = qi - h;
    const bool dn_ok = StackJacobian(model, catch_frame, q_try_);
    const detail::LogManip dn = dn_ok ? LogW5() : detail::LogManip{};

    q_try_(i) = qi;  // restore before any early exit, so the buffer stays == q
    if (!up.valid || !dn.valid)
      return false;
    grad_(i) = (up.log_w - dn.log_w) / (2.0 * h);
  }
  return grad_.allFinite();
}

CatchPoseIkResult CatchPoseIk::Solve(rtc_urdf_bridge::RtModelHandle& model,
                                     pinocchio::FrameIndex catch_frame, const Eigen::Vector3d& p_c,
                                     const Eigen::Vector3d& v_ball,
                                     const Eigen::Ref<const Eigen::VectorXd>& q_seed,
                                     const CatchPoseIkOptions& opt) noexcept {
  CatchPoseIkResult r;

  if (!OptionsUsable(opt)) {
    r.reason = CatchPoseReason::kOptionsInvalid;
    return r;
  }

  const pinocchio::Model& pin = model.GetModel();
  // nq ≠ nv means at least one joint has a non-Euclidean configuration
  // parameterisation (a continuous joint carries cosθ/sinθ), and then adding a
  // joint-space step to q componentwise is simply wrong. Reject rather than
  // produce a plausible answer from an invalid update.
  if (nv_ <= 0 || model.nv() != nv_ || model.nq() != model.nv() || catch_frame == 0 ||
      static_cast<std::size_t>(catch_frame) >= pin.frames.size()) {
    r.reason = CatchPoseReason::kModelInvalid;
    return r;
  }
  r.nv = nv_;

  if (q_seed.size() != nv_ || !q_seed.allFinite()) {
    r.reason = CatchPoseReason::kSeedNonFinite;
    return r;
  }
  if (!p_c.allFinite()) {
    r.reason = CatchPoseReason::kTargetNonFinite;
    return r;
  }
  if (!v_ball.allFinite()) {
    r.reason = CatchPoseReason::kVelocityNonFinite;
    return r;
  }
  // Both halves of the NUM-7 defence L3 §4.2 prescribes: finiteness above, and
  // the speed floor here. ‖v̂‖ below v_eps is a rejection, not a max() — a
  // clamped speed would hand the caller an approach axis derived from noise.
  const double speed = v_ball.norm();
  if (!(speed >= opt.v_eps)) {
    r.reason = CatchPoseReason::kSpeedTooLow;
    return r;
  }
  const Eigen::Vector3d a_d = -v_ball / speed;

  // The seed itself is clamped: a wait pose outside the model's limits would
  // otherwise make the first iterate the only one that ever violated them.
  q_ = q_seed;
  ClampToLimits(pin, q_);

  // Note 5: one cold start per CANDIDATE. Iterations inside this call may warm
  // start from each other — that is deterministic and wanted — but nothing from
  // the previous candidate may survive into this one.
  qp_.ResetWarmStart();
  qp_.SetEpsAbs(opt.qp_eps_abs);
  qp_.SetMaxIter(opt.qp_max_iter);
  qp_status_ = -1;
  qp_iterations_ = 0;

  bool accepted_any = false;
  bool manip_converged = (opt.k_manip == 0.0);  // nothing to converge without the term
  double grad_norm = 0.0;
  int iterations = 0;

  for (int iter = 0; iter < opt.max_iter; ++iter) {
    iterations = iter + 1;

    if (!StackJacobian(model, catch_frame, q_)) {
      r.reason = CatchPoseReason::kJacobianNonFinite;
      return r;
    }

    const Eigen::Vector3d p_frame = model.GetFramePosition(catch_frame);
    const Eigen::Matrix3d r_wc = model.GetFrameRotation(catch_frame);
    const Eigen::Vector3d a_local = r_wc.transpose() * a_d;
    const math::se3::AxisAlignErrorResult axis =
        math::se3::AxisAlignError(Eigen::Vector3d::UnitZ(), a_local);
    if (!axis.IsValid()) {
      r.reason = CatchPoseReason::kAxisAlignInvalid;
      return r;
    }

    const Eigen::Vector3d e_pos = p_c - p_frame;
    const double pos_error = e_pos.norm();
    // θ = ‖e_a‖ by construction of the rotation-vector error, so this is the
    // cone test zᵀa_d ≥ cos α_max written in a form that stays well-conditioned
    // at large angles (L3 §4.2).
    const double theta = axis.error.norm();
    const bool meets_task = (pos_error < opt.eps_pos) && (theta <= opt.alpha_max);
    if (meets_task) {
      q_acc_ = q_;
      accepted_any = true;
      r.pos_error = pos_error;
      r.theta = theta;
    }

    // W·J₅ and W·e with W = diag(1,1,1,ρ,ρ). ρ is a task weight on BOTH sides,
    // not a gain on the residual — see note 1 in the header.
    J5w_.topRows(3) = J6_.topRows(3);
    J5w_.row(3) = opt.rho * J6_.row(3);
    J5w_.row(4) = opt.rho * J6_.row(4);
    e5w_.head(3) = e_pos;
    e5w_(3) = opt.rho * axis.error.x();
    e5w_(4) = opt.rho * axis.error.y();
    // e_a ⟂ ê_z, so its z component is structurally zero and S drops nothing.

    const compliance::DifferentialIk::Result dls = dls_.Compute(J5w_, opt.sigma0, opt.lambda_max);
    if (!dls.ok) {
      // The ONLY thing a false ok reports is a non-finite J (#310). A singular
      // pose comes back ok with σ_min ≈ 0 and is judged later, by w.
      r.reason = CatchPoseReason::kJacobianNonFinite;
      return r;
    }
    r.sigma_min = dls.sigma_min;
    r.lambda_sq = dls.lambda_sq;

    qdot_sec_.setZero();
    grad_norm = 0.0;
    if (opt.k_manip != 0.0) {
      if (ManipGradient(model, catch_frame, q_, opt.fd_step)) {
        // Progress is measured on the PROJECTED gradient: a component of
        // ∇log w₅ that the task Jacobian cancels is not available to spend, so
        // ‖∇log w₅‖ alone would never fall below the tolerance and the loop
        // would always run to max_iter.
        qdot_n_.noalias() = dls_.NullspaceProjector() * grad_;
        grad_norm = qdot_n_.norm();
        qdot_sec_.noalias() += opt.k_manip * grad_;
      } else {
        // A non-finite probe drops the ascent for this iteration instead of
        // substituting a direction (NUM-7). The task step still runs, and the
        // zero grad_norm is reported rather than hidden.
        grad_.setZero();
      }
      manip_converged = (grad_norm < opt.manip_grad_tol);
    }
    if (opt.k_null != 0.0)
      qdot_sec_.noalias() += opt.k_null * (q_seed - q_);

    if (meets_task && manip_converged)
      break;

    // The update law, in the two pieces it is actually made of:
    //
    //     q̇_clik = the box-constrained task QP's solution   (D-26)
    //     q̇_n    = N q̇_sec     — a joint velocity IN the null space
    //     q̇_d    = q̇_clik + q̇_n
    //
    // Written as the sum rather than as one opaque Δq because that is what the
    // law is: the secondary task contributes a VELOCITY that the projector has
    // already made invisible to the primary task, and adding it can therefore
    // never move the primary residual (to first order). Collapsing the two into
    // a single step hides the priority that N is there to enforce.
    if (!QpTaskStep(pin, opt)) {
      // Fail closed. A non-converged QP means the task step is unknown, and
      // substituting the unconstrained pseudo-inverse would quietly hand back a
      // pose computed by a different law than the one the map recorded.
      r.qp_status = qp_status_;
      r.qp_iterations = qp_iterations_;
      r.qp_failures = 1;
      r.iterations = iterations;
      r.reason = CatchPoseReason::kQpFailed;
      return r;
    }
    qdot_n_.noalias() = dls_.NullspaceProjector() * qdot_sec_;
    qdot_d_ = qdot_clik_ + qdot_n_;

    const double step_inf = qdot_d_.lpNorm<Eigen::Infinity>();
    // Unreachable with a finite, factorable J and a finite residual; kept so a
    // future change cannot turn an overflow into a plausible joint velocity.
    if (!std::isfinite(step_inf)) {
      r.reason = CatchPoseReason::kJacobianNonFinite;
      return r;
    }
    // Scaled, not clipped per component. Clipping would rotate q̇_d away from
    // the direction the QP chose — and q̇_n is the part that would be
    // rotated hardest, because it is usually the smaller of the two. Scaling
    // keeps the direction and only shortens it.
    if (step_inf > opt.dq_step_max)
      qdot_d_ *= (opt.dq_step_max / step_inf);

    // Δt = 1. This is an offline root-finding iteration, not a servo tick:
    // there is no sample period here, so `dq_step_max` bounds ‖q̇_d‖∞ per
    // iteration and integrating over one unit step is the whole integrator.
    q_ += qdot_d_;
    ClampToLimits(pin, q_);
  }

  r.iterations = iterations;
  r.qp_status = qp_status_;
  r.qp_iterations = qp_iterations_;
  r.manip_grad_norm = grad_norm;
  r.manip_converged = manip_converged;

  if (!accepted_any) {
    r.reason = CatchPoseReason::kNotConverged;
    return r;
  }

  for (int i = 0; i < nv_; ++i)
    r.q[static_cast<std::size_t>(i)] = q_acc_(i);

  // Re-evaluate at q*: the loop's last Jacobian belongs to the last ITERATE,
  // which is not q* when the run ended on a step that failed the tolerances.
  if (!StackJacobian(model, catch_frame, q_acc_)) {
    r.reason = CatchPoseReason::kJacobianNonFinite;
    return r;
  }
  const detail::LogManip l5 = LogW5();
  const detail::LogManip l6 = LogW6();
  r.w5_valid = l5.valid;
  r.w6_valid = l6.valid;
  r.w5 = l5.valid ? std::exp(l5.log_w) : 0.0;
  r.w6 = l6.valid ? std::exp(l6.log_w) : 0.0;

  const bool use_w5 = (opt.definition == ManipDefinition::kArm5Row);
  const bool gate_valid = use_w5 ? l5.valid : l6.valid;
  if (!gate_valid) {
    r.reason = CatchPoseReason::kRankDeficient;
    return r;
  }
  const double w = use_w5 ? r.w5 : r.w6;
  if (!(w >= opt.manipulability_min)) {
    r.reason = CatchPoseReason::kBelowManipMin;
    return r;
  }

  r.reason = CatchPoseReason::kNone;
  r.accepted = true;
  return r;
}

bool CatchPoseIk::QpTaskStep(const pinocchio::Model& pin, const CatchPoseIkOptions& opt) noexcept {
  // ½‖W(J₅q̇ − e)‖² + ½μ‖q̇‖²  ⇒  H = JᵀJ + μI,  g = −Jᵀe, with J = W J₅.
  //
  // μ is not cosmetic: J₅wᵀJ₅w is nv×nv but has rank ≤ 5, so it is singular for
  // every arm this runs on and the QP has no unique solution without it. It is
  // what plays the role §6.5's adaptive λ plays for the projector — except that
  // it is a constant, which is why it is validated (> 0) and why its default is
  // a measured provisional rather than a guess (plan §4.4).
  JtJ_.noalias() = J5w_.transpose() * J5w_;
  qp_data_.H = JtJ_;
  qp_data_.H.diagonal().array() += opt.mu;
  // Two statements, not `g.noalias() = -(Jᵀe)`. `noalias()` only suppresses the
  // temporary for a bare product; wrapping the product in a unary minus makes
  // the assignment a CwiseUnaryOp over a Product, which Eigen evaluates into a
  // RUNTIME-SIZED temporary — one heap allocation per IK iteration, on a path
  // that may run at SCHED_FIFO (RT-1). Negating in place afterwards is a
  // component-wise operation with no temporary at all.
  qp_data_.g.noalias() = J5w_.transpose() * e5w_;
  qp_data_.g = -qp_data_.g;

  for (int i = 0; i < nv_; ++i) {
    const double lo = pin.lowerPositionLimit(i);
    const double hi = pin.upperPositionLimit(i);
    qp_data_.l(i) = std::max(lo - q_(i), -opt.dq_step_max);
    qp_data_.u(i) = std::min(hi - q_(i), opt.dq_step_max);
    // q already outside its bound (an unlimited joint carries ±inf and never
    // gets here). Collapsing to a zero-width box is the only feasible reading:
    // an inverted one would make the whole QP infeasible and lose the other
    // joints' steps too.
    if (!(qp_data_.l(i) <= qp_data_.u(i)))
      qp_data_.l(i) = qp_data_.u(i) = 0.0;
  }

  const tsid::SolveResult& res = qp_.Solve(qp_data_);
  qp_status_ = res.status;
  qp_iterations_ = res.iterations;
  if (!res.converged)
    return false;
  qdot_clik_ = res.x_opt.head(nv_);
  return qdot_clik_.allFinite();
}

void CatchPoseIk::ClampToLimits(const pinocchio::Model& pin, Eigen::VectorXd& q) noexcept {
  // Limits come from the model, never from a constant here (ARCH-1). A joint
  // the URDF left unlimited carries ±inf, for which this is a no-op; an
  // inverted pair is left alone rather than std::clamp'd, which would be UB.
  for (int i = 0; i < q.size(); ++i) {
    const double lo = pin.lowerPositionLimit(i);
    const double hi = pin.upperPositionLimit(i);
    if (std::isnan(lo) || std::isnan(hi) || lo > hi)
      continue;
    q(i) = std::clamp(q(i), lo, hi);
  }
}

}  // namespace rtc::catching
