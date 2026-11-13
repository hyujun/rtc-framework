/// @file mpc_handler_core.cpp
/// @brief Implementation of the shared MPC solve pipeline used by both
///        `ContactLightMPC` and `ContactRichMPC`.

#include "handler/internal/mpc_handler_core.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include "rtc_mpc/feedback/riccati_feedback.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rtc::mpc::internal {

namespace {

using Solver = aligator::SolverProxDDPTpl<double>;

/// @brief Clamp an int `v` against the compile-time capacity `cap`,
///        returning true when the input fit.
[[nodiscard]] bool CapacityOk(int v, int cap) noexcept {
  return v >= 0 && v <= cap;
}

/// @brief A node spacing the solution can carry: finite and positive. A NaN
///        passes every `dt <= 0` check downstream, so it is caught here.
[[nodiscard]] bool NodeDtValid(double dt) noexcept {
  return std::isfinite(dt) && dt > 0.0;
}

/// @brief The ranges of `MPCSolverConfig` (its field comments). A NaN
///        tolerance fails the `> 0` tests.
[[nodiscard]] bool SolverConfigValid(const MPCSolverConfig& cfg) noexcept {
  if (!(cfg.prim_tol > 0.0) || !(cfg.dual_tol > 0.0) || !(cfg.mu_init > 0.0) ||
      cfg.max_iters <= 0 || cfg.max_al_iters <= 0) {
    return false;
  }
  if (cfg.max_initial_iters < 0 ||
      (cfg.max_initial_iters > 0 && cfg.max_initial_iters < cfg.max_iters) ||
      cfg.max_online_al_iters < 0) {
    return false;
  }
  return cfg.linear_solver == MPCLinearSolver::kSerial && cfg.num_threads == 1 &&
         cfg.force_initial_condition;
}

}  // namespace

void MPCHandlerCore::RecomputeContactVarDim(const RobotModelHandler& model) noexcept {
  int total = 0;
  for (const auto& frame : model.contact_frames()) {
    total += frame.dim;
  }
  n_contact_vars_ = total;
}

MPCInitError MPCHandlerCore::Init(const MPCSolverConfig& solver_cfg, const RobotModelHandler& model,
                                  const OCPLimits& limits, const PhaseContext& initial_ctx,
                                  OCPHandlerBase& ocp) noexcept {
  initialised_ = false;

  if (!model.Initialised()) {
    return MPCInitError::kModelNotInitialised;
  }
  // A joint-torque OCP (u = τ, nu == nv) cannot drive a free-flyer, and a
  // kinodynamic OCP has no wrench block without one.
  const ControlModel control = ocp.control_model();
  if (control == ControlModel::kJointTorque && model.floating_base()) {
    return MPCInitError::kFloatingBaseUnsupported;
  }
  if (control == ControlModel::kKinodynamicWrench && !model.floating_base()) {
    return MPCInitError::kFixedBaseUnsupported;
  }
  if (!SolverConfigValid(solver_cfg)) {
    return MPCInitError::kInvalidSolverConfig;
  }
  if (initial_ctx.ocp_type != ocp.ocp_type()) {
    return MPCInitError::kOcpTypeMismatch;
  }
  if (!NodeDtValid(initial_ctx.cost_config.dt)) {
    return MPCInitError::kOCPBuildFailed;
  }

  const OCPBuildError be = ocp.Build(initial_ctx, model, limits);
  if (be != OCPBuildError::kNoError) {
    return MPCInitError::kOCPBuildFailed;
  }

  try {
    solver_ = std::make_unique<Solver>(solver_cfg.prim_tol, solver_cfg.mu_init,
                                       static_cast<std::size_t>(solver_cfg.max_iters));
    solver_->setDualTolerance(solver_cfg.dual_tol);
    solver_->max_al_iters = static_cast<std::size_t>(solver_cfg.max_al_iters);
    solver_->verbose_ = solver_cfg.verbose ? aligator::VERBOSE : aligator::QUIET;
    // Set before setup(), which picks the LQ backend; the thread count stays
    // Aligator's default of one (the config check refused anything else).
    solver_->rollout_type_ = solver_cfg.rollout_type == MPCRolloutType::kNonlinear
                                 ? aligator::RolloutType::NONLINEAR
                                 : aligator::RolloutType::LINEAR;
    solver_->linear_solver_choice = aligator::LQSolverChoice::SERIAL;
    solver_->force_initial_condition_ = true;
    solver_->setup(ocp.problem());
  } catch (...) {
    solver_.reset();
    return MPCInitError::kSolverSetupFailed;
  }

  model_ = &model;
  limits_ = limits;
  solver_cfg_ = solver_cfg;

  horizon_length_ = ocp.horizon_length();
  nq_ = model.nq();
  nv_ = model.nv();
  nu_ = model.nu();
  ndx_ = 2 * nv_;  // state tangent [dq; dv], fixed or floating base
  control_model_ = control;
  RecomputeContactVarDim(model);

  x_current_ = Eigen::VectorXd::Zero(nq_ + nv_);
  q_current_ = Eigen::VectorXd::Zero(nq_);
  tau_g_ = Eigen::VectorXd::Zero(nv_);
  pdata_ = std::make_unique<pinocchio::Data>(model.model());

  // Cache dt from the first cost_config so MPCSolution carries a valid
  // spacing; UpdateReferences / Build on later ticks may update it.
  dt_ = initial_ctx.cost_config.dt;

  // The OCP names its cold seed (gravity compensation is joint-torque only).
  use_gravity_comp_seed_ =
      ocp.cold_seed() == ColdSeed::kGravityCompensation && control == ControlModel::kJointTorque;
  use_reference_control_seed_ = ocp.cold_seed() == ColdSeed::kReferenceControl;

  // Allocate warm-start buffers matching the solver's internal shapes.
  xs_warm_.assign(static_cast<std::size_t>(horizon_length_ + 1), Eigen::VectorXd::Zero(nq_ + nv_));
  us_warm_.assign(static_cast<std::size_t>(horizon_length_), Eigen::VectorXd::Zero(nu_));

  first_solve_after_build_ = true;
  initial_solve_pending_ = true;
  initialised_ = true;
  return MPCInitError::kNoError;
}

void MPCHandlerCore::ColdSeedGuess(const OCPHandlerBase& ocp, const Eigen::VectorXd& q_current,
                                   const Eigen::VectorXd& x_current) noexcept {
  // Populate xs_warm_: broadcast current state across every node.
  for (auto& x : xs_warm_) {
    if (x.size() == x_current.size()) {
      x = x_current;
    }
  }
  if (use_gravity_comp_seed_) {
    // Risk #14 mitigation for ContactRich: gravity-comp τ drives the
    // constraint-force equilibrium λ → 0 at neutral pose.
    pinocchio::computeGeneralizedGravity(model_->model(), *pdata_, q_current);
    tau_g_ = pdata_->g;
    for (auto& u : us_warm_) {
      if (u.size() == tau_g_.size()) {
        u = tau_g_;
      }
    }
  } else if (use_reference_control_seed_) {
    // The OCP's own per-node reference (UpdateReferences ran this tick):
    // a start that satisfies what its constraints ask of the input.
    for (std::size_t k = 0; k < us_warm_.size(); ++k) {
      if (!ocp.ReferenceControl(static_cast<int>(k), us_warm_[k])) {
        us_warm_[k].setZero();
      }
    }
  } else {
    // ContactLight: zero torque is the cheaper seed — no self-gravity
    // drift to wrestle the cost landscape against.
    for (auto& u : us_warm_) {
      u.setZero();
    }
  }
}

namespace {

/// `out` is a reused slot: a failed tick must leave it with a status that
/// matches the error it reports, so the slot never looks like a solve that
/// happened. Mapped once here, written once at Solve's exit.
[[nodiscard]] constexpr SolverStatus StatusForError(MPCSolveError err) noexcept {
  switch (err) {
    case MPCSolveError::kSolutionTruncated:
      return SolverStatus::kTruncated;
    case MPCSolveError::kSolverException:
      return SolverStatus::kException;
    case MPCSolveError::kSolverDiverged:
      return SolverStatus::kDiverged;
    default:
      return SolverStatus::kNotSolved;
  }
}

}  // namespace

MPCSolveError MPCHandlerCore::Solve(const PhaseContext& ctx, const MPCStateSnapshot& state,
                                    OCPHandlerBase& ocp, MPCSolution& out) noexcept {
  const MPCSolveError err = SolveImpl(ctx, state, ocp, out);
  if (err != MPCSolveError::kNoError) {
    out.status = StatusForError(err);
  }
  return err;
}

MPCSolveError MPCHandlerCore::SolveImpl(const PhaseContext& ctx, const MPCStateSnapshot& state,
                                        OCPHandlerBase& ocp, MPCSolution& out) noexcept {
  if (!initialised_ || solver_ == nullptr || model_ == nullptr) {
    return MPCSolveError::kNotInitialised;
  }
  if (state.nq != nq_ || state.nv != nv_) {
    return MPCSolveError::kStateDimMismatch;
  }
  if (ctx.ocp_type != ocp.ocp_type()) {
    return MPCSolveError::kRebuildRequired;
  }
  // Checked before the context reaches the OCP: its own validation is a
  // `dt <= 0` test, which a NaN passes.
  if (!NodeDtValid(ctx.cost_config.dt)) {
    return MPCSolveError::kOCPRebuildFailed;
  }
  if (!CapacityOk(horizon_length_, kMaxHorizon) || !CapacityOk(nq_, kMaxNq) ||
      !CapacityOk(nv_, kMaxNv) || !CapacityOk(nu_, kMaxNu) || !CapacityOk(ndx_, kMaxNdx) ||
      !CapacityOk(n_contact_vars_, kMaxContactVars)) {
    return MPCSolveError::kSolutionTruncated;
  }
  // A non-finite state would become the problem's initial condition, and the
  // solver can hand such a trajectory back as converged. Reject it before
  // anything is written, so the warm start of the next tick stays intact.
  if (!StateFinite(state)) {
    return MPCSolveError::kStateDimMismatch;
  }
  // A non-unit base quaternion gives a finite but wrong state, so it is
  // refused, not normalised (the solver's own steps renormalise).
  if (!model_->layout().BaseQuaternionIsUnit(
          Eigen::Map<const Eigen::VectorXd>(state.q.data(), nq_))) {
    return MPCSolveError::kStateDimMismatch;
  }

  // Copy RT-side state into pre-allocated Eigen buffers.
  for (int i = 0; i < nq_; ++i)
    q_current_[i] = state.q[static_cast<std::size_t>(i)];
  for (int i = 0; i < nq_; ++i)
    x_current_[i] = state.q[static_cast<std::size_t>(i)];
  for (int i = 0; i < nv_; ++i)
    x_current_[nq_ + i] = state.v[static_cast<std::size_t>(i)];

  // Attempt reference update; fall through to a full rebuild on topology
  // change. Every tick pays the ref-update cost so continuously-moving
  // references (EE target slewing within a phase) stay in sync.
  const OCPBuildError upd = ocp.UpdateReferences(ctx);
  if (upd == OCPBuildError::kInvalidPhaseContext) {
    // An OCP whose config fixes its topology cannot be rebuilt into another
    // one; trying would only allocate.
    if (!ocp.RebuildsOnContextChange()) {
      return MPCSolveError::kOCPRebuildFailed;
    }
    const OCPBuildError be = ocp.Build(ctx, *model_, limits_);
    if (be != OCPBuildError::kNoError) {
      return MPCSolveError::kOCPRebuildFailed;
    }
    try {
      solver_->setup(ocp.problem());
    } catch (...) {
      return MPCSolveError::kOCPRebuildFailed;
    }
    horizon_length_ = ocp.horizon_length();
    first_solve_after_build_ = true;
    initial_solve_pending_ = true;
    // Re-size warm buffers in case horizon changed.
    xs_warm_.assign(static_cast<std::size_t>(horizon_length_ + 1),
                    Eigen::VectorXd::Zero(nq_ + nv_));
    us_warm_.assign(static_cast<std::size_t>(horizon_length_), Eigen::VectorXd::Zero(nu_));
  } else if (upd != OCPBuildError::kNoError) {
    return MPCSolveError::kOCPRebuildFailed;
  }
  dt_ = ctx.cost_config.dt;

  // Update the problem's initial-state constraint so the solver enforces
  // x_0 = x_current (force_initial_condition_ defaults to true).
  try {
    ocp.problem().setInitState(x_current_);
  } catch (...) {
    return MPCSolveError::kSolverException;
  }

  const bool cold_seeded = first_solve_after_build_;
  if (cold_seeded) {
    ColdSeedGuess(ocp, q_current_, x_current_);
    first_solve_after_build_ = false;
  } else {
    // Aligator's canonical shift-warm-start: rotates xs/us/vs/lams/gains
    // left by one node in-place (no alloc), re-seeds xs.front() = x0.
    try {
      solver_->results_.cycleAppend(ocp.problem(), x_current_);
    } catch (...) {
      DiscardIterates();  // the rotation may have stopped partway
      return MPCSolveError::kSolverException;
    }
  }

  // Stage the warm-start in separate xs_warm_ / us_warm_ buffers so
  // Aligator's assign_no_resize doesn't alias results_.xs onto itself
  // inside check_initial_guess_and_assign.
  {
    auto& xs_r = solver_->results_.xs;
    auto& us_r = solver_->results_.us;
    if (cold_seeded || xs_warm_.size() != xs_r.size() || us_warm_.size() != us_r.size()) {
      // Cold path populated xs_warm_/us_warm_ directly (ColdSeedGuess).
      // Nothing extra to do.
    } else {
      // Shifted results_ already hold the warm-start (from cycleAppend);
      // mirror them into the separate buffers we pass to run().
      for (std::size_t i = 0; i < xs_warm_.size(); ++i) {
        if (xs_warm_[i].size() == xs_r[i].size())
          xs_warm_[i] = xs_r[i];
      }
      for (std::size_t i = 0; i < us_warm_.size(); ++i) {
        if (us_warm_[i].size() == us_r[i].size())
          us_warm_[i] = us_r[i];
      }
    }
  }

  const bool initial_solve = initial_solve_pending_;
  ApplyBudget();
  const auto t0 = std::chrono::steady_clock::now();
  bool converged = false;
  try {
    converged = solver_->run(ocp.problem(), xs_warm_, us_warm_);
  } catch (...) {
    // Aligator accepts a step into results_ before its own NaN checks throw,
    // so a throw can leave the rejected iterate behind.
    DiscardIterates();
    return MPCSolveError::kSolverException;
  }
  const auto t1 = std::chrono::steady_clock::now();
  const auto dur_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());

  // A non-finite value anywhere in what PackSolution ships invalidates the
  // solve, converged or not: the solver's own criteria can pass with a
  // non-finite iterate (a control the dynamics never read, for one). A
  // finite, non-converged solve used its budget and is still usable.
  if (!SolutionFinite()) {
    DiscardIterates();
    return MPCSolveError::kSolverDiverged;
  }

  PackSolution(ocp, state, out, converged, initial_solve, dur_ns);
  if (converged) {
    initial_solve_pending_ = false;
  }
  return MPCSolveError::kNoError;
}

void MPCHandlerCore::ApplyBudget() noexcept {
  if (initial_solve_pending_) {
    solver_->max_iters = static_cast<std::size_t>(solver_cfg_.InitialIters());
    solver_->max_al_iters = static_cast<std::size_t>(solver_cfg_.max_al_iters);
  } else {
    solver_->max_iters = static_cast<std::size_t>(solver_cfg_.max_iters);
    solver_->max_al_iters = static_cast<std::size_t>(solver_cfg_.OnlineAlIters());
  }
}

bool MPCHandlerCore::SolutionFinite() const noexcept {
  const auto& results = solver_->results_;
  // The residual summary is shipped too.
  if (!std::isfinite(results.prim_infeas) || !std::isfinite(results.dual_infeas) ||
      !std::isfinite(results.traj_cost_) || !std::isfinite(results.merit_value_)) {
    return false;
  }
  for (const Eigen::VectorXd& x : results.xs) {
    if (!x.allFinite()) {
      return false;
    }
  }
  for (const Eigen::VectorXd& u : results.us) {
    if (!u.allFinite()) {
      return false;
    }
  }
  // Not shipped, but the next tick's warm start shifts them in.
  for (const Eigen::VectorXd& l : results.lams) {
    if (!l.allFinite()) {
      return false;
    }
  }
  for (const Eigen::VectorXd& v : results.vs) {
    if (!v.allFinite()) {
      return false;
    }
  }
  // Only the gain block PackRiccatiGain copies is shipped.
  const std::size_t gain_nodes = std::min(
      static_cast<std::size_t>(std::min(horizon_length_, kMaxGainNodes)), results.gains_.size());
  for (std::size_t k = 0; k < gain_nodes; ++k) {
    const Eigen::MatrixXd& gk = results.gains_[k];
    if (gk.rows() >= nu_ && gk.cols() >= ndx_ + 1 && !gk.topRightCorner(nu_, ndx_).allFinite()) {
      return false;
    }
  }
  return true;
}

void MPCHandlerCore::DiscardIterates() noexcept {
  // run() keeps results_' multipliers when it is given none, and a later
  // SeedWarmStart may overwrite only part of xs / us — so every slot is reset,
  // not just flagged: the measured state at every node, zero elsewhere.
  auto& results = solver_->results_;
  for (Eigen::VectorXd& x : results.xs) {
    if (x.size() == x_current_.size()) {
      x = x_current_;
    } else {
      x.setZero();
    }
  }
  for (Eigen::VectorXd& u : results.us) {
    u.setZero();
  }
  for (Eigen::VectorXd& l : results.lams) {
    l.setZero();
  }
  for (Eigen::VectorXd& v : results.vs) {
    v.setZero();
  }
  for (Eigen::MatrixXd& g : results.gains_) {
    g.setZero();
  }
  first_solve_after_build_ = true;
  initial_solve_pending_ = true;
}

void MPCHandlerCore::SeedWarmStart(const MPCSolution& prev, OCPHandlerBase& ocp) noexcept {
  if (!initialised_ || solver_ == nullptr)
    return;
  if (!prev.IsValid())
    return;

  auto& xs = solver_->results_.xs;
  auto& us = solver_->results_.us;
  const int H = horizon_length_;
  const int Hp = std::min(prev.horizon_length, H);

  // Copy states (nodes 0..Hp). Dim mismatches silently truncate / pad.
  const int nq_copy = std::min(prev.nq, nq_);
  const int nv_copy = std::min(prev.nv, nv_);
  for (int k = 0; k <= Hp && static_cast<std::size_t>(k) < xs.size(); ++k) {
    if (xs[static_cast<std::size_t>(k)].size() != nq_ + nv_)
      continue;
    for (int i = 0; i < nq_copy; ++i) {
      xs[static_cast<std::size_t>(k)][i] =
          prev.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < nv_copy; ++i) {
      xs[static_cast<std::size_t>(k)][nq_ + i] =
          prev.v_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
    }
  }
  // Copy controls (nodes 0..Hp-1).
  const int nu_copy = std::min(prev.nu, nu_);
  for (int k = 0; k < Hp && static_cast<std::size_t>(k) < us.size(); ++k) {
    if (us[static_cast<std::size_t>(k)].size() != nu_)
      continue;
    for (int i = 0; i < nu_copy; ++i) {
      us[static_cast<std::size_t>(k)][i] =
          prev.u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
    }
  }

  // Skip the cold-seed on the very next Solve — we have a warm trajectory.
  first_solve_after_build_ = false;
  (void)ocp;
}

void MPCHandlerCore::PackRiccatiGain(std::size_t k, MPCSolution& out) const noexcept {
  // `out` is a reused slot, so a node we cannot pack must not keep the gain
  // of an earlier solve under this solve's dimensions: clear it instead.
  std::array<double, MPCSolution::kRiccatiGainFlatSize>& flat = out.K_riccati[k];
  const std::vector<Eigen::MatrixXd>& gains = solver_->results_.gains_;
  if (k >= gains.size()) {
    flat.fill(0.0);
    return;
  }
  const Eigen::MatrixXd& gk = gains[k];
  if (gk.rows() < nu_ || gk.cols() < ndx_ + 1) {
    flat.fill(0.0);
    return;
  }

  // gains_[k] is (nu + nc + ndx2) × (1 + ndx1) ColMajor: column 0 is the
  // feedforward term, the top-nu × right-ndx block is the feedback gain.
  // The flat layout (row-major, stride = actual ndx) is owned by
  // PackRiccatiGainRowMajor so writer and reader share one definition.
  // Solve() has already checked nu_ and ndx_ against the capacities, so a
  // false here means the capacity check and the pack disagree — clear.
  if (!PackRiccatiGainRowMajor(gk.topRightCorner(nu_, ndx_), flat.data(), flat.size())) {
    flat.fill(0.0);
  }
}

void MPCHandlerCore::PackSolution(const OCPHandlerBase& ocp, const MPCStateSnapshot& state,
                                  MPCSolution& out, bool converged, bool initial_solve,
                                  std::uint64_t solve_duration_ns) const noexcept {
  const auto& results = solver_->results_;
  const auto& xs = results.xs;
  const auto& us = results.us;

  out.timestamp_ns =
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  out.solve_duration_ns = solve_duration_ns;
  out.horizon_length = horizon_length_;
  out.dt_node = dt_;
  // The solver returned finite iterates (the caller rejected NaN), so a
  // non-converged solve is a budget-exhausted, still-usable one.
  MarkSolved(out, converged);
  out.initial_solve = initial_solve;
  out.iterations = static_cast<int>(results.num_iters);
  out.residuals.prim_inf = results.prim_infeas;
  out.residuals.dual_inf = results.dual_infeas;
  out.residuals.traj_cost = results.traj_cost_;
  out.residuals.merit = results.merit_value_;
  out.observation_stamp_ns = state.timestamp_ns;
  out.observation_sequence = state.sequence;
  out.model_epoch = state.model_epoch;
  // plan_id is stamped by the publisher; the receive stamp by the consumer.
  out.nq = nq_;
  out.nv = nv_;
  out.nu = nu_;
  out.nx = nq_ + nv_;
  out.ndx = ndx_;
  out.n_contact_vars = n_contact_vars_;

  // State + control trajectories.
  const std::size_t H = static_cast<std::size_t>(horizon_length_);
  for (std::size_t k = 0; k <= H && k < xs.size(); ++k) {
    const auto& x = xs[k];
    if (x.size() == nq_ + nv_) {
      for (int i = 0; i < nq_; ++i) {
        out.q_traj[k][static_cast<std::size_t>(i)] = x[i];
      }
      for (int i = 0; i < nv_; ++i) {
        out.v_traj[k][static_cast<std::size_t>(i)] = x[nq_ + i];
      }
    }
  }
  for (std::size_t k = 0; k < H && k < us.size(); ++k) {
    const auto& u = us[k];
    if (u.size() == nu_) {
      for (int i = 0; i < nu_; ++i) {
        out.u_traj[k][static_cast<std::size_t>(i)] = u[i];
      }
    }
  }

  // Feed-forward acceleration: a copy of τ for the joint-torque OCPs (the
  // whole-body layer downstream recomputes the mapping); not defined yet for
  // a kinodynamic one, so it is shipped as 0 rather than left stale.
  // Contact forces: the wrench block at the front of a kinodynamic `u`
  // (inactive slots included — the consumer masks them by node mode); a
  // joint-torque OCP ships none.
  (void)ocp;
  const bool kinodynamic = control_model_ == ControlModel::kKinodynamicWrench;
  const std::size_t n_lambda = std::min(static_cast<std::size_t>(n_contact_vars_),
                                        static_cast<std::size_t>(kMaxContactVars));
  for (std::size_t k = 0; k < H; ++k) {
    for (int i = 0; i < nv_; ++i) {
      out.a_ff_traj[k][static_cast<std::size_t>(i)] =
          (kinodynamic || i >= nu_) ? 0.0 : out.u_traj[k][static_cast<std::size_t>(i)];
    }
    for (std::size_t i = 0; i < n_lambda; ++i) {
      out.lambda_traj[k][i] = kinodynamic ? out.u_traj[k][i] : 0.0;
    }
  }

  // Riccati gains — only the leading nodes are shipped (see kMaxGainNodes).
  out.gain_nodes = std::min(horizon_length_, kMaxGainNodes);
  for (std::size_t k = 0; k < static_cast<std::size_t>(out.gain_nodes); ++k) {
    PackRiccatiGain(k, out);
  }
}

}  // namespace rtc::mpc::internal
