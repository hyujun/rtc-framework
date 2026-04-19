/// @file mpc_handler_core.cpp
/// @brief Implementation of the shared MPC solve pipeline used by both
///        `LightContactMPC` and `ContactRichMPC`.

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
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <chrono>

namespace rtc::mpc::internal {

namespace {

using Solver = aligator::SolverProxDDPTpl<double>;

/// @brief Clamp an int `v` against the compile-time capacity `cap`,
///        returning true when the input fit.
[[nodiscard]] bool CapacityOk(int v, int cap) noexcept {
  return v >= 0 && v <= cap;
}

} // namespace

void MPCHandlerCore::RecomputeContactVarDim(
    const RobotModelHandler &model) noexcept {
  int total = 0;
  for (const auto &frame : model.contact_frames()) {
    total += frame.dim;
  }
  n_contact_vars_ = total;
}

MPCInitError MPCHandlerCore::Init(const MPCSolverConfig &solver_cfg,
                                  const RobotModelHandler &model,
                                  const OCPLimits &limits,
                                  const PhaseContext &initial_ctx,
                                  OCPHandlerBase &ocp) noexcept {
  initialised_ = false;

  if (!model.Initialised()) {
    return MPCInitError::kModelNotInitialised;
  }
  if (solver_cfg.prim_tol <= 0.0 || solver_cfg.dual_tol <= 0.0 ||
      solver_cfg.mu_init <= 0.0 || solver_cfg.max_iters <= 0 ||
      solver_cfg.max_al_iters <= 0) {
    return MPCInitError::kInvalidSolverConfig;
  }
  if (initial_ctx.ocp_type != ocp.ocp_type()) {
    return MPCInitError::kOcpTypeMismatch;
  }

  const OCPBuildError be = ocp.Build(initial_ctx, model, limits);
  if (be != OCPBuildError::kNoError) {
    return MPCInitError::kOCPBuildFailed;
  }

  try {
    solver_ = std::make_unique<Solver>(
        solver_cfg.prim_tol, solver_cfg.mu_init,
        static_cast<std::size_t>(solver_cfg.max_iters));
    solver_->setDualTolerance(solver_cfg.dual_tol);
    solver_->max_al_iters = static_cast<std::size_t>(solver_cfg.max_al_iters);
    solver_->verbose_ =
        solver_cfg.verbose ? aligator::VERBOSE : aligator::QUIET;
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
  ndx_ = 2 * nv_; // fixed-base: state-tangent is [v; a]
  RecomputeContactVarDim(model);

  x_current_ = Eigen::VectorXd::Zero(nq_ + nv_);
  q_current_ = Eigen::VectorXd::Zero(nq_);
  tau_g_ = Eigen::VectorXd::Zero(nv_);
  pdata_ = std::make_unique<pinocchio::Data>(model.model());

  // Cache dt from the first cost_config so MPCSolution carries a valid
  // spacing; UpdateReferences / Build on later ticks may update it.
  dt_ = initial_ctx.cost_config.dt;

  // Only ContactRich needs gravity-comp τ to skirt Risk #14. LightContact's
  // rigid-contact prox solve doesn't need it and is actually faster to
  // converge from zero controls.
  use_gravity_comp_seed_ = (ocp.ocp_type() == std::string_view{"contact_rich"});

  // Allocate warm-start buffers matching the solver's internal shapes.
  xs_warm_.assign(static_cast<std::size_t>(horizon_length_ + 1),
                  Eigen::VectorXd::Zero(nq_ + nv_));
  us_warm_.assign(static_cast<std::size_t>(horizon_length_),
                  Eigen::VectorXd::Zero(nv_));

  first_solve_after_build_ = true;
  initialised_ = true;
  return MPCInitError::kNoError;
}

void MPCHandlerCore::ColdSeedGuess(const Eigen::VectorXd &q_current,
                                   const Eigen::VectorXd &x_current) noexcept {
  // Populate xs_warm_: broadcast current state across every node.
  for (auto &x : xs_warm_) {
    if (x.size() == x_current.size()) {
      x = x_current;
    }
  }
  if (use_gravity_comp_seed_) {
    // Risk #14 mitigation for ContactRich: gravity-comp τ drives the
    // constraint-force equilibrium λ → 0 at neutral pose.
    pinocchio::computeGeneralizedGravity(model_->model(), *pdata_, q_current);
    tau_g_ = pdata_->g;
    for (auto &u : us_warm_) {
      if (u.size() == tau_g_.size()) {
        u = tau_g_;
      }
    }
  } else {
    // LightContact: zero torque is the cheaper seed — no self-gravity
    // drift to wrestle the cost landscape against.
    for (auto &u : us_warm_) {
      u.setZero();
    }
  }
}

MPCSolveError MPCHandlerCore::Solve(const PhaseContext &ctx,
                                    const MPCStateSnapshot &state,
                                    OCPHandlerBase &ocp,
                                    MPCSolution &out) noexcept {
  if (!initialised_ || solver_ == nullptr || model_ == nullptr) {
    return MPCSolveError::kNotInitialised;
  }
  if (state.nq != nq_ || state.nv != nv_) {
    return MPCSolveError::kStateDimMismatch;
  }
  if (ctx.ocp_type != ocp.ocp_type()) {
    return MPCSolveError::kRebuildRequired;
  }
  if (!CapacityOk(horizon_length_, kMaxHorizon) || !CapacityOk(nq_, kMaxNq) ||
      !CapacityOk(nv_, kMaxNv) || !CapacityOk(nu_, kMaxNu) ||
      !CapacityOk(n_contact_vars_, kMaxContactVars)) {
    return MPCSolveError::kSolutionTruncated;
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
    // Re-size warm buffers in case horizon changed.
    xs_warm_.assign(static_cast<std::size_t>(horizon_length_ + 1),
                    Eigen::VectorXd::Zero(nq_ + nv_));
    us_warm_.assign(static_cast<std::size_t>(horizon_length_),
                    Eigen::VectorXd::Zero(nv_));
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

  if (first_solve_after_build_) {
    ColdSeedGuess(q_current_, x_current_);
    first_solve_after_build_ = false;
  } else {
    // Aligator's canonical shift-warm-start: rotates xs/us/vs/lams/gains
    // left by one node in-place (no alloc), re-seeds xs.front() = x0.
    try {
      solver_->results_.cycleAppend(ocp.problem(), x_current_);
    } catch (...) {
      return MPCSolveError::kSolverException;
    }
  }

  // Stage the warm-start in separate xs_warm_ / us_warm_ buffers so
  // Aligator's assign_no_resize doesn't alias results_.xs onto itself
  // inside check_initial_guess_and_assign.
  {
    auto &xs_r = solver_->results_.xs;
    auto &us_r = solver_->results_.us;
    if (first_solve_after_build_ || xs_warm_.size() != xs_r.size() ||
        us_warm_.size() != us_r.size()) {
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

  const auto t0 = std::chrono::steady_clock::now();
  bool converged = false;
  try {
    converged = solver_->run(ocp.problem(), xs_warm_, us_warm_);
  } catch (...) {
    return MPCSolveError::kSolverException;
  }
  const auto t1 = std::chrono::steady_clock::now();
  const auto dur_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());

  // Divergence with NaN state invalidates the solution. `converged` means
  // the solver met its tolerance; false without NaN means "used the budget,
  // still usable as a warm-start".
  const auto &xs = solver_->results_.xs;
  if (!converged && !xs.empty() && xs.front().hasNaN()) {
    return MPCSolveError::kSolverDiverged;
  }

  PackSolution(ocp, out, converged, dur_ns);
  return MPCSolveError::kNoError;
}

void MPCHandlerCore::SeedWarmStart(const MPCSolution &prev,
                                   OCPHandlerBase &ocp) noexcept {
  if (!initialised_ || solver_ == nullptr)
    return;
  if (!prev.IsValid())
    return;

  auto &xs = solver_->results_.xs;
  auto &us = solver_->results_.us;
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
    if (us[static_cast<std::size_t>(k)].size() != nv_)
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

void MPCHandlerCore::PackRiccatiGain(std::size_t k,
                                     MPCSolution &out) const noexcept {
  const auto &gains = solver_->results_.gains_;
  if (k >= gains.size())
    return;
  const auto &gk = gains[k];
  if (gk.rows() < nu_ || gk.cols() < ndx_ + 1)
    return;

  // gains_[k] is (nu + nc + ndx2) × (1 + ndx1) ColMajor.
  // Top-nu rows, right-ndx cols = feedback gain block in control-space.
  const auto block = gk.topRightCorner(nu_, ndx_);
  auto &flat = out.K_riccati[k];
  // Row-major write: K_row[i, j] = block(i, j).
  for (int i = 0; i < nu_; ++i) {
    for (int j = 0; j < ndx_; ++j) {
      flat[static_cast<std::size_t>(i) * static_cast<std::size_t>(kMaxNx) +
           static_cast<std::size_t>(j)] = block(i, j);
    }
  }
}

void MPCHandlerCore::PackSolution(
    const OCPHandlerBase &ocp, MPCSolution &out, bool converged,
    std::uint64_t solve_duration_ns) const noexcept {
  const auto &results = solver_->results_;
  const auto &xs = results.xs;
  const auto &us = results.us;

  out.timestamp_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
  out.solve_duration_ns = solve_duration_ns;
  out.horizon_length = horizon_length_;
  out.dt_node = dt_;
  out.converged = converged;
  out.iterations = static_cast<int>(results.num_iters);
  out.nq = nq_;
  out.nv = nv_;
  out.nu = nu_;
  out.nx = nq_ + nv_;
  out.n_contact_vars = n_contact_vars_;

  // State + control trajectories.
  const std::size_t H = static_cast<std::size_t>(horizon_length_);
  for (std::size_t k = 0; k <= H && k < xs.size(); ++k) {
    const auto &x = xs[k];
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
    const auto &u = us[k];
    if (u.size() == nv_) {
      for (int i = 0; i < nv_; ++i) {
        out.u_traj[k][static_cast<std::size_t>(i)] = u[i];
      }
    }
  }

  // Feedforward acceleration is identical to τ for the fixed-base
  // `u = τ` actuation model used by both OCPs (identity M·a = τ - h in
  // the kinematic sense after dividing by mass matrix; the TSID
  // downstream layer recomputes the mapping). For Phase 5 we duplicate
  // the u trajectory into a_ff_traj so MPCSolutionManager has a
  // well-formed interpolation source; a more principled mapping lands
  // in Phase 6.
  for (std::size_t k = 0; k < H; ++k) {
    for (int i = 0; i < nv_; ++i) {
      out.a_ff_traj[k][static_cast<std::size_t>(i)] =
          out.u_traj[k][static_cast<std::size_t>(i)];
    }
  }

  // λ trajectories: Aligator's proximal-constraint forces live inside
  // `results.lams` (per-stage dynamics multipliers). We flatten active
  // contact-force slots into `lambda_traj`. For Phase 5 we leave this
  // zero — Phase 6 will wire it once MPCSolutionManager's contact
  // interpolation is exercised end-to-end.
  (void)ocp;
  for (std::size_t k = 0; k < H; ++k) {
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(n_contact_vars_) && i < kMaxContactVars;
         ++i) {
      out.lambda_traj[k][i] = 0.0;
    }
  }

  // Riccati gains.
  for (std::size_t k = 0; k < H; ++k) {
    PackRiccatiGain(k, out);
  }
}

} // namespace rtc::mpc::internal
