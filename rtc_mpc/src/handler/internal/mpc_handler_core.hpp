#ifndef RTC_MPC_SRC_HANDLER_INTERNAL_MPC_HANDLER_CORE_HPP_
#define RTC_MPC_SRC_HANDLER_INTERNAL_MPC_HANDLER_CORE_HPP_

/// @file mpc_handler_core.hpp
/// @brief Shared impl for `ContactLightMPC` / `ContactRichMPC` /
///        `KinodynamicMPC`. Internal —
///        not installed, not exported, only pulled by files under
///        `rtc_mpc/src/handler/`.
///
/// Owns the Aligator `SolverProxDDP` + warm-start bookkeeping and drives
/// the per-tick solve pipeline against an arbitrary `OCPHandlerBase`. The
/// concrete handler subclasses delegate their virtual methods into
/// this core so mode-specific code stays minimal.

#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <vector>

namespace rtc::mpc::internal {

/// @brief Non-owning, non-copyable helper holding the solver and warm-start
///        workspace used by the concrete `MPCHandlerBase` subclasses.
///
/// Lifetime: one instance per concrete handler. Owns its `SolverProxDDP`.
/// Does NOT own the `OCPHandlerBase` — the subclass owns it and passes it
/// by reference into every core call.
class MPCHandlerCore {
 public:
  MPCHandlerCore() = default;
  ~MPCHandlerCore() = default;

  MPCHandlerCore(const MPCHandlerCore&) = delete;
  MPCHandlerCore& operator=(const MPCHandlerCore&) = delete;
  MPCHandlerCore(MPCHandlerCore&&) = delete;
  MPCHandlerCore& operator=(MPCHandlerCore&&) = delete;

  /// @brief Build the OCP through @p ocp and set up the solver. Allocates.
  [[nodiscard]] MPCInitError Init(const MPCSolverConfig& solver_cfg, const RobotModelHandler& model,
                                  const OCPLimits& limits, const PhaseContext& initial_ctx,
                                  OCPHandlerBase& ocp) noexcept;

  /// @brief One MPC tick: (optional) rebuild, warm-start shift, solve,
  ///        pack `out`. `noexcept`; never allocates on steady-state
  ///        topology after first Init.
  [[nodiscard]] MPCSolveError Solve(const PhaseContext& ctx, const MPCStateSnapshot& state,
                                    OCPHandlerBase& ocp, MPCSolution& out) noexcept;

  /// @brief Copy xs/us from @p prev into the solver's `results_` so the
  ///        next `Solve` warm-starts from it. Used by `MPCFactory` to
  ///        transfer a trajectory across a cross-mode handler swap.
  void SeedWarmStart(const MPCSolution& prev, OCPHandlerBase& ocp) noexcept;

  [[nodiscard]] bool Initialised() const noexcept { return initialised_; }

  [[nodiscard]] int horizon_length() const noexcept { return horizon_length_; }

  [[nodiscard]] int nq() const noexcept { return nq_; }

  [[nodiscard]] int nv() const noexcept { return nv_; }

  [[nodiscard]] int nu() const noexcept { return nu_; }

  [[nodiscard]] int n_contact_vars() const noexcept { return n_contact_vars_; }

 private:
  /// @brief Solve body; `Solve` wraps it to stamp `out.status` once from the
  ///        returned error.
  [[nodiscard]] MPCSolveError SolveImpl(const PhaseContext& ctx, const MPCStateSnapshot& state,
                                        OCPHandlerBase& ocp, MPCSolution& out) noexcept;

  /// @brief True when every state, control and multiplier in `results_`, the
  ///        shipped part of every gain node and the residual summary are
  ///        finite. Non-allocating.
  [[nodiscard]] bool SolutionFinite() const noexcept;

  /// @brief Overwrite a rejected solve's iterates in `results_` (states with
  ///        the measured state, everything else with zero) and cold-seed the
  ///        next tick, so nothing non-finite survives into a later warm
  ///        start. Non-allocating.
  void DiscardIterates() noexcept;

  /// @brief Recompute @p n_contact_vars_ from the model's configured
  ///        `ContactFrameInfo` list — the sum of per-frame `dim` values.
  void RecomputeContactVarDim(const RobotModelHandler& model) noexcept;

  /// @brief Fill `results_.xs` with broadcast x0 and `results_.us` with
  ///        gravity-comp τ (Risk #14 cold-seed). Called on first solve
  ///        after Init or a topology rebuild. Non-allocating — uses the
  ///        pre-built `pdata_` and pre-allocated tau buffer.
  void ColdSeedGuess(const Eigen::VectorXd& q_current, const Eigen::VectorXd& x_current) noexcept;

  /// @brief Copy the top-nu × right-ndx block of `results_.gains_[k]`
  ///        (ColMajor) into `out.K_riccati[k]` in the shared row-major
  ///        layout (stride = ndx, see `PackRiccatiGainRowMajor`). `ndx_` is
  ///        the model's state-tangent dim (`2·nv`, set at Init), which is
  ///        also what the OCP's multibody phase space uses. A node that
  ///        cannot be packed is zeroed, never left stale.
  void PackRiccatiGain(std::size_t k, MPCSolution& out) const noexcept;

  /// @brief Pack xs/us/lams/K_riccati + metadata (status, residuals, the
  ///        provenance of @p state) into @p out after a successful solve.
  ///        Non-allocating.
  void PackSolution(const OCPHandlerBase& ocp, const MPCStateSnapshot& state, MPCSolution& out,
                    bool converged, std::uint64_t solve_duration_ns) const noexcept;

  // ── State ────────────────────────────────────────────────────────────
  bool initialised_{false};
  bool first_solve_after_build_{true};

  const RobotModelHandler* model_{nullptr};  // non-owning; set at Init
  OCPLimits limits_{};
  MPCSolverConfig solver_cfg_{};

  // Allocated in Init from the problem returned by `ocp.Build`.
  std::unique_ptr<aligator::SolverProxDDPTpl<double>> solver_{};

  int horizon_length_{0};
  int nq_{0};
  int nv_{0};
  int nu_{0};
  int ndx_{0};  // state-tangent dim for Riccati feedback columns
  int n_contact_vars_{0};
  double dt_{0.0};                     // node spacing (propagated to MPCSolution)
  bool use_gravity_comp_seed_{false};  // the OCP asked for a gravity-comp cold seed
  ControlModel control_model_{ControlModel::kJointTorque};

  // Pre-allocated scratch for the solve path.
  Eigen::VectorXd x_current_{};  // nq+nv
  Eigen::VectorXd q_current_{};  // nq
  Eigen::VectorXd tau_g_{};      // nv — gravity-comp buffer (joint torque: nu == nv)

  // Separate warm-start buffers passed to Aligator's `run(problem, xs, us)`
  // to avoid xs_in == xs_out aliasing quirks in assign_no_resize.
  std::vector<Eigen::VectorXd> xs_warm_{};
  std::vector<Eigen::VectorXd> us_warm_{};

  // Pinocchio data reused across ticks to keep RNEA alloc-free.
  std::unique_ptr<pinocchio::Data> pdata_{};
};

}  // namespace rtc::mpc::internal

#endif  // RTC_MPC_SRC_HANDLER_INTERNAL_MPC_HANDLER_CORE_HPP_
