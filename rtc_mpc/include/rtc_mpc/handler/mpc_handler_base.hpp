#ifndef RTC_MPC_HANDLER_MPC_HANDLER_BASE_HPP_
#define RTC_MPC_HANDLER_MPC_HANDLER_BASE_HPP_

/// @file mpc_handler_base.hpp
/// @brief Abstract MPC solve orchestrator — owns an `OCPHandlerBase` plus an
///        Aligator `SolverProxDDP` instance and drives warm-started solves.
///
/// Phase 5 contract (`git log --grep='rtc_mpc Phase 5'`):
/// - `Init` (off-RT): build the underlying OCP via `OCPHandlerBase::Build`,
///   call `solver.setup(problem)`, pre-allocate every workspace vector used
///   by `Solve`. Must be called once before the MPC thread starts ticking.
/// - `Solve` (off-RT but latency-sensitive): consume the latest RT state
///   snapshot + current `PhaseContext`, run one Aligator solve, pack the
///   result into the trivially-copyable `MPCSolution` that the MPC thread
///   ships to the RT side via `TripleBuffer`.
/// - `Solve` must not allocate after `Init` on steady-state topology. It
///   must not throw — Aligator calls are wrapped in `try/catch` and
///   converted into `MPCSolveError` enum values.
///
/// Warm-start strategy (common across concrete handlers):
/// - First solve after `Init`: cold seed via gravity-comp τ (closes Risk
///   #14 for contact-rich handlers) + `solver.run(problem, xs, us)`.
/// - Subsequent solves on unchanged topology: shift `results_` left via
///   `ResultsTpl::cycleAppend(problem, x_current)` and call
///   `solver.run(problem)` with empty inits so the solver re-uses
///   `results_.xs/us/vs/lams` directly — this is Aligator's canonical MPC
///   shift-warm-start path and does not allocate.
/// - On topology change (`ctx.phase_changed` with new horizon / active
///   contacts / weight-zero crossing): concrete handler re-Builds the OCP
///   and re-`setup`s the solver, then seeds the new workspace from the
///   previous solution where compatible. Cross-OCP-type transitions are
///   the factory's job (destroy + construct a different handler).
///
/// Cross-phase invariants upheld:
/// - Robot-agnostic: no robot identifiers touched here; everything flows
///   via `RobotModelHandler` + `PhaseContext` + `OCPLimits` (CLAUDE.md
///   rtc_* rule).
/// - RT-safety on `Solve`: no heap alloc, no throw, no mutex after first
///   tick on steady-state topology (CLAUDE.md Hard Rules +
///   `.claude/rules/rt-safety.md`).

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <yaml-cpp/yaml.h>

#include <string_view>

namespace rtc::mpc {

/// @brief Non-cost solver + handler configuration loaded once at
///        `MPCHandlerBase::Init`.
///
/// Proximal-DDP tolerances mirror Aligator's `SolverProxDDPTpl` ctor args.
/// Defaults are intentionally looser than Phase 3/4 fixtures so the typical
/// 20-node Panda solve converges in tens of iterations rather than hundreds.
struct MPCSolverConfig {
  double prim_tol{1e-3}; ///< primal infeasibility target
  double dual_tol{1e-2}; ///< dual infeasibility target
  double mu_init{1e-2};  ///< AL penalty initial value
  int max_iters{30};     ///< per-solve iteration ceiling
  int max_al_iters{20};  ///< AL outer-loop ceiling
  bool verbose{false};   ///< route Aligator's own stdout log
};

/// @brief Failure modes for @ref MPCHandlerBase::Init. Allocation is
///        permitted on this path; callers inspect the enum and either
///        retry with corrected inputs or abort before entering the MPC
///        loop.
enum class MPCInitError {
  kNoError = 0,
  kModelNotInitialised, ///< RobotModelHandler.Initialised() == false
  kOCPBuildFailed,      ///< underlying OCPHandlerBase::Build rejected
  kSolverSetupFailed,   ///< Aligator's setup() threw (caught internally)
  kInvalidSolverConfig, ///< tolerances / max_iters out of range
  kOcpTypeMismatch,     ///< ctx.ocp_type != this handler's dispatch key
};

/// @brief Failure modes for @ref MPCHandlerBase::Solve. **Never thrown** —
///        the solve path is `noexcept`; failures surface via this enum.
enum class MPCSolveError {
  kNoError = 0,
  kNotInitialised,    ///< Solve called before successful Init
  kStateDimMismatch,  ///< snapshot.{nq,nv} differ from handler's model
  kRebuildRequired,   ///< phase_changed with cross-mode OCP type — the
                      ///< MPCFactory / orchestration layer must swap the
                      ///< handler, not this instance
  kOCPRebuildFailed,  ///< in-handler topology rebuild (horizon / contacts
                      ///< / weight-zero crossing) failed
  kSolverException,   ///< Aligator threw during `run()` (caught, logged
                      ///< once in Debug builds, returned as an error so
                      ///< the RT thread can fall back to its last valid
                      ///< solution)
  kSolverDiverged,    ///< `run()` returned `false` AND NaN state — the
                      ///< produced `out` is NOT valid
  kSolutionTruncated, ///< horizon_length or a dim exceeds the fixed
                      ///< `MPCSolution` capacity (kMaxHorizon, kMaxNu,
                      ///< …). Indicates a configuration mistake, not a
                      ///< runtime condition.
};

/// @brief Abstract MPC handler. Concrete implementations pin themselves to
///        a single OCP dispatch key (`"light_contact"`, `"contact_rich"`)
///        and own exactly one OCP + one solver. Cross-key transitions are
///        done at the orchestration layer via `MPCFactory`.
class MPCHandlerBase {
public:
  MPCHandlerBase() = default;
  virtual ~MPCHandlerBase() = default;

  MPCHandlerBase(const MPCHandlerBase &) = delete;
  MPCHandlerBase &operator=(const MPCHandlerBase &) = delete;
  MPCHandlerBase(MPCHandlerBase &&) = delete;
  MPCHandlerBase &operator=(MPCHandlerBase &&) = delete;

  /// @brief Build the underlying OCP and set up the solver. Allocation-
  ///        heavy, called once off-RT before the MPC thread starts.
  ///
  /// @param solver_cfg  tolerances / iter ceilings (non-cost)
  /// @param limits      non-cost OCP limits (friction_mu, u_min/u_max)
  /// @param model       initialised RobotModelHandler (must outlive this)
  /// @param initial_ctx first phase context — the OCP is `Build()`ed
  ///                    against this and the solver is seeded from a
  ///                    gravity-comp control trajectory.
  /// @return `kNoError` on success; on failure the handler stays in the
  ///         uninitialised state (`Initialised() == false`).
  [[nodiscard]] virtual MPCInitError
  Init(const MPCSolverConfig &solver_cfg, const RobotModelHandler &model,
       const OCPLimits &limits, const PhaseContext &initial_ctx) noexcept = 0;

  /// @brief Run one MPC tick. Called off the 500 Hz RT path, but on a
  ///        latency-sensitive MPC thread — must be `noexcept` and must not
  ///        allocate after the first call on steady-state topology.
  ///
  /// @param ctx    current phase context (same `ocp_type` as this handler).
  ///               `phase_changed==true` + in-type topology change triggers
  ///               a rebuild; a cross-type change returns
  ///               `kRebuildRequired` so the orchestrator can swap handlers.
  /// @param state  latest RT-side state snapshot (q, v, nq, nv, timestamp)
  /// @param out    filled on success: `xs/us/λ/K_riccati/converged/iters`
  /// @return enum; `out` is only guaranteed valid when `kNoError`.
  [[nodiscard]] virtual MPCSolveError Solve(const PhaseContext &ctx,
                                            const MPCStateSnapshot &state,
                                            MPCSolution &out) noexcept = 0;

  /// @return true once `Init` has succeeded.
  [[nodiscard]] virtual bool Initialised() const noexcept = 0;

  /// @return `"light_contact"` or `"contact_rich"` — matches the underlying
  ///         OCP handler. Used by `MPCFactory` for cross-handler dispatch.
  [[nodiscard]] virtual std::string_view ocp_type() const noexcept = 0;

  /// @return current horizon length (may differ from `Init` after an
  ///         in-type topology rebuild).
  [[nodiscard]] virtual int horizon_length() const noexcept = 0;

  /// @return model-side dim accessors, valid after `Init`.
  [[nodiscard]] virtual int nq() const noexcept = 0;
  [[nodiscard]] virtual int nv() const noexcept = 0;
  [[nodiscard]] virtual int nu() const noexcept = 0;
  [[nodiscard]] virtual int n_contact_vars() const noexcept = 0;

  /// @brief Seed the solver with the given state trajectory for the next
  ///        Solve call. Used by `MPCFactory` after a cross-mode swap to
  ///        transfer warm-start data from the outgoing handler.
  ///
  /// The caller supplies trajectories of length `horizon_length()+1` for
  /// xs and `horizon_length()` for us; sizes outside that range are
  /// silently truncated / zero-padded (best-effort cross-topology seed).
  /// No-op if not Initialised.
  virtual void SeedWarmStart(const MPCSolution &prev_solution) noexcept = 0;
};

} // namespace rtc::mpc

#endif // RTC_MPC_HANDLER_MPC_HANDLER_BASE_HPP_
