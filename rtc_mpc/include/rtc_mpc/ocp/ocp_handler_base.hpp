#ifndef RTC_MPC_OCP_OCP_HANDLER_BASE_HPP_
#define RTC_MPC_OCP_OCP_HANDLER_BASE_HPP_

/// @file ocp_handler_base.hpp
/// @brief Abstract OCP builder interface — `PhaseContext → TrajOptProblem`.
///
/// `OCPHandlerBase` is the boundary between the phase-agnostic MPC
/// orchestration layer (Phase 5 `MPCHandler`) and concrete solver-specific
/// OCP assembly (Aligator-backed `LightContactOCP`, `ContactRichOCP`). It
/// lives on the **OCP build / reconfigure path**, not the 500Hz RT loop.
///
/// Two-method API split:
/// - `Build` — full topology rebuild (allocates). Called on first init and
///   whenever stage topology changes (horizon_length, per-stage active
///   contact set, `ocp_type`, nq / nv).
/// - `UpdateReferences` — mutate per-stage cost targets in place, no
///   allocation after first `Build`. Rejects topology changes so callers
///   must `Build` when topology diverges.
///
/// All methods are `noexcept`. Aligator ctors may throw; concrete handlers
/// wrap those calls in try/catch and convert to
/// `OCPBuildError::kAligatorInstantiationFailure`.

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/traj-opt-problem.hpp>
#pragma GCC diagnostic pop

#include <string_view>

namespace rtc::mpc {

/// @brief Non-cost, non-per-phase OCP limits. Loaded once from YAML at
///        handler construction; **not** an extension of `PhaseCostConfig`
///        (keeps the Phase 2 POD frozen).
///
/// Empty `u_min` / `u_max` vectors mean "no control box applied". If one
/// is populated both must match `RobotModelHandler::nu()`; mismatch causes
/// `OCPBuildError::kLimitsDimMismatch`.
struct OCPLimits {
  Eigen::VectorXd u_min{}; ///< size `nu`, or empty = unlimited
  Eigen::VectorXd u_max{}; ///< size `nu`, or empty = unlimited
  double friction_mu{0.7}; ///< shared across active contacts
  /// Number of polyhedral facets for friction-cone approximation.
  ///
  /// @note Currently UNUSED by ContactRichOCP, which uses
  ///       aligator::MultibodyFrictionConeResidualTpl — a smooth 2D
  ///       conic residual (‖f_tan‖ ≤ μ·f_n enforced via sqrt, no facets).
  ///       Retained in OCPLimits as a reserved slot for a future
  ///       polyhedral friction-cone variant (e.g., dense-QP compatible
  ///       or multi-facet analytic Jacobian). See Phase 4 Spike Notes Q3
  ///       in docs/mpc_implementation_progress.md.
  int n_friction_facets{4};
};

/// @brief Failure modes for `OCPHandlerBase::Build` / `::UpdateReferences`.
enum class OCPBuildError {
  kNoError = 0,
  kModelNotInitialised,      ///< passed-in RobotModelHandler is unusable
  kInvalidPhaseContext,      ///< unknown `ocp_type`, or topology change on
                             ///< UpdateReferences
  kInvalidCostConfig,        ///< PhaseCostConfig sizes / horizon / dt invalid
  kContactPlanModelMismatch, ///< frame_id in contact plan not present in model
  kLimitsDimMismatch,        ///< u_min/u_max non-empty with size != nu
  kOverlappingContactPhases, ///< ContactPlan::phases have overlapping time
                             ///< intervals
  kAligatorInstantiationFailure, ///< an Aligator ctor threw (caught internally)
};

/// @brief Abstract OCP builder. Concrete implementations map the OCP onto a
///        specific dynamics class (e.g. LightContactOCP, ContactRichOCP).
class OCPHandlerBase {
public:
  OCPHandlerBase() = default;
  virtual ~OCPHandlerBase() = default;

  OCPHandlerBase(const OCPHandlerBase &) = delete;
  OCPHandlerBase &operator=(const OCPHandlerBase &) = delete;
  OCPHandlerBase(OCPHandlerBase &&) = delete;
  OCPHandlerBase &operator=(OCPHandlerBase &&) = delete;

  /// @brief Full problem rebuild. Off-RT path; allocation is permitted.
  /// @param ctx     phase snapshot (contact plan + cost config + targets)
  /// @param model   initialised RobotModelHandler (its model must outlive the
  /// handler)
  /// @param limits  non-cost limits loaded once at handler init
  /// @return kNoError on success; on failure, the handler may be left
  ///         partially constructed — callers should treat `Built() == false`
  ///         and re-Build with corrected inputs.
  [[nodiscard]] virtual OCPBuildError
  Build(const PhaseContext &ctx, const RobotModelHandler &model,
        const OCPLimits &limits) noexcept = 0;

  /// @brief Mutate per-stage cost references without touching stage topology.
  ///
  /// Contract: **must not allocate** after the first successful `Build`. If
  /// `ctx` implies a topology change (horizon_length, dt, active-contact set,
  /// ocp_type), returns `kInvalidPhaseContext` **without** modifying stored
  /// state — caller must `Build` again. Weight crossings (0 ↔ positive)
  /// also count as topology changes because they add or remove cost terms.
  [[nodiscard]] virtual OCPBuildError
  UpdateReferences(const PhaseContext &ctx) noexcept = 0;

  /// @return true once a successful `Build` has completed.
  [[nodiscard]] virtual bool Built() const noexcept = 0;

  /// @return non-owning reference to the internal TrajOptProblem for solver
  ///         binding (Phase 5 `MPCHandler` calls `solver.setup(problem())`).
  ///         Precondition: `Built() == true`.
  [[nodiscard]] virtual aligator::TrajOptProblemTpl<double> &problem() = 0;

  /// @return horizon length used by the last successful Build.
  [[nodiscard]] virtual int horizon_length() const noexcept = 0;

  /// @return dispatch key string literal (`"light_contact"`, `"contact_rich"`).
  ///         Must match `PhaseContext::ocp_type` for Build to succeed.
  [[nodiscard]] virtual std::string_view ocp_type() const noexcept = 0;
};

} // namespace rtc::mpc

#endif // RTC_MPC_OCP_OCP_HANDLER_BASE_HPP_
