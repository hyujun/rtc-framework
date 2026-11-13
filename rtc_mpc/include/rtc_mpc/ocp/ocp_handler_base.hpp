#ifndef RTC_MPC_OCP_OCP_HANDLER_BASE_HPP_
#define RTC_MPC_OCP_OCP_HANDLER_BASE_HPP_

/// @file ocp_handler_base.hpp
/// @brief Abstract OCP builder interface — `PhaseContext → TrajOptProblem`.
///
/// `OCPHandlerBase` is the boundary between the phase-agnostic MPC
/// orchestration layer (Phase 5 `MPCHandler`) and concrete solver-specific
/// OCP assembly (Aligator-backed `ContactLightOCP`, `ContactRichOCP`,
/// `KinodynamicOCP`). It
/// lives on the **OCP build / reconfigure path**, not the RT loop.
///
/// Two-method API split:
/// - `Build` — full topology rebuild (allocates). Called on first init and
///   whenever stage topology changes.
/// - `UpdateReferences` — mutate per-stage cost targets in place, no
///   allocation after first `Build`. Rejects topology changes so callers
///   must `Build` when topology diverges.
///
/// What counts as topology is the concrete OCP's to define: horizon_length,
/// dt, `ocp_type` and nq / nv always; the joint-torque OCPs also count the
/// per-stage active contact set, while `KinodynamicOCP` switches contact
/// modes in place (every mode has the same rows and columns).
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

#include <cstddef>
#include <string_view>
#include <vector>

namespace rtc::mpc {

/// @brief Non-cost, non-per-phase OCP limits. Loaded once from YAML at
///        handler construction; **not** an extension of `PhaseCostConfig`
///        (keeps the Phase 2 POD frozen).
///
/// Empty `u_min` / `u_max` vectors mean "no control box applied". If one
/// is populated both must match `RobotModelHandler::nu()`; mismatch causes
/// `OCPBuildError::kLimitsDimMismatch`.
struct OCPLimits {
  Eigen::VectorXd u_min{};  ///< size `nu`, or empty = unlimited
  Eigen::VectorXd u_max{};  ///< size `nu`, or empty = unlimited
  double friction_mu{0.7};  ///< shared across active contacts
};

/// @brief Failure modes for `OCPHandlerBase::Build` / `::UpdateReferences`.
enum class OCPBuildError {
  kNoError = 0,
  kModelNotInitialised,           ///< passed-in RobotModelHandler is unusable
  kInvalidPhaseContext,           ///< unknown `ocp_type`, or topology change on
                                  ///< UpdateReferences
  kInvalidCostConfig,             ///< PhaseCostConfig sizes / horizon / dt invalid
  kContactPlanModelMismatch,      ///< frame_id in contact plan not present in model
  kLimitsDimMismatch,             ///< u_min/u_max non-empty with size != nu, or
                                  ///< non-empty for an OCP without a control box
  kOverlappingContactPhases,      ///< ContactPlan::phases have overlapping time
                                  ///< intervals
  kAligatorInstantiationFailure,  ///< an Aligator ctor threw (caught internally)
  kMissingHorizonReference,       ///< an OCP that tracks `PhaseContext::horizon` got null
  kInvalidHorizonReference,       ///< a horizon of the right shape with a value the OCP
                                  ///< refuses (non-finite, non-unit, out of range)
};

/// @brief What the OCP's control vector `u` is — the handler core sizes,
///        refuses and packs by it.
enum class ControlModel {
  kJointTorque,        ///< `u = τ` (size `nv`) on a fixed base
  kKinodynamicWrench,  ///< `u = [W_0 … W_{nc−1}; q̈_j]` on a floating base (`WbLayout`)
};

/// @brief The initial guess the handler core gives the solver after a build.
enum class ColdSeed {
  kZeroControl,          ///< measured state at every node, `u = 0`
  kGravityCompensation,  ///< measured state at every node, `u = g(q)` — joint torque
                         ///< only; a kinodynamic OCP gets the zero-control seed
  kReferenceControl,     ///< measured state at every node, `u` the OCP's own
                         ///< reference control of each node (`ReferenceControl`);
                         ///< a node it cannot give gets `u = 0`
};

/// @brief Abstract OCP builder. Concrete implementations map the OCP onto a
///        specific dynamics class (e.g. ContactLightOCP, ContactRichOCP).
class OCPHandlerBase {
 public:
  OCPHandlerBase() = default;
  virtual ~OCPHandlerBase() = default;

  OCPHandlerBase(const OCPHandlerBase&) = delete;
  OCPHandlerBase& operator=(const OCPHandlerBase&) = delete;
  OCPHandlerBase(OCPHandlerBase&&) = delete;
  OCPHandlerBase& operator=(OCPHandlerBase&&) = delete;

  /// @brief Full problem rebuild. Off-RT path; allocation is permitted.
  /// @param ctx     phase snapshot (contact plan + cost config + targets)
  /// @param model   initialised RobotModelHandler (its model must outlive the
  /// handler)
  /// @param limits  non-cost limits loaded once at handler init
  /// @return kNoError on success; on failure, the handler may be left
  ///         partially constructed — callers should treat `Built() == false`
  ///         and re-Build with corrected inputs.
  [[nodiscard]] virtual OCPBuildError Build(const PhaseContext& ctx, const RobotModelHandler& model,
                                            const OCPLimits& limits) noexcept = 0;

  /// @brief Mutate per-stage cost references without touching stage topology.
  ///
  /// Contract: **must not allocate** after the first successful `Build`. If
  /// `ctx` implies a topology change (as this OCP defines it — file comment),
  /// returns `kInvalidPhaseContext` **without** modifying stored state —
  /// caller must `Build` again. In the joint-torque OCPs weight crossings
  /// (0 ↔ positive) also count, because they add or remove cost terms. Any
  /// other error also leaves the stored state unchanged; the caller does not
  /// rebuild for it.
  [[nodiscard]] virtual OCPBuildError UpdateReferences(const PhaseContext& ctx) noexcept = 0;

  /// @return true once a successful `Build` has completed.
  [[nodiscard]] virtual bool Built() const noexcept = 0;

  /// @return non-owning reference to the internal TrajOptProblem for solver
  ///         binding (Phase 5 `MPCHandler` calls `solver.setup(problem())`).
  ///         Precondition: `Built() == true`.
  [[nodiscard]] virtual aligator::TrajOptProblemTpl<double>& problem() = 0;

  /// @return horizon length used by the last successful Build.
  [[nodiscard]] virtual int horizon_length() const noexcept = 0;

  /// @return dispatch key string literal (`"contact_light"`, `"contact_rich"`,
  ///         `"kinodynamic"`).
  ///         Must match `PhaseContext::ocp_type` for Build to succeed.
  [[nodiscard]] virtual std::string_view ocp_type() const noexcept = 0;

  /// @return what `u` is. A handler refuses a floating-base model for a
  ///         joint-torque OCP (`kFloatingBaseUnsupported`) and a fixed-base
  ///         one for a kinodynamic OCP (`kFixedBaseUnsupported`).
  [[nodiscard]] virtual ControlModel control_model() const noexcept {
    return ControlModel::kJointTorque;
  }

  /// @return whether a `kInvalidPhaseContext` from `UpdateReferences` can be
  ///        met by a `Build` of the same context (the default). An OCP whose
  ///        configuration fixes its topology returns false: the handler core
  ///        then refuses the tick (`kOCPRebuildFailed`) without a `Build` or a
  ///        solver setup, both of which allocate.
  [[nodiscard]] virtual bool RebuildsOnContextChange() const noexcept { return true; }

  /// @brief Called by the handler core right after it shifted the previous
  ///        solve's iterates one node on, before the solve: set to `+0.0` the
  ///        multipliers of the constraint rows that the stages' current modes
  ///        (as of this tick's `UpdateReferences`) make inactive. A node whose
  ///        mode differs from the one its shifted multipliers were solved under
  ///        would otherwise carry them into rows that no longer bind (an
  ///        inactive row has value and Jacobian zero, so the solver leaves its
  ///        multiplier where it is). Does not allocate.
  /// @param stage_multipliers  the solver's per-stage constraint multipliers
  ///                           (`vs`; entry k is stage k's rows, a terminal
  ///                           entry past the stages is left alone)
  /// @return how many entries it changed (the default: none — every row of
  ///         an OCP without modes is always active)
  virtual std::size_t ClearInactiveMultipliers(
      std::vector<Eigen::VectorXd>& stage_multipliers) const noexcept {
    static_cast<void>(stage_multipliers);
    return 0;
  }

  /// @return the cold-start guess this OCP wants after a build.
  [[nodiscard]] virtual ColdSeed cold_seed() const noexcept { return ColdSeed::kZeroControl; }

  /// @brief For `ColdSeed::kReferenceControl`: write stage @p k's reference
  ///        control (what its cost drives `u` toward, as of the last
  ///        `Build` / `UpdateReferences`) into @p u. Does not allocate.
  /// @return false, writing nothing, for an OCP without one, a stage out of
  ///         range or a @p u of another size (the default: always false).
  [[nodiscard]] virtual bool ReferenceControl(int k, Eigen::Ref<Eigen::VectorXd> u) const noexcept {
    static_cast<void>(k);
    static_cast<void>(u);
    return false;
  }
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_OCP_HANDLER_BASE_HPP_
