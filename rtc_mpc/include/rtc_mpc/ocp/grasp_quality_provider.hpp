#pragma once

/// @file grasp_quality_provider.hpp
/// @brief Pure-virtual extension seam for grasp-quality cost terms on the
///        running (and optionally terminal) stages of a `ContactRichOCP`.
///
/// Phase 4 ships this interface only — **no concrete implementation is
/// provided**. Concrete providers land in Phase 4.5+ alongside the first
/// real consumer (e.g. a gripper-specific force-closure or reachability
/// residual), where the residual's signature and dependencies can be
/// co-designed with an actual use-case rather than speculated up-front.
///
/// The implementation cost of shipping a useful grasp-quality residual is
/// substantial — it typically requires a custom `aligator::StageFunctionTpl`
/// subclass with custom Jacobians over contact frames — and is
/// explicitly out of scope for Phase 4 (see Open Decision #4 in
/// `docs/mpc_implementation_progress.md`). The seam exists so that a
/// downstream provider can plug in without re-touching `ContactRichOCP`.
///
/// ## Where the hooks are invoked
///
/// `ContactRichOCP::Build` calls `AppendRunningCost` once per running
/// stage, **after** `cost_factory::BuildRunningCost` has populated the
/// baseline (frame placement / state reg / control reg) **and after** the
/// per-contact `ContactForceResidualTpl` terms have been attached. The
/// provider receives the stage's active contact frame ids (may be empty
/// on free-flight stages) so it can key residuals on the same contact set
/// the rest of the OCP sees.
///
/// `AppendTerminalCost` is optional (default no-op) and runs after
/// `cost_factory::BuildTerminalCost`. Control-dependent residuals cannot
/// appear on the terminal stage — the terminal cost takes `x` only.
///
/// `UpdateReferences` runs from `ContactRichOCP::UpdateReferences` on
/// every tick where topology is unchanged. Reference mutation must be
/// alloc-free; see the ownership contract below.
///
/// ## Ownership contract
///
/// Residuals that a provider constructs inside `Append*Cost` are added to
/// the caller-owned `StageCost::stack` (an `aligator::CostStackTpl<double>`).
/// Aligator stores components as `xyz::polymorphic<T>` — they are
/// **value-copied** when the `CostStack` is later moved into a
/// `StageModelTpl`, so any raw pointer the provider caches inside
/// `Append*Cost` **will dangle** before the first `UpdateReferences` tick.
/// The same trap was observed and documented in the Phase 3 completion
/// notes; the safe pattern is identical:
///
/// - Add residuals via `stack.addCost(std::string{key}, QuadCost, weight)`
///   during `Append*Cost`. Do NOT save raw pointers at this time.
/// - Retrieve handles **after** `TrajOptProblem` construction through the
///   polymorphic chain:
///   `stage.getCost<CostStackTpl<double>>()
///        ->getComponent<QuadraticResidualCostTpl<double>>(key)
///        ->getResidual<Derived>();`
///
/// `ContactRichOCP` does not perform this retrieval for providers in
/// Phase 4; providers that need per-tick mutation must do the retrieval
/// themselves (typically inside their own `UpdateReferences`) or deposit
/// handles into their own state. A production-ready retrieval helper on
/// the OCP side is deferred to Phase 4.5 when the interface first has a
/// concrete consumer.
///
/// ## RT-safety
///
/// All methods execute on the **off-RT build / reconfigure path** (same
/// context as `OCPHandlerBase::Build` / `UpdateReferences`). They never
/// run on the 500 Hz solve loop. `noexcept` is still required on every
/// override so that Aligator's no-throw contract for stage construction
/// propagates through the provider.

#include "rtc_mpc/ocp/cost_factory.hpp"     // StageCost
#include "rtc_mpc/ocp/ocp_handler_base.hpp" // OCPBuildError
#include "rtc_mpc/phase/phase_context.hpp"  // PhaseContext

#include <string_view>
#include <vector>

namespace rtc::mpc {

class RobotModelHandler;

/// @brief Pure-virtual seam for adding grasp-quality residuals to a
///        `ContactRichOCP` stage cost stack.
///
/// Phase 4 ships this interface with no concrete subclass; Phase 4.5+ will
/// deliver the first implementation once a real consumer exists.
class GraspQualityResidualProvider {
public:
  virtual ~GraspQualityResidualProvider() = default;

  GraspQualityResidualProvider(const GraspQualityResidualProvider &) = delete;
  GraspQualityResidualProvider &
  operator=(const GraspQualityResidualProvider &) = delete;

  /// @brief Append grasp-quality residual(s) to a running stage's cost.
  ///
  /// Called once per running stage from `ContactRichOCP::Build`, after the
  /// baseline costs from `cost_factory::BuildRunningCost` and the
  /// per-contact `ContactForceResidualTpl` entries have been added.
  ///
  /// @param stage_cost  Mutable stage cost to extend. Add residuals through
  ///                    `stage_cost.stack.addCost(key, QuadCost, weight)`;
  ///                    set the matching flag in `stage_cost.keys` if a
  ///                    flag exists for the added component. Do NOT cache
  ///                    raw pointers to residuals added here — Aligator
  ///                    value-copies them into the stored stage later
  ///                    (see class-level ownership contract).
  /// @param model       Initialised `RobotModelHandler`; outlives the call.
  /// @param active_contact_frame_ids  Pinocchio frame ids that are in
  ///                                   contact on this stage. Empty on
  ///                                   free-flight stages; providers should
  ///                                   no-op in that case if their residual
  ///                                   is contact-dependent.
  /// @return kNoError on success; any other value causes the surrounding
  ///         `Build` to abort and return the same code to the caller.
  [[nodiscard]] virtual OCPBuildError AppendRunningCost(
      StageCost &stage_cost, const RobotModelHandler &model,
      const std::vector<int> &active_contact_frame_ids) noexcept = 0;

  /// @brief Optional: append grasp-quality residuals to the terminal cost.
  ///
  /// Runs once from `ContactRichOCP::Build` after
  /// `cost_factory::BuildTerminalCost`. The terminal cost is `x`-only, so
  /// any control-dependent residual must no-op here. Default implementation
  /// is a no-op (returns `kNoError`) for providers that do not need a
  /// terminal contribution.
  [[nodiscard]] virtual OCPBuildError AppendTerminalCost(
      StageCost & /*stage_cost*/, const RobotModelHandler & /*model*/,
      const std::vector<int> & /*active_contact_frame_ids*/) noexcept {
    return OCPBuildError::kNoError;
  }

  /// @brief Push new reference targets into provider-owned residual handles.
  ///
  /// Called from `ContactRichOCP::UpdateReferences` on every tick where
  /// topology is unchanged. Implementations that need per-tick mutation
  /// must retrieve their residual handles via the polymorphic chain
  /// (see ownership contract) and mutate them alloc-free. Default
  /// implementation is a no-op for providers that only depend on topology.
  virtual void UpdateReferences(const PhaseContext & /*ctx*/) noexcept {}

  /// @brief Short identifier used in log lines and error diagnostics.
  ///        Must be stable for the lifetime of the provider. Example:
  ///        `"force_closure_v1"`.
  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

protected:
  GraspQualityResidualProvider() = default;
};

} // namespace rtc::mpc
