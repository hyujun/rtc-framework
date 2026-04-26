#pragma once

/// @file contact_rich_ocp.hpp
/// @brief Concrete `OCPHandlerBase` for the "ContactRich" MPC mode on a
///        fixed-base manipulator.
///
/// Shares the `MultibodyConstraintFwdDynamicsTpl` backbone with
/// `LightContactOCP`; the differentiation (Phase 4 scope; `git log
/// --grep='rtc_mpc Phase 4'`):
/// - per-active-contact `ContactForceResidualTpl` in the running cost
///   when `cfg.w_contact_force > 0`
/// - per-active-contact `MultibodyFrictionConeResidualTpl` inequality
///   constraints via `NegativeOrthantTpl` when `limits.friction_mu > 0`
/// - extension seam for grasp-quality residuals via
///   `GraspQualityResidualProvider` (pure-virtual; no concrete
///   implementation ships in Phase 4)
///
/// Layout (identical to `LightContactOCP`):
/// - state  `x = [q; v]` ∈ R^{nq+nv}
/// - control `u = τ` ∈ R^{nv} (identity actuation; contact forces emerge
///   from the rigid-contact proximal solve as Lagrange multipliers λ, NOT
///   part of `u`)
/// - Riccati gain from Aligator: `K ∈ R^{nu × ndx}` with `ndx = 2·nv`;
///   fixed-base → `nu = nv`, so `rtc::mpc::RiccatiFeedback::SetGain`
///   `accel_only` reads the full `K`.

#include "rtc_mpc/ocp/cost_factory.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/traj-opt-problem.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <string_view>
#include <vector>

// Forward-declare Aligator residual templates — we only take raw pointers to
// them in `RichStageHandles`, so full definitions are not needed at header
// parse time. The `.cpp` pulls in the concrete headers.
namespace aligator {
template <typename _Scalar> struct FramePlacementResidualTpl;
template <typename _Scalar> struct StateErrorResidualTpl;
template <typename _Scalar> struct ControlErrorResidualTpl;
template <typename _Scalar> struct ContactForceResidualTpl;
} // namespace aligator

namespace rtc::mpc {

class RobotModelHandler;
class GraspQualityResidualProvider;

/// @brief Canonical cost-stack key prefix for contact-force residuals.
///        Full key is `kCostKeyContactForcePrefix + contact_frame_name`,
///        e.g. `"contact_force::panda_leftfinger"`. Kept at this TU-local
///        scope (not in `cost_factory.hpp`) because contact-force
///        residuals are dynamics-coupled (actuation, constraint models,
///        prox settings) and therefore owned by this OCP rather than the
///        dynamics-agnostic cost factory. See Phase 4 Spike Notes Q6.
inline constexpr std::string_view kCostKeyContactForcePrefix =
    "contact_force::";

/// @brief Non-owning raw-pointer handles to residuals stored inside a
///        contact-rich `StageModel`'s polymorphic cost tree. Populated
///        **after** problem assembly; used by `UpdateReferences` for
///        alloc-free reference mutation.
///
/// Null scalar entries indicate the corresponding weight was <= 0 at
/// `Build` and the residual is absent from the stage (no lookup possible).
/// `contact_force` is parallel to `stage_active_contacts_[k]` order.
///
/// Phase 4.5+: a `grasp_quality` handle slot is intentionally reserved
/// here (provider owns the concrete residual; the OCP would reference it
/// through the polymorphic-chain retrieval pattern, not cache-on-build).
struct RichStageHandles {
  aligator::FramePlacementResidualTpl<double> *frame_placement{nullptr};
  aligator::StateErrorResidualTpl<double> *state_reg{nullptr};
  aligator::ControlErrorResidualTpl<double> *control_reg{nullptr};
  /// Parallel to `stage_active_contacts_[k]` order.
  std::vector<aligator::ContactForceResidualTpl<double> *> contact_force{};
};

/// Contact-rich MPC OCP handler. Shares the MultibodyConstraintFwdDynamicsTpl
/// backbone with LightContactOCP; adds per-active-contact
/// ContactForceResidualTpl cost terms (when w_contact_force > 0) and
/// MultibodyFrictionConeResidualTpl inequality constraints (when
/// limits.friction_mu > 0). Use for CLOSURE / HOLD / MANIPULATE phases
/// where contact is load-bearing.
///
/// @note Cold-start requirement:
///       The solver's initial guess must be seeded with a well-posed
///       control trajectory before calling solve(). Uninitialized or
///       zero initial guess with active contact constraints produces
///       NaN at the first linearization because
///       computeConstraintDynamicsDerivatives is ill-conditioned when
///       the constraint-force equilibrium λ is undefined.
///
///       For tests without a grasped object (free fingertip — the
///       Phase 4 fixture), seeding with the robot's self-gravity
///       compensation is sufficient:
///
///         Eigen::VectorXd tau_g = pinocchio::computeGeneralizedGravity(
///                                     model, data, q);
///         for (auto& u : us) u = tau_g;
///
///       This drives the constraint-force equilibrium to λ ≈ 0. For
///       real grasp scenarios (object present, contact load-bearing),
///       this self-gravity seed is a cold-start bootstrap only — the
///       solver will adjust λ to match the true contact load on the
///       first few iterations. Steady-state MPC use should rely on
///       warm-start (MPCHandler, Phase 5) which seeds from the previous
///       tick's solution (already encodes object load).
///
///       Seeding is the caller's responsibility:
///         - Phase 4: test fixture (see
///                    test_utils/solver_seeding.hpp::SeedGravityCompensation)
///         - Phase 5+: MPCHandler::SeedInitialGuess()
///
///       See Phase 4 spike Q7 and Risk #14 (`git log --grep='rtc_mpc
///       Phase 4'`, closure `6e49bc9`).
class ContactRichOCP : public OCPHandlerBase {
public:
  ContactRichOCP() = default;
  ~ContactRichOCP() override = default;

  ContactRichOCP(const ContactRichOCP &) = delete;
  ContactRichOCP &operator=(const ContactRichOCP &) = delete;
  ContactRichOCP(ContactRichOCP &&) = delete;
  ContactRichOCP &operator=(ContactRichOCP &&) = delete;

  [[nodiscard]] OCPBuildError Build(const PhaseContext &ctx,
                                    const RobotModelHandler &model,
                                    const OCPLimits &limits) noexcept override;

  [[nodiscard]] OCPBuildError
  UpdateReferences(const PhaseContext &ctx) noexcept override;

  [[nodiscard]] bool Built() const noexcept override {
    return problem_ != nullptr;
  }

  [[nodiscard]] aligator::TrajOptProblemTpl<double> &problem() override {
    return *problem_;
  }

  [[nodiscard]] int horizon_length() const noexcept override {
    return horizon_length_;
  }

  [[nodiscard]] std::string_view ocp_type() const noexcept override {
    return std::string_view{"contact_rich"};
  }

  /// @brief Install a non-owning grasp-quality provider.
  ///
  /// Default is null, which matches the Phase 4 baseline behaviour: no
  /// grasp-quality residual is added and the OCP behaves as
  /// "LightContact + contact-force cost + friction cone". The effect of a
  /// non-null provider takes hold on the **next** `Build()` call; this
  /// setter does not invalidate an already-built problem or trigger a
  /// rebuild on its own.
  ///
  /// Ownership: the caller retains the provider and MUST guarantee it
  /// outlives every subsequent `Build`/`UpdateReferences` call on this
  /// handler. Passing `nullptr` detaches the provider. Never called on
  /// the 500 Hz RT solve path — only during off-RT reconfigure.
  void
  SetGraspQualityProvider(GraspQualityResidualProvider *provider) noexcept {
    grasp_quality_provider_ = provider;
  }

private:
  // Problem tree (ownership).
  std::unique_ptr<aligator::TrajOptProblemTpl<double>> problem_{};

  // Per-stage cached handles; size == horizon_length_ after Build.
  std::vector<RichStageHandles> stage_handles_{};
  RichStageHandles terminal_handles_{};

  // Cached topology + limits — used by UpdateReferences to detect changes
  // that would require a full Build.
  int horizon_length_{0};
  double dt_{0.0};
  int nq_{0};
  int nv_{0};
  int nu_{0};
  std::vector<std::vector<int>> stage_active_contacts_{};

  // Limits + contact-force weight snapshot taken at the last successful
  // Build. UpdateReferences treats a crossing of `w_contact_force` across
  // 0 (either direction) as a topology change; same for friction_mu and
  // n_friction_facets changes.
  OCPLimits limits_cached_{};
  double w_contact_force_cached_{0.0};

  // Dynamics bookkeeping (kept so Build state can be inspected; not reused
  // across Build calls — Build always rebuilds from scratch).
  Eigen::MatrixXd actuation_matrix_{};

  // Extension seam: non-owning pointer. Null by default = Phase 4 behaviour
  // (no grasp-quality cost). Provider lifetime is the caller's concern; see
  // SetGraspQualityProvider doc-comment.
  GraspQualityResidualProvider *grasp_quality_provider_{nullptr};
};

} // namespace rtc::mpc
