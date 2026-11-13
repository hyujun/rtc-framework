#ifndef RTC_MPC_OCP_KINODYNAMIC_OCP_HPP_
#define RTC_MPC_OCP_KINODYNAMIC_OCP_HPP_

/// @file kinodynamic_ocp.hpp
/// @brief The whole-body kinodynamic OCP (`ocp_type: kinodynamic`): state
///        `x = [q; v]` on a floating base, input `u = [W_0 … W_{nc−1}; q̈_j]`,
///        one contact mode per stage, all switched in place every tick.
///
/// ## Problem
///
/// `horizon_nodes` stages at spacing `dt`; stage k is
///
/// - dynamics: `KinodynamicDynamics::MakeStageDynamics(mode_k, dt, integrator)`;
/// - cost: `dt · ( ½‖x ⊖ x_ref,k‖²_Q + ½‖u − u_ref,k‖²_R + ½‖r_k(x, u)‖²_W )`,
///   with `r_k` the contact tracking residual (`ContactTrackingResidual`, `W`
///   = `contact_tracking_weights` on every contact block) — the running cost
///   integrated over the node interval, as the reference formulation does;
///
///   plus the soft terms, also `× dt`: the contact wrench cone barrier
///   (`ContactConeBarrierCost` — friction and centre of pressure of the
///   stance contacts, the reference formulation's relaxed log barrier), the
///   joint position barrier (`JointLimitBarrierCost`) and, when
///   `constraints.collision.enabled`, the sphere-pair barrier
///   (`CollisionBarrierCost`);
/// - one constraint, `ContactConstraintResidual` as an equality: every
///   contact's rows of both phases, active by the stage's mode — the row
///   count is the same for every mode. Friction and centre of pressure are
///   not hard rows: as hard rows they stall a walking window (the solver
///   drives the rear foot's normal force of a double-support node to zero,
///   where the CoP rows hold trivially and the friction row stays violated by
///   `√ε`), as the barrier they do not;
///
/// and the terminal cost is `½‖x ⊖ x_ref,H‖²` weighted by
/// `terminal_scaling · terminal_weights` (not multiplied by `dt`; no soft
/// limit and no constraint on the terminal node).
///
/// ## Per tick
///
/// `UpdateReferences` reads `PhaseContext::horizon` (a `HorizonReference`,
/// one `StageReference` per node) and writes, per stage: the ODE's contact
/// mode (`SetMode` on the stage's own copy), `x_ref = [q; v]`, `u_ref =
/// [wrench; 0]`, the tracking residual's targets, scales and plane normal
/// and the contact equality's height targets and plane normal
/// (`SetFromStage`); the terminal node's `x_ref`. The constraints and the
/// collision barrier read the mode from the same ODE. A mode change is not a
/// topology change: the row and column counts of every stage are the same
/// for every mode, so nothing is rebuilt and nothing allocates.
///
/// - null `horizon` → `kMissingHorizonReference`;
/// - a horizon of another shape — `n_stages ≠ horizon_nodes + 1`, or another
///   `n_contacts` · `nq` · `nv` · `n_contact_vars` · `dt` — or a cost config
///   whose `horizon_length` · `dt` differ from this OCP's → `kInvalidPhaseContext`;
/// - a value a setter would refuse (non-finite, a non-unit base quaternion,
///   a mode naming a missing contact, a swing contact's `impact_proximity`
///   outside `[0, 1]`, a non-unit terrain normal) → `kInvalidHorizonReference`.
///
/// Every check runs before the first write: a refused horizon leaves every
/// stage as it was.
///
/// `Build` with a horizon applies it the same way (and fails as above); a
/// `Build` without one leaves every stage in full stance, `x_ref` the model's
/// neutral state, `u_ref = 0` and every contact scale 0. A `Build` refused by
/// any check — those above, a cost config of another horizon, a model of
/// another size, a control box (`OCPLimits::u_min` / `u_max`: this
/// formulation has no input box — hard joint and torque limits belong to the
/// whole-body QP — so a box is refused rather than dropped,
/// `kLimitsDimMismatch`) — changes nothing and allocates nothing, so a built
/// problem survives the handler core's rebuild attempt on `kInvalidPhaseContext`
/// (which no rebuild can satisfy: this OCP's config fixes the horizon).
///
/// ## Cold seed
///
/// `cold_seed()` is `kReferenceControl`: the handler core starts a cold
/// solve from each node's `u_ref` (`ReferenceControl`) — `u = 0` puts every
/// stance contact outside its friction cone (margin `−√ε`) and all four CoP
/// margins at zero, deep in the barrier's quadratic branch.
///
/// ## Threads and real time
///
/// `Build` is configure time (allocates). `UpdateReferences`,
/// `ReferenceControl` and `ConstraintResiduals` run on the MPC thread
/// between solves: noexcept, no throw; the first two allocate nothing, the
/// last nothing of its own (the ODE `forward` it evaluates allocates inside
/// the library).

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/ocp/barrier_costs.hpp"
#include "rtc_mpc/ocp/contact_constraints.hpp"
#include "rtc_mpc/ocp/contact_tracking_residual.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#pragma GCC diagnostic pop

#include <array>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace rtc::mpc {

/// @brief The constraint residuals of one trajectory, stage by stage, as the
///        solver sees them (`KinodynamicOCP::ConstraintResiduals`).
struct ConstraintResidualReport {
  /// Largest `|c|` over the rows of the stance contacts (gain-weighted, the
  /// solver's own units).
  double stance_equality_max{0.0};
  /// Largest `|c|` over the swing rows.
  double swing_equality_max{0.0};
  /// Largest `max(0, c)` over the wrench cone rows of the stance contacts
  /// (the barrier penalises, it does not forbid, an iterate outside).
  double inequality_violation_max{0.0};
  /// Per contact, over the nodes it is in stance: the smallest friction
  /// margin `μ(f_z + f_grip) − √(f_x² + f_y² + ε)` [N], and the smallest
  /// distance [m] of the centre of pressure inside its rectangle (negative
  /// outside, `−inf` at a node with no positive normal force; 6-D contacts).
  /// `+inf` for a contact never in stance.
  std::array<double, kMaxContacts> min_friction_margin{};
  std::array<double, kMaxContacts> min_cop_margin{};
};

class KinodynamicOCP final : public OCPHandlerBase {
 public:
  static constexpr std::string_view kType{"kinodynamic"};

  /// Keys of the terms of every stage's `CostStack` (collision only when enabled).
  static constexpr std::string_view kStateCostKey{"state"};
  static constexpr std::string_view kInputCostKey{"input"};
  static constexpr std::string_view kContactTrackingCostKey{"contact_tracking"};
  static constexpr std::string_view kContactConeCostKey{"contact_cone"};
  static constexpr std::string_view kJointLimitCostKey{"joint_limits"};
  static constexpr std::string_view kCollisionCostKey{"collision"};

  /// Index of the one constraint of every stage.
  static constexpr std::size_t kEqualityConstraint = 0;

  using QuadStateCost = aligator::QuadraticStateCostTpl<double>;
  using QuadControlCost = aligator::QuadraticControlCostTpl<double>;

  /// @param cfg  loaded for the model `Build` will be given
  explicit KinodynamicOCP(KinodynamicOCPConfig cfg);

  [[nodiscard]] OCPBuildError Build(const PhaseContext& ctx, const RobotModelHandler& model,
                                    const OCPLimits& limits) noexcept override;

  [[nodiscard]] OCPBuildError UpdateReferences(const PhaseContext& ctx) noexcept override;

  [[nodiscard]] bool Built() const noexcept override { return problem_.has_value(); }

  [[nodiscard]] aligator::TrajOptProblemTpl<double>& problem() override { return *problem_; }

  [[nodiscard]] int horizon_length() const noexcept override { return cfg_.horizon_nodes; }

  [[nodiscard]] std::string_view ocp_type() const noexcept override { return kType; }

  [[nodiscard]] ControlModel control_model() const noexcept override {
    return ControlModel::kKinodynamicWrench;
  }

  [[nodiscard]] ColdSeed cold_seed() const noexcept override { return ColdSeed::kReferenceControl; }

  /// @brief Stage @p k's `u_ref = [wrench; 0]`.
  [[nodiscard]] bool ReferenceControl(int k, Eigen::Ref<Eigen::VectorXd> u) const noexcept override;

  /// @brief Evaluate every stage's constraints at `(xs[k], us[k])` with the
  ///        stage's current mode and targets, into @p out.
  /// @return false, writing nothing, before a `Build`, for `xs` / `us` of
  ///         another length or entry size, for a non-finite entry or
  ///         residual, or if an evaluation threw.
  [[nodiscard]] bool ConstraintResiduals(const std::vector<Eigen::VectorXd>& xs,
                                         const std::vector<Eigen::VectorXd>& us,
                                         ConstraintResidualReport& out) noexcept;

  [[nodiscard]] const KinodynamicOCPConfig& config() const noexcept { return cfg_; }

 private:
  /// What `UpdateReferences` writes in stage k, resolved once after the
  /// problem is built (the problem owns the objects).
  struct StageHandles {
    KinodynamicDynamics::Ode* ode{nullptr};
    QuadStateCost* state{nullptr};
    QuadControlCost* input{nullptr};
    ContactTrackingResidual* tracking{nullptr};
    ContactConstraintResidual* equality{nullptr};
    ContactConeBarrierCost* cone{nullptr};
    CollisionBarrierCost* collision{nullptr};  ///< null when collision is disabled
  };

  /// The checks of `UpdateReferences` against @p model, before any write.
  [[nodiscard]] OCPBuildError CheckContext(const PhaseContext& ctx,
                                           const RobotModelHandler& model) const noexcept;
  void WriteHorizon(const HorizonReference& h) noexcept;
  void WriteStateTarget(const StageReference& s, QuadStateCost& cost) noexcept;

  KinodynamicOCPConfig cfg_;
  const RobotModelHandler* model_{nullptr};  // non-owning; set by Build
  KinodynamicDynamics dyn_{};
  std::optional<aligator::TrajOptProblemTpl<double>> problem_{};
  std::vector<StageHandles> stages_{};
  QuadStateCost* terminal_{nullptr};
  int nq_{0};
  int nv_{0};
  int nu_{0};
  int n_contact_vars_{0};
  Eigen::VectorXd x_buf_{};  ///< nq + nv — one node's x_ref
  Eigen::VectorXd u_buf_{};  ///< nu — one stage's u_ref (joint accelerations stay 0)
  /// `ConstraintResiduals` evaluation buffers (one of each, reused per stage).
  std::shared_ptr<aligator::StageFunctionDataTpl<double>> eq_data_{};
  std::shared_ptr<aligator::StageFunctionDataTpl<double>> ineq_data_{};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_KINODYNAMIC_OCP_HPP_
