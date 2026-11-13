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
/// and the terminal cost is `½‖x ⊖ x_ref,H‖²` weighted by
/// `terminal_scaling · terminal_weights` (not multiplied by `dt`). No
/// constraint rows — those are a later addition, at a fixed row count.
///
/// ## Per tick
///
/// `UpdateReferences` reads `PhaseContext::horizon` (a `HorizonReference`,
/// one `StageReference` per node) and writes, per stage: the ODE's contact
/// mode (`SetMode` on the stage's own copy), `x_ref = [q; v]`, `u_ref =
/// [wrench; 0]`, and the residual's targets, scales and plane normal
/// (`SetFromStage`); the terminal node's `x_ref`. A mode change is not a
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
/// another size, a control box (`OCPLimits::u_min` / `u_max`: there are no
/// constraint rows yet, so a box is refused rather than dropped,
/// `kLimitsDimMismatch`) — changes nothing and allocates nothing, so a built
/// problem survives the handler core's rebuild attempt on `kInvalidPhaseContext`
/// (which no rebuild can satisfy: this OCP's config fixes the horizon).
///
/// ## Threads and real time
///
/// `Build` is configure time (allocates). `UpdateReferences` runs on the MPC
/// thread between solves: noexcept, no allocation, no throw.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
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

#include <optional>
#include <string_view>
#include <vector>

namespace rtc::mpc {

class KinodynamicOCP final : public OCPHandlerBase {
 public:
  static constexpr std::string_view kType{"kinodynamic"};

  /// Keys of the three terms of every stage's `CostStack`.
  static constexpr std::string_view kStateCostKey{"state"};
  static constexpr std::string_view kInputCostKey{"input"};
  static constexpr std::string_view kContactTrackingCostKey{"contact_tracking"};

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

  [[nodiscard]] const KinodynamicOCPConfig& config() const noexcept { return cfg_; }

 private:
  /// What `UpdateReferences` writes in stage k, resolved once after the
  /// problem is built (the problem owns the objects).
  struct StageHandles {
    KinodynamicDynamics::Ode* ode{nullptr};
    QuadStateCost* state{nullptr};
    QuadControlCost* input{nullptr};
    ContactTrackingResidual* tracking{nullptr};
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
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_KINODYNAMIC_OCP_HPP_
