#ifndef RTC_MPC_OCP_INTERNAL_KINODYNAMIC_STAGE_MODEL_HPP_
#define RTC_MPC_OCP_INTERNAL_KINODYNAMIC_STAGE_MODEL_HPP_

/// @file kinodynamic_stage_model.hpp
/// @brief Internal: the running stage of the kinodynamic OCP when its
///        contact residuals share one evaluation of the stage's kinodynamic
///        ODE and of the contact-frame kinematics
///        (`KinodynamicOCP::StageKinematics::kShared`).
///
/// Not installed. Not part of the public `rtc_mpc` API.
///
/// ## What is evaluated where
///
/// A kinodynamic stage evaluates its ODE in the integrator (`dynamics_`) and,
/// left to themselves, once more in each of the contact equality
/// (`ContactConstraintResidual`) and the contact tracking residual
/// (`ContactTrackingResidual`, inside the `QuadraticResidualCost` of the
/// stage's `CostStack`), each followed by the frame kinematics of
/// `(q, v, v̇)`. Both residuals are bound to the integrator's own ODE, and
/// the integrator's continuous data holds that ODE at the stage's `(x, u)`
/// after its step — for the semi-implicit Euler and the RK2 integrator
/// alike (RK2's second evaluation, at the midpoint, goes to a second data
/// object). So this stage evaluates, per `(x, u)`:
///
/// - `evaluate`: the dynamics step (`ode.forward` into the continuous data),
///   then `ContactKinematics::ForwardKinematics` once on that data's `v̇`,
///   then the constraints and the cost as Aligator's stage does;
/// - `computeFirstOrderDerivatives`: the dynamics derivatives
///   (`ode.dForward` into the same data), then
///   `ContactKinematics::KinematicsDerivatives` once, then the constraint
///   Jacobians and the cost gradients.
///
/// The residual data `createData` finds (the equality constraint at its
/// index, the tracking cost's residual data under its key) gets a
/// non-owning pointer to the one `ContactKinematics` of this stage's data
/// and reads it instead of evaluating its own. The operations and their
/// inputs are those of the per-residual path, so the results are the same
/// bits. A residual bound to another ODE than this stage's, or data of
/// another type, is left to evaluate on its own.
///
/// ## Threads and real time
///
/// The sharing stays inside one stage's data: stage-parallel evaluation runs
/// different stages (different data) concurrently, never one stage's
/// residuals. `createData` allocates (configure time); `evaluate` and
/// `computeFirstOrderDerivatives` allocate nothing of their own (the ODE's
/// `dForward` allocates inside the library, as on the per-residual path).

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/ocp/contact_kinematics.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-data.hpp>
#include <aligator/core/stage-model.hpp>
#pragma GCC diagnostic pop

#include <cstddef>
#include <memory>
#include <string>

namespace rtc::mpc::internal {

class KinodynamicStageModel;

/// @brief Aligator's stage data plus the one `ContactKinematics` the stage's
///        contact residuals read.
struct KinodynamicStageData final : aligator::StageDataTpl<double> {
  using Base = aligator::StageDataTpl<double>;

  /// @brief Builds every member's data, then points the residual data of
  ///        @p stage's contact equality and tracking cost at `kin`.
  ///        Allocates (configure time).
  /// @throws std::logic_error when @p stage's dynamics is not an explicit
  ///         integrator of the kinodynamic ODE.
  explicit KinodynamicStageData(const KinodynamicStageModel& stage);

  KinodynamicStageData(const KinodynamicStageData&) = delete;
  KinodynamicStageData& operator=(const KinodynamicStageData&) = delete;
  KinodynamicStageData(KinodynamicStageData&&) = delete;
  KinodynamicStageData& operator=(KinodynamicStageData&&) = delete;
  ~KinodynamicStageData() override = default;

  const KinodynamicDynamics::Ode* ode{nullptr};  ///< the stage's own ODE (non-owning)
  ContactKinematics kin;                         ///< on the integrator's continuous data
  bool shared{false};                            ///< some residual data reads `kin`
};

/// @brief A kinodynamic running stage whose contact residuals read one
///        shared evaluation of the ODE and the frame kinematics. Copyable
///        (a stage lives in `xyz::polymorphic<StageModelTpl>`).
class KinodynamicStageModel final : public aligator::StageModelTpl<double> {
 public:
  using Base = aligator::StageModelTpl<double>;

  /// @param equality      index of the `ContactConstraintResidual` among
  ///                      the stage's constraints (added after construction)
  /// @param tracking_key  key of the `QuadraticResidualCost` over the
  ///                      `ContactTrackingResidual` in the `CostStack` @p cost
  /// @throws std::invalid_argument when @p dynamics is not an explicit
  ///         integrator of the kinodynamic ODE; as Aligator's stage otherwise.
  KinodynamicStageModel(const PolyCost& cost, const PolyDynamics& dynamics, std::size_t equality,
                        std::string tracking_key);

  /// @brief Dynamics, the shared kinematics, constraints, cost.
  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override;

  /// @brief Dynamics derivatives, the shared kinematics derivatives,
  ///        constraint Jacobians, cost gradients. After the `evaluate` of the
  ///        same `(x, u)` on the same data.
  void computeFirstOrderDerivatives(const ConstVectorRef& x, const ConstVectorRef& u,
                                    Data& data) const override;

  /// @brief A `KinodynamicStageData`. Allocates; throws as its constructor
  ///        and the members' `createData`.
  [[nodiscard]] std::shared_ptr<Data> createData() const override;

  [[nodiscard]] std::size_t equality_index() const noexcept { return equality_; }

  [[nodiscard]] const std::string& tracking_key() const noexcept { return tracking_key_; }

 private:
  std::size_t equality_;
  std::string tracking_key_;
};

}  // namespace rtc::mpc::internal

#endif  // RTC_MPC_OCP_INTERNAL_KINODYNAMIC_STAGE_MODEL_HPP_
