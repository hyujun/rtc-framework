#ifndef RTC_MPC_OCP_COST_FACTORY_HPP_
#define RTC_MPC_OCP_COST_FACTORY_HPP_

/// @file cost_factory.hpp
/// @brief Builds per-stage Aligator `CostStack` from `PhaseCostConfig`.
///
/// Scope is the three **robot-agnostic**, **OCP-backbone-independent**
/// residuals: frame placement, state regularisation, control regularisation.
/// Contact-force and centroidal-momentum residuals live inside the concrete
/// OCP handler (e.g. `LightContactOCP`) because they depend on dynamics-
/// specific data (rigid-contact constraint models, actuation matrix, prox
/// settings) that the handler already owns.
///
/// Ownership contract (Phase 3 spike — `git log --grep='rtc_mpc Phase 3'`):
/// Aligator stores stage components as `xyz::polymorphic<T>` (value-type,
/// copy-on-construct). The `CostStackTpl` returned here is a fresh object
/// that the caller copies into a `StageModelTpl`. External pointers to the
/// residuals constructed inside this function would **not** reach the stored
/// copies. Callers that need alloc-free reference updates must retrieve
/// handles **after** stage assembly via:
/// ```
/// stage.getCost<CostStackTpl<double>>()
///     ->getComponent<QuadraticResidualCostTpl<double>>(kKey<…>)
///     ->getResidual<Derived>();
/// ```
/// `StageCost::keys` tells the caller which lookups will succeed for a given
/// stage (weight <= 0 entries are omitted, no lookup is valid).

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_cost_config.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <string_view>

namespace rtc::mpc {

/// @brief Canonical string keys for component lookup on the stored
///        `CostStackTpl`. Stable across phases; must match the strings used
///        inside `cost_factory::Build*Cost`.
inline constexpr std::string_view kCostKeyFramePlacement = "frame_placement";
inline constexpr std::string_view kCostKeyStateReg = "state_reg";
inline constexpr std::string_view kCostKeyControlReg = "control_reg";

/// @brief Flags telling the caller which components the returned stack
///        actually contains. Components are omitted when their scalar
///        weight is <= 0 (avoids zero-contribution terms in the Hessian).
struct StageComponentKeys {
  bool has_frame_placement{false};
  bool has_state_reg{false};
  bool has_control_reg{false};
};

/// @brief Return bundle: the freshly built cost stack + which components
///        are present. The stack is a value object intended to be passed by
///        move or copy into a `StageModelTpl` ctor.
struct StageCost {
  aligator::CostStackTpl<double> stack;
  StageComponentKeys keys{};
};

/// @brief Failure modes for the cost factory. Distinct from
///        `OCPBuildError` so the factory can be reused by
///        non-OCPHandler callers (e.g. future `FullDynamicsOCP`, tests).
enum class CostFactoryError {
  kNoError = 0,
  kModelNotInitialised,
  kInvalidCostConfig,            ///< horizon_length <= 0 or dt <= 0
  kPostureRefDimMismatch,        ///< q_posture_ref.size() != nq
  kAligatorInstantiationFailure, ///< an Aligator ctor threw
};

namespace cost_factory {

/// @brief Running-stage cost: frame placement + state reg + control reg.
///
/// Weight-gated — a residual is only added when its scalar weight is > 0.
/// The returned `StageCost::keys` reflects which components were added.
///
/// @param cfg        Per-phase cost weights + references (Phase 2 POD)
/// @param model      Initialised `RobotModelHandler`; provides `nq`, `nv`,
///                   `nu`, EE frame id, and the Pinocchio model used for
///                   residual construction.
/// @param ee_target  SE3 target for the frame-placement residual.
/// @param[out] out_error  Non-null: receives `CostFactoryError::kNoError`
///                         on success, otherwise the first failure cause.
/// @return A `StageCost` bundle. On error, the returned stack is empty and
///         all keys are false.
[[nodiscard]] StageCost BuildRunningCost(const PhaseCostConfig &cfg,
                                         const RobotModelHandler &model,
                                         const pinocchio::SE3 &ee_target,
                                         CostFactoryError *out_error) noexcept;

/// @brief Terminal-stage cost: frame placement + state reg only. Control
///        reg and any `u`-dependent residuals are absent (the terminal
///        cost takes `x` only).
[[nodiscard]] StageCost BuildTerminalCost(const PhaseCostConfig &cfg,
                                          const RobotModelHandler &model,
                                          const pinocchio::SE3 &ee_target,
                                          CostFactoryError *out_error) noexcept;

} // namespace cost_factory

} // namespace rtc::mpc

#endif // RTC_MPC_OCP_COST_FACTORY_HPP_
