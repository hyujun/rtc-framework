#include "rtc_mpc/ocp/cost_factory.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/vector-space.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/modelling/state-error.hpp>
#pragma GCC diagnostic pop

#include <exception>
#include <string>
#include <utility>

namespace rtc::mpc {
namespace cost_factory {

namespace {

using CostStack = aligator::CostStackTpl<double>;
using QuadCost = aligator::QuadraticResidualCostTpl<double>;
using FrameRes = aligator::FramePlacementResidualTpl<double>;
using StateRes = aligator::StateErrorResidualTpl<double>;
using ControlRes = aligator::ControlErrorResidualTpl<double>;
using PhaseSpace = aligator::MultibodyPhaseSpace<double>;

// Empty stack + all-false keys. We cannot construct a `PhaseSpace` from an
// uninitialised model (would deref a null `pinocchio::Model*`), so we fall
// back to a 1-dim VectorSpace. Callers use this only for error returns.
StageCost MakeEmpty(const RobotModelHandler &model) noexcept {
  // -1 == Eigen::Dynamic; spelled literally because
  // `pinocchio::Dynamic` shadows the Eigen constant in some TUs.
  using FallbackSpace = aligator::VectorSpaceTpl<double, -1>;
  const int nu = model.Initialised() ? model.nu() : 1;
  if (model.Initialised()) {
    PhaseSpace space(model.model());
    return StageCost{CostStack(space, nu), StageComponentKeys{}};
  }
  FallbackSpace fallback(1);
  return StageCost{CostStack(fallback, nu), StageComponentKeys{}};
}

[[nodiscard]] bool ValidateCommon(const PhaseCostConfig &cfg,
                                  const RobotModelHandler &model,
                                  CostFactoryError &err) noexcept {
  if (!model.Initialised()) {
    err = CostFactoryError::kModelNotInitialised;
    return false;
  }
  if (cfg.horizon_length <= 0 || cfg.dt <= 0.0) {
    err = CostFactoryError::kInvalidCostConfig;
    return false;
  }
  if (cfg.q_posture_ref.size() != model.nq()) {
    err = CostFactoryError::kPostureRefDimMismatch;
    return false;
  }
  return true;
}

// Add frame-placement cost if w_frame_placement > 0. Caller owns `space`
// and `stack`; `keys` is mutated on success. Aligator ctors may throw →
// caller must be within a try/catch.
void AddFramePlacement(CostStack &stack, StageComponentKeys &keys,
                       const PhaseSpace &space, const PhaseCostConfig &cfg,
                       const RobotModelHandler &model,
                       const pinocchio::SE3 &ee_target) {
  if (cfg.w_frame_placement <= 0.0) {
    return;
  }
  const int ndx = space.ndx();
  const int nu = model.nu();
  FrameRes residual(
      ndx, nu, model.model(), ee_target,
      static_cast<pinocchio::FrameIndex>(model.end_effector_frame_id()));
  Eigen::MatrixXd W =
      cfg.w_frame_placement * cfg.W_placement.asDiagonal().toDenseMatrix();
  QuadCost qcost(space, residual, W);
  stack.addCost(std::string(kCostKeyFramePlacement), qcost, 1.0);
  keys.has_frame_placement = true;
}

// State error residual: target = [q_posture_ref; 0]. Weight is scalar I.
void AddStateReg(CostStack &stack, StageComponentKeys &keys,
                 const PhaseSpace &space, const PhaseCostConfig &cfg,
                 const RobotModelHandler &model) {
  if (cfg.w_state_reg <= 0.0) {
    return;
  }
  const int nq = model.nq();
  const int nv = model.nv();
  const int ndx = space.ndx();
  Eigen::VectorXd x_target(nq + nv);
  x_target.head(nq) = cfg.q_posture_ref;
  x_target.tail(nv).setZero();
  StateRes residual(space, model.nu(), x_target);
  Eigen::MatrixXd W = Eigen::MatrixXd::Identity(ndx, ndx) * cfg.w_state_reg;
  QuadCost qcost(space, residual, W);
  stack.addCost(std::string(kCostKeyStateReg), qcost, 1.0);
  keys.has_state_reg = true;
}

// Control error residual: target = 0 (u_ref). Weight scalar I.
void AddControlReg(CostStack &stack, StageComponentKeys &keys,
                   const PhaseSpace &space, const PhaseCostConfig &cfg,
                   const RobotModelHandler &model) {
  if (cfg.w_control_reg <= 0.0) {
    return;
  }
  const int nu = model.nu();
  const int ndx = space.ndx();
  Eigen::VectorXd u_ref = Eigen::VectorXd::Zero(nu);
  ControlRes residual(ndx, u_ref);
  Eigen::MatrixXd W = Eigen::MatrixXd::Identity(nu, nu) * cfg.w_control_reg;
  QuadCost qcost(space, residual, W);
  stack.addCost(std::string(kCostKeyControlReg), qcost, 1.0);
  keys.has_control_reg = true;
}

} // namespace

StageCost BuildRunningCost(const PhaseCostConfig &cfg,
                           const RobotModelHandler &model,
                           const pinocchio::SE3 &ee_target,
                           CostFactoryError *out_error) noexcept {
  CostFactoryError err = CostFactoryError::kNoError;
  if (!ValidateCommon(cfg, model, err)) {
    if (out_error != nullptr) {
      *out_error = err;
    }
    return MakeEmpty(model);
  }

  try {
    PhaseSpace space(model.model());
    CostStack stack(space, model.nu());
    StageComponentKeys keys{};

    AddFramePlacement(stack, keys, space, cfg, model, ee_target);
    AddStateReg(stack, keys, space, cfg, model);
    AddControlReg(stack, keys, space, cfg, model);

    if (out_error != nullptr) {
      *out_error = CostFactoryError::kNoError;
    }
    return StageCost{std::move(stack), keys};
  } catch (const std::exception &) {
    if (out_error != nullptr) {
      *out_error = CostFactoryError::kAligatorInstantiationFailure;
    }
    return MakeEmpty(model);
  } catch (...) {
    if (out_error != nullptr) {
      *out_error = CostFactoryError::kAligatorInstantiationFailure;
    }
    return MakeEmpty(model);
  }
}

StageCost BuildTerminalCost(const PhaseCostConfig &cfg,
                            const RobotModelHandler &model,
                            const pinocchio::SE3 &ee_target,
                            CostFactoryError *out_error) noexcept {
  CostFactoryError err = CostFactoryError::kNoError;
  if (!ValidateCommon(cfg, model, err)) {
    if (out_error != nullptr) {
      *out_error = err;
    }
    return MakeEmpty(model);
  }

  try {
    PhaseSpace space(model.model());
    CostStack stack(space, model.nu());
    StageComponentKeys keys{};

    AddFramePlacement(stack, keys, space, cfg, model, ee_target);
    AddStateReg(stack, keys, space, cfg, model);
    // No control reg on terminal (x-only cost).

    if (out_error != nullptr) {
      *out_error = CostFactoryError::kNoError;
    }
    return StageCost{std::move(stack), keys};
  } catch (const std::exception &) {
    if (out_error != nullptr) {
      *out_error = CostFactoryError::kAligatorInstantiationFailure;
    }
    return MakeEmpty(model);
  } catch (...) {
    if (out_error != nullptr) {
      *out_error = CostFactoryError::kAligatorInstantiationFailure;
    }
    return MakeEmpty(model);
  }
}

} // namespace cost_factory
} // namespace rtc::mpc
