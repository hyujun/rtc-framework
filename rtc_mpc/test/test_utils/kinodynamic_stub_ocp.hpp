#pragma once

/// @file kinodynamic_stub_ocp.hpp
/// @brief A kinodynamic OCP stand-in for driving the handler core on a
///        floating base: every stage is `KinodynamicDynamics::MakeStageDynamics`
///        of one contact mode, the cost a state regulariser plus a control
///        regulariser that leaves the inactive contacts' wrench slots out (so
///        those slots are inert: neither dynamics nor cost reads them). Not a
///        controller — nothing about convergence is promised. Not installed —
///        test-local by design.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"
#include "rtc_mpc/phase/phase_context.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/quad-costs.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#pragma GCC diagnostic pop

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace rtc::mpc::test_utils {

class KinodynamicStubOCP final : public OCPHandlerBase {
 public:
  static constexpr std::string_view kType{"kinodynamic_stub"};

  /// @param horizon     stage count
  /// @param mode        the contact mode of every stage
  /// @param integrator  the stages' discretisation
  KinodynamicStubOCP(int horizon, std::uint32_t mode,
                     KinodynamicIntegrator integrator = KinodynamicIntegrator::kSemiImplicitEuler)
      : horizon_(horizon), mode_(mode), integrator_(integrator) {}

  OCPBuildError Build(const PhaseContext& ctx, const RobotModelHandler& model,
                      const OCPLimits& /*limits*/) noexcept override {
    problem_.reset();
    if (dyn_.Init(model) != KinodynamicError::kNoError) {
      return OCPBuildError::kModelNotInitialised;
    }
    try {
      const KinodynamicDynamics::Space& space = *dyn_.space();
      const int nu = dyn_.nu();
      const int ndx = space.ndx();
      Eigen::VectorXd w_u = Eigen::VectorXd::Constant(nu, 1e-4);
      for (int c = 0; c < dyn_.n_contacts(); ++c) {
        if (((mode_ >> c) & 1U) == 0U) {
          w_u.segment(c * dyn_.force_size(), dyn_.force_size()).setZero();
        }
      }
      aligator::CostStackTpl<double> cost(space, nu);
      cost.addCost("x_reg", aligator::QuadraticStateCostTpl<double>(
                                space, nu, space.neutral(), Eigen::MatrixXd::Identity(ndx, ndx)));
      cost.addCost("u_reg",
                   aligator::QuadraticControlCostTpl<double>(space, Eigen::VectorXd::Zero(nu),
                                                             Eigen::MatrixXd(w_u.asDiagonal())));
      std::vector<xyz::polymorphic<aligator::StageModelTpl<double>>> stages;
      stages.reserve(static_cast<std::size_t>(horizon_));
      for (int k = 0; k < horizon_; ++k) {
        std::optional<KinodynamicDynamics::StageDynamics> dyn;
        if (dyn_.MakeStageDynamics(mode_, ctx.cost_config.dt, integrator_, dyn) !=
            KinodynamicError::kNoError) {
          return OCPBuildError::kInvalidCostConfig;
        }
        stages.emplace_back(aligator::StageModelTpl<double>(cost, *dyn));
      }
      problem_.emplace(space.neutral(), stages, aligator::CostStackTpl<double>(space, nu));
    } catch (...) {
      problem_.reset();
      return OCPBuildError::kAligatorInstantiationFailure;
    }
    return OCPBuildError::kNoError;
  }

  OCPBuildError UpdateReferences(const PhaseContext& /*ctx*/) noexcept override {
    return OCPBuildError::kNoError;
  }

  bool Built() const noexcept override { return problem_.has_value(); }

  aligator::TrajOptProblemTpl<double>& problem() override { return *problem_; }

  int horizon_length() const noexcept override { return horizon_; }

  std::string_view ocp_type() const noexcept override { return kType; }

  ControlModel control_model() const noexcept override { return ControlModel::kKinodynamicWrench; }

 private:
  int horizon_;
  std::uint32_t mode_;
  KinodynamicIntegrator integrator_;
  KinodynamicDynamics dyn_;
  std::optional<aligator::TrajOptProblemTpl<double>> problem_;
};

}  // namespace rtc::mpc::test_utils
