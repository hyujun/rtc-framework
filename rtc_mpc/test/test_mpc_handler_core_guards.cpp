/// @file test_mpc_handler_core_guards.cpp
/// @brief The handler core's seeding guards: the first solve after a build
///        starts from the cold seed, and a clean warm start stays valid.
///
/// The OCP here is a stub whose controls enter neither the dynamics nor the
/// cost (identity dynamics with an exact zero control Jacobian, constant
/// cost). From a guess that already satisfies the dynamics, ProxDDP meets its
/// tolerance on iteration 0 without a line search, so the solution it returns
/// is the guess the core handed it.

#include "handler/internal/mpc_handler_core.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
#include "test_utils/panda_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/core/vector-space.hpp>
#include <aligator/modelling/costs/constant-cost.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

namespace {

using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::SolverStatus;
using rtc::mpc::test_utils::kPandaTwoFingerYaml;
using rtc::mpc::test_utils::kPandaUrdf;
using rtc::mpc::test_utils::NeutralStateSnapshot;

constexpr int kHorizon = 8;
constexpr double kNodeDt = 0.01;
constexpr std::string_view kStubType{"inert_stub"};

/// x_{k+1} = x_k, with an exact zero control Jacobian: no control value can
/// reach the next state, not even as 0 · inf.
struct HoldStateDynamics : aligator::ExplicitDynamicsModelTpl<double> {
  using Base = aligator::ExplicitDynamicsModelTpl<double>;

  HoldStateDynamics(int nx, int nu) : Base(aligator::VectorSpaceTpl<double>(nx), nu) {}

  void forward(const ConstVectorRef& x, const ConstVectorRef& /*u*/, Data& data) const override {
    data.xnext_ = x;
  }

  void dForward(const ConstVectorRef& /*x*/, const ConstVectorRef& /*u*/,
                Data& data) const override {
    data.Jx().setIdentity();
    data.Ju().setZero();
  }
};

/// OCP whose controls are inert: identity dynamics and a constant cost.
class InertOCP final : public rtc::mpc::OCPHandlerBase {
 public:
  rtc::mpc::OCPBuildError Build(const rtc::mpc::PhaseContext& /*ctx*/,
                                const rtc::mpc::RobotModelHandler& model,
                                const rtc::mpc::OCPLimits& /*limits*/) noexcept override {
    try {
      const int nx = model.nq() + model.nv();
      const int nu = model.nu();
      const aligator::VectorSpaceTpl<double> space(nx);
      const aligator::ConstantCostTpl<double> cost(space, nu, 0.0);
      std::vector<xyz::polymorphic<aligator::StageModelTpl<double>>> stages;
      for (int k = 0; k < kHorizon; ++k) {
        stages.emplace_back(aligator::StageModelTpl<double>(cost, HoldStateDynamics(nx, nu)));
      }
      problem_.emplace(Eigen::VectorXd::Zero(nx), stages, cost);
    } catch (...) {
      return rtc::mpc::OCPBuildError::kAligatorInstantiationFailure;
    }
    return rtc::mpc::OCPBuildError::kNoError;
  }

  rtc::mpc::OCPBuildError UpdateReferences(
      const rtc::mpc::PhaseContext& /*ctx*/) noexcept override {
    return rtc::mpc::OCPBuildError::kNoError;
  }

  bool Built() const noexcept override { return problem_.has_value(); }

  aligator::TrajOptProblemTpl<double>& problem() override { return *problem_; }

  int horizon_length() const noexcept override { return kHorizon; }

  std::string_view ocp_type() const noexcept override { return kStubType; }

 private:
  std::optional<aligator::TrajOptProblemTpl<double>> problem_;
};

[[nodiscard]] bool SolutionFinite(const MPCSolution& s) {
  for (int k = 0; k <= s.horizon_length; ++k) {
    for (int i = 0; i < s.nq; ++i) {
      if (!std::isfinite(s.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)])) {
        return false;
      }
    }
    for (int i = 0; i < s.nv; ++i) {
      if (!std::isfinite(s.v_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)])) {
        return false;
      }
    }
  }
  for (int k = 0; k < s.horizon_length; ++k) {
    for (int i = 0; i < s.nu; ++i) {
      if (!std::isfinite(s.u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)])) {
        return false;
      }
    }
  }
  return true;
}

class MPCHandlerCoreGuardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);
    ASSERT_EQ(handler_.Init(model_, YAML::Load(kPandaTwoFingerYaml)),
              rtc::mpc::RobotModelInitError::kNoError);

    ctx_.ocp_type = std::string{kStubType};
    ctx_.cost_config.dt = kNodeDt;

    solver_cfg_.prim_tol = 1e-6;
    solver_cfg_.dual_tol = 1e-6;
    solver_cfg_.mu_init = 1e-2;
    solver_cfg_.max_iters = 5;
    solver_cfg_.max_al_iters = 2;

    ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_),
              rtc::mpc::MPCInitError::kNoError);
    state_ = NeutralStateSnapshot(model_, handler_);
  }

  /// One clean tick: the stub converges on iteration 0 from the cold seed.
  void SolveClean(MPCSolution& out) {
    ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
    ASSERT_TRUE(out.IsValid());
    ASSERT_TRUE(SolutionFinite(out));
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseContext ctx_;
  rtc::mpc::OCPLimits limits_;
  rtc::mpc::MPCSolverConfig solver_cfg_;
  InertOCP ocp_;
  rtc::mpc::internal::MPCHandlerCore core_;
  MPCStateSnapshot state_{};
};

TEST_F(MPCHandlerCoreGuardTest, FirstSolveStartsFromTheColdSeed) {
  // Away from the problem's placeholder initial state, so a guess that is
  // not the cold seed cannot pass for it.
  for (int i = 0; i < state_.nq; ++i) {
    state_.q[static_cast<std::size_t>(i)] = 0.1 * (i + 1);
  }
  MPCSolution out{};
  SolveClean(out);
  // The stub is solved by its guess, so the result is the seed itself: the
  // measured state at every node and zero controls (a non-gravity OCP type).
  EXPECT_EQ(out.iterations, 0);
  for (int k = 0; k <= out.horizon_length; ++k) {
    for (int i = 0; i < out.nq; ++i) {
      EXPECT_EQ(out.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)],
                state_.q[static_cast<std::size_t>(i)])
          << "node " << k << " q[" << i << "]";
    }
  }
  for (int k = 0; k < out.horizon_length; ++k) {
    for (int i = 0; i < out.nu; ++i) {
      EXPECT_EQ(out.u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)], 0.0);
    }
  }
}

TEST_F(MPCHandlerCoreGuardTest, CleanWarmStartStaysValid) {
  MPCSolution first{};
  SolveClean(first);
  core_.SeedWarmStart(first, ocp_);
  MPCSolution second{};
  SolveClean(second);
}

}  // namespace
