/// @file test_mpc_handler_core_floating.cpp
/// @brief The handler core on a floating base, driven by a kinodynamic stub
///        OCP (`ControlModel::kKinodynamicWrench`) on the public G1: sizes by
///        `nu`, packs the wrench block as `lambda_traj` and a zero `a_ff_traj`,
///        carries a warm start of size `nu`, refuses a non-unit base
///        quaternion; and the shipped joint-torque OCPs still refuse a
///        floating model. Zero cold seed; convergence is not asserted.

#include "handler/internal/mpc_handler_core.hpp"
#include "rtc_mpc/handler/contact_light_mpc.hpp"
#include "rtc_mpc/handler/contact_rich_mpc.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_stub_ocp.hpp"
#include "test_utils/panda_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <cstddef>
#include <filesystem>

namespace {

using rtc::mpc::MPCInitError;
using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::test_utils::KinodynamicStubOCP;

constexpr int kHorizon = 6;
constexpr double kNodeDt = 0.035;
/// Only contact 1 in contact: contact 0's wrench slots are inert.
constexpr std::uint32_t kMode = 0b10;

class MPCHandlerCoreFloatingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(rtc::mpc::test_utils::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << rtc::mpc::test_utils::kG1Urdf;
    }
    model_ = rtc::mpc::test_utils::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(rtc::mpc::test_utils::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ctx_.ocp_type = std::string{KinodynamicStubOCP::kType};
    ctx_.cost_config.dt = kNodeDt;
    solver_cfg_.prim_tol = 1e-4;
    solver_cfg_.dual_tol = 1e-4;
    solver_cfg_.mu_init = 1e-2;
    solver_cfg_.max_iters = 5;
    solver_cfg_.max_al_iters = 2;

    const Eigen::VectorXd q = pinocchio::neutral(model_);
    state_.nq = model_.nq;
    state_.nv = model_.nv;
    for (int i = 0; i < model_.nq; ++i) {
      state_.q[static_cast<std::size_t>(i)] = q[i];
    }
    state_.q[2] = 0.75;  // pelvis height
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseContext ctx_;
  rtc::mpc::OCPLimits limits_;
  rtc::mpc::MPCSolverConfig solver_cfg_;
  KinodynamicStubOCP ocp_{kHorizon, kMode};
  rtc::mpc::internal::MPCHandlerCore core_;
  MPCStateSnapshot state_{};
};

TEST_F(MPCHandlerCoreFloatingTest, PacksTheKinodynamicControl) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  const rtc::mpc::WbLayout& layout = handler_.layout();
  EXPECT_EQ(core_.nu(), layout.nu);

  MPCSolution out{};
  for (auto& row : out.lambda_traj) {
    row.fill(7.0);
  }
  for (auto& row : out.a_ff_traj) {
    row.fill(7.0);
  }
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  ASSERT_TRUE(out.IsValid());
  EXPECT_EQ(out.nu, layout.nu);
  EXPECT_EQ(out.nv, model_.nv);
  EXPECT_EQ(out.nq, model_.nq);
  EXPECT_EQ(out.ndx, 2 * model_.nv);
  EXPECT_EQ(out.n_contact_vars, layout.n_wrench_u);
  for (int k = 0; k < out.horizon_length; ++k) {
    const auto ks = static_cast<std::size_t>(k);
    for (int i = 0; i < layout.n_wrench_u; ++i) {
      const auto is = static_cast<std::size_t>(i);
      EXPECT_EQ(out.lambda_traj[ks][is], out.u_traj[ks][is]) << "node " << k << " slot " << i;
    }
    for (int i = 0; i < out.nv; ++i) {
      EXPECT_EQ(out.a_ff_traj[ks][static_cast<std::size_t>(i)], 0.0) << "node " << k;
    }
  }
}

// A seeded control of size nu reaches the solver: contact 0's slots are
// inert here, so a value seeded there comes back unchanged.
TEST_F(MPCHandlerCoreFloatingTest, WarmStartOfSizeNuIsUsed) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  MPCSolution first{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, first), MPCSolveError::kNoError);
  MPCSolution seed = first;
  for (int k = 0; k < kHorizon; ++k) {
    seed.u_traj[static_cast<std::size_t>(k)][0] = 123.0;
  }
  core_.SeedWarmStart(seed, ocp_);
  MPCSolution out{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
  for (int k = 0; k < kHorizon; ++k) {
    EXPECT_EQ(out.u_traj[static_cast<std::size_t>(k)][0], 123.0) << "node " << k;
  }
}

TEST_F(MPCHandlerCoreFloatingTest, RefusesANonUnitBaseQuaternion) {
  ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_), MPCInitError::kNoError);
  MPCStateSnapshot bad = state_;
  for (int i = 3; i < 7; ++i) {
    bad.q[static_cast<std::size_t>(i)] *= 1.001;
  }
  MPCSolution out{};
  EXPECT_EQ(core_.Solve(ctx_, bad, ocp_, out), MPCSolveError::kStateDimMismatch);
  EXPECT_FALSE(out.IsValid());
  // Within the layout's tolerance it is accepted as is.
  MPCStateSnapshot close = state_;
  close.q[6] *= 1.0 + 1e-8;
  EXPECT_EQ(core_.Solve(ctx_, close, ocp_, out), MPCSolveError::kNoError);
}

TEST_F(MPCHandlerCoreFloatingTest, ControlModelMustMatchTheBase) {
  // The shipped joint-torque OCPs refuse a floating model.
  rtc::mpc::PhaseContext light = ctx_;
  light.ocp_type = "contact_light";
  rtc::mpc::ContactLightMPC light_mpc;
  EXPECT_EQ(light_mpc.Init(solver_cfg_, handler_, limits_, light),
            MPCInitError::kFloatingBaseUnsupported);
  rtc::mpc::PhaseContext rich = ctx_;
  rich.ocp_type = "contact_rich";
  rtc::mpc::ContactRichMPC rich_mpc;
  EXPECT_EQ(rich_mpc.Init(solver_cfg_, handler_, limits_, rich),
            MPCInitError::kFloatingBaseUnsupported);

  // A kinodynamic OCP refuses a fixed base.
  if (!std::filesystem::exists(rtc::mpc::test_utils::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  pinocchio::Model fixed;
  pinocchio::urdf::buildModel(rtc::mpc::test_utils::kPandaUrdf, fixed);
  rtc::mpc::RobotModelHandler fixed_handler;
  ASSERT_EQ(fixed_handler.Init(fixed, YAML::Load(rtc::mpc::test_utils::kPandaTwoFingerYaml)),
            rtc::mpc::RobotModelInitError::kNoError);
  rtc::mpc::internal::MPCHandlerCore core;
  KinodynamicStubOCP ocp(kHorizon, kMode);
  EXPECT_EQ(core.Init(solver_cfg_, fixed_handler, limits_, ctx_, ocp),
            MPCInitError::kFixedBaseUnsupported);
  EXPECT_FALSE(core.Initialised());
}

}  // namespace
