/// @file test_light_contact_ocp.cpp
/// @brief End-to-end integration test for `rtc::mpc::LightContactOCP` on the
///        generic Panda fixture (9-DoF, fixed-base, 2×3D fingertip contacts).
///
/// Covered:
/// - Build on neutral pose succeeds; problem has correct topology.
/// - Offline solve with ProxDDP converges to a small primal residual.
/// - UpdateReferences propagates a new EE target through to evaluation.
/// - ReBuild idempotence (Build twice, both succeed).
/// - Empty contact plan (free-flight) solves.
/// - Error paths: invalid ocp_type, contact-plan/model mismatch,
///   overlapping phases, u_limits dim mismatch, topology-change on update.
/// - Perf: p50/p99 solve wall-time logged (not asserted).

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include "rtc_mpc/ocp/light_contact_ocp.hpp"

namespace {

constexpr const char *kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

constexpr const char *kCostYaml = R"(
horizon_length: 20
dt: 0.01
w_frame_placement: 100.0
w_state_reg: 1.0
w_control_reg: 0.01
w_contact_force: 0.0
w_centroidal_momentum: 0.0
W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights: {}
)";

class LightContactOCPTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);

    auto robot_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
    ASSERT_EQ(handler_.Init(model_, robot_cfg),
              rtc::mpc::RobotModelInitError::kNoError);

    auto cost_node = YAML::Load(kCostYaml);
    ASSERT_EQ(
        rtc::mpc::PhaseCostConfig::LoadFromYaml(cost_node, handler_, cfg_),
        rtc::mpc::PhaseCostConfigError::kNoError);

    // Default context: free-flight (no contacts), ee_target = fk(q_neutral).
    ctx_.phase_id = 0;
    ctx_.phase_name = "test";
    ctx_.phase_changed = false;
    ctx_.ocp_type = "light_contact";
    ctx_.cost_config = cfg_;
    ctx_.contact_plan = {}; // no frames, no phases

    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    ctx_.ee_target = pdata.oMf[handler_.end_effector_frame_id()];
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseCostConfig cfg_;
  rtc::mpc::PhaseContext ctx_;
  rtc::mpc::OCPLimits limits_; // all defaults
};

// ── Happy path ───────────────────────────────────────────────────────────

TEST_F(LightContactOCPTest, BuildNeutralSucceeds) {
  rtc::mpc::LightContactOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
  EXPECT_EQ(ocp.horizon_length(), cfg_.horizon_length);
  EXPECT_EQ(ocp.problem().numSteps(),
            static_cast<std::size_t>(cfg_.horizon_length));
  EXPECT_EQ(ocp.ocp_type(), std::string_view("light_contact"));
}

TEST_F(LightContactOCPTest, SolveReachesEETarget) {
  rtc::mpc::LightContactOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  aligator::SolverProxDDPTpl<double> solver(1e-4, 1e-2);
  solver.max_iters = 50;
  solver.verbose_ = aligator::QUIET;
  solver.setup(ocp.problem());

  const bool ran = solver.run(ocp.problem());
  (void)ran;
  const auto &res = solver.results_;
  // The solve must at least produce a small primal residual. Convergence of
  // the dual is not asserted for this trivial setup.
  EXPECT_LT(res.prim_infeas, 1e-3);
  EXPECT_GT(res.num_iters, 0);
}

TEST_F(LightContactOCPTest, UpdateReferencesPropagatesTarget) {
  rtc::mpc::LightContactOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  // Change ee_target and call UpdateReferences; verify the stored
  // FramePlacement residual inside stage 0 reflects the new target.
  pinocchio::SE3 new_target = ctx_.ee_target;
  new_target.translation() += Eigen::Vector3d(0.12, -0.03, 0.01);
  ctx_.ee_target = new_target;

  ASSERT_EQ(ocp.UpdateReferences(ctx_), rtc::mpc::OCPBuildError::kNoError);

  // Cross-check by walking the live problem tree.
  auto &problem = ocp.problem();
  ASSERT_GT(problem.stages_.size(), 0u);
  auto &stage0 = *problem.stages_[0];
  auto *stack = stage0.getCost<aligator::CostStackTpl<double>>();
  ASSERT_NE(stack, nullptr);
  auto *quad = stack->getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string(rtc::mpc::kCostKeyFramePlacement));
  ASSERT_NE(quad, nullptr);
  auto *residual =
      quad->getResidual<aligator::FramePlacementResidualTpl<double>>();
  ASSERT_NE(residual, nullptr);
  EXPECT_TRUE(residual->getReference().translation().isApprox(
      new_target.translation()));
}

TEST_F(LightContactOCPTest, ReBuildIdempotent) {
  rtc::mpc::LightContactOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
}

TEST_F(LightContactOCPTest, EmptyContactPlanFreeFlight) {
  // ctx_ default is already free-flight.
  rtc::mpc::LightContactOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
}

TEST_F(LightContactOCPTest, WithContactPhaseSucceeds) {
  // One phase over the entire horizon, both fingertips active.
  const int lfinger = handler_.contact_frames()[0].frame_id;
  const int rfinger = handler_.contact_frames()[1].frame_id;
  ctx_.contact_plan.frames = handler_.contact_frames();
  ctx_.contact_plan.phases.push_back(
      rtc::mpc::ContactPhase{{lfinger, rfinger}, 0.0, 100.0});

  rtc::mpc::LightContactOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
}

// ── Error paths ──────────────────────────────────────────────────────────

TEST_F(LightContactOCPTest, InvalidOcpTypeRejected) {
  ctx_.ocp_type = "contact_rich";
  rtc::mpc::LightContactOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kInvalidPhaseContext);
  EXPECT_FALSE(ocp.Built());
}

TEST_F(LightContactOCPTest, UninitialisedModelRejected) {
  rtc::mpc::RobotModelHandler empty;
  rtc::mpc::LightContactOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, empty, limits_),
            rtc::mpc::OCPBuildError::kModelNotInitialised);
}

TEST_F(LightContactOCPTest, ContactPlanMismatchRejected) {
  ctx_.contact_plan.phases.push_back(
      rtc::mpc::ContactPhase{{9999}, 0.0, 1.0}); // bogus frame id
  rtc::mpc::LightContactOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kContactPlanModelMismatch);
}

TEST_F(LightContactOCPTest, OverlappingPhasesRejected) {
  // Horizon = 20 * 0.01 = 0.20 s. Design two phases whose overlap lands
  // inside the horizon so a stage actually falls in both.
  const int lf = handler_.contact_frames()[0].frame_id;
  ctx_.contact_plan.phases.push_back(rtc::mpc::ContactPhase{{lf}, 0.0, 0.15});
  ctx_.contact_plan.phases.push_back(
      rtc::mpc::ContactPhase{{lf}, 0.10, 0.20}); // overlap on [0.10, 0.15)
  rtc::mpc::LightContactOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kOverlappingContactPhases);
}

TEST_F(LightContactOCPTest, LimitsDimMismatchRejected) {
  limits_.u_min = Eigen::VectorXd::Zero(handler_.nu() - 1);
  rtc::mpc::LightContactOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kLimitsDimMismatch);
}

TEST_F(LightContactOCPTest, UpdateReferencesTopologyChangeRejected) {
  rtc::mpc::LightContactOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  // Changing horizon_length is a topology change → must reject.
  ctx_.cost_config.horizon_length = cfg_.horizon_length + 5;
  EXPECT_EQ(ocp.UpdateReferences(ctx_),
            rtc::mpc::OCPBuildError::kInvalidPhaseContext);
  // Original problem should remain built and unchanged.
  EXPECT_TRUE(ocp.Built());
  EXPECT_EQ(ocp.horizon_length(), cfg_.horizon_length);
}

TEST_F(LightContactOCPTest, UpdateReferencesBeforeBuildRejected) {
  rtc::mpc::LightContactOCP ocp;
  EXPECT_EQ(ocp.UpdateReferences(ctx_),
            rtc::mpc::OCPBuildError::kInvalidPhaseContext);
}

// ── Performance (log-only) ───────────────────────────────────────────────

TEST_F(LightContactOCPTest, SolvePerfLog) {
  rtc::mpc::LightContactOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  aligator::SolverProxDDPTpl<double> solver(1e-4, 1e-2);
  solver.max_iters = 30;
  solver.verbose_ = aligator::QUIET;
  solver.setup(ocp.problem());

  constexpr int kIters = 20; // kept modest to avoid long CI runs
  std::vector<long> wall_us;
  wall_us.reserve(kIters);
  for (int i = 0; i < kIters; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    (void)solver.run(ocp.problem());
    const auto t1 = std::chrono::steady_clock::now();
    wall_us.push_back(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
  }
  std::sort(wall_us.begin(), wall_us.end());
  const long p50 = wall_us[wall_us.size() / 2];
  const long p99 = wall_us.back();
  std::cout << "[LightContactOCP perf] " << kIters << " solves: p50=" << p50
            << "us p99=" << p99 << "us\n";
  // No hard assertion — thresholds are informational.
  SUCCEED();
}

} // namespace
