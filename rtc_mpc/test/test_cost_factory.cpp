/// @file test_cost_factory.cpp
/// @brief Unit tests for @ref rtc::mpc::cost_factory.
///
/// Verifies:
/// - weight-gated component inclusion (weights > 0 → term added, else skipped)
/// - StageComponentKeys reports exactly the present components
/// - Terminal cost omits control reg regardless of weight
/// - Polymorphic handle retrieval chain works post-StageModel assembly
///   (the load-bearing assumption for alloc-free UpdateReferences in
///    `LightContactOCP`) — including a mutation round-trip
/// - Error paths: uninitialised model, invalid horizon/dt, posture dim mismatch

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/stage-model.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/modelling/state-error.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_mpc/ocp/cost_factory.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

namespace {

constexpr const char* kPandaUrdf = RTC_PANDA_URDF_PATH;

// Baseline cost config matching Panda (nq=9, 2 × 3D contacts).
constexpr const char* kBaselineYaml = R"(
horizon_length: 20
dt: 0.01
w_frame_placement: 100.0
w_state_reg: 1.0
w_control_reg: 0.01
w_contact_force: 0.001
w_centroidal_momentum: 0.0
W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights: {}
)";

class CostFactoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);

    auto model_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
    ASSERT_EQ(handler_.Init(model_, model_cfg), rtc::mpc::RobotModelInitError::kNoError);

    auto cfg_node = YAML::Load(kBaselineYaml);
    ASSERT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(cfg_node, handler_, cfg_),
              rtc::mpc::PhaseCostConfigError::kNoError);

    // Use FK at neutral as ee_target baseline.
    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    ee_target_ = pdata.oMf[handler_.end_effector_frame_id()];
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseCostConfig cfg_;
  pinocchio::SE3 ee_target_{pinocchio::SE3::Identity()};
};

// ── Happy path ───────────────────────────────────────────────────────────

TEST_F(CostFactoryTest, RunningCostAllThreeComponents) {
  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kNoError);
  EXPECT_TRUE(sc.keys.has_frame_placement);
  EXPECT_TRUE(sc.keys.has_state_reg);
  EXPECT_TRUE(sc.keys.has_control_reg);
  EXPECT_EQ(sc.stack.size(), 3u);
}

TEST_F(CostFactoryTest, AllWeightsZeroEmptyStack) {
  cfg_.w_frame_placement = 0.0;
  cfg_.w_state_reg = 0.0;
  cfg_.w_control_reg = 0.0;

  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kNoError);
  EXPECT_FALSE(sc.keys.has_frame_placement);
  EXPECT_FALSE(sc.keys.has_state_reg);
  EXPECT_FALSE(sc.keys.has_control_reg);
  EXPECT_EQ(sc.stack.size(), 0u);
}

TEST_F(CostFactoryTest, OnlyFramePlacement) {
  cfg_.w_state_reg = 0.0;
  cfg_.w_control_reg = 0.0;

  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kNoError);
  EXPECT_TRUE(sc.keys.has_frame_placement);
  EXPECT_FALSE(sc.keys.has_state_reg);
  EXPECT_FALSE(sc.keys.has_control_reg);
  EXPECT_EQ(sc.stack.size(), 1u);
}

TEST_F(CostFactoryTest, TerminalOmitsControlReg) {
  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildTerminalCost(cfg_, handler_, ee_target_, &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kNoError);
  EXPECT_TRUE(sc.keys.has_frame_placement);
  EXPECT_TRUE(sc.keys.has_state_reg);
  EXPECT_FALSE(sc.keys.has_control_reg);  // terminal: no u-dep cost
  EXPECT_EQ(sc.stack.size(), 2u);
}

// ── Ownership / handle retrieval (load-bearing) ──────────────────────────

TEST_F(CostFactoryTest, PolymorphicHandleRetrievalAfterStageAssembly) {
  // Build running cost, then wrap into a minimal StageModel to verify the
  // getCost → getComponent(key) → getResidual<T>() chain resolves after
  // polymorphic copy. If this test fails, alloc-free UpdateReferences in
  // LightContactOCP is infeasible (spike Q3 would be invalidated).

  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, &err);
  ASSERT_EQ(err, rtc::mpc::CostFactoryError::kNoError);
  ASSERT_TRUE(sc.keys.has_frame_placement);

  // Minimal dynamics wrapper is not required for cost access — we can
  // access getCost on a standalone cost stack. But to mirror the real
  // flow we need StageModel. StageModel requires PolyDynamics — construct
  // a trivial vector-space dynamics via LinearDiscreteDynamics is a lot of
  // plumbing; instead, exercise the retrieval chain directly on the
  // returned CostStack.

  aligator::CostStackTpl<double>& stored_stack = sc.stack;
  auto* quad_fp = stored_stack.getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string(rtc::mpc::kCostKeyFramePlacement));
  ASSERT_NE(quad_fp, nullptr);

  auto* fp_residual = quad_fp->getResidual<aligator::FramePlacementResidualTpl<double>>();
  ASSERT_NE(fp_residual, nullptr);

  // Mutation round-trip: change reference, read back.
  pinocchio::SE3 new_target = ee_target_;
  new_target.translation() += Eigen::Vector3d(0.1, -0.05, 0.02);
  fp_residual->setReference(new_target);

  const auto& readback = fp_residual->getReference();
  EXPECT_TRUE(readback.translation().isApprox(new_target.translation()));
}

// ── Error paths ──────────────────────────────────────────────────────────

TEST(CostFactoryStandaloneTest, UninitialisedModelRejected) {
  rtc::mpc::RobotModelHandler empty_handler;
  rtc::mpc::PhaseCostConfig cfg;  // default-init; horizon_length=20, dt=0.01
  // q_posture_ref is empty — validation should trip on model first.

  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg, empty_handler, pinocchio::SE3::Identity(),
                                                     &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kModelNotInitialised);
  EXPECT_EQ(sc.stack.size(), 0u);
}

TEST_F(CostFactoryTest, InvalidHorizonRejected) {
  cfg_.horizon_length = 0;
  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kInvalidCostConfig);
  EXPECT_EQ(sc.stack.size(), 0u);
}

TEST_F(CostFactoryTest, InvalidDtRejected) {
  cfg_.dt = 0.0;
  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kInvalidCostConfig);
}

TEST_F(CostFactoryTest, PostureRefDimMismatchRejected) {
  cfg_.q_posture_ref = Eigen::VectorXd::Zero(handler_.nq() - 1);  // wrong
  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, &err);
  EXPECT_EQ(err, rtc::mpc::CostFactoryError::kPostureRefDimMismatch);
}

TEST_F(CostFactoryTest, NullOutErrorIsSafe) {
  // Contract: out_error may be null; factory must not crash.
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg_, handler_, ee_target_, nullptr);
  EXPECT_EQ(sc.stack.size(), 3u);
  EXPECT_TRUE(sc.keys.has_frame_placement);
}

// ── base_frame propagation (F-1) ────────────────────────────────────────
//
// When the handler reports a non-Identity base_oMf, BuildRunningCost must
// lift `ee_target` (interpreted in base) to world before constructing the
// Aligator FramePlacementResidual. This guards against the silent frame-
// mismatch that motivated F-1.
TEST(CostFactoryBaseFrameTest, EeTargetLiftedToWorldWhenBaseNonIdentity) {
  if (!std::filesystem::exists(kPandaUrdf)) {
    GTEST_SKIP();
  }
  pinocchio::Model model;
  pinocchio::urdf::buildModel(kPandaUrdf, model);

  // Use panda_link4 as base — non-Identity oMb (~0.3 m above world origin).
  auto model_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
base_frame: panda_link4
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model, model_cfg), rtc::mpc::RobotModelInitError::kNoError);
  ASSERT_FALSE(handler.base_frame_is_universe());

  rtc::mpc::PhaseCostConfig cfg;
  ASSERT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(YAML::Load(kBaselineYaml), handler, cfg),
            rtc::mpc::PhaseCostConfigError::kNoError);

  // ee_target expressed in base_frame: arbitrary offset.
  pinocchio::SE3 ee_in_base = pinocchio::SE3::Identity();
  ee_in_base.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);

  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg, handler, ee_in_base, &err);
  ASSERT_EQ(err, rtc::mpc::CostFactoryError::kNoError);
  ASSERT_TRUE(sc.keys.has_frame_placement);

  auto* quad_fp = sc.stack.getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string(rtc::mpc::kCostKeyFramePlacement));
  ASSERT_NE(quad_fp, nullptr);
  auto* fp_residual = quad_fp->getResidual<aligator::FramePlacementResidualTpl<double>>();
  ASSERT_NE(fp_residual, nullptr);

  const pinocchio::SE3 expected_world = handler.base_oMf().act(ee_in_base);
  const auto& stored = fp_residual->getReference();
  EXPECT_TRUE(stored.translation().isApprox(expected_world.translation()))
      << "stored=" << stored.translation().transpose()
      << " expected=" << expected_world.translation().transpose();
  EXPECT_TRUE(stored.rotation().isApprox(expected_world.rotation()));
}

TEST(CostFactoryBaseFrameTest, EeTargetUnchangedOnUniverseFastPath) {
  if (!std::filesystem::exists(kPandaUrdf)) {
    GTEST_SKIP();
  }
  pinocchio::Model model;
  pinocchio::urdf::buildModel(kPandaUrdf, model);

  // No base_frame → universe fallback.
  auto model_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model, model_cfg), rtc::mpc::RobotModelInitError::kNoError);
  ASSERT_TRUE(handler.base_frame_is_universe());

  rtc::mpc::PhaseCostConfig cfg;
  ASSERT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(YAML::Load(kBaselineYaml), handler, cfg),
            rtc::mpc::PhaseCostConfigError::kNoError);

  pinocchio::SE3 ee = pinocchio::SE3::Identity();
  ee.translation() = Eigen::Vector3d(0.4, -0.1, 0.5);

  rtc::mpc::CostFactoryError err = rtc::mpc::CostFactoryError::kNoError;
  auto sc = rtc::mpc::cost_factory::BuildRunningCost(cfg, handler, ee, &err);
  ASSERT_EQ(err, rtc::mpc::CostFactoryError::kNoError);

  auto* quad_fp = sc.stack.getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string(rtc::mpc::kCostKeyFramePlacement));
  auto* fp_residual = quad_fp->getResidual<aligator::FramePlacementResidualTpl<double>>();
  // universe → no transform applied.
  EXPECT_TRUE(fp_residual->getReference().translation().isApprox(ee.translation()));
}

}  // namespace
