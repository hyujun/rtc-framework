// ── test_runtime_controls.cpp ────────────────────────────────────────────────
// Atomic setter/getter pairs for runtime solver / physics controls.
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <array>

namespace rtc {
namespace {

class RuntimeControls : public ::testing::Test {
 protected:
  void SetUp() override {
    sim_ = std::make_unique<MuJoCoSimulator>(test::MakeMinimalConfig());
    ASSERT_TRUE(sim_->Initialize());
  }

  std::unique_ptr<MuJoCoSimulator> sim_;
};

TEST_F(RuntimeControls, IntegratorRoundtrip) {
  sim_->SetIntegrator(mjINT_RK4);
  EXPECT_EQ(sim_->GetIntegrator(), mjINT_RK4);
  sim_->SetIntegrator(mjINT_IMPLICIT);
  EXPECT_EQ(sim_->GetIntegrator(), mjINT_IMPLICIT);
}

TEST_F(RuntimeControls, SolverTypeRoundtrip) {
  sim_->SetSolverType(mjSOL_CG);
  EXPECT_EQ(sim_->GetSolverType(), mjSOL_CG);
}

TEST_F(RuntimeControls, SolverIterationsClamped) {
  sim_->SetSolverIterations(50);
  EXPECT_EQ(sim_->GetSolverIterations(), 50);
  sim_->SetSolverIterations(-5);
  EXPECT_EQ(sim_->GetSolverIterations(), 1);  // clamped to min=1
  sim_->SetSolverIterations(5000);
  EXPECT_EQ(sim_->GetSolverIterations(), 1000);  // clamped to max=1000
}

TEST_F(RuntimeControls, SolverToleranceClampedNonnegative) {
  sim_->SetSolverTolerance(1e-5);
  EXPECT_DOUBLE_EQ(sim_->GetSolverTolerance(), 1e-5);
  sim_->SetSolverTolerance(-1.0);
  EXPECT_DOUBLE_EQ(sim_->GetSolverTolerance(), 0.0);
}

TEST_F(RuntimeControls, ContactEnabledToggle) {
  sim_->SetContactEnabled(false);
  EXPECT_FALSE(sim_->IsContactEnabled());
  sim_->SetContactEnabled(true);
  EXPECT_TRUE(sim_->IsContactEnabled());
}

TEST_F(RuntimeControls, ImpratioGuardsNegative) {
  sim_->SetImpratio(2.5);
  EXPECT_DOUBLE_EQ(sim_->GetImpratio(), 2.5);
  sim_->SetImpratio(-1.0);
  EXPECT_DOUBLE_EQ(sim_->GetImpratio(), 1.0);  // reset to default
}

TEST_F(RuntimeControls, NoslipIterationsClamped) {
  sim_->SetNoslipIterations(5);
  EXPECT_EQ(sim_->GetNoslipIterations(), 5);
  sim_->SetNoslipIterations(-1);
  EXPECT_EQ(sim_->GetNoslipIterations(), 0);
  sim_->SetNoslipIterations(2000);
  EXPECT_EQ(sim_->GetNoslipIterations(), 1000);
}

TEST_F(RuntimeControls, MaxRtfGuardsNegative) {
  sim_->SetMaxRtf(2.0);
  EXPECT_DOUBLE_EQ(sim_->GetMaxRtf(), 2.0);
  sim_->SetMaxRtf(-1.0);
  EXPECT_DOUBLE_EQ(sim_->GetMaxRtf(), 0.0);
}

TEST_F(RuntimeControls, WorldGravityRoundtrip) {
  // World gravity is on by default so free objects in the scene fall normally,
  // even while a robot group is in position-servo (per-body gravcomp) mode.
  EXPECT_TRUE(sim_->IsWorldGravityEnabled());
  sim_->EnableWorldGravity(false);
  EXPECT_FALSE(sim_->IsWorldGravityEnabled());
  sim_->EnableWorldGravity(true);
  EXPECT_TRUE(sim_->IsWorldGravityEnabled());
}

// An out-of-range body must be REPORTED as rejected, not merely survived. The
// call used to return void, so "no crash" was the whole assertion and a caller
// had no way to learn its request went nowhere — the silent-failure path the
// /sim/set_external_wrench service exists to close (#135).
TEST_F(RuntimeControls, ExternalForceOnInvalidBodyIsRejected) {
  std::array<double, 6> wrench = {0, 0, 10, 0, 0, 0};
  EXPECT_FALSE(sim_->SetExternalForce(-1, wrench));    // body_id <= 0
  EXPECT_FALSE(sim_->SetExternalForce(0, wrench));     // world body
  EXPECT_FALSE(sim_->SetExternalForce(9999, wrench));  // out of range
  EXPECT_FALSE(sim_->SetExternalWrenchAtPoint(9999, {0.1, 0.0, 0.0}, wrench));
  sim_->ClearExternalForce();  // no crash
}

TEST_F(RuntimeControls, CommandModeFlagRoundtrip) {
  sim_->SetControlMode(0, true);
  EXPECT_TRUE(sim_->IsInTorqueMode(0));
  sim_->SetControlMode(0, false);
  EXPECT_FALSE(sim_->IsInTorqueMode(0));
}

TEST_F(RuntimeControls, ControlModeEnumRoundtrip) {
  sim_->SetControlMode(0, JointControlMode::kTorque);
  EXPECT_EQ(sim_->GetControlMode(0), JointControlMode::kTorque);
  sim_->SetControlMode(0, JointControlMode::kPdFeedforward);
  EXPECT_EQ(sim_->GetControlMode(0), JointControlMode::kPdFeedforward);
  sim_->SetControlMode(0, JointControlMode::kPosition);
  EXPECT_EQ(sim_->GetControlMode(0), JointControlMode::kPosition);
}

TEST_F(RuntimeControls, PdFeedforwardDisablesGravcomp) {
  // Only kPosition keeps MuJoCo gravity comp; pd_feedforward leaves it to the
  // controller's feedforward torque (which must include gravity).
  sim_->StageCommand(0, JointControlMode::kPdFeedforward, {0.0, 0.0}, {0.0, 0.0}, {}, {});
  EXPECT_EQ(sim_->GetControlMode(0), JointControlMode::kPdFeedforward);
  EXPECT_FALSE(sim_->IsGroupGravcompEnabled(0));
}

TEST_F(RuntimeControls, PdFeedforwardInjectsAppliedTorque) {
  sim_->StageCommand(0, JointControlMode::kPdFeedforward, {0.0, 0.0}, {1.5, -2.5}, {}, {});
  sim_->StepForTest();
  EXPECT_DOUBLE_EQ(sim_->GetAppliedForceForTest(0, 0), 1.5);
  EXPECT_DOUBLE_EQ(sim_->GetAppliedForceForTest(0, 1), -2.5);
}

TEST_F(RuntimeControls, ExitingPdFeedforwardClearsAppliedTorque) {
  sim_->StageCommand(0, JointControlMode::kPdFeedforward, {0.0, 0.0}, {3.0, 4.0}, {}, {});
  sim_->StepForTest();
  ASSERT_DOUBLE_EQ(sim_->GetAppliedForceForTest(0, 0), 3.0);
  // Switch back to position servo — feedforward must be zeroed, not stale.
  sim_->StageCommand(0, JointControlMode::kPosition, {0.0, 0.0}, {}, {}, {});
  sim_->StepForTest();
  EXPECT_DOUBLE_EQ(sim_->GetAppliedForceForTest(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(sim_->GetAppliedForceForTest(0, 1), 0.0);
}

TEST_F(RuntimeControls, RuntimeGainUpdateAppliesToActuator) {
  sim_->StageCommand(0, JointControlMode::kPdFeedforward, {0.0, 0.0}, {0.0, 0.0}, {123.0, 456.0},
                     {5.0, 6.0});
  sim_->StepForTest();
  EXPECT_DOUBLE_EQ(sim_->GetActuatorGainForTest(0, 0), 123.0);
  EXPECT_DOUBLE_EQ(sim_->GetActuatorGainForTest(0, 1), 456.0);
}

TEST_F(RuntimeControls, StageCommandIgnoresMismatchedGainArrays) {
  // Apply a known gain first, then a malformed (wrong-size) gain update: the
  // StageCommand guard requires kp.size()==kd.size()==num_command_joints, so the
  // bad update must be a no-op and the prior gain must persist.
  sim_->StageCommand(0, JointControlMode::kPdFeedforward, {0.0, 0.0}, {0.0, 0.0}, {200.0, 200.0},
                     {5.0, 5.0});
  sim_->StepForTest();
  ASSERT_DOUBLE_EQ(sim_->GetActuatorGainForTest(0, 0), 200.0);
  sim_->StageCommand(0, JointControlMode::kPdFeedforward, {0.0, 0.0}, {0.0, 0.0}, {999.0}, {});
  sim_->StepForTest();
  EXPECT_DOUBLE_EQ(sim_->GetActuatorGainForTest(0, 0), 200.0);
}

}  // namespace
}  // namespace rtc
