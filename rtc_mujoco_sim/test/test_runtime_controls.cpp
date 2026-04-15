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

TEST_F(RuntimeControls, GravityLockBlocksEnable) {
  // Default: in servo mode, gravity is locked
  EXPECT_TRUE(sim_->IsGravityLockedByServo());
  sim_->EnableGravity(true);
  EXPECT_FALSE(sim_->IsGravityEnabled());  // lock prevents enable
}

TEST_F(RuntimeControls, ExternalForceOnInvalidBodyIsNoOp) {
  std::array<double, 6> wrench = {0, 0, 10, 0, 0, 0};
  sim_->SetExternalForce(-1, wrench);    // body_id <= 0 → reject
  sim_->SetExternalForce(9999, wrench);  // out of range → reject
  sim_->ClearExternalForce();  // no crash
  SUCCEED();
}

TEST_F(RuntimeControls, CommandModeFlagRoundtrip) {
  sim_->SetControlMode(0, true);
  EXPECT_TRUE(sim_->IsInTorqueMode(0));
  sim_->SetControlMode(0, false);
  EXPECT_FALSE(sim_->IsInTorqueMode(0));
}

}  // namespace
}  // namespace rtc
