// ── test_sim_effort_force.cpp ────────────────────────────────────────────────
// Live-sim coverage for effort read-back and external-force injection. Drives
// the simulator synchronously via StepForTest() (SimLoop thread NOT running).
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>

namespace rtc {
namespace {

class SimEffortForce : public ::testing::Test {
 protected:
  void SetUp() override {
    sim_ = std::make_unique<MuJoCoSimulator>(test::MakeMinimalConfig());
    ASSERT_TRUE(sim_->Initialize());
  }

  std::unique_ptr<MuJoCoSimulator> sim_;
};

// GetEfforts returns one entry per state joint, all finite after a real step
// (efforts = qfrc_actuator + qfrc_applied, computed in ReadState).
TEST_F(SimEffortForce, EffortsSizedAndFiniteAfterStep) {
  sim_->StepForTest();
  const auto efforts = sim_->GetEfforts(0);
  ASSERT_EQ(efforts.size(), static_cast<std::size_t>(sim_->NumStateJoints(0)));
  for (double e : efforts) {
    EXPECT_TRUE(std::isfinite(e));
  }
}

// SetExternalForce on a valid body stages a wrench that PreparePhysicsStep
// memcpy's into data_->xfrc_applied on the next tick.
TEST_F(SimEffortForce, ExternalForceWritesXfrcApplied) {
  const int body_id = 1;  // link1 (world body is index 0)
  ASSERT_LT(body_id, sim_->GetModel()->nbody);

  const std::array<double, 6> wrench = {0.0, 0.0, 7.0, 0.0, 0.0, 0.0};
  sim_->SetExternalForce(body_id, wrench);
  sim_->StepForTest();

  const std::size_t off = static_cast<std::size_t>(body_id) * 6;
  const auto* xfrc = sim_->GetData()->xfrc_applied;
  EXPECT_DOUBLE_EQ(xfrc[off + 2], 7.0);  // Fz applied to link1
  EXPECT_DOUBLE_EQ(xfrc[off + 0], 0.0);
}

// ClearExternalForce drops the dirty flag so PreparePhysicsStep zeroes
// xfrc_applied on the following tick.
TEST_F(SimEffortForce, ClearExternalForceZerosXfrcApplied) {
  const int body_id = 1;
  const std::array<double, 6> wrench = {0.0, 0.0, 7.0, 0.0, 0.0, 0.0};
  sim_->SetExternalForce(body_id, wrench);
  sim_->StepForTest();
  const std::size_t off = static_cast<std::size_t>(body_id) * 6;
  ASSERT_DOUBLE_EQ(sim_->GetData()->xfrc_applied[off + 2], 7.0);

  sim_->ClearExternalForce();
  sim_->StepForTest();
  EXPECT_DOUBLE_EQ(sim_->GetData()->xfrc_applied[off + 2], 0.0);
}

}  // namespace
}  // namespace rtc
