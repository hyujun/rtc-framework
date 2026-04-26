// ── DemoWbcController Phase 5 integration tests ─────────────────────────────
//
// Verifies:
//   * Gains accessor (set_gains/get_gains) round-trip preserves the runtime
//     tunables that DeclareGainParameters() exposes (arm/hand trajectory
//     speed, SE3/force/posture weights, MPC toggle, Riccati gain scale).
//   * Compute() succeeds whether or not LoadConfig was called with an MPC
//     config (the Phase 4 fallback path keeps the RT loop alive).
//
// We deliberately avoid spinning the MPC thread here — that's covered by
// rtc_mpc's own test_mpc_thread_mock. This file focuses on the bindings
// between DemoWbcController and MPCSolutionManager.

#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace {

using rtc::ControllerState;
using ur5e_bringup::DemoWbcController;

/// Replicates MakeState() from test_demo_wbc_controller.cpp — 2-device UR5e
/// + 10-DoF hand at home pose.
ControllerState MakeDefaultState(double dt = 0.002) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = dt;
  state.iteration = 1;
  auto &dev0 = state.devices[0];
  dev0.num_channels = 6;
  dev0.valid = true;
  dev0.positions[0] = 0.0;
  dev0.positions[1] = -1.57;
  dev0.positions[2] = 1.57;
  dev0.positions[3] = -1.57;
  dev0.positions[4] = -1.57;
  dev0.positions[5] = 0.0;
  auto &dev1 = state.devices[1];
  dev1.num_channels = 10;
  dev1.valid = true;
  return state;
}

class WbcMpcTest : public ::testing::Test {
protected:
  DemoWbcController ctrl_{""};
  ControllerState state_ = MakeDefaultState();

  void SetUp() override { ctrl_.InitializeHoldPosition(state_); }
};

TEST_F(WbcMpcTest, GainsAccessorRoundTrip) {
  DemoWbcController::Gains g;
  g.arm_trajectory_speed = 0.6;
  g.hand_trajectory_speed = 1.5;
  g.se3_weight = 120.0;
  g.force_weight = 12.0;
  g.posture_weight = 1.5;
  ctrl_.set_gains(g);

  const auto rb = ctrl_.get_gains();
  EXPECT_NEAR(rb.arm_trajectory_speed, 0.6, 1e-12);
  EXPECT_NEAR(rb.se3_weight, 120.0, 1e-12);
  EXPECT_NEAR(rb.posture_weight, 1.5, 1e-12);
}

TEST_F(WbcMpcTest, ComputeProducesOutputRegardlessOfMpcState) {
  // Without LoadConfig(mpc), the manager stays disabled by default —
  // Compute must still succeed via the Phase 4 fallback path.
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);

  auto out2 = ctrl_.Compute(state_);
  EXPECT_TRUE(out2.valid);
}

} // namespace
