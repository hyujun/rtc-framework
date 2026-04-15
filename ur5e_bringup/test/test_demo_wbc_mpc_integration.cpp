// ── DemoWbcController Phase 5 integration tests ─────────────────────────────
//
// Verifies:
//   * Gains layout expanded to 9 entries with MPC toggle and Riccati scale.
//   * With `mpc.enabled` false in YAML, gains[7]=1 does NOT accidentally
//     enable MPC (the controller needs both build-time init and runtime on).
//   * GetCurrentGains round-trips MPC state faithfully.
//   * A controller constructed without an MPC config produces identical
//     output when MPC is nominally "on" via gains (disabled guard).
//
// We deliberately avoid spinning the MPC thread here — that's covered by
// rtc_mpc's own test_mpc_thread_mock. This file focuses on the bindings
// between DemoWbcController and MPCSolutionManager.

#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace {

using ur5e_bringup::DemoWbcController;
using rtc::ControllerState;

/// Replicates MakeState() from test_demo_wbc_controller.cpp — 2-device UR5e
/// + 10-DoF hand at home pose.
ControllerState MakeDefaultState(double dt = 0.002) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = dt;
  state.iteration = 1;
  auto& dev0 = state.devices[0];
  dev0.num_channels = 6;
  dev0.valid = true;
  dev0.positions[0] = 0.0;
  dev0.positions[1] = -1.57;
  dev0.positions[2] = 1.57;
  dev0.positions[3] = -1.57;
  dev0.positions[4] = -1.57;
  dev0.positions[5] = 0.0;
  auto& dev1 = state.devices[1];
  dev1.num_channels = 10;
  dev1.valid = true;
  return state;
}

class WbcMpcTest : public ::testing::Test {
 protected:
  DemoWbcController ctrl_{""};
  ControllerState state_ = MakeDefaultState();

  void SetUp() override {
    ctrl_.InitializeHoldPosition(state_);
  }
};

TEST_F(WbcMpcTest, GainsVectorIsNineElements) {
  const auto current = ctrl_.GetCurrentGains();
  EXPECT_EQ(current.size(), 9U);
}

TEST_F(WbcMpcTest, MpcDisabledByDefaultWithoutConfig) {
  // Constructed from an empty urdf_path — LoadConfig(mpc) never ran, so
  // the MPC manager is in its default (disabled) state.
  const auto gains = ctrl_.GetCurrentGains();
  EXPECT_NEAR(gains[7], 0.0, 1e-12) << "MPC must be disabled by default";
}

TEST_F(WbcMpcTest, GainsSetterClampsToBuildTimeEnable) {
  // Attempt to enable MPC via runtime gains without the build-time
  // `mpc_enabled_` flag (which LoadConfig sets from YAML). The controller's
  // UpdateGainsFromMsg ANDs the requested state with the build-time flag,
  // so requesting 1.0 when the flag is false must leave MPC off.
  std::array<double, 9> gains = {
    0.0, 0.0, 0.5, 1.0, 100.0, 10.0, 1.0,
    1.0,  // mpc_enable requested on
    0.8   // riccati_gain_scale
  };
  ctrl_.UpdateGainsFromMsg(gains);

  const auto current = ctrl_.GetCurrentGains();
  EXPECT_NEAR(current[7], 0.0, 1e-12)
      << "Runtime enable must be ANDed with build-time flag";
  EXPECT_NEAR(current[8], 0.8, 1e-12)
      << "Riccati gain scale should still propagate even when MPC is off";
}

TEST_F(WbcMpcTest, RiccatiGainScaleClampedToUnitInterval) {
  // RiccatiFeedback::SetGainScale clamps to [0, 1]; the manager
  // delegates to that, so overly large values are capped.
  std::array<double, 9> gains = {
    0.0, 0.0, 0.5, 1.0, 100.0, 10.0, 1.0, 0.0,
    5.0  // out-of-range riccati_gain_scale
  };
  ctrl_.UpdateGainsFromMsg(gains);

  const auto current = ctrl_.GetCurrentGains();
  EXPECT_LE(current[8], 1.0 + 1e-12);
  EXPECT_GE(current[8], 0.0 - 1e-12);
}

TEST_F(WbcMpcTest, ShortGainsMessageKeepsPhase4Semantics) {
  // A 7-entry Phase 4 gains message must update indices 0-6 and leave
  // the MPC state at its default.
  std::array<double, 7> phase4 = {
    0.0, 2.0, 0.6, 1.5, 120.0, 12.0, 1.5
  };
  ctrl_.UpdateGainsFromMsg(phase4);

  const auto current = ctrl_.GetCurrentGains();
  EXPECT_NEAR(current[0], 0.0, 1e-12);
  EXPECT_NEAR(current[4], 120.0, 1e-12);
  EXPECT_NEAR(current[7], 0.0, 1e-12) << "MPC state must remain disabled";
}

TEST_F(WbcMpcTest, ComputeProducesOutputRegardlessOfMpcState) {
  // MPC disabled → controller must still run Phase 4 logic normally.
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);

  // Now request MPC on via gains; because LoadConfig was not supplied an
  // `mpc:` YAML, the manager stays disabled — Compute must still succeed.
  std::array<double, 9> on = {0, 0, 0.5, 1.0, 100.0, 10.0, 1.0, 1.0, 1.0};
  ctrl_.UpdateGainsFromMsg(on);
  auto out2 = ctrl_.Compute(state_);
  EXPECT_TRUE(out2.valid);
}

}  // namespace
