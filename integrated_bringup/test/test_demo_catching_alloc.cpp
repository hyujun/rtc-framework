// ── DemoCatchingController RT-1 zero-allocation gate ─────────────────────────
//
// Its own binary because alloc_gate.hpp installs a replacement global
// `operator new` — one TU per binary (alloc_gate.hpp contract item 1).
//
// WHAT THIS GATE SEES. Global `operator new`, across translation units, so it
// does cover the real Compute() compiled into the integrated_bringup library.
// It does NOT see Eigen's aligned_malloc — irrelevant here, since this
// controller's tick touches no Eigen at all: it copies fixed arrays and clamps.
// The positive control below is what makes the zero mean anything.

#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using rtc::ControllerState;

constexpr int kArmDof = 6;
constexpr int kHandDof = 4;

std::string MinimalYaml() {
  return R"(
command_type: "position"
diagnostic:
  hand_step: true
catching:
  robot:
    hand:
      provisional: true
      rho_eps: 0.02
      q_open:  [0.0, 0.0, 0.0, 0.0]
      q_pre:   [0.1, 0.1, 0.1, 0.1]
      q_close: [0.6, 0.6, 0.6, 0.6]
      caging_mask: [true, true, true, true]
      eta_close: 0.9
      T_close_e2e: TBD
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
)";
}

std::map<std::string, rtc::DeviceNameConfig> MakeConfigs() {
  std::map<std::string, rtc::DeviceNameConfig> configs;
  rtc::DeviceNameConfig arm;
  arm.device_name = "arm";
  arm.joint_state_names = {"a0", "a1", "a2", "a3", "a4", "a5"};
  rtc::DeviceBackendBinding arm_backend;
  arm_backend.type = "mujoco_native";
  arm.backend = arm_backend;
  configs["arm"] = std::move(arm);

  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  hand.joint_state_names = {"h0", "h1", "h2", "h3"};
  rtc::DeviceJointLimits limits;
  limits.position_lower = {-1.0, -1.0, -1.0, -1.0};
  limits.position_upper = {1.0, 1.0, 1.0, 1.0};
  hand.joint_limits = limits;
  rtc::DeviceBackendBinding hand_backend;
  hand_backend.type = "mujoco_native";
  hand.backend = hand_backend;
  configs["hand"] = std::move(hand);
  return configs;
}

ControllerState MakeState() {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = 0.002;
  state.iteration = 1;
  state.devices[0].num_channels = kArmDof;
  state.devices[0].valid = true;
  state.devices[1].num_channels = kHandDof;
  state.devices[1].valid = true;
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.1 * static_cast<double>(i);
  }
  for (int i = 0; i < kHandDof; ++i) {
    state.devices[1].positions[static_cast<std::size_t>(i)] = 0.05 * static_cast<double>(i);
  }
  return state;
}

TEST(DemoCatchingAlloc, ComputeAllocatesNothingOnTheHoldPath) {
  DemoCatchingController ctrl{""};
  ctrl.LoadConfig(YAML::Load(MinimalYaml()));
  ctrl.SetDeviceNameConfigs(MakeConfigs());
  const ControllerState state = MakeState();
  static_cast<void>(ctrl.Compute(state));  // warm the latches outside the gate

  rtc::testing::ScopedAllocGate gate;
  for (int i = 0; i < 100; ++i) {
    static_cast<void>(ctrl.Compute(state));
  }
  EXPECT_EQ(gate.count(), 0U);
}

TEST(DemoCatchingAlloc, ComputeAllocatesNothingOnTheStepPath) {
  // The drain path: the mailbox pop, ApplyPendingTarget and the clamp all run
  // on the RT tick, so they are inside the gate. SetDeviceTarget is off-RT and
  // is called OUTSIDE it.
  DemoCatchingController ctrl{""};
  ctrl.LoadConfig(YAML::Load(MinimalYaml()));
  ctrl.SetDeviceNameConfigs(MakeConfigs());
  const ControllerState state = MakeState();
  static_cast<void>(ctrl.Compute(state));

  const std::array<double, kHandDof> target{0.5, -0.5, 0.25, 0.75};
  std::size_t worst = 0;
  for (int i = 0; i < 20; ++i) {
    ctrl.SetDeviceTarget(1, std::span<const double>(target));
    rtc::testing::ScopedAllocGate gate;
    static_cast<void>(ctrl.Compute(state));
    worst = std::max(worst, gate.count());
  }
  EXPECT_EQ(worst, 0U);
}

TEST(DemoCatchingAlloc, PositiveControlTheGateIsArmed) {
  rtc::testing::ScopedAllocGate gate;
  std::vector<double> v;
  v.reserve(64);
  EXPECT_GT(gate.count(), 0U) << "the gate is inert — every zero above proves nothing";
}

}  // namespace
