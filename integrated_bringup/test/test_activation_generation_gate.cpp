// ── test_activation_generation_gate.cpp ──────────────────────────────────────
// Activation generation gate (issue #196 §3) for the integrated_bringup demo
// controllers.
//
// A controller's target subscriptions stay alive while it is Inactive
// (rclcpp_lifecycle gates publishers only), so goals addressed to a deactivated
// controller keep landing on its pending-target queue. After re-activation the
// first RT tick must hold the measured state, not replay whatever was queued in
// the meantime.
//
// The hand (device 1) is used as the observable because its command is plain
// joint space: `commands[i]` holds the seeded pose until a target moves it.
//
// The base contract lives in
// rtc_controller_interface/test/test_activation_generation.cpp; the rtc_controllers
// counterpart in rtc_controllers/test/test_activation_generation_gate.cpp.
// ─────────────────────────────────────────────────────────────────────────────
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace {

using rtc::ControllerState;

constexpr int kHandChannels = 10;
constexpr double kHandRest = 0.5;     // where the hand actually is
constexpr double kStaleTarget = 0.9;  // commanded while Inactive
constexpr double kTolerance = 1e-6;
constexpr int kSettleTicks = 300;  // 300 x 2 ms = 0.6 s > hand trajectory duration

const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive");

ControllerState MakeState() {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = 0.002;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = 6;
  dev0.valid = true;

  auto& dev1 = state.devices[1];
  dev1.num_channels = kHandChannels;
  dev1.valid = true;
  for (int i = 0; i < kHandChannels; ++i) {
    dev1.positions[static_cast<std::size_t>(i)] = kHandRest;
  }
  return state;
}

// activate → tick (hand hold seeds at kHandRest) → deactivate → a target lands
// on the still-live subscription → re-activate → tick. The hand must still be
// commanded to its measured rest pose.
template <typename Controller>
void ExpectStaleHandTargetDropped(Controller& ctrl) {
  auto state = MakeState();

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  (void)ctrl.Compute(state);
  state.iteration = 2;
  (void)ctrl.Compute(state);  // hand seed is deferred one tick in some controllers

  ASSERT_EQ(ctrl.on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  std::array<double, kHandChannels> stale{};
  stale.fill(kStaleTarget);
  ctrl.SetDeviceTarget(1, stale);

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);

  // Run past the hand trajectory duration: a stale target that survived the
  // cycle would have driven the fingers most of the way to kStaleTarget by now,
  // whereas a dropped one leaves the hold pose untouched.
  rtc::ControllerOutput out{};
  for (int i = 0; i < kSettleTicks; ++i) {
    state.iteration = static_cast<std::uint64_t>(i + 3);
    out = ctrl.Compute(state);
  }

  ASSERT_GT(out.num_devices, 1);
  for (int i = 0; i < kHandChannels; ++i) {
    EXPECT_NEAR(out.devices[1].commands[static_cast<std::size_t>(i)], kHandRest, kTolerance)
        << "finger " << i << ": target queued while Inactive overwrote the measured-pose hold";
  }
}

// The activation-generation half, for the bindings whose Compute() cannot run
// model-less. `ActivationGeneration()` lives in the base and is bumped by the
// base's on_activate; an override that forgets to delegate leaves it frozen at
// 0, which never re-arms the self-init latch and never invalidates a queued
// target. Two cycles rather than one: a generation that is SET rather than
// INCREMENTED passes the first assertion and fails the second.
template <typename Controller>
void ExpectActivationDelegatesToBase(Controller& ctrl) {
  EXPECT_EQ(ctrl.ActivationGeneration(), 0U);

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.ActivationGeneration(), 1U);

  ASSERT_EQ(ctrl.on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.ActivationGeneration(), 2U);
}

}  // namespace

TEST(IntegratedActivationGenerationGate, DemoJointDropsTargetQueuedWhileInactive) {
  integrated_bringup::DemoJointController ctrl{""};
  ExpectStaleHandTargetDropped(ctrl);
}

// DemoTask / DemoWbc / DemoCompliance need a real Pinocchio model before
// Compute() will run its non-E-STOP path, which a unit test cannot cheaply
// provide. Their drain guard is the same idiom covered end-to-end above and in
// rtc_controllers, so what is pinned here is the part that is actually easy to
// regress: an on_activate override that forgets to delegate to the base and
// therefore never bumps the generation nor re-arms the self-init latch.
TEST(IntegratedActivationGenerationGate, DemoTaskActivationDelegatesToBase) {
  integrated_bringup::DemoTaskController ctrl{"", integrated_bringup::DemoTaskController::Gains{}};
  ExpectActivationDelegatesToBase(ctrl);
}

TEST(IntegratedActivationGenerationGate, DemoWbcActivationDelegatesToBase) {
  integrated_bringup::DemoWbcController ctrl{""};
  ExpectActivationDelegatesToBase(ctrl);
}

// #469 S2 shipped this binding as a rename-copy of demo_task, so today this
// case asserts what its twin above does. It is listed by hand anyway because
// S3 gives the copy its own on_activate work (the admittance deviation has to
// be reset there, §7), and that is exactly the edit that can forget to call
// down — at which point this case is the only one that fails.
TEST(IntegratedActivationGenerationGate, DemoComplianceActivationDelegatesToBase) {
  integrated_bringup::DemoComplianceController ctrl{
      "", integrated_bringup::DemoComplianceController::Gains{}};
  ExpectActivationDelegatesToBase(ctrl);
}

// Positive control: a target delivered while the controller is Active is still
// consumed — the gate must reject only stale generations.
TEST(IntegratedActivationGenerationGate, DemoJointAppliesTargetSentWhileActive) {
  integrated_bringup::DemoJointController ctrl{""};
  auto state = MakeState();

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  (void)ctrl.Compute(state);
  state.iteration = 2;
  (void)ctrl.Compute(state);

  std::array<double, kHandChannels> fresh{};
  fresh.fill(kStaleTarget);
  ctrl.SetDeviceTarget(1, fresh);

  // The hand follows a trajectory, so assert it is moving toward the goal
  // rather than pinned to the hold pose.
  rtc::ControllerOutput out{};
  for (int i = 0; i < kSettleTicks; ++i) {
    state.iteration = static_cast<std::uint64_t>(i + 3);
    out = ctrl.Compute(state);
  }
  EXPECT_GT(out.devices[1].commands[0], kHandRest + 1e-3)
      << "target sent while Active never reached the hand";
}
