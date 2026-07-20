// ── test_activation_generation_gate.cpp ──────────────────────────────────────
// Per-controller half of the activation generation gate (issue #196 §3).
//
// The base contract (counter advances per activation, stale stamps rejected,
// ResetTargetInitialization fires) is pinned in
// rtc_controller_interface/test/test_activation_generation.cpp. Here we assert
// that each concrete controller actually honours it end-to-end:
//
//   1. a target delivered while the controller is Inactive does not reach the
//      target slot after re-activation — the first tick holds the current
//      measured state instead;
//   2. a target delivered after re-activation still lands.
//
// None of these four controllers had an on_activate override before this
// change, so (1) previously failed for all of them: the pre-deactivation goal
// stayed in the slot and the queued goal was drained straight into it.
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/direct/operational_space_controller.hpp"
#include "rtc_controllers/indirect/clik_controller.hpp"
#include "rtc_controllers/indirect/p_controller.hpp"
#include "test_urdf_path.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <string>

namespace {

constexpr int kDof = 6;
constexpr double kHoldPosition = 0.5;  // where the robot actually is
constexpr double kStaleTarget = 1.25;  // commanded while Inactive
constexpr double kFreshTarget = 0.75;  // commanded after re-activation
constexpr double kPoseTolerance = 1e-9;

const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive");

std::string GetTestUrdfPath() {
  if (const char* env = std::getenv("RTC_TEST_URDF_PATH")) {
    return env;
  }
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

rtc::ControllerState MakeState() {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = 0.002;
  state.devices[0].num_channels = kDof;
  state.devices[0].valid = true;
  for (int i = 0; i < kDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = kHoldPosition;
  }
  return state;
}

std::array<double, kDof> Fill(double v) {
  std::array<double, kDof> a{};
  a.fill(v);
  return a;
}

// Activate → tick (self-init seeds the hold target) → deactivate → deliver a
// target on the still-live subscription → re-activate → tick. `observe` reads
// whatever field mirrors the controller's target slot.
template <typename Controller, typename Observe>
void ExpectStaleTargetDropped(Controller& ctrl, Observe observe) {
  const auto state = MakeState();

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  (void)ctrl.Compute(state);

  ASSERT_EQ(ctrl.on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto stale = Fill(kStaleTarget);
  ctrl.SetDeviceTarget(0, stale);

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto out = ctrl.Compute(state);

  EXPECT_NEAR(observe(out), kHoldPosition, kPoseTolerance)
      << "target queued while Inactive overwrote the current-state hold";
}

template <typename Controller, typename Observe>
void ExpectFreshTargetApplied(Controller& ctrl, Observe observe) {
  const auto state = MakeState();

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  (void)ctrl.Compute(state);

  const auto fresh = Fill(kFreshTarget);
  ctrl.SetDeviceTarget(0, fresh);
  const auto out = ctrl.Compute(state);

  EXPECT_NEAR(observe(out), kFreshTarget, kPoseTolerance)
      << "target sent while Active must still be honoured";
}

// ── P controller ─────────────────────────────────────────────────────────────
// goal_positions mirrors slot.targets[0] directly.
double PGoal(const rtc::ControllerOutput& out) {
  return out.devices[0].goal_positions[0];
}

}  // namespace

TEST(ActivationGenerationGate, PControllerDropsTargetQueuedWhileInactive) {
  rtc::PController ctrl(GetTestUrdfPath());
  ExpectStaleTargetDropped(ctrl, PGoal);
}

TEST(ActivationGenerationGate, PControllerAppliesTargetSentWhileActive) {
  rtc::PController ctrl(GetTestUrdfPath());
  ExpectFreshTargetApplied(ctrl, PGoal);
}

// ── Joint PD controller ──────────────────────────────────────────────────────
// goal_positions mirrors slot.targets[0]; target_positions is the trajectory
// reference, which ramps, so the goal is the stable observable.
TEST(ActivationGenerationGate, JointPDControllerDropsTargetQueuedWhileInactive) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  ExpectStaleTargetDropped(ctrl, PGoal);
}

TEST(ActivationGenerationGate, JointPDControllerAppliesTargetSentWhileActive) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  ExpectFreshTargetApplied(ctrl, PGoal);
}

// ── Operational-space controller ─────────────────────────────────────────────
// Device 0 carries a 6-DoF pose target, not joint angles: goal_positions[0] is
// the commanded TCP x. The self-init seeds it from FK of the measured
// configuration, so the hold value is the TCP x at q = kHoldPosition rather
// than kHoldPosition itself — capture it from the first post-activation tick.
TEST(ActivationGenerationGate, OperationalSpaceDropsTargetQueuedWhileInactive) {
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), {});
  const auto state = MakeState();

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto seeded = ctrl.Compute(state);
  const double hold_x = seeded.devices[0].goal_positions[0];
  ASSERT_NE(hold_x, kStaleTarget);

  ASSERT_EQ(ctrl.on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto stale = Fill(kStaleTarget);
  ctrl.SetDeviceTarget(0, stale);

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto out = ctrl.Compute(state);
  EXPECT_NEAR(out.devices[0].goal_positions[0], hold_x, kPoseTolerance)
      << "pose target queued while Inactive overwrote the current-pose hold";
}

TEST(ActivationGenerationGate, OperationalSpaceAppliesTargetSentWhileActive) {
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), {});
  ExpectFreshTargetApplied(ctrl, PGoal);
}

// ── CLIK controller ──────────────────────────────────────────────────────────
// task_goal_positions[0] mirrors slot.tcp_target[0] (commanded TCP x).
TEST(ActivationGenerationGate, ClikDropsTargetQueuedWhileInactive) {
  rtc::ClikController ctrl(GetTestUrdfPath(), {});
  const auto state = MakeState();

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto seeded = ctrl.Compute(state);
  const double hold_x = seeded.task_goal_positions[0];
  ASSERT_NE(hold_x, kStaleTarget);

  ASSERT_EQ(ctrl.on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto stale = Fill(kStaleTarget);
  ctrl.SetDeviceTarget(0, stale);

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto out = ctrl.Compute(state);
  EXPECT_NEAR(out.task_goal_positions[0], hold_x, kPoseTolerance)
      << "TCP target queued while Inactive overwrote the current-pose hold";
}

TEST(ActivationGenerationGate, ClikAppliesTargetSentWhileActive) {
  rtc::ClikController ctrl(GetTestUrdfPath(), {});
  const auto state = MakeState();

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  (void)ctrl.Compute(state);

  const auto fresh = Fill(kFreshTarget);
  ctrl.SetDeviceTarget(0, fresh);
  const auto out = ctrl.Compute(state);
  EXPECT_NEAR(out.task_goal_positions[0], kFreshTarget, kPoseTolerance);
}

// Re-activation must also re-seed the hold target itself: before this change
// none of the four controllers reset target_initialized_ on activation, so a
// controller re-activated after the robot had moved kept commanding the
// position it held when it was deactivated.
TEST(ActivationGenerationGate, ReactivationReseedsHoldFromCurrentState) {
  rtc::PController ctrl(GetTestUrdfPath());

  auto state = MakeState();
  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  (void)ctrl.Compute(state);
  ASSERT_EQ(ctrl.on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);

  // Robot is moved by whoever was active in the meantime.
  constexpr double kMovedPosition = -0.4;
  for (int i = 0; i < kDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = kMovedPosition;
  }

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto out = ctrl.Compute(state);
  EXPECT_NEAR(out.devices[0].goal_positions[0], kMovedPosition, kPoseTolerance)
      << "re-activated controller kept its pre-deactivation hold target";
}
