// ── DemoCatchingController (dynamic_catching S4.0/S4.1) ──────────────────────
//
// What this suite pins, and why each one is here rather than assumed:
//
//   1. The arm is HELD. The whole sim-only argument (E-8) rests on the arm
//      command being a constant from the first readable tick, so the constancy
//      is asserted bit-exactly over many ticks with the measurement MOVING
//      underneath — a controller that tracked the measurement would pass a
//      one-tick check.
//   2. The hand step reaches the wire UNSHAPED. G6-B says the slot target IS
//      the backend command, and the CM half of that is already pinned in
//      test_rt_loop_pipeline (it copies, it does not clamp or filter). What is
//      left to prove is the binding half: what this controller writes equals
//      clamp(target), bit for bit, from the tick the target is accepted. If it
//      were shaped, S4.2 would be measuring the controller, not the hand.
//   3. The refusals refuse: a non-simulator backend, a hand target with the
//      diagnostic off, and an arm target at any time.
//   4. A target queued while Inactive does not leak into the next activation.
//
// Deliberately NOT in the four shared per-controller suites (testing-debug.md
// §28): device-readability and gate-closure assertions live HERE instead of in
// test_device_readability_gate because half of that suite's contract is the
// E-STOP safe-position ramp, which this controller does not have (it overrides
// no E-STOP hook — that IS the E-8 decision). The momentum-observer embedding
// suite does not apply either: this skeleton runs no arm dynamics and
// registers no observer. Both are revisited in S5.1, when the controller grows
// the law those suites are about.

#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controllers/catching/catching_params.hpp"
#include "shipped_config_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::kCatchingHandDeviceIdx;
using rtc::ControllerOutput;
using rtc::ControllerState;

constexpr int kArmDof = 6;
constexpr int kHandDof = 4;
constexpr double kHandLower = -1.0;
constexpr double kHandUpper = 1.0;

/// A 4-joint hand profile: every joint cages, and q_close is inside the
/// limits below so a clamp only fires when a test asks for one.
std::string MinimalYaml(bool hand_step) {
  return std::string(R"(
command_type: "position"
diagnostic:
  hand_step: )") +
         (hand_step ? "true" : "false") + R"(
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

/// The same profile with the consumed subset RESOLVED: not provisional, and
/// `T_close_e2e` filled in. Used for the cases that need the controller to be
/// armable — which is the consumed subset's verdict, not the whole schema's
/// (`reference.*` and `planner.*` are still TBD here, exactly as they are in
/// the shipped profile, and neither blocks this step).
std::string ClearedYaml(bool hand_step) {
  std::string yaml = MinimalYaml(hand_step);
  const auto replace = [&yaml](std::string_view from, std::string_view to) {
    const auto at = yaml.find(from);
    if (at != std::string::npos) {
      yaml.replace(at, from.size(), to);
    }
  };
  replace("provisional: true", "provisional: false");
  replace("T_close_e2e: TBD", "T_close_e2e: 0.28");
  return yaml;
}

std::map<std::string, rtc::DeviceNameConfig> MakeConfigs(const std::string& arm_backend,
                                                         const std::string& hand_backend) {
  std::map<std::string, rtc::DeviceNameConfig> configs;

  rtc::DeviceNameConfig arm;
  arm.device_name = "arm";
  arm.joint_state_names = {"a0", "a1", "a2", "a3", "a4", "a5"};
  if (!arm_backend.empty()) {
    rtc::DeviceBackendBinding b;
    b.type = arm_backend;
    b.state_topic = "/arm/joint_states";
    b.command_topic = "/arm/joint_command";
    arm.backend = b;
  }
  configs["arm"] = std::move(arm);

  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  hand.joint_state_names = {"h0", "h1", "h2", "h3"};
  rtc::DeviceJointLimits limits;
  limits.position_lower = {kHandLower, kHandLower, kHandLower, kHandLower};
  limits.position_upper = {kHandUpper, kHandUpper, kHandUpper, kHandUpper};
  hand.joint_limits = limits;
  if (!hand_backend.empty()) {
    rtc::DeviceBackendBinding b;
    b.type = hand_backend;
    b.state_topic = "/hand/joint_states";
    b.command_topic = "/hand/joint_command";
    hand.backend = b;
  }
  configs["hand"] = std::move(hand);

  return configs;
}

/// Fully reported state. `bias` shifts every measured position so a caller can
/// make the measurement move between ticks.
ControllerState MakeState(double bias = 0.0, int hand_channels = kHandDof) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = 0.002;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = kArmDof;
  dev0.valid = true;
  dev0.hole_mask = 0;
  for (int i = 0; i < kArmDof; ++i) {
    dev0.positions[static_cast<std::size_t>(i)] = 0.1 * static_cast<double>(i + 1) + bias;
  }

  auto& dev1 = state.devices[1];
  dev1.num_channels = hand_channels;
  dev1.valid = true;
  dev1.hole_mask = 0;
  for (int i = 0; i < hand_channels; ++i) {
    dev1.positions[static_cast<std::size_t>(i)] = 0.05 * static_cast<double>(i + 1) + bias;
  }

  return state;
}

/// Bring a controller up the way the CM does, minus the lifecycle node.
void BringUp(DemoCatchingController& ctrl, bool hand_step = true) {
  ctrl.LoadConfig(YAML::Load(MinimalYaml(hand_step)));
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
}

// ── 1. The arm is held ──────────────────────────────────────────────────────

TEST(DemoCatchingController, ArmCommandIsLatchedOnTheFirstTickAndNeverMovesAgain) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl);

  const ControllerOutput first = ctrl.Compute(MakeState(0.0));
  ASSERT_EQ(first.devices[0].num_channels, kArmDof);

  // The measurement drifts away under the controller; the command must not
  // follow it. A controller that tracked the measurement would pass a
  // single-tick comparison and fail here on tick 2.
  for (int tick = 1; tick < 50; ++tick) {
    const ControllerOutput out = ctrl.Compute(MakeState(0.01 * static_cast<double>(tick)));
    ASSERT_EQ(out.devices[0].num_channels, kArmDof) << "tick " << tick;
    for (int i = 0; i < kArmDof; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      EXPECT_EQ(out.devices[0].commands[idx], first.devices[0].commands[idx])
          << "arm joint " << i << " moved on tick " << tick;
    }
  }
}

TEST(DemoCatchingController, ArmLatchSeedsFromTheFirstREADABLETickNotTheFirstTick) {
  // Latching from an unreadable device would freeze a hold whose unreported
  // joints are 0 — "go to the origin" — and the latch makes that permanent.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);

  ControllerState blind = MakeState(0.0);
  blind.devices[0].valid = false;
  const ControllerOutput silent = ctrl.Compute(blind);
  EXPECT_EQ(silent.devices[0].num_channels, 0) << "an unreadable arm must be silenced, not zeroed";

  const ControllerState good = MakeState(0.5);
  const ControllerOutput out = ctrl.Compute(good);
  ASSERT_EQ(out.devices[0].num_channels, kArmDof);
  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[0].commands[idx], good.devices[0].positions[idx]);
  }
}

TEST(DemoCatchingController, SilencedTickReportsThePARKEDPositionNotZeros) {
  // The log lane is bounded by the DEVICE's channel count, so an untouched
  // reference row would be stored as zeros and read as an origin command.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);

  ControllerState blind = MakeState(0.0);
  blind.devices[0].valid = false;
  blind.devices[1].valid = false;
  const ControllerOutput out = ctrl.Compute(blind);

  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[0].trajectory_positions[idx], blind.devices[0].positions[idx]);
  }
}

TEST(DemoCatchingController, AWideDeviceIsNotAFault) {
  // More channels than the model covers is a legal device, not a gate closure.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ControllerState wide = MakeState(0.0, kHandDof + 2);
  wide.devices[1].positions[kHandDof] = 0.9;
  wide.devices[1].positions[kHandDof + 1] = 0.9;

  const ControllerOutput out = ctrl.Compute(wide);
  EXPECT_EQ(out.devices[1].num_channels, kHandDof + 2);
}

TEST(DemoCatchingController, AWideDeviceWithHolesIsLatchedOnlyUpToTheFirstHole) {
  // `readable` vouches for [0, dof) only. The channels past it can be wire
  // slots no state message ever wrote, which read 0.0 out of the persistent
  // cache — latching those would freeze "go to the origin" on them for the
  // whole activation, and this latch never re-seeds.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ControllerState holed = MakeState(0.0, kHandDof + 2);
  holed.devices[1].positions[kHandDof] = 0.0;      // never written
  holed.devices[1].positions[kHandDof + 1] = 0.0;  // never written
  holed.devices[1].hole_mask =
      (1ULL << static_cast<unsigned>(kHandDof)) | (1ULL << static_cast<unsigned>(kHandDof + 1));

  const ControllerOutput out = ctrl.Compute(holed);
  EXPECT_EQ(out.devices[1].num_channels, kHandDof)
      << "a never-written slot was latched and is now commanded every tick";
}

// ── 2. The hand step reaches the wire unshaped ───────────────────────────────

TEST(DemoCatchingController, AcceptedStepIsCommandedBitExactFromTheAcceptingTick) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  static_cast<void>(ctrl.Compute(MakeState(0.0)));  // latch the holds

  const std::array<double, kHandDof> target{0.123456789, -0.25, 0.5, 0.75};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, std::span<const double>(target));

  // Every tick from the first drain onward: exactly the target, and nothing
  // between it and the previous hold.
  for (int tick = 0; tick < 5; ++tick) {
    const ControllerOutput out = ctrl.Compute(MakeState(0.0));
    ASSERT_EQ(out.devices[1].num_channels, kHandDof) << "tick " << tick;
    for (int i = 0; i < kHandDof; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      EXPECT_EQ(out.devices[1].commands[idx], target[idx])
          << "hand joint " << i << " was shaped on tick " << tick;
    }
  }
  EXPECT_EQ(ctrl.GetHandStepAppliedCount(), 1U);
}

TEST(DemoCatchingController, OutOfRangeStepIsClampedOnTheWireAndRawInTheGoalLane) {
  // The CM neither clamps nor filters, so the position clamp is this
  // controller's job. The goal lane keeps the raw ask, so a clamp shows up in
  // the CSV as a divergence rather than silently rewriting the request.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  static_cast<void>(ctrl.Compute(MakeState(0.0)));

  const std::array<double, kHandDof> target{5.0, -5.0, 0.5, -0.5};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, std::span<const double>(target));
  const ControllerOutput out = ctrl.Compute(MakeState(0.0));

  EXPECT_EQ(out.devices[1].commands[0], kHandUpper);
  EXPECT_EQ(out.devices[1].commands[1], kHandLower);
  EXPECT_EQ(out.devices[1].commands[2], 0.5);
  EXPECT_EQ(out.devices[1].commands[3], -0.5);

  EXPECT_EQ(out.devices[1].goal_positions[0], 5.0);
  EXPECT_EQ(out.devices[1].goal_positions[1], -5.0);
}

TEST(DemoCatchingController, HandHoldsItsActivationPoseUntilAStepArrives) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  const ControllerState seed = MakeState(0.0);
  static_cast<void>(ctrl.Compute(seed));

  const ControllerOutput out = ctrl.Compute(MakeState(0.3));
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[1].commands[idx], seed.devices[1].positions[idx]);
  }
}

// ── 3. The refusals refuse ──────────────────────────────────────────────────

TEST(DemoCatchingController, ATaskGoalOnTheHandIsRefusedRatherThanSteppedAsJoints) {
  // The base's default SetDeviceTaskTarget forwards a task goal straight to
  // SetDeviceTarget, so without an override the six Cartesian numbers would be
  // written into the first six HAND JOINTS as an unshaped step — clamped to the
  // joint limits, and with the base's limit warning deliberately skipped
  // because "joint limits do not apply to a Cartesian goal".
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  const ControllerState seed = MakeState(0.0);
  static_cast<void>(ctrl.Compute(seed));

  rtc_msgs::msg::RobotTarget msg;
  msg.goal_type = "task";
  msg.task_target = {0.4, -0.2, 0.6, 0.0, 1.57, 0.0};
  ctrl.DeliverTargetMessage("hand", kCatchingHandDeviceIdx, msg);
  const ControllerOutput out = ctrl.Compute(MakeState(0.0));

  EXPECT_EQ(ctrl.GetTaskTargetRejectCount(), 1U);
  EXPECT_EQ(ctrl.GetHandStepAppliedCount(), 0U);
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[1].commands[idx], seed.devices[1].positions[idx])
        << "a Cartesian pose value reached hand joint " << i;
  }
}

TEST(DemoCatchingController, AShortHandGoalIsRefusedRatherThanHalfApplied) {
  // This controller has no persistent target row: the overlay covers
  // [0, width) and the REST comes from the activation latch. So a short goal
  // would not leave the untouched joints where the previous step put them — it
  // would snap them back to the activation pose, unshaped.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  static_cast<void>(ctrl.Compute(MakeState(0.0)));

  const std::array<double, kHandDof> full{0.11, 0.22, 0.33, 0.44};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, std::span<const double>(full));
  static_cast<void>(ctrl.Compute(MakeState(0.0)));

  const std::array<double, 2> partial{0.9, 0.9};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, std::span<const double>(partial));
  const ControllerOutput out = ctrl.Compute(MakeState(0.0));

  EXPECT_EQ(ctrl.GetHandTargetWidthRejectCount(), 1U);
  EXPECT_EQ(ctrl.GetHandStepAppliedCount(), 1U) << "the short goal was applied";
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[1].commands[idx], full[idx])
        << "hand joint " << i << " left the previous step";
  }
}

TEST(DemoCatchingController, HandStepIsRefusedWhenTheDiagnosticFlagIsOff) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl, /*hand_step=*/false);
  const ControllerState seed = MakeState(0.0);
  static_cast<void>(ctrl.Compute(seed));

  const std::array<double, kHandDof> target{0.4, 0.4, 0.4, 0.4};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, std::span<const double>(target));
  const ControllerOutput out = ctrl.Compute(MakeState(0.0));

  EXPECT_EQ(ctrl.GetHandStepDisabledRejectCount(), 1U);
  EXPECT_EQ(ctrl.GetHandStepAppliedCount(), 0U);
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[1].commands[idx], seed.devices[1].positions[idx])
        << "a refused step still moved hand joint " << i;
  }
}

TEST(DemoCatchingController, ArmTargetsAreAlwaysRefusedAndCounted) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  const ControllerState seed = MakeState(0.0);
  static_cast<void>(ctrl.Compute(seed));

  const std::array<double, kArmDof> target{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(target));
  const ControllerOutput out = ctrl.Compute(MakeState(0.0));

  EXPECT_EQ(ctrl.GetArmTargetRejectCount(), 1U);
  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[0].commands[idx], seed.devices[0].positions[idx]);
  }
}

// ── 4. A stale target does not survive re-activation ────────────────────────

TEST(DemoCatchingController, TargetQueuedWhileInactiveIsDroppedOnReactivation) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  const rclcpp_lifecycle::State unconfigured;
  ASSERT_EQ(ctrl.on_activate(unconfigured), DemoCatchingController::CallbackReturn::SUCCESS);
  const ControllerState seed = MakeState(0.0);
  static_cast<void>(ctrl.Compute(seed));

  ASSERT_EQ(ctrl.on_deactivate(unconfigured), DemoCatchingController::CallbackReturn::SUCCESS);
  const std::array<double, kHandDof> stale{0.9, 0.9, 0.9, 0.9};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, std::span<const double>(stale));
  ASSERT_EQ(ctrl.on_activate(unconfigured), DemoCatchingController::CallbackReturn::SUCCESS);

  const ControllerState reseed = MakeState(0.2);
  const ControllerOutput out = ctrl.Compute(reseed);
  EXPECT_EQ(ctrl.GetHandStepAppliedCount(), 0U) << "a stale goal reached the RT slot";
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[1].commands[idx], reseed.devices[1].positions[idx]);
  }
}

TEST(DemoCatchingController, ActivationDelegatesToTheBase) {
  DemoCatchingController ctrl{""};
  const rclcpp_lifecycle::State unconfigured;
  ASSERT_EQ(ctrl.on_activate(unconfigured), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.ActivationGeneration(), 1U);
  ASSERT_EQ(ctrl.on_deactivate(unconfigured), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_activate(unconfigured), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.ActivationGeneration(), 2U);
}

// ── 5. configure-time refusals (needs a LifecycleNode) ──────────────────────

class CatchingConfigureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_cfg_" +
                                                              std::to_string(++counter_));
  }

  void TearDown() override { node_.reset(); }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  static inline int counter_ = 0;
};

TEST_F(CatchingConfigureTest, ConfiguresOnAnAllMujocoProfile) {
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl.GetNonSimGroups().empty());
  EXPECT_TRUE(ctrl.IsHandStepEnabled());
  EXPECT_EQ(ctrl.GetHandDof(), kHandDof);
}

// ── Parking a real-arm configuration (A-S5-1) ───────────────────────────────
//
// WHAT CHANGED AT S5.1, and why these tests keep their assertions. Through
// S4.0 the rule was "refuse to activate unless every claimed device has proven
// it is the simulator", standing in for the then-pending E-8 decision about
// commanding a real arm at all. E-8 was approved on 2026-09-22, so the
// stand-in is gone and what remains is the rule it stood in for: a provisional
// or still-TBD value blocks a REAL-ARM configuration and only warns in sim
// (L0 §5.3, L8 §5.4).
//
// The profile these tests load is provisional, so the OUTCOME for a real
// backend is the same as before — parked at configure, activation refused —
// and the assertions are unchanged. What the reason change costs is a
// distinction these tests could not previously express, so a new one is added
// below for it: a real-arm configuration whose profile is CLEARED now
// configures and activates, which is the entire content of the E-8 approval.
// Deleting that case would leave the suite passing on a controller that still
// refuses hardware outright.
//
// Activation is still where the refusal bites: nothing is commanded until a
// controller is active, and refusing the CONFIGURE would take the whole robot
// down with it (sim and real share `config/<variant>/controllers/`, and CM
// latches `bring_up_failed` on any controller's configure failure).

TEST_F(CatchingConfigureTest, ConfiguresDisabledOnARealDriverBackend) {
  // The profile is provisional (`robot.hand.provisional: true`), which on the
  // real-arm axis is a failure on a key this controller consumes.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("ur_driver_native", "udp_hand_native"));
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << "a configure failure here refuses EVERY controller on the real robot";
  EXPECT_TRUE(ctrl.IsSimOnlyDisabled());
  EXPECT_EQ(ctrl.GetNonSimGroups().size(), 2U);
}

TEST_F(CatchingConfigureTest, RefusesToActivateOnARealDriverBackend) {
  // Where the refusal bites: nothing is commanded until a controller is
  // active, so a parked controller cannot reach the arm. The generation
  // assertion is what says the base activation did not run anyway — a refusal
  // that still bumped it would leave the RT tick believing an activation
  // boundary was crossed.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("ur_driver_native", "udp_hand_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
  EXPECT_EQ(ctrl.ActivationGeneration(), 0U) << "the base activation ran anyway";
}

TEST_F(CatchingConfigureTest, ConfiguresDisabledOnADeviceThatDeclaresNoBackendAtAll) {
  // Silence does not prove the device is the simulator, so the strict axis
  // applies and the provisional profile parks the instance.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", ""));
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl.IsSimOnlyDisabled());
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
}

TEST_F(CatchingConfigureTest, ADisabledInstanceExposesNoProfileParameters) {
  // A disabled instance stops before DeclareProfileParameters, so the real
  // robot does not even advertise the diagnostic step lane's profile.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("ur_driver_native", "udp_hand_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_FALSE(node_->has_parameter("hand.q_close"));
  EXPECT_FALSE(node_->has_parameter("diagnostic.hand_step"));
}

TEST_F(CatchingConfigureTest, ConfigureIsReentrantOnTheSameNode) {
  // The profile parameters are read_only, so on_cleanup cannot undeclare them.
  // Without a has_parameter guard the second declare throws and turns a legal
  // re-configure into a configure failure.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ctrl.IsSimOnlyDisabled());
}

TEST_F(CatchingConfigureTest, ADisabledInstanceRecoversWhenTheBackendsBecomeSim) {
  // The verdict is re-decided per configure — a disabled instance must not
  // stay disabled once it is given simulator backends.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("ur_driver_native", "udp_hand_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_TRUE(ctrl.IsSimOnlyDisabled());
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);

  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ctrl.IsSimOnlyDisabled());
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

// The case the previous rule could not express: E-8 is approved, so a real
// arm whose profile is CLEARED configures and activates. Without this test the
// suite would still pass on a controller that refuses hardware outright, which
// is what S4.0 did and what S5.1 exists to stop doing.
TEST_F(CatchingConfigureTest, RealArmWithAClearedProfileConfiguresAndActivates) {
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("ur_driver_native", "udp_hand_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(ClearedYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl.IsRealArmConfig()) << "precondition: judged on the strict axis";
  EXPECT_FALSE(ctrl.IsSimOnlyDisabled());
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

// ── The arm latch (A-S5-3) and P-1 (c) ──────────────────────────────────────

TEST_F(CatchingConfigureTest, ActivationDoesNotArmTheController) {
  // An activation is not an arming. A controller that armed itself here would
  // resume catching after any deactivate/activate cycle — including the one an
  // E-STOP recovery goes through, which is precisely the resume P-1 (c) bans.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(ClearedYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);

  EXPECT_FALSE(ctrl.IsArmRequested());
  ctrl.Compute(MakeState(0.0));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);
}

TEST_F(CatchingConfigureTest, TheEnableParameterArmsTheSupervisor) {
  // The operator channel end to end: the parameter is what moves the latch,
  // and the TICK is what acts on it. Asserting the mode rather than the latch
  // is deliberate — a callback that stored the flag somewhere the tick never
  // reads would pass an IsArmRequested()-only check.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(ClearedYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_TRUE(node_->has_parameter(integrated_bringup::kCatchingEnableParam));

  ctrl.Compute(MakeState(0.0));
  ASSERT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);

  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  EXPECT_TRUE(ctrl.IsArmRequested());
  ctrl.Compute(MakeState(0.01));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kArmed);
  EXPECT_EQ(ctrl.GetLastReason(), rtc::catching::Reason::kNone);

  // Disarming sends it back to IDLE through the documented reuse of
  // kParamsTbd for "an ARMED precondition stopped holding" (L7 §4.5).
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, false));
  ctrl.Compute(MakeState(0.02));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);
  EXPECT_EQ(ctrl.GetLastReason(), rtc::catching::Reason::kParamsTbd);
}

TEST_F(CatchingConfigureTest, EstopDisarmsAndClearingItDoesNotResume) {
  // G7-H (c). The point is what does NOT happen after the clear: the latch
  // stays down and the supervisor stays in IDLE for as long as anyone cares to
  // tick it. A controller that resumed on the clear would show kArmed here,
  // and on a real robot that is a catching attempt nobody asked for.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(ClearedYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  ctrl.Compute(MakeState(0.0));
  ASSERT_EQ(ctrl.GetMode(), rtc::catching::Mode::kArmed) << "precondition: it was armed";

  ctrl.TriggerEstop();
  ctrl.Compute(MakeState(0.05));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);
  EXPECT_EQ(ctrl.GetLastReason(), rtc::catching::Reason::kEstop);
  EXPECT_FALSE(ctrl.IsArmRequested()) << "the stop did not disarm";

  ctrl.ClearEstop();
  for (int i = 0; i < 20; ++i) {
    ctrl.Compute(MakeState(0.05 + 0.01 * static_cast<double>(i)));
    ASSERT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle) << "resumed on its own at tick " << i;
  }
  EXPECT_FALSE(ctrl.IsArmRequested());

  // Re-arming is a deliberate act, and it works.
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  ctrl.Compute(MakeState(0.3));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kArmed);
}

TEST_F(CatchingConfigureTest, RefusesAProfileWhoseHandWidthDisagreesWithTheDevice) {
  DemoCatchingController ctrl{""};
  auto configs = MakeConfigs("mujoco_native", "mujoco_native");
  configs["hand"].joint_state_names = {"h0", "h1", "h2"};  // 3, profile declares 4
  ctrl.SetDeviceNameConfigs(configs);
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::FAILURE);
}

TEST_F(CatchingConfigureTest, RefusesAnAbsentCatchingSection) {
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  YAML::Node yaml = YAML::Load(MinimalYaml(true));
  yaml.remove("catching");
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, yaml), DemoCatchingController::CallbackReturn::FAILURE);
}

TEST_F(CatchingConfigureTest, RefusesACagingJointThatDoesNotTravel) {
  // L6 §4.2: rho divides by |q_close - q_pre| on the caging set.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  YAML::Node yaml = YAML::Load(MinimalYaml(true));
  yaml["catching"]["robot"]["hand"]["q_close"][0] = 0.1;  // == q_pre[0], and masked
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, yaml), DemoCatchingController::CallbackReturn::FAILURE);
}

TEST_F(CatchingConfigureTest, StillConfiguresWhileTCloseE2eIsTbd) {
  // T_close_e2e is the number S4.2 produces WITH this controller; gating on it
  // would make the measurement its own precondition. The minimal YAML above
  // ships it as TBD, so every passing case here already covers this — assert it
  // by name so a future widening of the gate is a red test and not a silent
  // deadlock.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl.GetCatchingParams().hand.T_close_e2e.tbd);
  EXPECT_FALSE(ctrl.GetValidationReport().armable)
      << "the report itself must still say 'not armable' — only the GATE is narrower";
}

// A successful configure must leave a real `joint_goal` ENDPOINT behind, not
// just a parsed `topics:` entry. Those are different things: the YAML parses
// and the group lands in topic_config_ whether or not CreateOwnedTopics ever
// runs, and a controller that skips it drops every step on the floor with no
// error on any lane — the step simply never arrives. Asserting through
// DeliverTargetMessage cannot see this, because that call bypasses the
// subscription the operator and S4.2's runner actually publish to.
TEST_F(CatchingConfigureTest, ConfigureCreatesTheDeclaredJointGoalEndpoints) {
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::SUCCESS);

  // Graph discovery is not instantaneous even for a node's own endpoints, so
  // poll rather than sample once.
  const auto wait_for_subscriber = [this](const std::string& topic) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (node_->count_subscribers(topic) > 0) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  };

  EXPECT_TRUE(wait_for_subscriber("/hand/joint_goal"))
      << "the hand step lane has no endpoint — S4.2 would publish into nothing";
  EXPECT_TRUE(wait_for_subscriber("/arm/joint_goal"))
      << "the arm lane has no endpoint, so its targets are never counted as rejected";
}

// ── 6. The shipped profiles (S4.1) ──────────────────────────────────────────

class ShippedCatchingProfile : public ::testing::TestWithParam<std::pair<std::string, int>> {};

TEST_P(ShippedCatchingProfile, ParsesWithinLimitsAndMatchesTheHandWidth) {
  const auto& [profile, expected_dof] = GetParam();
  const YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  ASSERT_TRUE(node["catching"]) << profile << ": no catching section";

  const auto params = rtc::catching::ParseCatchingParams(node["catching"]);
  const auto& hand = params.hand;
  EXPECT_EQ(hand.dof, expected_dof);
  EXPECT_FALSE(hand.tbd);
  EXPECT_FALSE(hand.q_open_tbd);
  EXPECT_FALSE(hand.eta_close.tbd);
  EXPECT_TRUE(hand.provisional) << "S4.1 profiles are provisional until S4.2 confirms them";

  // Every pose inside the shipped device limits. The limits live in the
  // profile's device YAML, which is a different file — read it rather than
  // restating the numbers, or this check pins a copy instead of the config.
  const std::string base = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/";
  YAML::Node devices;
  // Profiles differ in which file carries `devices:` (ur5e_p1b splits a
  // `_base.yaml` out, iiwa7_leap keeps it in `sim.yaml`), and a profile that
  // has neither must fail loudly rather than skip the limit check.
  for (const char* candidate : {"_base.yaml", "sim.yaml"}) {
    if (!std::filesystem::exists(base + candidate)) {
      continue;
    }
    const YAML::Node root = YAML::LoadFile(base + candidate);
    const YAML::Node d = root["/**"]["ros__parameters"]["devices"];
    if (d && d.IsMap()) {
      devices = d;
      break;
    }
  }
  ASSERT_TRUE(devices) << profile << ": no devices block found";

  // The hand group is the SECOND entry of the controller's own topics list —
  // taken from there rather than hard-coded, so this stays robot-agnostic.
  ASSERT_TRUE(node["topics"] && node["topics"].size() >= 2);
  auto topic_it = node["topics"].begin();
  ++topic_it;
  const auto hand_group = topic_it->first.as<std::string>();

  const YAML::Node hand_dev = devices[hand_group];
  ASSERT_TRUE(hand_dev) << profile << ": no device '" << hand_group << "'";
  EXPECT_EQ(static_cast<int>(hand_dev["joint_state_names"].size()), expected_dof)
      << profile << ": profile width must equal the hand device's channel count (L6 §5.1)";

  const YAML::Node limits = hand_dev["joint_limits"];
  ASSERT_TRUE(limits) << profile << ": hand device declares no joint_limits";
  const auto lower = limits["position_lower"].as<std::vector<double>>();
  const auto upper = limits["position_upper"].as<std::vector<double>>();
  ASSERT_EQ(static_cast<int>(lower.size()), expected_dof);

  for (int i = 0; i < expected_dof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    for (const auto& [name, pose] :
         {std::pair{"q_open", &hand.q_open}, std::pair{"q_pre", &hand.q_pre},
          std::pair{"q_close", &hand.q_close}}) {
      EXPECT_GE((*pose)[idx], lower[idx]) << profile << " " << name << "[" << i << "]";
      EXPECT_LE((*pose)[idx], upper[idx]) << profile << " " << name << "[" << i << "]";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(BothCatchingRobots, ShippedCatchingProfile,
                         ::testing::Values(std::pair<std::string, int>{"ur5e_p1b", 10},
                                           std::pair<std::string, int>{"iiwa7_leap", 16}),
                         [](const auto& info) { return info.param.first; });

// ── 5. S5.1: the P-1 minimum E-STOP/fault contract (E-8) ────────────────────
//
// The four parts are tested separately because they fail separately: (a) is
// about WHO writes, (b) about WHEN, (c) about what a clear does NOT do, and
// (d) about two latches staying independent. A single "estop works" test would
// pass with three of them broken.

TEST(DemoCatchingEstop, HooksOnlyRequestAndTheTickIsTheWriter) {
  // P-1 (a). The hooks are called here the way CM calls them — off the tick,
  // between ticks — and then NOTHING is allowed to have changed until a tick
  // runs. A hook that reset state directly would move the counter and the mode
  // on these lines, before any Compute().
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.Compute(MakeState(0.0));
  const std::uint64_t resets_before = ctrl.GetRtResetCount();

  ctrl.TriggerEstop();
  ctrl.ClearEstop();
  ctrl.ResetFault();
  EXPECT_EQ(ctrl.GetRtResetCount(), resets_before)
      << "a hook performed a reset instead of requesting one";

  ctrl.Compute(MakeState(0.1));
  EXPECT_GT(ctrl.GetRtResetCount(), resets_before) << "the tick never serviced the request";
}

TEST(DemoCatchingEstop, IsEstoppedTracksTheRequestWithoutATick) {
  // The CM polls this from off the RT thread and must not need a tick to get a
  // true answer — it is the REQUEST's observable, not the tick's.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  EXPECT_FALSE(ctrl.IsEstopped());
  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());
  ctrl.ClearEstop();
  EXPECT_FALSE(ctrl.IsEstopped());
}

TEST(DemoCatchingEstop, ReactivationCommandsTheNewPoseNotTheOldOne) {
  // G7-H (a). The controller is activated, holds a pose, is deactivated, the
  // arm is then moved by something else, and is reactivated. The first tick of
  // the new activation must command where the arm IS, not where it was.
  //
  // This is the failure the activation-generation reset exists for: the hold
  // latch survives the object, so without a reset keyed to the activation
  // boundary the first command of the second activation is the first
  // activation's pose — a step of whatever distance the other controller moved.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);

  const ControllerState first_pose = MakeState(0.0);
  const ControllerOutput first = ctrl.Compute(first_pose);
  ASSERT_EQ(first.devices[0].num_channels, kArmDof);

  // Deactivate, someone else moves the arm, reactivate. on_activate is the
  // real path: the base bumps the activation generation and calls
  // ResetTargetInitialization, and the generation bump is what the tick reads.
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);

  const ControllerState moved = MakeState(0.75);
  const ControllerOutput after = ctrl.Compute(moved);
  ASSERT_EQ(after.devices[0].num_channels, kArmDof);
  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(after.devices[0].commands[idx], moved.devices[0].positions[idx])
        << "arm joint " << i << " was commanded to the PREVIOUS activation's pose";
  }
}

TEST(DemoCatchingEstop, EstopCycleReseedsTheHoldFromTheMeasuredPose) {
  // P-1 (b) on the arm command lane. An E-STOP is not an activation, so the
  // generation does not move — the epoch is what carries the reset, and the
  // command after the cycle must come from the pose the arm is actually in.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.Compute(MakeState(0.0));

  ctrl.TriggerEstop();
  ctrl.Compute(MakeState(0.2));  // the tick that services the trigger
  ctrl.ClearEstop();

  const ControllerState recovered = MakeState(0.9);
  const ControllerOutput out = ctrl.Compute(recovered);
  ASSERT_EQ(out.devices[0].num_channels, kArmDof);
  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[0].commands[idx], recovered.devices[0].positions[idx])
        << "arm joint " << i << " carried a pre-E-STOP command across the stop";
  }
}

TEST(DemoCatchingEstop, ATriggerAndClearBetweenTwoTicksStillReseeds) {
  // The reason the E-STOP request is an EPOCH and not a flag. A stop that is
  // raised and cleared inside one tick period leaves the flag back at false —
  // a tick that only looked at the flag would see nothing happened and carry
  // q_c straight across a stop.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.Compute(MakeState(0.0));
  const std::uint64_t resets_before = ctrl.GetRtResetCount();

  ctrl.TriggerEstop();
  ctrl.ClearEstop();
  EXPECT_FALSE(ctrl.IsEstopped()) << "precondition: the flag is back to false";

  const ControllerState after_pose = MakeState(0.4);
  const ControllerOutput out = ctrl.Compute(after_pose);
  EXPECT_GT(ctrl.GetRtResetCount(), resets_before);
  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[0].commands[idx], after_pose.devices[0].positions[idx]);
  }
}

TEST(DemoCatchingEstop, AQueuedHandStepDoesNotSurviveAnEstop) {
  // P-1 (b) on the target lane. The base's generation gate drops goals queued
  // across an ACTIVATION, but an E-STOP does not bump that generation — so
  // without DiscardPendingTargets a step issued just before the stop lands on
  // the hand just after it.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.Compute(MakeState(0.0));

  const std::array<double, kHandDof> step{0.3, 0.3, 0.3, 0.3};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, step);
  ctrl.TriggerEstop();

  const ControllerState state = MakeState(0.0);
  const ControllerOutput out = ctrl.Compute(state);
  ASSERT_EQ(out.devices[1].num_channels, kHandDof);
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[1].commands[idx], state.devices[1].positions[idx])
        << "hand joint " << i << " took a step queued before the E-STOP";
  }
}

TEST(DemoCatchingEstop, ResetFaultDoesNotClearTheEstopAndNoFaultIsLatchedByDefault) {
  // P-1 (d), the half that is reachable at S5.1. The two paths are separate:
  // a fault reset must not lower the global stop. The other half — a LATCHED
  // fault surviving ClearEstop — needs a fault source, and the only one in the
  // design is the QP failure streak that arrives with S5.3; this test would be
  // asserting on a latch nothing can raise.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  EXPECT_FALSE(ctrl.HasLatchedFault());

  ctrl.TriggerEstop();
  ctrl.ResetFault();
  ctrl.Compute(MakeState(0.0));
  EXPECT_TRUE(ctrl.IsEstopped()) << "a fault reset lowered the global E-STOP";
  EXPECT_FALSE(ctrl.HasLatchedFault());
}

// ── 6. S5.1: the supervisor mode and the arm latch ──────────────────────────

TEST(DemoCatchingEstop, ConcurrentHooksNeverResetOutsideATick) {
  // G7-H (b). Hooks are hammered from other threads while ticks run, which is
  // what CM actually does: the E-STOP propagation and the reset service run on
  // executor threads with no relationship to the RT loop.
  //
  // The invariant that makes this test mean something is RESETS <= TICKS. The
  // tick performs at most one reset per tick by construction, so a count that
  // outran the ticks could only have come from a hook doing the work itself —
  // which is the P-1 (a) violation, and the one that no single-threaded test
  // can see. (Run under TSAN as well: this asserts the COUNT, and TSAN is what
  // judges the accesses.)
  DemoCatchingController ctrl{""};
  BringUp(ctrl);

  std::atomic<bool> stop{false};
  std::thread trigger([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      ctrl.TriggerEstop();
      ctrl.ClearEstop();
    }
  });
  std::thread faults([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      ctrl.ResetFault();
      static_cast<void>(ctrl.HasLatchedFault());
      static_cast<void>(ctrl.IsEstopped());
    }
  });

  constexpr int kTicks = 2000;
  for (int i = 0; i < kTicks; ++i) {
    static_cast<void>(ctrl.Compute(MakeState(0.001 * static_cast<double>(i))));
  }
  stop.store(true, std::memory_order_relaxed);
  trigger.join();
  faults.join();

  EXPECT_LE(ctrl.GetRtResetCount(), static_cast<std::uint64_t>(kTicks))
      << "more resets than ticks — something other than the tick performed one";

  // And the controller is still coherent afterwards: one more tick with the
  // stop cleared commands the pose the arm is in.
  ctrl.ClearEstop();
  const ControllerState settled = MakeState(0.5);
  const ControllerOutput out = ctrl.Compute(settled);
  ASSERT_EQ(out.devices[0].num_channels, kArmDof);
  for (int i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[0].commands[idx], settled.devices[0].positions[idx]);
  }
}

TEST(DemoCatchingSupervisor, StaysIdleWhileTheProfileIsNotArmable) {
  // G0-C on the consumed subset: BringUp never runs on_configure, so nothing
  // has cleared the profile and the supervisor must not advance out of IDLE
  // however many ticks it gets.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  for (int i = 0; i < 10; ++i) {
    ctrl.Compute(MakeState(0.01 * static_cast<double>(i)));
  }
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);
  EXPECT_EQ(ctrl.GetLastReason(), rtc::catching::Reason::kParamsTbd);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
