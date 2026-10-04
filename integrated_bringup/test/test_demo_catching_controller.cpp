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
// Deliberately NOT in the four shared per-controller suites (docs/testing.md
// §Sensor Matrix, the catching-binding row): device-readability and
// gate-closure assertions live HERE instead of in
// test_device_readability_gate because half of that suite's contract is the
// E-STOP safe-position ramp, which this controller does not have (it overrides
// no E-STOP hook — that IS the E-8 decision). The momentum-observer embedding
// suite does not apply either: this skeleton runs no arm dynamics and
// registers no observer. Both are revisited in S5.1, when the controller grows
// the law those suites are about.

#include "catching_cloud_fixture.hpp"
#include "csv_log_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/catching_params.hpp"
#include "rtc_controllers/catching/decel_mpc.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/unit_speed.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "shipped_config_test_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::kCatchingHandDeviceIdx;
using rtc::ControllerOutput;
using rtc::ControllerState;

constexpr int kArmDof = 6;
/// The shipped `control_rate` both catching profiles run at (`_base.yaml` /
/// `sim.yaml`). Stated rather than read because the validator needs a rate
/// before the config is parsed; a profile that changed it would show up as a
/// discretisation warning in the shipped-profile test below.
constexpr double kShippedControlRateHz = 500.0;
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
  io:
    expected_frame: "world"
    n_min: 7
    t_stale: 0.2
    future_tol: 0.01
    horizon_min: 0.3
    track:
      eval_offset: 0.05
  prediction:
    dt_expected: 0.05
  sim:
    io:
      future_tol: 0.2
  reference:
    provisional: true
    omega: 10.0
    zeta: 1.0
    v_max: 2.0
    a_max: 15.0
  joint_cmd:
    K_p: 20.0
    K_a: 8.0
    K_n: 1.0
    w_task: 1.0
    w_a: 0.5
    w_arm: 0.01
    w_smooth: 0.001
    damping_sq: 0.0001
    qp:
      max_iter: 20
    lag:
      provisional: false
      T_arm: 0.0
  supervisor:
    track_err_abort: 0.3
    n_qp: 3
    deadline:
      provisional: false
  robot:
    arm:
      limit_margin: 0.05
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
  // Replaces EVERY occurrence: the profile carries more than one provisional
  // flag (the hand profile and the L4 reference block), and clearing only the
  // first leaves a "cleared" profile that a real arm still refuses — which
  // reads as the controller being broken rather than as the fixture being
  // half-written.
  const auto replace = [&yaml](std::string_view from, std::string_view to) {
    std::size_t at = 0;
    while ((at = yaml.find(from, at)) != std::string::npos) {
      yaml.replace(at, from.size(), to);
      at += to.size();
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
  (void)ctrl.Compute(MakeState(0.0));
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

  (void)ctrl.Compute(MakeState(0.0));
  ASSERT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);

  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  EXPECT_TRUE(ctrl.IsArmRequested());
  (void)ctrl.Compute(MakeState(0.01));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kArmed);
  EXPECT_EQ(ctrl.GetLastReason(), rtc::catching::Reason::kNone);

  // Disarming sends it back to IDLE through the documented reuse of
  // kParamsTbd for "an ARMED precondition stopped holding" (L7 §4.5).
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, false));
  (void)ctrl.Compute(MakeState(0.02));
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
  (void)ctrl.Compute(MakeState(0.0));
  ASSERT_EQ(ctrl.GetMode(), rtc::catching::Mode::kArmed) << "precondition: it was armed";

  ctrl.TriggerEstop();
  (void)ctrl.Compute(MakeState(0.05));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);
  EXPECT_EQ(ctrl.GetLastReason(), rtc::catching::Reason::kEstop);
  EXPECT_FALSE(ctrl.IsArmRequested()) << "the stop did not disarm";

  ctrl.ClearEstop();
  for (int i = 0; i < 20; ++i) {
    (void)ctrl.Compute(MakeState(0.05 + 0.01 * static_cast<double>(i)));
    ASSERT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle) << "resumed on its own at tick " << i;
  }
  EXPECT_FALSE(ctrl.IsArmRequested());

  // Re-arming is a deliberate act, and it works.
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  (void)ctrl.Compute(MakeState(0.3));
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kArmed);
}

// ── The vision lane, end to end (S5.2: G1-B, G1-I, G1-J) ───────────────────
//
// These go through a REAL subscription rather than calling the decoder: what
// they are about is the path — a message published on the configured topic,
// decoded on the non-RT callback group, handed across a SeqLock, and judged by
// the tick — and every one of those steps has its own way of being wired
// wrong while the decoder stays correct.

class CatchingVisionTest : public CatchingConfigureTest {
 protected:
  /// The profile plus a vision lane pointed at this test's own topic.
  ///
  /// The topic is set on the PARSED node rather than spliced into the text:
  /// the profile already carries the rest of the `io:` block, and appending a
  /// second one would leave the document with two keys of the same name —
  /// which parses, silently keeps one, and makes the test depend on which.
  static YAML::Node VisionYaml(const std::string& topic) {
    YAML::Node yaml = YAML::Load(ClearedYaml(/*hand_step=*/true));
    yaml["catching"]["io"]["traj_topic"] = topic;
    return yaml;
  }

  void BringUpWithVision() {
    topic_ = "/test_catching_vision/prediction";
    ctrl_.SetDeviceNameConfigs(MakeConfigs("mujoco_native", "mujoco_native"));
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl_.on_configure(prev, node_, VisionYaml(topic_)),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_EQ(ctrl_.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    rclcpp::QoS qos{rclcpp::KeepLast(1)};
    qos.best_effort();
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);
  }

  void TearDown() override {
    pub_.reset();
    if (executor_) {
      executor_->remove_node(node_->get_node_base_interface());
      executor_.reset();
    }
    CatchingConfigureTest::TearDown();
  }

  /// Publish and let the callback run. The message's stamp is built from the
  /// CURRENT wall clock so the D-2 conversion sees a plausible origin delay —
  /// a fixed stamp would be hundreds of seconds in the past by the time the
  /// test runs and would be refused as malformed rather than decoded.
  void PublishPrediction(std::uint64_t sequence, std::uint64_t generation = 42,
                         std::int64_t origin_delay_ns = 5'000'000) {
    integrated_bringup::testing::CloudSpec spec;
    spec.n = 8;
    spec.sequence = sequence;
    spec.generation = generation;
    spec.origin_delay_ns = origin_delay_ns;
    auto msg = integrated_bringup::testing::MakeCloud(spec);
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const std::int64_t wall =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - origin_delay_ns;
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
    pub_->publish(msg);
  }

  void Spin() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(20));
      if (ctrl_.GetTrajInput().AcceptCount() > accepted_before_) {
        break;
      }
    }
    accepted_before_ = ctrl_.GetTrajInput().AcceptCount();
  }

  DemoCatchingController ctrl_{""};
  std::string topic_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::uint64_t accepted_before_{0};
};

TEST_F(CatchingVisionTest, APublishedPredictionArmsAndThenTracks) {
  BringUpWithVision();

  // Armed, no ball: ARMED is the waiting state, and it must be reachable
  // without vision — readiness is about the robot.
  (void)ctrl_.Compute(MakeState(0.0));
  ASSERT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kArmed);

  PublishPrediction(/*sequence=*/1);
  Spin();
  ASSERT_EQ(ctrl_.GetTrajInput().AcceptCount(), 1U) << "the message never reached the callback";

  (void)ctrl_.Compute(MakeState(0.01));
  EXPECT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kTracking);
  EXPECT_FALSE(ctrl_.GetTrajView().stale);
  EXPECT_TRUE(ctrl_.GetTrajView().is_new);

  // Same snapshot on the next tick: still usable, no longer new.
  (void)ctrl_.Compute(MakeState(0.02));
  EXPECT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kTracking);
  EXPECT_FALSE(ctrl_.GetTrajView().is_new);
}

TEST_F(CatchingVisionTest, AStaleLaneSendsTrackingBackToArmed) {
  // The lane going quiet is the failure mode `t_stale` exists for, and it is
  // indistinguishable from a lane being refused — which is why the reject
  // counters are asserted to be empty here: this test would otherwise pass on
  // a controller that was rejecting every message for an unrelated reason.
  BringUpWithVision();
  // One tick to leave IDLE: the supervisor takes ONE edge per tick, so
  // reaching TRACKING from IDLE is two ticks whatever vision does.
  (void)ctrl_.Compute(MakeState(0.0));
  ASSERT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kArmed);
  PublishPrediction(/*sequence=*/1, /*generation=*/42, /*origin_delay_ns=*/5'000'000);
  Spin();
  (void)ctrl_.Compute(MakeState(0.005));
  ASSERT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kTracking);
  ASSERT_EQ(ctrl_.GetTrajInput().RejectCount(integrated_bringup::CloudReject::kMalformed), 0U);

  // Nothing new arrives; t_stale is 0.2 s in this profile.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  (void)ctrl_.Compute(MakeState(0.01));
  EXPECT_TRUE(ctrl_.GetTrajView().stale);
  EXPECT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kArmed);
  EXPECT_EQ(ctrl_.GetLastReason(), rtc::catching::Reason::kBallStale);
}

TEST_F(CatchingVisionTest, ABacklogIsCollapsedToTheNewestSnapshot) {
  // G1-I / ARCH-6. The callback is not spun while three predictions are
  // published, so they queue in the subscription — and a depth-1 KEEP_LAST
  // queue keeps the LAST one. A deeper queue would make the controller work
  // through stale predictions after any hiccup, each one arriving as "new".
  BringUpWithVision();
  (void)ctrl_.Compute(MakeState(0.0));  // IDLE → ARMED (one edge per tick)
  PublishPrediction(/*sequence=*/1);
  PublishPrediction(/*sequence=*/2);
  PublishPrediction(/*sequence=*/3);
  Spin();

  EXPECT_EQ(ctrl_.GetTrajInput().AcceptCount(), 1U)
      << "more than one queued prediction was delivered — the depth is not 1";
  // And the one that survived is the NEWEST. A queue that kept the oldest
  // would also deliver exactly one here, so the sequence is what separates
  // "depth 1" from "depth 1, wrong end".
  EXPECT_EQ(ctrl_.GetTrajInput().LastDiagnostics().accepted_sequence, 3U);
  (void)ctrl_.Compute(MakeState(0.005));
  EXPECT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kTracking);
}

TEST_F(CatchingVisionTest, ATrajectoryReceivedWhileInactiveIsNotConsumedOnReactivation) {
  // G1-J / D-23. The subscription outlives deactivation — lifecycle gates
  // publishers, not subscriptions — so the message IS decoded and stored while
  // the controller is inactive. The refusal has to happen at the RT read, on
  // the activation generation the snapshot carries.
  BringUpWithVision();
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);

  PublishPrediction(/*sequence=*/1);
  Spin();
  ASSERT_EQ(ctrl_.GetTrajInput().AcceptCount(), 1U)
      << "precondition: the subscription is still alive while inactive";

  ASSERT_EQ(ctrl_.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  (void)ctrl_.Compute(MakeState(0.0));
  EXPECT_TRUE(ctrl_.GetTrajView().stale)
      << "a trajectory received while inactive was consumed by the new activation";
  EXPECT_EQ(ctrl_.GetMode(), rtc::catching::Mode::kArmed);
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

/// Device configs built FROM the shipped YAML for the two groups the
/// controller's own `topics:` block claims, tagged as the simulator.
///
/// Read rather than restated: a copy of the joint names here would pass while
/// the shipped config said something else, which is the whole failure mode the
/// test below exists to catch.
std::map<std::string, rtc::DeviceNameConfig> ShippedSimConfigs(const std::string& profile,
                                                               const YAML::Node& node) {
  std::map<std::string, rtc::DeviceNameConfig> configs;
  const std::string base = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/";
  YAML::Node devices;
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
  if (!devices) {
    return configs;
  }
  for (auto it = node["topics"].begin(); it != node["topics"].end(); ++it) {
    const auto group = it->first.as<std::string>();
    const YAML::Node dev = devices[group];
    if (!dev || !dev["joint_state_names"]) {
      continue;
    }
    rtc::DeviceNameConfig cfg;
    cfg.device_name = group;
    cfg.joint_state_names = dev["joint_state_names"].as<std::vector<std::string>>();
    if (dev["motor_state_names"]) {
      cfg.motor_state_names = dev["motor_state_names"].as<std::vector<std::string>>();
    }
    if (const YAML::Node limits = dev["joint_limits"]; limits) {
      // Every field the CM carries: the shipped `mpc` law (MD-89) and the
      // dynamic CLIK read the torque and velocity ratings, not only the box.
      const auto per_joint = [&limits](const char* key) {
        return limits[key] ? limits[key].as<std::vector<double>>() : std::vector<double>{};
      };
      rtc::DeviceJointLimits jl;
      jl.max_velocity = per_joint("max_velocity");
      jl.max_torque = per_joint("max_torque");
      jl.position_lower = limits["position_lower"].as<std::vector<double>>();
      jl.position_upper = limits["position_upper"].as<std::vector<double>>();
      cfg.joint_limits = jl;
    }
    rtc::DeviceBackendBinding backend;
    backend.type = integrated_bringup::kCatchingSimBackendType;
    cfg.backend = backend;
    configs[group] = std::move(cfg);
  }
  return configs;
}

/// The profile's system model as the CM builds it: every entry of the shipped
/// `urdf:` block, read from the file. Only the package is swapped, for the
/// vendored `robot_descriptions` copy at the same relative path — the one
/// model CI can acquire (ur5e_p1b_test_fixture.hpp has why). Since MD-89 the
/// shipped DECEL law is `mpc`, which parks without a model, so a bring-up
/// test of the shipped file that left the model out would configure a
/// controller production never builds.
rtc_urdf_bridge::ModelConfig ShippedModelConfig(const std::string& profile) {
  const std::string base = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/";
  YAML::Node urdf;
  for (const char* candidate : {"_base.yaml", "sim.yaml"}) {
    if (!std::filesystem::exists(base + candidate)) {
      continue;
    }
    const YAML::Node u = YAML::LoadFile(base + candidate)["/**"]["ros__parameters"]["urdf"];
    if (u && u.IsMap()) {
      urdf = u;
      break;
    }
  }
  // Throws rather than ASSERT_: the callers would go on with no model.
  if (!urdf) {
    throw std::runtime_error(profile + ": no shipped urdf block");
  }
  const std::string share = ament_index_cpp::get_package_share_directory("robot_descriptions");
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = share + "/" + urdf["path"].as<std::string>();
  cfg.root_joint_type = urdf["root_joint_type"].as<std::string>("fixed");
  if (urdf["extended"].as<bool>(false)) {
    cfg.closure_yaml_path = share + "/" + urdf["closure_path"].as<std::string>();
  }
  // Document order: controllers take the FIRST sub-model as the arm.
  for (auto it = urdf["sub_models"].begin(); it != urdf["sub_models"].end(); ++it) {
    cfg.sub_models.push_back({it->first.as<std::string>(),
                              it->second["root_link"].as<std::string>(),
                              it->second["tip_link"].as<std::string>()});
  }
  for (auto it = urdf["tree_models"].begin(); it != urdf["tree_models"].end(); ++it) {
    cfg.tree_models.push_back({it->first.as<std::string>(),
                               it->second["root_link"].as<std::string>(),
                               it->second["tip_links"].as<std::vector<std::string>>()});
  }
  for (auto it = urdf["extra_frames"].begin(); it != urdf["extra_frames"].end(); ++it) {
    rtc_urdf_bridge::ExtraFrameConfig frame;
    frame.name = it->first.as<std::string>();
    frame.parent = it->second["parent"].as<std::string>();
    const auto xyz = it->second["xyz"].as<std::vector<double>>();
    const auto rpy = it->second["rpy"].as<std::vector<double>>(std::vector<double>(3, 0.0));
    frame.xyz = Eigen::Vector3d(xyz.at(0), xyz.at(1), xyz.at(2));
    frame.rpy = Eigen::Vector3d(rpy.at(0), rpy.at(1), rpy.at(2));
    frame.provisional = it->second["provisional"].as<bool>(true);
    cfg.extra_frames.push_back(frame);
  }
  return cfg;
}

/// One builder per profile per process — the xacro → Pinocchio parse is the
/// expensive part, and production shares one builder the same way.
std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> ShippedModelBuilder(
    const std::string& profile) {
  static std::map<std::string, std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder>> builders;
  auto& builder = builders[profile];
  if (!builder) {
    builder = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(ShippedModelConfig(profile));
  }
  return builder;
}

/// The CM's bring-up order for a controller of the shipped profile: model,
/// builder, rate, devices (iiwa7_leap_test_fixture.hpp). on_configure is the
/// caller's.
void BringUpShipped(DemoCatchingController& ctrl, const std::string& profile,
                    const std::map<std::string, rtc::DeviceNameConfig>& configs) {
  ctrl.SetSystemModelConfig(ShippedModelConfig(profile));
  ctrl.SetSharedModelBuilder(ShippedModelBuilder(profile));
  ctrl.SetControlRate(kShippedControlRateHz);
  ctrl.SetDeviceNameConfigs(configs);
}

// The sensor whose absence let a bring-up-killing profile ship.
//
// Every other case in this suite configures a profile the TEST wrote, and
// those all passed on 2026-09-22 while the shipped `ur5e_p1b` sim profile
// refused to configure at all — S5.3 moved `reference.*` into the consumed
// subset and the shipped file had no `reference:` block, so a consumed TBD
// refused the configure. CM latches `bring_up_failed` on ANY controller's
// configure failure and then refuses to configure EVERY controller, so the
// symptom was not "no catching" but "no robot".
//
// This runs the same on_configure the CM runs, on the file that ships, with
// the devices and the model the file declares, on the SIM axis — which is the
// one that refuses rather than parks.
TEST_P(ShippedCatchingProfile, ConfiguresAndIsArmableOnTheSimAxis) {
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  ASSERT_TRUE(node["catching"]);

  auto configs = ShippedSimConfigs(profile, node);
  ASSERT_EQ(configs.size(), 2U) << profile << ": could not build both device configs";

  auto node_handle =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_shipped_" + profile);
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, configs);
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile
      << ": the shipped profile does not configure in sim. CM refuses EVERY controller "
         "when one fails, so this is a robot-wide bring-up failure";
  EXPECT_FALSE(ctrl.IsRealArmConfig()) << profile << ": precondition — judged on the sim axis";
  EXPECT_FALSE(ctrl.IsSimOnlyDisabled());
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS)
      << profile << ": the shipped profile configures but cannot activate";
  ASSERT_EQ(ctrl.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

// A REAL arm is still blocked by the same profile — the provisional values are
// what does it, and a profile that stopped blocking would be indistinguishable
// from one that had been cleared for hardware.
TEST_P(ShippedCatchingProfile, IsStillParkedOnTheRealArmAxis) {
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  auto configs = ShippedSimConfigs(profile, node);
  ASSERT_EQ(configs.size(), 2U);
  for (auto& [name, cfg] : configs) {
    static_cast<void>(name);
    cfg.backend->type = "ur_driver_native";
  }
  auto node_handle =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_shipped_real_" + profile);
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, configs);
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile << ": a real-arm configuration is PARKED, never refused (A-S5-1)";
  EXPECT_TRUE(ctrl.IsRealArmConfig());
  EXPECT_TRUE(ctrl.IsSimOnlyDisabled())
      << profile << ": the provisional values stopped blocking a real arm";
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

// MD-89: the shipped law is `mpc`, which needs the arm model (its catch
// sub-model and the CLIK). Without one the shipped profile still configures —
// the robot comes up — but this controller parks and refuses to activate,
// never falls back to the closed form. The positive control is
// ConfiguresAndIsArmableOnTheSimAxis, the same file with the model.
TEST_P(ShippedCatchingProfile, WithoutAModelTheShippedMpcLawParks) {
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  ASSERT_EQ(node["catching"]["supervisor"]["decel"]["mode"].as<std::string>(), "mpc")
      << profile << ": precondition — the shipped law";
  auto node_handle =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_shipped_no_model_" + profile);
  DemoCatchingController ctrl{""};
  ctrl.SetControlRate(kShippedControlRateHz);
  ctrl.SetDeviceNameConfigs(ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile << ": a missing model parks this controller, it must not fail the bring-up";
  EXPECT_FALSE(ctrl.IsRealArmConfig()) << "precondition: judged on the sim axis";
  EXPECT_TRUE(ctrl.IsSimOnlyDisabled()) << profile;
  EXPECT_EQ(ctrl.GetParkReason(), integrated_bringup::CatchingParkReason::kDecelModeUnmet)
      << profile;
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

// ── S6-A: the shipped profiles with the planner thread ──────────────────────
//
// The planner is ON in these cases whatever the file says (and the oracle
// OFF), because the property under test is "the shipped profile runs the
// planner through the whole lifecycle" — the ur5e_p1b file keeps its oracle on
// until S6-B, and testing only what each file enables today would leave that
// profile's planner path unexercised.

YAML::Node ShippedWithPlanner(const std::string& profile, bool planner, bool oracle) {
  YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  node["catching"]["planner"]["enabled"] = planner;
  node["diagnostic"]["oracle_plan"]["enabled"] = oracle;
  if (!planner) {
    // The shipped law (`mpc`, MD-89) follows the planner's segments, so a
    // profile with the planner off runs v1's law — the only one it can.
    node["catching"]["supervisor"]["decel"]["mode"] = "closed_form";
  }
  return node;
}

rclcpp_lifecycle::LifecycleNode::SharedPtr NodeWithProfile(const std::string& name,
                                                           const std::string& layout_profile) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rt_layout_profile", layout_profile)});
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, options);
}

TEST_P(ShippedCatchingProfile, RunsThePlannerThroughTheWholeLifecycle) {
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node = ShippedWithPlanner(profile, /*planner=*/true, /*oracle=*/false);
  auto configs = ShippedSimConfigs(profile, node);
  ASSERT_EQ(configs.size(), 2U);
  auto node_handle = NodeWithProfile("catching_shipped_planner_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, configs);
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_FALSE(ctrl.IsSimOnlyDisabled());
  EXPECT_TRUE(ctrl.GetPlannerParams().enabled);
  EXPECT_GE(ctrl.GetPlannerWakeFd(), 0) << "configure made no wake eventfd";
  EXPECT_EQ(ctrl.GetPlannerThread(), nullptr) << "the thread is spawned at activation, not before";

  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  const auto* thread = ctrl.GetPlannerThread();
  ASSERT_NE(thread, nullptr);
  EXPECT_TRUE(thread->Running());
  EXPECT_FALSE(thread->Paused());

  ASSERT_EQ(ctrl.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(thread->Paused());
  // Re-activation reuses the thread (lazy spawn is once per controller).
  ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.GetPlannerThread(), thread);
  EXPECT_FALSE(thread->Paused());
  ASSERT_EQ(ctrl.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

TEST_P(ShippedCatchingProfile, MirrorsTheLeadFloorAndTheClikFormAsRun) {
  // MPC E1-F10 (MD-72, MD-74): the two values a tuning overlay moves per arm.
  // The lead floor is the commit lead while `planner.slice.t_lead_min` is
  // absent and the key's value once it is set; every shipped profile runs the
  // dynamic CLIK form. Each case moves the loaded value off the shipped one,
  // as above.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);

  struct Case {
    const char* name;
    bool set_floor;
    bool box;
  };

  for (const Case& c : {Case{"shipped", false, false}, Case{"moved", true, true}}) {
    YAML::Node node = ShippedWithPlanner(profile, true, false);
    YAML::Node planner = node["catching"]["planner"];
    const double t_freeze = planner["freeze"]["T_freeze"].as<double>();
    ASSERT_FALSE(planner["slice"]["t_lead_min"].IsDefined())
        << profile << ": the shipped profile sets the floor — pick another baseline";
    const double floor = c.set_floor ? t_freeze + 0.07 : t_freeze;
    if (c.set_floor) {
      planner["slice"]["t_lead_min"] = floor;
    }
    if (c.box) {
      node["catching"]["joint_cmd"]["accel_constraint"] = "box";
      node["catching"]["joint_cmd"].remove("eta_tau");  // the dynamic form's key
    }
    auto node_handle =
        NodeWithProfile("catching_shipped_tuning_mirror_" + profile + "_" + c.name, "mpc_on");
    DemoCatchingController ctrl{""};
    BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
              DemoCatchingController::CallbackReturn::SUCCESS)
        << profile << " " << c.name;
    EXPECT_DOUBLE_EQ(node_handle->get_parameter("planner.slice.t_lead_min").as_double(), floor)
        << profile << " " << c.name;
    EXPECT_EQ(node_handle->get_parameter("joint_cmd.accel_constraint").as_string(),
              c.box ? "box" : "dynamic")
        << profile << " " << c.name;
  }
}

TEST_P(ShippedCatchingProfile, MirrorsTheTrialRunnerInputsTheControllerLoaded) {
  // S8-A: the trial runner reads the wait pose, T_freeze and the lead axis
  // from these read-only parameters instead of the installed YAML, because a
  // sim overlay changes them without changing the file. So the loaded values
  // are MOVED off the shipped ones here: a mirror that re-read the file (or a
  // default) would still pass an equality check against the file.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);  // the hand's; the wait pose is the arm's
  YAML::Node node = ShippedWithPlanner(profile, true, false);
  YAML::Node planner = node["catching"]["planner"];
  std::vector<double> wait = planner["wait_pose"].as<std::vector<double>>();
  ASSERT_FALSE(wait.empty()) << profile;
  wait[0] += 0.01;
  planner["wait_pose"] = wait;
  const double t_freeze = planner["freeze"]["T_freeze"].as<double>() + 0.01;
  planner["freeze"]["T_freeze"] = t_freeze;
  // Lead on with the shipped T_arm (0): the freeze-window check is unchanged,
  // so this is the only mirrored value that moves the lead switch alone.
  node["catching"]["joint_cmd"]["lag"]["lead_enable"] = true;
  const double t_arm = node["catching"]["joint_cmd"]["lag"]["T_arm"].as<double>();

  auto node_handle = NodeWithProfile("catching_shipped_mirror_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_FALSE(ctrl.IsSimOnlyDisabled());

  const auto mirrored = node_handle->get_parameter("planner.wait_pose").as_double_array();
  ASSERT_EQ(mirrored.size(), wait.size());
  for (std::size_t i = 0; i < wait.size(); ++i) {
    EXPECT_DOUBLE_EQ(mirrored[i], wait[i]) << profile << " joint " << i;
  }
  EXPECT_DOUBLE_EQ(node_handle->get_parameter("planner.freeze.T_freeze").as_double(), t_freeze);
  EXPECT_DOUBLE_EQ(node_handle->get_parameter("joint_cmd.lag.T_arm").as_double(), t_arm);
  EXPECT_TRUE(node_handle->get_parameter("joint_cmd.lag.lead_enable").as_bool());
  // S8-C: RETREAT's hand timeout. A profile that does not set the key gets
  // the DERIVED value (ur5e_p1b); one that sets it from a measurement gets
  // exactly that (iiwa7_leap since S8-D, #537 — the derived 1.454 s was
  // shorter than its measured release). Either way the mirror is the only
  // place a reader can see the value the controller runs with.
  const auto& hand = ctrl.GetCatchingParams().hand;
  const YAML::Node set = node["catching"]["robot"]["hand"]["T_release_timeout"];
  EXPECT_EQ(hand.T_release_timeout_derived, !set) << profile;
  ASSERT_FALSE(hand.T_release_timeout.tbd) << profile;
  if (set) {
    EXPECT_DOUBLE_EQ(hand.T_release_timeout.value, set.as<double>()) << profile;
  }
  EXPECT_GT(hand.T_release_timeout.value, hand.T_close_e2e.value) << profile;
  EXPECT_DOUBLE_EQ(node_handle->get_parameter("hand.T_release_timeout").as_double(),
                   hand.T_release_timeout.value)
      << profile;
  // Read-only, like the hand profile: the runner must not be able to "fix" a
  // mismatch by writing the value it expected.
  const auto result = node_handle->set_parameter(rclcpp::Parameter("planner.freeze.T_freeze", 0.5));
  EXPECT_FALSE(result.successful);
}

TEST_P(ShippedCatchingProfile, AKeyOfEachFragmentReachesTheController) {
  // MD-90: the profile is a main file plus three `include:` fragments
  // (catching/search_grid, planner_closed_form, planner_mpc). One mirrored key
  // per fragment is moved in the composed tree — the place a CM override
  // writes — and read back from the configured controller.
  //
  // The main file read on its own must NOT hold these keys: that is what makes
  // each line below a statement about its fragment, and what turns the suite
  // red when an entry is dropped from the include list.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);

  struct Case {
    const char* fragment;
    std::vector<std::string> path;  // under `catching:`
    const char* mirror;
    double moved_by;
  };

  const std::vector<Case> cases = {
      {"catching/search_grid.yaml", {"planner", "time", "margin"}, "planner.time.margin", 0.01},
      {"catching/planner_closed_form.yaml", {"reference", "omega"}, "reference.omega", -1.0},
      {"catching/planner_mpc.yaml",
       {"planner", "decel_mpc", "catch", "gamma_ref"},
       "planner.decel_mpc.catch.gamma_ref",
       -0.1},
      {"catching/planner_mpc.yaml",
       {"supervisor", "decel", "switch_margin"},
       "supervisor.decel.switch_margin",
       -0.1},
  };
  const auto at = [](const YAML::Node& root, const std::vector<std::string>& path) {
    YAML::Node cursor = YAML::Clone(root);  // a const walk: operator[] must not insert
    for (const auto& key : path) {
      const YAML::Node& view = cursor;
      const YAML::Node next = view[key];
      if (!next.IsDefined()) {
        return YAML::Node(YAML::NodeType::Undefined);
      }
      cursor.reset(next);
    }
    return cursor;
  };

  const std::string main_path = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile +
                                "/controllers/demo_catching_controller.yaml";
  const YAML::Node main_only = YAML::LoadFile(main_path)["demo_catching_controller"]["catching"];
  YAML::Node node = ShippedWithPlanner(profile, true, false);

  std::vector<double> moved;
  for (const Case& c : cases) {
    ASSERT_FALSE(at(main_only, c.path).IsDefined())
        << profile << ": " << c.mirror << " is in the main file, not in " << c.fragment;
    const YAML::Node shipped = at(node["catching"], c.path);
    ASSERT_TRUE(shipped.IsDefined()) << profile << ": " << c.mirror
                                     << " is missing from the composed tree (" << c.fragment << ")";
    moved.push_back(shipped.as<double>() + c.moved_by);
    YAML::Node parent = node["catching"];
    for (std::size_t i = 0; i + 1 < c.path.size(); ++i) {
      parent.reset(parent[c.path[i]]);
    }
    parent[c.path.back()] = moved.back();
  }

  auto node_handle = NodeWithProfile("catching_shipped_fragments_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile;
  for (std::size_t i = 0; i < cases.size(); ++i) {
    EXPECT_DOUBLE_EQ(node_handle->get_parameter(cases[i].mirror).as_double(), moved[i])
        << profile << ": " << cases[i].mirror << " (" << cases[i].fragment << ")";
  }
}

// The acceleration box is two plain keys of the search fragment. The mirror is
// what every reader of the loaded value (the search, the stop and homing ramp,
// CLIK's `box` form, the tools) sees, so it is the place to pin the value.
TEST_P(ShippedCatchingProfile, MirrorsTheShippedAccelerationBox) {
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  const std::vector<double> expected =
      profile == "ur5e_p1b" ? std::vector<double>(6, 2.03052) : std::vector<double>(7, 9.197629);

  const std::string main_path = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile +
                                "/controllers/demo_catching_controller.yaml";
  const YAML::Node main_arm =
      YAML::LoadFile(main_path)["demo_catching_controller"]["catching"]["robot"]["arm"];
  EXPECT_FALSE(main_arm["qdd_max"].IsDefined()) << profile << ": the box belongs to the search";
  EXPECT_FALSE(main_arm["qdd_provisional"].IsDefined()) << profile;

  YAML::Node node = ShippedWithPlanner(profile, true, false);
  auto node_handle = NodeWithProfile("catching_shipped_box_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile;
  const auto mirrored = node_handle->get_parameter("robot.arm.qdd_max").as_double_array();
  ASSERT_EQ(mirrored.size(), expected.size()) << profile;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_DOUBLE_EQ(mirrored[i], expected[i]) << profile << " joint " << i;
  }
  EXPECT_FALSE(node_handle->has_parameter("robot.arm.accel_limits_path"))
      << profile << ": the box file is gone, so is its mirror";
}

// ur5e_p1b's sim.yaml replaces the box with the executed envelope. The CM
// applies that block like a sim_overlay — it writes the block's leaves into
// the composed tree — so the same write is made here and read back from the
// mirror. The block's arrays must be float arrays (a ROS parameter array has one
// type; `[20, ...]` would be an integer array).
TEST_P(ShippedCatchingProfile, TheP1bSimOverrideSetsTheEnvelopeBox) {
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  if (profile != "ur5e_p1b") {
    GTEST_SKIP() << "only ur5e_p1b's sim.yaml overrides the box";
  }
  const std::vector<double> envelope = {20.2531, 30.859, 36.9851, 21.5036, 14.2481, 29.0196};

  const YAML::Node sim = YAML::LoadFile(std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile +
                                        "/sim.yaml")["/**"]["ros__parameters"];
  const YAML::Node arm = sim["demo_catching_controller"]["catching"]["robot"]["arm"];
  ASSERT_TRUE(arm.IsDefined());
  ASSERT_EQ(arm.size(), 2U) << "the override carries the box and its flag, nothing else";
  ASSERT_TRUE(arm["qdd_max"].IsSequence());
  for (const auto& v : arm["qdd_max"]) {
    EXPECT_NE(v.Scalar().find('.'), std::string::npos) << v.Scalar() << ": not a float literal";
  }
  EXPECT_TRUE(arm["qdd_provisional"].as<bool>());

  YAML::Node node = ShippedWithPlanner(profile, true, false);
  node["catching"]["robot"]["arm"]["qdd_max"] = arm["qdd_max"];
  node["catching"]["robot"]["arm"]["qdd_provisional"] = arm["qdd_provisional"];
  auto node_handle = NodeWithProfile("catching_shipped_envelope_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS);
  const auto mirrored = node_handle->get_parameter("robot.arm.qdd_max").as_double_array();
  ASSERT_EQ(mirrored.size(), envelope.size());
  for (std::size_t i = 0; i < envelope.size(); ++i) {
    EXPECT_DOUBLE_EQ(mirrored[i], envelope[i]) << "joint " << i;
  }
}

TEST_P(ShippedCatchingProfile, RefusesToActivateThePlannerUnderTheMpcOffProfile) {
  // Same gate, same place as DemoWbc's (#350): on_activate's first statement,
  // before any side effect — above all, no planner thread on a core the
  // shield no longer holds.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node = ShippedWithPlanner(profile, true, false);
  auto node_handle = NodeWithProfile("catching_shipped_mpc_off_" + profile, "mpc_off");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << "the profile gate refuses ACTIVATION; configure must still succeed";
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
  EXPECT_EQ(ctrl.GetPlannerThread(), nullptr);

  // The planner OFF under the same profile is a normal activation: the gate
  // refuses a configuration, not a controller.
  YAML::Node off = ShippedWithPlanner(profile, false, false);
  auto node_off = NodeWithProfile("catching_shipped_mpc_off_noplan_" + profile, "mpc_off");
  DemoCatchingController ctrl_off{""};
  BringUpShipped(ctrl_off, profile, ShippedSimConfigs(profile, off));
  ASSERT_EQ(ctrl_off.on_configure(prev, node_off, off),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl_off.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl_off.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

TEST_P(ShippedCatchingProfile, AConsumedKeyLeftTbdParksTheSimProfileInsteadOfFailingIt) {
  // A-S5-12, positive control: the shipped sim profile with one consumed key
  // removed. Before A-S5-12 this refused the configure, and CM then refused
  // EVERY controller — no robot. Now the robot comes up and only this
  // controller declines to activate.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node = ShippedWithPlanner(profile, false, false);
  ASSERT_TRUE(node["catching"]["io"]["t_stale"]) << "precondition: the key is shipped";
  node["catching"]["io"].remove("t_stale");
  auto node_handle = NodeWithProfile("catching_shipped_tbd_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile << ": a consumed TBD in sim must park, not refuse (A-S5-12)";
  EXPECT_FALSE(ctrl.IsRealArmConfig()) << "precondition: judged on the sim axis";
  EXPECT_TRUE(ctrl.IsSimOnlyDisabled());
  EXPECT_EQ(ctrl.GetParkReason(), integrated_bringup::CatchingParkReason::kConsumedValues);
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
  ASSERT_EQ(ctrl.on_cleanup(prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

TEST_P(ShippedCatchingProfile, ThePlannerAndTheOracleTogetherPark) {
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node = ShippedWithPlanner(profile, /*planner=*/true, /*oracle=*/true);
  auto node_handle = NodeWithProfile("catching_shipped_both_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS);
  EXPECT_TRUE(ctrl.IsSimOnlyDisabled());
  EXPECT_EQ(ctrl.GetParkReason(), integrated_bringup::CatchingParkReason::kPlannerOracleConflict);
  EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
  EXPECT_EQ(ctrl.GetPlannerThread(), nullptr);
}

TEST_P(ShippedCatchingProfile, AnUnsetPlannerDecisionParksInsteadOfGuessing) {
  // S6-B: sub_model, T_freeze, catch_box, d_eff and r_cap are decisions with
  // no default. Each one removed parks the controller and names the key; the
  // robot still comes up. Positive control: the shipped file itself does not
  // park (RunsThePlannerThroughTheWholeLifecycle).
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  const std::vector<std::pair<const char*, const char*>> keys{{"sub_model", nullptr},
                                                              {"freeze", "T_freeze"},
                                                              {"workspace", "catch_box"},
                                                              {"hand", "d_eff"},
                                                              {"hand", "r_cap"}};
  int n = 0;
  for (const auto& [section, key] : keys) {
    YAML::Node node = ShippedWithPlanner(profile, true, false);
    YAML::Node planner = node["catching"]["planner"];
    ASSERT_TRUE(key == nullptr ? static_cast<bool>(planner[section])
                               : static_cast<bool>(planner[section][key]))
        << profile << ": precondition — the shipped file sets " << section;
    if (key == nullptr) {
      planner.remove(section);
    } else {
      planner[section].remove(key);
    }
    auto node_handle =
        NodeWithProfile("catching_shipped_unset_" + profile + "_" + std::to_string(n++), "mpc_on");
    DemoCatchingController ctrl{""};
    BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
    const rclcpp_lifecycle::State prev;
    ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
              DemoCatchingController::CallbackReturn::SUCCESS)
        << profile << " without " << section;
    EXPECT_TRUE(ctrl.IsSimOnlyDisabled()) << profile << " without " << section;
    EXPECT_EQ(ctrl.GetParkReason(), integrated_bringup::CatchingParkReason::kPlannerUnset)
        << profile << " without " << section;
    EXPECT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::FAILURE);
  }
}

TEST_P(ShippedCatchingProfile, TheWaitPoseIsOnePerArmJointAndInsideTheArmLimits) {
  // Decision L: arm joint order. Read against the arm device's own limits
  // rather than restated numbers.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  const YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  const auto planner = rtc::catching::ParsePlannerParams(node["catching"]);
  const auto configs = ShippedSimConfigs(profile, node);
  const auto arm_group = node["topics"].begin()->first.as<std::string>();
  const auto it = configs.find(arm_group);
  ASSERT_NE(it, configs.end());
  const auto& arm = it->second;
  ASSERT_EQ(planner.wait_pose_n, static_cast<int>(arm.joint_state_names.size()))
      << profile << ": planner.wait_pose must carry one value per arm joint";
  ASSERT_TRUE(arm.joint_limits.has_value()) << profile << ": arm declares no joint_limits";
  for (int i = 0; i < planner.wait_pose_n; ++i) {
    const auto u = static_cast<std::size_t>(i);
    EXPECT_GE(planner.wait_pose[u], arm.joint_limits->position_lower[u])
        << profile << "[" << i << "]";
    EXPECT_LE(planner.wait_pose[u], arm.joint_limits->position_upper[u])
        << profile << "[" << i << "]";
  }
}

// ── S8-I: the wait pose the arm was switched in at ───────────────────────────
namespace {

constexpr int kP1bHandChannels = 10;

/// The p1b sim profile with the planner on and `planner.wait_pose_source` set.
struct WaitPoseRig {
  explicit WaitPoseRig(const char* source, const std::string& node_name) {
    node = ShippedWithPlanner("ur5e_p1b", /*planner=*/true, /*oracle=*/false);
    // No arm model here (the S8-I refusal below needs none), so not the
    // shipped `mpc` law, which parks without one (MD-89): v1's law, as the
    // profile shipped until then.
    node["catching"]["supervisor"]["decel"]["mode"] = "closed_form";
    if (source != nullptr) {
      node["catching"]["planner"]["wait_pose_source"] = source;
    }
    yaml_wait = node["catching"]["planner"]["wait_pose"].as<std::vector<double>>();
    handle = NodeWithProfile(node_name, "mpc_on");
    ctrl.SetControlRate(kShippedControlRateHz);
    ctrl.SetDeviceNameConfigs(ShippedSimConfigs("ur5e_p1b", node));
  }

  void ConfigureAndActivate() {
    ASSERT_EQ(ctrl.on_configure(prev, handle, node),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_FALSE(ctrl.IsSimOnlyDisabled());
    ASSERT_EQ(ctrl.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  }

  void ExpectWaitPoseIs(const std::vector<double>& expect, const char* why) const {
    const auto pose = ctrl.GetWaitPoseForTesting();
    const auto rt = ctrl.GetPlannerRtState();
    const auto rec = ctrl.GetLastTickRecord();
    for (std::size_t i = 0; i < expect.size(); ++i) {
      EXPECT_DOUBLE_EQ(pose[i], expect[i]) << why << " joint " << i;
      EXPECT_DOUBLE_EQ(rec.wait_pose[i], expect[i]) << why << " diag joint " << i;
      if (rt.wait_pose_adopted) {
        EXPECT_DOUBLE_EQ(rt.wait_pose[i], expect[i]) << why << " planner joint " << i;
      }
    }
  }

  YAML::Node node;
  std::vector<double> yaml_wait;
  rclcpp_lifecycle::LifecycleNode::SharedPtr handle;
  DemoCatchingController ctrl{""};
  const rclcpp_lifecycle::State prev;
};

std::vector<double> ArmPositionsOf(const ControllerState& s) {
  return {s.devices[0].positions.begin(), s.devices[0].positions.begin() + kArmDof};
}

}  // namespace

TEST_P(ShippedCatchingProfile, ShipsTheApproachStopGridSwitchedOn) {
  // MPC E1-F08 (#661, MD-54 · MD-55): the shipped profiles carry the
  // APPROACH–stop grid — 7 x 0.05 s after the catch, up to 6 x 0.1 s before
  // it. Since MD-89 (2026-10-03) they also ship the decel MPC on and DECEL
  // on `mpc` — the law the grid serves; before it they shipped both off and
  // the closed form. The CODE defaults are neither (14 x 0.025, n_pre_max 0,
  // `closed_form`), so only reading the file shows what a robot runs.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  const YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  const auto planner = rtc::catching::ParsePlannerParams(node["catching"]);
  const auto& d = planner.decel;
  EXPECT_TRUE(d.horizon_explicit) << profile;
  EXPECT_EQ(d.n_nodes, 7) << profile;
  EXPECT_EQ(d.DtNs(), 50'000'000) << profile;
  ASSERT_EQ(d.n_blocks, 4) << profile;
  EXPECT_EQ((std::array<int, 4>{d.blocks[0], d.blocks[1], d.blocks[2], d.blocks[3]}),
            (std::array<int, 4>{1, 1, 2, 3}))
      << profile;
  EXPECT_EQ(d.k_max, 2) << profile;
  EXPECT_EQ(d.n_pre_max, 6) << profile;
  EXPECT_EQ(d.DtPreNs(), 100'000'000) << profile;
  EXPECT_TRUE(d.replan_same_point) << profile;
  // The budgets must leave the planner's wake inside the decel admission age
  // bound the RT judges a segment by (50 ms): configure parks mode mpc unless
  // budget.replan_s + 3 ticks is below it (DecelModeUnmet).
  EXPECT_GT(d.budget_first_s, 0.0) << profile;
  EXPECT_GT(d.budget_replan_s, 0.0) << profile;
  EXPECT_GT(d.catch_pos_err_max, 0.0) << profile;
  ASSERT_TRUE(node["catching"]["supervisor"]["decel"]["mode"]) << profile;
  EXPECT_EQ(node["catching"]["supervisor"]["decel"]["mode"].as<std::string>(), "mpc") << profile;
}

TEST_P(ShippedCatchingProfile, ShipsTheVelocitySlackWrittenAndOff) {
  // MPC MD-91: the relative-velocity slack row's keys are WRITTEN in
  // catching/planner_mpc.yaml — not left to the code default — and both 0:
  // the row is off, so the shipped solve is the one before the keys existed.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  const YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  const YAML::Node dcatch = node["catching"]["planner"]["decel_mpc"]["catch"];
  ASSERT_TRUE(dcatch.IsMap()) << profile;
  ASSERT_TRUE(dcatch["rho_v"].IsDefined()) << profile << ": catch.rho_v is not written";
  ASSERT_TRUE(dcatch["v_rel_allow"].IsDefined()) << profile << ": catch.v_rel_allow is not written";
  const std::string main_path = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile +
                                "/controllers/demo_catching_controller.yaml";
  const YAML::Node main_planner =
      YAML::LoadFile(main_path)["demo_catching_controller"]["catching"]["planner"];
  EXPECT_FALSE(main_planner["decel_mpc"].IsDefined())
      << profile << ": the decel MPC keys belong to catching/planner_mpc.yaml";
  const auto planner = rtc::catching::ParsePlannerParams(node["catching"]);
  EXPECT_EQ(planner.decel.rho_v, 0.0) << profile;
  EXPECT_EQ(planner.decel.v_rel_allow, 0.0) << profile;
}

TEST_P(ShippedCatchingProfile, MirrorsTheVelocitySlackKeysItRunsWith) {
  // The two keys moved in the composed tree — where a CM override writes —
  // reach the controller: its read-only mirrors carry the moved values and the
  // decel planner configures with the slack row on (every catch core is built
  // and warmed with it). A mirror declared from the field's default would read
  // 0 here.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node = ShippedWithPlanner(profile, true, false);
  YAML::Node dcatch = node["catching"]["planner"]["decel_mpc"]["catch"];
  dcatch["rho_v"] = 2.0;
  dcatch["v_rel_allow"] = 0.3;

  auto node_handle = NodeWithProfile("catching_shipped_v_slack_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile;
  EXPECT_TRUE(ctrl.IsDecelPlannerConfigured()) << profile;
  EXPECT_DOUBLE_EQ(node_handle->get_parameter("planner.decel_mpc.catch.rho_v").as_double(), 2.0)
      << profile;
  EXPECT_DOUBLE_EQ(node_handle->get_parameter("planner.decel_mpc.catch.v_rel_allow").as_double(),
                   0.3)
      << profile;
  EXPECT_DOUBLE_EQ(ctrl.GetPlannerParams().decel.rho_v, 2.0) << profile;
  EXPECT_DOUBLE_EQ(ctrl.GetPlannerParams().decel.v_rel_allow, 0.3) << profile;
  // Read-only, like every other mirror of the profile.
  EXPECT_FALSE(node_handle->set_parameter(rclcpp::Parameter("planner.decel_mpc.catch.rho_v", 0.0))
                   .successful);
}

TEST_P(ShippedCatchingProfile, ShipsTheDesignKeysWrittenAtTheCodeDefaults) {
  // MPC MD-92 / #698: every design value of the decel MPC core, of the catch
  // pose IK, of the unit-speed solve and of the switch step bound is WRITTEN in
  // the shipped fragments, and at what the code used before the key existed —
  // so the shipped solves, rankings and tests did not move.
  const auto& [profile, total_dof] = GetParam();
  static_cast<void>(total_dof);
  const YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  const YAML::Node catching = node["catching"];
  // The arm's joint count: `expected_dof` of the parameter counts the hand too.
  const auto arm_dof = static_cast<int>(catching["robot"]["arm"]["qdd_max"].size());
  const YAML::Node mpc = catching["planner"]["decel_mpc"];
  const YAML::Node ik = catching["planner"]["ik"];

  struct Key {
    YAML::Node section;
    const char* name;
  };

  for (const Key& k : {Key{mpc["cost"], "jerk_weight"},
                       Key{mpc["cost"], "u_scale"},
                       Key{mpc["cost"], "w_delta"},
                       Key{mpc["cost"], "rho_tau"},
                       Key{mpc["catch"], "axis_theta_max"},
                       Key{mpc["linearization"], "delta_tr"},
                       Key{mpc["linearization"], "reference_rest_tol"},
                       Key{mpc["linearization"], "ref_speed_fraction"},
                       Key{mpc["solver"], "max_iter"},
                       Key{mpc["solver"], "max_iter_in"},
                       Key{mpc["solver"], "eps_abs"},
                       Key{mpc["solver"], "eps_rel"},
                       Key{ik, "eps_pos"},
                       Key{ik, "alpha_max"},
                       Key{ik, "rho"},
                       Key{ik, "sigma0"},
                       Key{ik, "lambda_max"},
                       Key{ik, "dq_step_max"},
                       Key{ik, "mu"},
                       Key{ik, "qp_eps_abs"},
                       Key{ik, "qp_max_iter"},
                       Key{ik, "k_null"},
                       Key{ik, "manip_grad_tol"},
                       Key{ik, "fd_step"},
                       Key{ik, "v_eps"},
                       Key{catching["planner"]["gamma"], "unit_speed_damping"},
                       Key{catching["planner"]["switch"], "samples"}}) {
    ASSERT_TRUE(k.section.IsMap()) << profile << ": the section of " << k.name << " is missing";
    EXPECT_TRUE(k.section[k.name].IsDefined()) << profile << ": " << k.name << " is not written";
  }

  // The core: the planner's parse equals DecelMpcParams{} field by field (the
  // fields the planner overwrites — grid, eta, m_q, catch terms — are not
  // compared). jerk_weight is written, one 1.0 per arm joint, which is the
  // core's empty = all-ones.
  const auto planner = rtc::catching::ParsePlannerParams(catching);
  const rtc::catching::DecelMpcParams core{};
  const auto& d = planner.decel;
  ASSERT_EQ(d.jerk_weight.size(), static_cast<std::size_t>(arm_dof)) << profile;
  for (const double w : d.jerk_weight) {
    EXPECT_EQ(w, 1.0) << profile;
  }
  EXPECT_EQ(d.u_scale, core.u_scale) << profile;
  EXPECT_EQ(d.w_delta, core.w_delta) << profile;
  EXPECT_EQ(d.rho_tau, core.rho_tau) << profile;
  EXPECT_EQ(d.axis_theta_max, core.axis_theta_max) << profile;
  EXPECT_EQ(d.delta_tr, core.delta_tr) << profile;
  EXPECT_EQ(d.reference_rest_tol, core.reference_rest_tol) << profile;
  EXPECT_EQ(d.solver_max_iter, core.solver.max_iter) << profile;
  EXPECT_EQ(d.solver_max_iter_in, core.solver.max_iter_in) << profile;
  EXPECT_EQ(d.solver_eps_abs, core.solver.eps_abs) << profile;
  EXPECT_EQ(d.solver_eps_rel, core.solver.eps_rel) << profile;
  EXPECT_EQ(d.ref_speed_fraction, 0.9) << profile;  // the planner's former constant
  EXPECT_EQ(core.w_perp, 0.0);                      // the default; `cost.w_perp` is tested below

  // The search: the IK options equal CatchPoseIkOptions{} field by field —
  // except the two the profile has always set (k_manip, max_iter) and the
  // gate's threshold; alpha_max is now a SET value, no longer a TBD default.
  const auto ik_cfg = rtc::catching::ParseCatchPoseIkParams(catching);
  const rtc::catching::CatchPoseIkOptions def{};
  const auto& o = ik_cfg.options;
  EXPECT_EQ(o.eps_pos, def.eps_pos) << profile;
  EXPECT_EQ(o.alpha_max, def.alpha_max) << profile;
  EXPECT_FALSE(ik_cfg.alpha_max.tbd) << profile << ": alpha_max must be a decided value";
  EXPECT_EQ(o.rho, def.rho) << profile;
  EXPECT_EQ(o.sigma0, def.sigma0) << profile;
  EXPECT_EQ(o.lambda_max, def.lambda_max) << profile;
  EXPECT_EQ(o.dq_step_max, def.dq_step_max) << profile;
  EXPECT_EQ(o.mu, def.mu) << profile;
  EXPECT_EQ(o.qp_eps_abs, def.qp_eps_abs) << profile;
  EXPECT_EQ(o.qp_max_iter, def.qp_max_iter) << profile;
  EXPECT_EQ(o.k_null, def.k_null) << profile;
  EXPECT_EQ(o.manip_grad_tol, def.manip_grad_tol) << profile;
  EXPECT_EQ(o.fd_step, def.fd_step) << profile;
  EXPECT_EQ(o.v_eps, def.v_eps) << profile;
  EXPECT_EQ(o.definition, def.definition) << profile;

  // The unit-speed damping and the switch samples: the code's former constants.
  EXPECT_EQ(planner.unit_speed_damping, rtc::catching::kUnitSpeedDamping) << profile;
  EXPECT_EQ(planner.switch_samples, 9) << profile;
}

TEST_P(ShippedCatchingProfile, MirrorsTheDesignKeysItRunsWith) {
  // The twelve decel-MPC design keys moved in the composed tree — where a CM
  // override writes — reach the controller: each read-only mirror carries the
  // moved value, and the decel planner configures with the cores built on them.
  // A mirror declared from a default would read the shipped value here.
  const auto& [profile, total_dof] = GetParam();
  static_cast<void>(total_dof);
  YAML::Node node = ShippedWithPlanner(profile, true, false);
  YAML::Node mpc = node["catching"]["planner"]["decel_mpc"];
  const auto arm_dof = static_cast<int>(node["catching"]["robot"]["arm"]["qdd_max"].size());
  YAML::Node weights;
  for (int i = 0; i < arm_dof; ++i) {
    weights.push_back(1.0 + 0.25 * i);
  }
  mpc["cost"]["jerk_weight"] = weights;
  mpc["cost"]["u_scale"] = 500.0;
  mpc["cost"]["w_delta"] = 2.5;
  mpc["cost"]["rho_tau"] = 7.0;
  mpc["catch"]["axis_theta_max"] = 1.2;
  mpc["linearization"]["delta_tr"] = 0.2;
  mpc["linearization"]["reference_rest_tol"] = 2.0e-5;
  mpc["linearization"]["ref_speed_fraction"] = 0.8;
  mpc["solver"]["max_iter"] = 300;
  mpc["solver"]["max_iter_in"] = 150;
  mpc["solver"]["eps_abs"] = 2.0e-7;
  mpc["solver"]["eps_rel"] = 1.0e-5;

  auto node_handle = NodeWithProfile("catching_shipped_design_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile;
  EXPECT_TRUE(ctrl.IsDecelPlannerConfigured()) << profile;
  const auto dbl = [&](const char* name) { return node_handle->get_parameter(name).as_double(); };
  const auto integer = [&](const char* name) { return node_handle->get_parameter(name).as_int(); };
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.cost.u_scale"), 500.0) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.cost.w_delta"), 2.5) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.cost.rho_tau"), 7.0) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.catch.axis_theta_max"), 1.2) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.linearization.delta_tr"), 0.2) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.linearization.reference_rest_tol"), 2.0e-5) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.linearization.ref_speed_fraction"), 0.8) << profile;
  EXPECT_EQ(integer("planner.decel_mpc.solver.max_iter"), 300) << profile;
  EXPECT_EQ(integer("planner.decel_mpc.solver.max_iter_in"), 150) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.solver.eps_abs"), 2.0e-7) << profile;
  EXPECT_DOUBLE_EQ(dbl("planner.decel_mpc.solver.eps_rel"), 1.0e-5) << profile;
  const auto jw =
      node_handle->get_parameter("planner.decel_mpc.cost.jerk_weight").as_double_array();
  ASSERT_EQ(jw.size(), static_cast<std::size_t>(arm_dof)) << profile;
  for (int i = 0; i < arm_dof; ++i) {
    EXPECT_DOUBLE_EQ(jw[static_cast<std::size_t>(i)], 1.0 + 0.25 * i) << profile << " " << i;
  }
  EXPECT_DOUBLE_EQ(ctrl.GetPlannerParams().decel.ref_speed_fraction, 0.8) << profile;
  EXPECT_FALSE(node_handle->set_parameter(rclcpp::Parameter("planner.decel_mpc.cost.u_scale", 1.0))
                   .successful);
}

TEST_P(ShippedCatchingProfile, ShipsTheStopPathWeightWrittenAndOff) {
  // #698: `cost.w_perp` is WRITTEN in catching/planner_mpc.yaml — not left to
  // the code default — and 0: the stop-path term is off, so the shipped solve
  // is the one before the key existed (the core's own default).
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  const YAML::Node node =
      integrated_bringup::testfx::ShippedControllerNode(profile, "demo_catching_controller");
  const YAML::Node cost = node["catching"]["planner"]["decel_mpc"]["cost"];
  ASSERT_TRUE(cost.IsMap()) << profile;
  ASSERT_TRUE(cost["w_perp"].IsDefined()) << profile << ": cost.w_perp is not written";
  EXPECT_EQ(cost["w_perp"].as<double>(), 0.0) << profile;
  const std::string fragment = std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile +
                               "/controllers/catching/planner_mpc.yaml";
  const YAML::Node in_fragment = YAML::LoadFile(
      fragment)["demo_catching_controller"]["catching"]["planner"]["decel_mpc"]["cost"]["w_perp"];
  EXPECT_TRUE(in_fragment.IsDefined()) << profile << ": the key belongs to planner_mpc.yaml";
  const auto planner = rtc::catching::ParsePlannerParams(node["catching"]);
  EXPECT_EQ(planner.decel.w_perp, 0.0) << profile;
  EXPECT_EQ(planner.decel.w_perp, rtc::catching::DecelMpcParams{}.w_perp) << profile;
}

TEST_P(ShippedCatchingProfile, MirrorsTheStopPathWeightItRunsWith) {
  // The key moved in the composed tree — where a CM override writes — reaches
  // the controller: the read-only mirror carries the moved value, and the
  // decel planner configures with the term ON, which means every stop core and
  // every catch core was built with it and warmed up on a line the planner
  // built (a warm-up that fails fails the configure). A mirror declared from
  // the field's default would read 0 here.
  const auto& [profile, expected_dof] = GetParam();
  static_cast<void>(expected_dof);
  YAML::Node node = ShippedWithPlanner(profile, true, false);
  node["catching"]["planner"]["decel_mpc"]["cost"]["w_perp"] = 40.0;

  auto node_handle = NodeWithProfile("catching_shipped_w_perp_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl.on_configure(prev, node_handle, node),
            DemoCatchingController::CallbackReturn::SUCCESS)
      << profile;
  EXPECT_TRUE(ctrl.IsDecelPlannerConfigured()) << profile;
  EXPECT_DOUBLE_EQ(node_handle->get_parameter("planner.decel_mpc.cost.w_perp").as_double(), 40.0)
      << profile;
  EXPECT_DOUBLE_EQ(ctrl.GetPlannerParams().decel.w_perp, 40.0) << profile;
  // Read-only, like every other mirror of the profile.
  EXPECT_FALSE(node_handle->set_parameter(rclcpp::Parameter("planner.decel_mpc.cost.w_perp", 0.0))
                   .successful);
}

TEST_P(ShippedCatchingProfile, RefusesAJerkWeightListOfTheWrongLengthAtConfigure) {
  // The parser has no joint count, so the length is checked where the arm is
  // known: the configure fails, and the planner is not configured (the
  // controller parks rather than running a core built on the wrong list).
  const auto& [profile, total_dof] = GetParam();
  static_cast<void>(total_dof);
  YAML::Node node = ShippedWithPlanner(profile, true, false);
  const auto arm_dof = static_cast<int>(node["catching"]["robot"]["arm"]["qdd_max"].size());
  YAML::Node weights;
  for (int i = 0; i < arm_dof - 1; ++i) {
    weights.push_back(1.0);
  }
  node["catching"]["planner"]["decel_mpc"]["cost"]["jerk_weight"] = weights;
  auto node_handle = NodeWithProfile("catching_shipped_jw_len_" + profile, "mpc_on");
  DemoCatchingController ctrl{""};
  BringUpShipped(ctrl, profile, ShippedSimConfigs(profile, node));
  const rclcpp_lifecycle::State prev;
  const auto rc = ctrl.on_configure(prev, node_handle, node);
  EXPECT_FALSE(rc == DemoCatchingController::CallbackReturn::SUCCESS &&
               ctrl.IsDecelPlannerConfigured())
      << profile << ": a list one short was accepted";
}

TEST(DemoCatchingWaitPose, WithoutAnArmBoxTheSwitchedInPoseIsRefusedAndTheYamlPoseStands) {
  // S8-I, the fail-closed half. This fixture has no arm model, so the arm has
  // no margined joint box to admit a pose against: `current` must refuse (and
  // count) rather than adopt an unchecked pose. The adoption, E-STOP and
  // re-activation semantics run under a model in
  // test_catching_supervisor_scenarios.cpp.
  WaitPoseRig rig("current", "catching_wait_pose_current");
  rig.ConfigureAndActivate();
  EXPECT_EQ(rig.handle->get_parameter("planner.wait_pose_source").as_string(), "current");
  // The read-only mirror is the YAML value whatever the source.
  EXPECT_EQ(rig.handle->get_parameter("planner.wait_pose").as_double_array(), rig.yaml_wait);

  (void)rig.ctrl.Compute(MakeState(0.3, kP1bHandChannels));
  EXPECT_FALSE(rig.ctrl.IsWaitPoseAdoptedForTesting());
  EXPECT_FALSE(rig.ctrl.GetPlannerRtState().wait_pose_adopted);
  EXPECT_FALSE(rig.ctrl.GetLastTickRecord().wait_pose_adopted);
  EXPECT_EQ(rig.ctrl.GetWaitPoseRefusedCount(), 1U) << "refused once, on the first readable tick";
  rig.ExpectWaitPoseIs(rig.yaml_wait, "refused → YAML");
  // Not retried every tick: a refusal is one event per activation, like an
  // adoption; a new activation asks again.
  (void)rig.ctrl.Compute(MakeState(0.3, kP1bHandChannels));
  EXPECT_EQ(rig.ctrl.GetWaitPoseRefusedCount(), 1U);
  ASSERT_EQ(rig.ctrl.on_deactivate(rig.prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(rig.ctrl.on_activate(rig.prev), DemoCatchingController::CallbackReturn::SUCCESS);
  (void)rig.ctrl.Compute(MakeState(0.3, kP1bHandChannels));
  EXPECT_EQ(rig.ctrl.GetWaitPoseRefusedCount(), 2U);
  ASSERT_EQ(rig.ctrl.on_deactivate(rig.prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(rig.ctrl.on_cleanup(rig.prev), DemoCatchingController::CallbackReturn::SUCCESS);
}

TEST(DemoCatchingWaitPose, TheYamlSourceIsTheDefaultAndNeverAdopts) {
  WaitPoseRig rig(nullptr, "catching_wait_pose_yaml");
  rig.ConfigureAndActivate();
  EXPECT_EQ(rig.handle->get_parameter("planner.wait_pose_source").as_string(), "yaml");
  const ControllerState first = MakeState(0.3, kP1bHandChannels);
  (void)rig.ctrl.Compute(first);
  EXPECT_FALSE(rig.ctrl.IsWaitPoseAdoptedForTesting());
  EXPECT_FALSE(rig.ctrl.GetLastTickRecord().wait_pose_adopted);
  EXPECT_FALSE(rig.ctrl.GetPlannerRtState().wait_pose_adopted);
  rig.ExpectWaitPoseIs(rig.yaml_wait, "YAML");
  EXPECT_EQ(rig.ctrl.GetWaitPoseRefusedCount(), 0U);
  ASSERT_EQ(rig.ctrl.on_deactivate(rig.prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(rig.ctrl.on_cleanup(rig.prev), DemoCatchingController::CallbackReturn::SUCCESS);
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
  (void)ctrl.Compute(MakeState(0.0));
  const std::uint64_t resets_before = ctrl.GetRtResetCount();

  ctrl.TriggerEstop();
  ctrl.ClearEstop();
  ctrl.ResetFault();
  EXPECT_EQ(ctrl.GetRtResetCount(), resets_before)
      << "a hook performed a reset instead of requesting one";

  (void)ctrl.Compute(MakeState(0.1));
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
  (void)ctrl.Compute(MakeState(0.0));

  ctrl.TriggerEstop();
  (void)ctrl.Compute(MakeState(0.2));  // the tick that services the trigger
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
  (void)ctrl.Compute(MakeState(0.0));
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
  (void)ctrl.Compute(MakeState(0.0));

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

TEST(DemoCatchingEstop, AStepIssuedDuringAStopNeverReachesTheHand) {
  // The second layer on the target lane. CM substitutes its own hold for this
  // controller's whole output while the global latch is up, so this cannot be
  // observed on an actuator today — which is the point: the layer doing the
  // work belongs to another component, and this asserts the one this
  // controller owns. It is also what makes a step issued mid-stop not arrive
  // LATE, once the stop clears.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  (void)ctrl.Compute(MakeState(0.0));

  ctrl.TriggerEstop();
  (void)ctrl.Compute(MakeState(0.0));  // services the trigger

  // Now a step arrives while the stop is up, on a tick that performs no reset.
  const std::array<double, kHandDof> step{0.4, 0.4, 0.4, 0.4};
  ctrl.SetDeviceTarget(kCatchingHandDeviceIdx, step);
  const ControllerState held = MakeState(0.0);
  const ControllerOutput during = ctrl.Compute(held);
  ASSERT_EQ(during.devices[1].num_channels, kHandDof);
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(during.devices[1].commands[idx], held.devices[1].positions[idx])
        << "hand joint " << i << " moved on a stopped tick";
  }

  // And it does not arrive after the clear either: the stop discarded it.
  ctrl.ClearEstop();
  const ControllerState after = MakeState(0.0);
  const ControllerOutput out = ctrl.Compute(after);
  for (int i = 0; i < kHandDof; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_EQ(out.devices[1].commands[idx], after.devices[1].positions[idx])
        << "hand joint " << i << " took a step issued during the stop, one clear later";
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
  (void)ctrl.Compute(MakeState(0.0));
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
    (void)ctrl.Compute(MakeState(0.01 * static_cast<double>(i)));
  }
  EXPECT_EQ(ctrl.GetMode(), rtc::catching::Mode::kIdle);
  EXPECT_EQ(ctrl.GetLastReason(), rtc::catching::Reason::kParamsTbd);
}

// ── 9. The per-tick record (S5.4, D-20 + PROC-7) ────────────────────────────
//
// PROC-7 says every tick publishes a body — including the ones that decide to
// do nothing — and that a block the tick did not compute is CLEARED rather
// than left holding the previous tick's value. The mechanism is that the
// record is default-constructed at the top of Compute(), so these tests are
// written to fail if that line goes away: each one drives a tick that fills a
// block and then a tick that does not, and asserts the second one is empty.
//
// THE IDENTITY FIELDS ARE THE POINT of the "this tick's body" naming (the
// EstopTickPublishesThisTicksBody precedent). A publisher that skipped a tick
// and a publisher that republished the previous tick's numbers are
// indistinguishable downstream unless the row says WHICH tick it is.

/// A state whose iteration and session time move, so a record that belonged to
/// a different tick is visible rather than plausible.
ControllerState MakeStateAt(std::uint64_t tick, double bias = 0.0) {
  ControllerState state = MakeState(bias);
  state.iteration = tick;
  state.t_relative_s = static_cast<double>(tick) * state.dt;
  return state;
}

TEST(DemoCatchingRecord, EveryTickPublishesItsOwnBody) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  for (std::uint64_t tick = 1; tick <= 20; ++tick) {
    const ControllerState state = MakeStateAt(tick, 0.001 * static_cast<double>(tick));
    (void)ctrl.Compute(state);
    const auto rec = ctrl.GetLastTickRecord();
    ASSERT_EQ(rec.tick, tick) << "the record is not from this tick";
    EXPECT_DOUBLE_EQ(rec.t_relative_s, state.t_relative_s);
    EXPECT_EQ(rec.mode, static_cast<std::uint8_t>(ctrl.GetMode()));
    EXPECT_EQ(rec.reason, static_cast<std::uint8_t>(ctrl.GetLastReason()));
  }
}

TEST(DemoCatchingRecord, EstopTickPublishesThisTicksBody) {
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  (void)ctrl.Compute(MakeStateAt(1));
  ASSERT_FALSE(ctrl.GetLastTickRecord().estop_active) << "precondition: not stopped";

  ctrl.TriggerEstop();
  (void)ctrl.Compute(MakeStateAt(2));
  const auto rec = ctrl.GetLastTickRecord();
  EXPECT_EQ(rec.tick, 2U) << "the stop tick did not publish a body of its own";
  EXPECT_TRUE(rec.estop_active);
  EXPECT_EQ(rec.reason, static_cast<std::uint8_t>(rtc::catching::Reason::kEstop));
  EXPECT_FALSE(rec.armed) << "the tick must lower the arm latch on a stop (P-1 (c))";
}

TEST(DemoCatchingRecord, AStaleInputTickPublishesThisTicksBody) {
  // No vision has ever arrived, which is the shape every bring-up starts in
  // and the one a reader is most likely to mistake for "the publisher is
  // late". The row has to say the snapshot is stale AND be this tick's.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  (void)ctrl.Compute(MakeStateAt(7));
  const auto rec = ctrl.GetLastTickRecord();
  EXPECT_EQ(rec.tick, 7U);
  EXPECT_TRUE(rec.input_stale);
  EXPECT_FALSE(rec.input_valid);
  EXPECT_EQ(rec.input_n, 0);
  EXPECT_EQ(rec.input_generation, 0U);
}

TEST(DemoCatchingRecord, ANoPlanTickCarriesAnEmptyPlanBlock) {
  // There is no planner before S6 and no oracle in this profile, so the plan
  // block must be empty on every tick — not merely absent from the topic.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  for (std::uint64_t tick = 1; tick <= 5; ++tick) {
    (void)ctrl.Compute(MakeStateAt(tick));
    const auto rec = ctrl.GetLastTickRecord();
    ASSERT_FALSE(rec.plan_valid);
    EXPECT_DOUBLE_EQ(rec.plan_p_c[0], 0.0);
    EXPECT_DOUBLE_EQ(rec.plan_gamma_f, 0.0);
    EXPECT_EQ(rec.plan_id, 0U);
  }
}

TEST(DemoCatchingRecord, AHoldingTickStillCarriesTheArmCommandAndTheMeasurement) {
  // A hold IS a command. A reader that saw an empty q_cmd here could not tell
  // a held arm from a tick that produced no command at all.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  const ControllerState state = MakeStateAt(3, 0.25);
  (void)ctrl.Compute(state);
  const auto rec = ctrl.GetLastTickRecord();
  ASSERT_EQ(rec.num_arm_joints, kArmDof);
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(rec.q_meas[u], state.devices[0].positions[u]);
  }
}

TEST(DemoCatchingRecord, AHoldingTicksCommandColumnIsTheCommandThatWentOut) {
  // The row above proves the MEASUREMENT is carried. This one is about the
  // COMMAND, and it is the half that was wrong: `q_cmd` was written as 0.0
  // whenever the law had not taken over, while `WriteDeviceCommand` was
  // really sending the hold latch. Every holding row therefore claimed the
  // arm was commanded to the origin, and ‖q_meas − q_cmd‖ computed offline
  // from these two columns showed a multi-radian error that does not exist
  // on the wire (2026-09-23 review). The column has to equal the output.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  const ControllerState state = MakeStateAt(3, 0.25);
  const ControllerOutput out = ctrl.Compute(state);
  const auto rec = ctrl.GetLastTickRecord();
  ASSERT_EQ(rec.num_arm_joints, kArmDof);
  ASSERT_GE(out.devices[0].num_channels, kArmDof) << "precondition: the arm was commanded";
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(rec.q_cmd[u], out.devices[0].commands[u])
        << "joint " << i << ": the record and the wire disagree about the command";
    EXPECT_NE(rec.q_cmd[u], 0.0) << "joint " << i << " still reports the old 0.0 placeholder";
  }
}

TEST(DemoCatchingRecord, ATickWithNoCommandOnTheWireReportsNaNRatherThanZero) {
  // Before the hold latch is set the output is SILENCED — the drive keeps its
  // own setpoint and no command exists. 0.0 would be a number a reader can
  // average; NaN is the only encoding that says "there was nothing here".
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ControllerState state = MakeStateAt(1);
  // An unreadable arm is what keeps the latch unset: it only latches on a
  // readable tick, precisely so it never freezes "go to the origin".
  state.devices[0].valid = false;
  const ControllerOutput out = ctrl.Compute(state);
  ASSERT_EQ(out.devices[0].num_channels, 0) << "precondition: the arm output is silenced";
  const auto rec = ctrl.GetLastTickRecord();
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_TRUE(std::isnan(rec.q_cmd[static_cast<std::size_t>(i)]))
        << "joint " << i << " reported a command on a tick that sent none";
  }
}

TEST(DemoCatchingRecord, AVisionLaneThatNeverReceivedReportsTheNeverSentinel) {
  // `traj_recv_ns` is 0 before the first prediction, so an age measured
  // against it is the steady clock's own origin distance. Measured on a real
  // session 2026-09-23: 373966.66 s on all 28476 rows before the first
  // prediction — a number that is unbounded, plausible, and indistinguishable
  // from an age to anything that thresholds or averages it. -1 is the same
  // "never" the fingertip lane already uses.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  (void)ctrl.Compute(MakeStateAt(5));
  const auto rec = ctrl.GetLastTickRecord();
  ASSERT_FALSE(rec.input_valid) << "precondition: no prediction has arrived";
  EXPECT_LT(rec.input_age_s, 0.0) << "an unreceived lane reported an age of " << rec.input_age_s
                                  << " s instead of the sentinel";
}

TEST(DemoCatchingRecord, TheLawBlocksStayEmptyWhileTheLawIsNotWired) {
  // This binding runs with no URDF, so CLIK never configures. The record must
  // say so rather than publish a zeroed solve that reads as "status 0".
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  (void)ctrl.Compute(MakeStateAt(1));
  const auto rec = ctrl.GetLastTickRecord();
  EXPECT_FALSE(rec.law_enabled);
  EXPECT_FALSE(rec.clik_ran);
  EXPECT_FALSE(rec.ref_valid);
  EXPECT_EQ(rec.clik_status, -1) << "-1 is 'not solved'; 0 is ProxQP's SOLVED";
  EXPECT_DOUBLE_EQ(rec.track_err_rad, 0.0);
}

TEST(DemoCatchingRecord, AFingertipLaneThatNeverReportedIsNeverReadAsFresh) {
  // The message contract says `tip_age_s < 0` means "never received" and 0
  // means "arrived this instant". A POD's zero-init gives the second for free,
  // and the wire arrays are sized from the device's SENSOR NAMES while the
  // fill loop is bounded by `num_inference_groups` — so a backend that reports
  // no groups (mujoco with no `fingertip_wrench_topics`, which is the shipped
  // sim) would publish every configured fingertip as fresh.
  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ControllerState state = MakeStateAt(1);
  ASSERT_EQ(state.devices[kCatchingHandDeviceIdx].num_inference_groups, 0)
      << "precondition: this fixture's hand reports no inference groups";
  (void)ctrl.Compute(state);
  const auto rec = ctrl.GetLastTickRecord();
  for (std::size_t i = 0; i < integrated_bringup::CatchingDiagLogPod::kMaxTips; ++i) {
    EXPECT_LT(rec.tip_age_s[i], 0.0) << "slot " << i << " reads as just-received";
  }
}

TEST_F(CatchingVisionTest, ReActivationForgetsThePreviousActivationsIngressDiagnostics) {
  // `traj_input_.Reset()` puts the jump back to "not compared" and forgets the
  // accepted sequence, but the BOX the publish thread reads is a separate
  // object — so without a re-Store the topic keeps reporting a jump measured
  // against a trajectory this activation has no reason to think is relevant.
  BringUpWithVision();
  PublishPrediction(/*sequence=*/1);
  Spin();
  ASSERT_EQ(ctrl_.GetIngressSnapshot().accept_count, 1U);
  ASSERT_EQ(ctrl_.GetIngressSnapshot().diag.accepted_sequence, 1U);

  const rclcpp_lifecycle::State prev;
  ASSERT_EQ(ctrl_.on_deactivate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl_.on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);

  const auto after = ctrl_.GetIngressSnapshot();
  EXPECT_EQ(after.diag.accepted_sequence, 0U)
      << "the previous activation's accepted sequence is still on the wire";
  EXPECT_LT(after.diag.jump_m, 0.0) << "a jump from the previous activation is still published";
  // The CUMULATIVE counters are NOT reset, and that is the contract: they are
  // lifetime totals like the CSV drop counts, and an operator reading "how
  // many messages has this lane refused" wants the run, not the activation.
  // Asserting it here stops a future reset from being added quietly.
  EXPECT_EQ(after.accept_count, 1U) << "the lifetime accept counter was reset";
}

TEST_F(CatchingVisionTest, AnAcceptedPredictionShowsUpInThisTicksInputBlock) {
  BringUpWithVision();
  (void)ctrl_.Compute(MakeStateAt(1));
  PublishPrediction(/*sequence=*/11, /*generation=*/77);
  Spin();
  ASSERT_EQ(ctrl_.GetTrajInput().AcceptCount(), 1U);

  (void)ctrl_.Compute(MakeStateAt(2, 0.01));
  const auto rec = ctrl_.GetLastTickRecord();
  EXPECT_EQ(rec.tick, 2U);
  EXPECT_TRUE(rec.input_valid);
  EXPECT_FALSE(rec.input_stale);
  EXPECT_TRUE(rec.input_new);
  EXPECT_EQ(rec.input_generation, 77U);
  EXPECT_EQ(rec.input_snapshot_sequence, 11U);
  EXPECT_EQ(rec.input_n, 8);
  // The horizon the message actually carries, not the configured minimum: a
  // publisher one sample short has to look different from one that is not.
  EXPECT_GT(rec.input_horizon_s, 0.0);
  EXPECT_GE(rec.input_age_s, 0.0);
}

TEST_F(CatchingVisionTest, TheIngressCountersReachThePublishSideWhetherOrNotAMessageIsAccepted) {
  // A lane being REFUSED and a lane being SILENT are the same thing from the
  // RT side, so these counters are the only way to tell them apart — and they
  // are published from the subscription thread through their own SeqLock
  // because reading the ingress members across threads is a race.
  BringUpWithVision();
  ASSERT_EQ(ctrl_.GetIngressSnapshot().accept_count, 0U);

  // A message from the wrong frame: decoded far enough to be judged, then
  // refused. The counters must move even though nothing was stored.
  integrated_bringup::testing::CloudSpec spec;
  spec.n = 8;
  spec.sequence = 1;
  spec.frame_id = "not_the_configured_frame";
  auto bad = integrated_bringup::testing::MakeCloud(spec);
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const std::int64_t wall = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  bad.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
  bad.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
  pub_->publish(bad);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
  while (std::chrono::steady_clock::now() < deadline &&
         ctrl_.GetIngressSnapshot()
                 .rejects[static_cast<std::size_t>(integrated_bringup::CloudReject::kFrameId)] ==
             0U) {
    executor_->spin_some(std::chrono::milliseconds(20));
  }
  const auto refused = ctrl_.GetIngressSnapshot();
  EXPECT_EQ(refused.accept_count, 0U);
  EXPECT_EQ(refused.rejects[static_cast<std::size_t>(integrated_bringup::CloudReject::kFrameId)],
            1U)
      << "a refused message left no trace on the publish side";

  PublishPrediction(/*sequence=*/2);
  Spin();
  const auto accepted = ctrl_.GetIngressSnapshot();
  EXPECT_EQ(accepted.accept_count, 1U);
  EXPECT_EQ(accepted.diag.accepted_sequence, 2U);
}

// ── pre-S10 R4: an empty cloud is the publisher's idle state ────────────────

/// Every WARN that reaches the log, for the one test below that asserts on a
/// line NOT being written. The handler is process-global; the test restores it.
class WarnSink {
 public:
  WarnSink() : previous_(rcutils_logging_get_output_handler()) {
    {
      const std::lock_guard<std::mutex> lock(Mutex());
      Lines().clear();
    }
    rcutils_logging_set_output_handler(&WarnSink::Handler);
  }

  ~WarnSink() { rcutils_logging_set_output_handler(previous_); }

  WarnSink(const WarnSink&) = delete;
  WarnSink& operator=(const WarnSink&) = delete;

  [[nodiscard]] static std::vector<std::string> Containing(const std::string& needle) {
    const std::lock_guard<std::mutex> lock(Mutex());
    std::vector<std::string> hits;
    for (const auto& line : Lines()) {
      if (line.find(needle) != std::string::npos) {
        hits.push_back(line);
      }
    }
    return hits;
  }

 private:
  static void Handler(const rcutils_log_location_t* /*location*/, int severity,
                      const char* /*name*/, rcutils_time_point_value_t /*timestamp*/,
                      const char* format, va_list* args) {
    if (severity != RCUTILS_LOG_SEVERITY_WARN) {
      return;
    }
    // Value-initialised: on an encoding failure vsnprintf need not terminate.
    char buffer[1024]{};
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    const std::lock_guard<std::mutex> lock(Mutex());
    Lines().emplace_back(buffer);
  }

  static std::vector<std::string>& Lines() {
    static std::vector<std::string> lines;
    return lines;
  }

  static std::mutex& Mutex() {
    static std::mutex m;
    return m;
  }

  rcutils_logging_output_handler_t previous_;
};

TEST_F(CatchingVisionTest, AnEmptyCloudIsCountedAsNoTrackAndWarnsNobody) {
  using integrated_bringup::CloudReject;
  BringUpWithVision();

  const auto stamp_now = [](sensor_msgs::msg::PointCloud2& msg) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const std::int64_t wall = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
  };
  const auto spin_until_counted = [this](CloudReject which) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
    while (std::chrono::steady_clock::now() < deadline &&
           ctrl_.GetIngressSnapshot().rejects[static_cast<std::size_t>(which)] == 0U) {
      executor_->spin_some(std::chrono::milliseconds(20));
    }
    return ctrl_.GetIngressSnapshot().rejects[static_cast<std::size_t>(which)];
  };

  // The refusal warning is throttled per call site, and an earlier test in
  // this binary may have fired it. Wait the window out, or "no line" below
  // would be the throttle's doing and pass with the warning still in place.
  std::this_thread::sleep_for(
      std::chrono::milliseconds(::integrated_bringup::logging::kThrottleSlowMs + 100));
  const WarnSink sink;

  integrated_bringup::testing::CloudSpec idle;
  idle.n = 0;
  auto empty = integrated_bringup::testing::MakeCloud(idle);
  stamp_now(empty);
  pub_->publish(empty);
  ASSERT_EQ(spin_until_counted(CloudReject::kNoTrack), 1U) << "the empty cloud was not counted";

  const auto counted = ctrl_.GetIngressSnapshot();
  EXPECT_EQ(counted.accept_count, 0U);
  EXPECT_EQ(counted.rejects[static_cast<std::size_t>(CloudReject::kShape)], 0U);
  EXPECT_TRUE(WarnSink::Containing("vision message refused").empty())
      << WarnSink::Containing("vision message refused").front();

  // Positive control, same sink and same call site: a DEFECT does warn. It
  // also shows the throttle was open when the empty cloud went through.
  integrated_bringup::testing::CloudSpec wrong;
  wrong.n = 8;
  wrong.frame_id = "not_the_configured_frame";
  auto bad = integrated_bringup::testing::MakeCloud(wrong);
  stamp_now(bad);
  pub_->publish(bad);
  ASSERT_EQ(spin_until_counted(CloudReject::kFrameId), 1U);
  EXPECT_EQ(WarnSink::Containing("vision message refused (frame_id)").size(), 1U);
}

// ── 10. catching_diag.csv (L8 §5.2) ─────────────────────────────────────────
//
// A BOUND HANDLE AND A REAL FILE. These fixtures bring the controller up
// through LoadConfig, which leaves every production log handle unbound — and a
// row assertion written against that state passes with the push deleted
// outright (#424). So the channel is registered here through the SHARED
// registration helper (which puts its `catching_diag_enabled` gate under test
// too) and read back off disk.

struct CatchingDiagChannel {
  rtc::LogHandle<integrated_bringup::CatchingDiagLogPod> handle;
  std::filesystem::path path;
};

struct CatchingLogEntry {
  std::string msg_type;
  std::string instance;
};

CatchingDiagChannel BindCatchingDiagChannel(rtc::ControllerLogSet& log_set,
                                            const std::vector<std::string>& arm_joints,
                                            const std::vector<std::string>& tips) {
  const std::vector<CatchingLogEntry> entries{
      {std::string(integrated_bringup::kCatchingDiagLogMsgType),
       std::string(integrated_bringup::kCatchingDiagLogInstance)}};
  integrated_bringup::LogRegistrationContext ctx{
      .logger = rclcpp::get_logger("catching_diag_test"),
      .log_set = log_set,
      .catching_diag_enabled = true,
      .catching_diag_arm_joint_names = arm_joints,
      .catching_diag_tip_names = tips,
  };
  auto reg = integrated_bringup::RegisterControllerLogs(entries, ctx);
  EXPECT_EQ(reg.status, integrated_bringup::LogRegistrationStatus::kSuccess);
  CatchingDiagChannel out;
  out.handle = std::move(reg.handles.catching_diag);
  for (const auto& ch : log_set.Channels()) {
    if (ch.first == integrated_bringup::kCatchingDiagLogInstance) {
      out.path = ch.second;
    }
  }
  return out;
}

const std::vector<std::string> kArmJointNames{"a0", "a1", "a2", "a3", "a4", "a5"};
const std::vector<std::string> kTipNames{"thumb", "index"};

TEST(CatchingDiagLog, EveryTickIsARowAndTheHeaderMatchesTheRowWidth) {
  integrated_bringup::testfx::ScopedSessionDir session{"catching_diag"};
  rtc::ControllerLogSet log_set{"catching_diag_rows"};
  auto ch = BindCatchingDiagChannel(log_set, kArmJointNames, kTipNames);
  ASSERT_TRUE(ch.handle) << "the catching_diag channel did not bind";

  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.SetCatchingDiagLogHandleForTesting(std::move(ch.handle));

  constexpr int kTicks = 37;
  for (std::uint64_t tick = 1; tick <= kTicks; ++tick) {
    (void)ctrl.Compute(MakeStateAt(tick, 0.001 * static_cast<double>(tick)));
  }
  log_set.DrainAll();
  EXPECT_EQ(log_set.TotalDropCount(), 0U) << "the ring overflowed; rows are missing";

  const auto csv = integrated_bringup::testfx::ReadCsv(ch.path);
  ASSERT_EQ(csv.rows.size(), static_cast<std::size_t>(kTicks))
      << "a tick did not produce a row (PROC-7)";
  for (std::size_t r = 0; r < csv.rows.size(); ++r) {
    ASSERT_EQ(csv.rows[r].size(), csv.header.size())
        << "row " << r << " does not line up with the header";
  }
  // The tick column is what makes a gap readable as a dropped row rather than
  // as a tick the controller chose not to log.
  for (std::size_t r = 0; r < csv.rows.size(); ++r) {
    EXPECT_DOUBLE_EQ(csv.At(r, "tick"), static_cast<double>(r + 1));
  }
}

TEST(CatchingDiagLog, TheHeaderNamesTheJointsAndTipsItsColumnsAreIn) {
  // A stored file has to decode without that run's YAML (#234 P-14). Column
  // POSITION cannot carry that: the arm width comes from the robot's config.
  integrated_bringup::testfx::ScopedSessionDir session{"catching_diag"};
  rtc::ControllerLogSet log_set{"catching_diag_names"};
  auto ch = BindCatchingDiagChannel(log_set, kArmJointNames, kTipNames);
  ASSERT_TRUE(ch.handle);

  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.SetCatchingDiagLogHandleForTesting(std::move(ch.handle));
  (void)ctrl.Compute(MakeStateAt(1));
  log_set.DrainAll();

  const auto csv = integrated_bringup::testfx::ReadCsv(ch.path);
  for (const auto& j : kArmJointNames) {
    EXPECT_TRUE(csv.Has("q_cmd_" + j)) << "no commanded column for " << j;
    EXPECT_TRUE(csv.Has("q_meas_" + j)) << "no measured column for " << j;
  }
  for (const auto& t : kTipNames) {
    EXPECT_TRUE(csv.Has("tip_force_" + t));
    EXPECT_TRUE(csv.Has("tip_age_s_" + t));
  }
  EXPECT_FALSE(csv.Has("q_cmd_a6")) << "a column exists for a joint the device does not have";
}

TEST(CatchingDiagLog, TheDecelBlockIsWrittenAsNumbersAndIsZeroWhenTheLaneDidNotRun) {
  // E1-F05 (#631): the tick record's decel block reaches the file. This
  // controller runs `closed_form`, so the lane never judges a segment and the
  // block is the fresh POD's — which is what a reader sees on every tick of a
  // closed_form log. `decel_refusal` is a std::uint8_t: streamed without a
  // cast it would be a raw NUL byte, not "0", and the width check above would
  // not notice.
  integrated_bringup::testfx::ScopedSessionDir session{"catching_diag"};
  rtc::ControllerLogSet log_set{"catching_diag_decel"};
  auto ch = BindCatchingDiagChannel(log_set, kArmJointNames, kTipNames);
  ASSERT_TRUE(ch.handle);

  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.SetCatchingDiagLogHandleForTesting(std::move(ch.handle));
  (void)ctrl.Compute(MakeStateAt(1));
  log_set.DrainAll();

  const auto csv = integrated_bringup::testfx::ReadCsv(ch.path);
  ASSERT_EQ(csv.rows.size(), 1U);
  for (const char* col :
       {"decel_judged", "decel_refusal", "decel_event", "decel_following", "decel_seq", "decel_k0",
        "decel_held", "decel_p_d_x", "decel_p_d_y", "decel_p_d_z", "decel_v_ff_x", "decel_v_ff_y",
        "decel_v_ff_z", "decel_rho", "decel_dq_max", "decel_dqd_max"}) {
    ASSERT_TRUE(csv.Has(col)) << col;
    EXPECT_DOUBLE_EQ(csv.At(0, col), 0.0) << col;
  }
  EXPECT_EQ(csv.Text(0, "decel_refusal"), "0");
  EXPECT_EQ(csv.Text(0, "decel_event"), "0");
  ASSERT_TRUE(csv.Has("decel_gate_joint"));
  EXPECT_DOUBLE_EQ(csv.At(0, "decel_gate_joint"), -1.0) << "no gate was judged";
}

TEST(CatchingDiagLog, AStoppedTickIsARowLikeAnyOtherAndSaysSo) {
  integrated_bringup::testfx::ScopedSessionDir session{"catching_diag"};
  rtc::ControllerLogSet log_set{"catching_diag_estop"};
  auto ch = BindCatchingDiagChannel(log_set, kArmJointNames, kTipNames);
  ASSERT_TRUE(ch.handle);

  DemoCatchingController ctrl{""};
  BringUp(ctrl);
  ctrl.SetCatchingDiagLogHandleForTesting(std::move(ch.handle));
  (void)ctrl.Compute(MakeStateAt(1));
  ctrl.TriggerEstop();
  (void)ctrl.Compute(MakeStateAt(2));
  (void)ctrl.Compute(MakeStateAt(3));
  log_set.DrainAll();

  const auto csv = integrated_bringup::testfx::ReadCsv(ch.path);
  ASSERT_EQ(csv.rows.size(), 3U) << "the stopped ticks are missing from the file";
  EXPECT_DOUBLE_EQ(csv.At(0, "estop_active"), 0.0);
  EXPECT_DOUBLE_EQ(csv.At(1, "estop_active"), 1.0);
  // Both spellings on purpose: the raw value survives an enum gaining a
  // member, the name makes the file readable without this build's header.
  EXPECT_EQ(csv.Text(1, "reason_name"), "estop");
  EXPECT_DOUBLE_EQ(csv.At(1, "reason"), static_cast<double>(rtc::catching::Reason::kEstop));
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
