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

TEST_F(CatchingConfigureTest, RefusesARealDriverBackend) {
  // E-8: holding a real arm is an E-STOP-path change pending approval.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("ur_driver_native", "udp_hand_native"));
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::FAILURE);
  EXPECT_EQ(ctrl.GetNonSimGroups().size(), 2U);
}

TEST_F(CatchingConfigureTest, RefusesADeviceThatDeclaresNoBackendAtAll) {
  // Silence does not prove the device is the simulator.
  DemoCatchingController ctrl{""};
  ctrl.SetDeviceNameConfigs(MakeConfigs("mujoco_native", ""));
  const rclcpp_lifecycle::State prev;
  EXPECT_EQ(ctrl.on_configure(prev, node_, YAML::Load(MinimalYaml(true))),
            DemoCatchingController::CallbackReturn::FAILURE);
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

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
