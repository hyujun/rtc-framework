// ── DemoJointController on THREE device groups: a body and two hands (#715) ──
//
// The controller was written for "one body + one hand", and a third `topics:`
// group used to fall through three holes with no symptom: its RobotTarget
// subscription was skipped (owned_topics bound 2), its device slot was never
// commanded (every hand output wrote devices[1] alone) and its E-STOP hold was
// never emitted — so a simulator that waits for every group's command never
// left its first step. What this file pins, on a synthetic fixture:
//   - on_configure subscribes a target for EVERY group, or refuses the list;
//   - after self-init, every device is commanded at its own width and held at
//     its own measurement;
//   - each hand follows its OWN joint goal and leaves the other where it was;
//   - the E-STOP tick holds both hands at their measurement while the body ramps;
//   - a fourth group is refused at configure (one body + kDemoJointMaxHands).
//
// Fingertip FK, TF slots, GraspState and contact-stop stay on the first hand
// by design (demo_joint_controller.hpp, kDemoJointMaxHands) — none of that is
// asserted on the second hand here. The late / narrow second-hand seed cases
// live with the other seed tests in test_demo_joint_controller.cpp.
//
// The fixture (rtc_urdf_bridge/test/urdf/trunk_two_hands.urdf) is synthetic
// and owned by this repository: a body tree with two branches and a 3-finger
// hand on each, the two hands different in every finger.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "test_urdf_path.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;
using integrated_bringup::DemoJointController;
using rtc::ControllerOutput;
using rtc::ControllerState;

constexpr int kBodyDof = 5;
constexpr int kHandDof = 3;
constexpr double kDt = 0.002;
constexpr double kBodyMaxVelocity = 5.0;

const char* const kRoot = "pelvis";
const char* const kBodyName = "body";
const char* const kRightName = "right_hand";
const char* const kLeftName = "left_hand";

const std::vector<std::string> kBodyJoints = {"waist_yaw", "left_shoulder", "left_elbow",
                                              "right_shoulder", "right_elbow"};
const std::vector<std::string> kRightJoints = {"right_finger_a", "right_finger_b",
                                               "right_finger_c"};
const std::vector<std::string> kLeftJoints = {"left_finger_a", "left_finger_b", "left_finger_c"};

// Every joint off zero, no two equal, and the two hands at different values:
// "held at its own measurement" is then distinguishable from zero, from the
// other hand's values and from any other channel of the same hand.
const std::vector<double> kBodyQ = {0.35, 0.40, -0.60, -0.50, 0.70};
const std::vector<double> kRightQ = {0.30, -0.40, 0.50};
const std::vector<double> kLeftQ = {-0.25, 0.20, 0.45};

std::string Yaml(int hand_groups) {
  std::string yaml = R"(arm_dof: 5
robot_trajectory_speed: 2.0
hand_trajectory_speed: 3.0
robot_max_traj_velocity: 3.14
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  body:
    subscribe:
      - topic: "body/joint_goal"
        role: "target"
    publish:
      - topic: "transforms"
        role: "robot_transforms"
)";
  const std::array<const char*, 3> hands = {kRightName, kLeftName, "extra_hand"};
  for (int h = 0; h < hand_groups; ++h) {
    const std::string name = hands[static_cast<std::size_t>(h)];
    yaml += "  " + name + ":\n    subscribe:\n      - topic: \"" + name +
            "/joint_goal\"\n        role: \"target\"\n";
  }
  return yaml;
}

rtc::DeviceNameConfig BodyDevice() {
  rtc::DeviceNameConfig body;
  body.device_name = kBodyName;
  body.joint_state_names = kBodyJoints;
  rtc::DeviceUrdfConfig urdf;
  urdf.root_link = kRoot;  // a tree: no tip of its own; the controller reads
  body.urdf = urdf;        // the arm tip off the first hand's mount
  rtc::DeviceJointLimits limits;
  limits.max_velocity.assign(kBodyJoints.size(), kBodyMaxVelocity);
  limits.position_lower.assign(kBodyJoints.size(), -2.0);
  limits.position_upper.assign(kBodyJoints.size(), 2.0);
  body.joint_limits = limits;
  return body;
}

rtc::DeviceNameConfig HandDevice(const char* name, const char* mount,
                                 const std::vector<std::string>& joints) {
  rtc::DeviceNameConfig hand;
  hand.device_name = name;
  hand.joint_state_names = joints;
  rtc::DeviceUrdfConfig urdf;
  urdf.root_link = mount;
  hand.urdf = urdf;
  return hand;
}

rub::ModelConfig MakeModelConfig() {
  rub::ModelConfig cfg;
  cfg.urdf_path = rtc::test::TestUrdfPath("trunk_two_hands.urdf");
  cfg.root_joint_type = "fixed";
  cfg.tree_models.push_back({kBodyName, kRoot, {"left_forearm", "right_forearm"}});
  cfg.tree_models.push_back(
      {kRightName, "right_palm", {"right_tip_a", "right_tip_b", "right_tip_c"}});
  cfg.tree_models.push_back({kLeftName, "left_palm", {"left_tip_a", "left_tip_b", "left_tip_c"}});
  return cfg;
}

struct Rig {
  const rub::ModelConfig* model{nullptr};
  std::shared_ptr<rub::PinocchioModelBuilder> builder;
  std::map<std::string, rtc::DeviceNameConfig> devices;
  std::string yaml;
};

/// Body + right hand + left hand, in `topics:` order (device 0, 1, 2).
Rig ThreeGroupRig() {
  static const rub::ModelConfig cfg = MakeModelConfig();
  static const auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);
  Rig rig;
  rig.model = &cfg;
  rig.builder = builder;
  rig.devices[kBodyName] = BodyDevice();
  rig.devices[kRightName] = HandDevice(kRightName, "right_palm", kRightJoints);
  rig.devices[kLeftName] = HandDevice(kLeftName, "left_palm", kLeftJoints);
  rig.yaml = Yaml(/*hand_groups=*/2);
  return rig;
}

/// The same robot with a fourth `topics:` group — one hand more than the
/// controller has lanes for.
Rig FourGroupRig() {
  Rig rig = ThreeGroupRig();
  rig.devices["extra_hand"] = HandDevice("extra_hand", "left_palm", kLeftJoints);
  rig.yaml = Yaml(/*hand_groups=*/3);
  return rig;
}

ControllerState MakeState() {
  ControllerState state{};
  state.num_devices = 3;
  state.dt = kDt;
  state.iteration = 1;
  const std::array<const std::vector<double>*, 3> q = {&kBodyQ, &kRightQ, &kLeftQ};
  for (std::size_t d = 0; d < q.size(); ++d) {
    auto& dev = state.devices[d];
    dev.num_channels = static_cast<int>(q[d]->size());
    dev.valid = true;
    dev.hole_mask = 0;
    for (std::size_t i = 0; i < q[d]->size(); ++i) {
      dev.positions[i] = (*q[d])[i];
    }
  }
  return state;
}

std::unique_ptr<DemoJointController> Make(const Rig& rig) {
  auto ctrl = std::make_unique<DemoJointController>("");
  ctrl->SetSystemModelConfig(*rig.model);
  ctrl->SetSharedModelBuilder(rig.builder);
  ctrl->SetControlRate(1.0 / kDt);
  return ctrl;
}

/// Config, then device configs — enough for Compute, with no node involved.
std::unique_ptr<DemoJointController> BringUp(const Rig& rig) {
  auto ctrl = Make(rig);
  ctrl->LoadConfig(YAML::Load(rig.yaml));
  ctrl->SetDeviceNameConfigs(rig.devices);
  return ctrl;
}

ControllerOutput RunTicks(DemoJointController& ctrl, ControllerState& state, int n) {
  ControllerOutput out = ctrl.Compute(state);
  for (int i = 0; i < n; ++i) {
    state.iteration += 1;
    out = ctrl.Compute(state);
  }
  return out;
}

void ExpectHeldAt(const rtc::DeviceOutput& out, const std::vector<double>& q, const char* what) {
  ASSERT_EQ(out.num_channels, static_cast<int>(q.size())) << what;
  for (std::size_t i = 0; i < q.size(); ++i) {
    EXPECT_NEAR(out.commands[i], q[i], 1e-9) << what << " channel " << i;
  }
}

// ── Canary: the fixture has two different hands ─────────────────────────────

TEST(JointThreeGroups, FixtureHasABodyTreeAndTwoDistinctHands) {
  const Rig rig = ThreeGroupRig();
  const pinocchio::Model& full = *rig.builder->GetFullModel();
  ASSERT_EQ(full.nq, kBodyDof + 2 * kHandDof);
  EXPECT_EQ(rig.builder->GetTreeModel(kBodyName)->nv, kBodyDof);
  EXPECT_EQ(rig.builder->GetTreeModel(kRightName)->nv, kHandDof);
  EXPECT_EQ(rig.builder->GetTreeModel(kLeftName)->nv, kHandDof);
  for (const auto& n : kRightJoints) {
    EXPECT_TRUE(full.existJointName(n)) << n;
  }
  for (const auto& n : kLeftJoints) {
    EXPECT_TRUE(full.existJointName(n)) << n;
  }
}

// ── Every group is commanded, each at its own width and measurement ─────────

TEST(JointThreeGroups, EveryGroupIsHeldAtItsOwnMeasurementAfterSelfInit) {
  const Rig rig = ThreeGroupRig();
  auto ctrl = BringUp(rig);
  ControllerState state = MakeState();
  const ControllerOutput out = RunTicks(*ctrl, state, 40);

  ASSERT_TRUE(out.valid);
  ASSERT_EQ(out.num_devices, 3);
  ExpectHeldAt(out.devices[0], kBodyQ, "body");
  ExpectHeldAt(out.devices[1], kRightQ, "right hand");
  ExpectHeldAt(out.devices[2], kLeftQ, "left hand");
}

// A goal for the second hand moves the second hand and nothing else; a goal
// for the first hand then moves the first and leaves the second where its own
// goal put it. Both lanes run the same law on the same gains.
TEST(JointThreeGroups, EachHandFollowsItsOwnGoalAndLeavesTheOtherAlone) {
  const Rig rig = ThreeGroupRig();
  auto ctrl = BringUp(rig);
  ControllerState state = MakeState();
  (void)RunTicks(*ctrl, state, 3);

  const std::array<double, kHandDof> left_goal = {0.9, -0.8, 1.1};
  ctrl->SetDeviceTarget(2, left_goal);
  // |Δq| ≤ 1.15 rad at hand_trajectory_speed 3 rad/s, max velocity 6.28:
  // duration ≈ 0.38 s, so 0.6 s of ticks lands on the goal.
  ControllerOutput out = RunTicks(*ctrl, state, 300);
  ASSERT_EQ(out.devices[2].num_channels, kHandDof);
  for (std::size_t i = 0; i < left_goal.size(); ++i) {
    EXPECT_NEAR(out.devices[2].commands[i], left_goal[i], 1e-6) << "left channel " << i;
  }
  ExpectHeldAt(out.devices[1], kRightQ, "right hand while the left moves");
  ExpectHeldAt(out.devices[0], kBodyQ, "body while the left hand moves");

  const std::array<double, kHandDof> right_goal = {-0.7, 0.6, -0.9};
  ctrl->SetDeviceTarget(1, right_goal);
  out = RunTicks(*ctrl, state, 300);
  ASSERT_EQ(out.devices[1].num_channels, kHandDof);
  for (std::size_t i = 0; i < right_goal.size(); ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], right_goal[i], 1e-6) << "right channel " << i;
    EXPECT_NEAR(out.devices[2].commands[i], left_goal[i], 1e-6)
        << "left channel " << i << " moved when the RIGHT hand was commanded";
  }
}

// ── E-STOP: both hands hold, the body ramps toward its safe position ────────

TEST(JointThreeGroups, EstopHoldsBothHandsAtMeasuredWhileTheBodyRamps) {
  const Rig rig = ThreeGroupRig();
  auto ctrl = BringUp(rig);
  ControllerState state = MakeState();
  (void)RunTicks(*ctrl, state, 3);

  ctrl->TriggerEstop();
  state.iteration += 1;
  const ControllerOutput out = ctrl->Compute(state);
  ASSERT_TRUE(ctrl->IsEstopped());
  ASSERT_EQ(out.num_devices, 3);

  ExpectHeldAt(out.devices[1], kRightQ, "right hand under E-STOP");
  ExpectHeldAt(out.devices[2], kLeftQ, "left hand under E-STOP");

  // The body ramps toward arm_safe_position (all zero): one step of
  // clamp(q_safe − q, ±max_velocity) · dt — every |q| here is far below the
  // 5 rad/s bound, so the step is exactly −q · dt. A step, not a hold and
  // not a jump.
  ASSERT_EQ(out.devices[0].num_channels, kBodyDof);
  for (std::size_t i = 0; i < kBodyQ.size(); ++i) {
    ASSERT_LT(std::abs(kBodyQ[i]), kBodyMaxVelocity) << "fixture: the ramp must not saturate";
    const double step = out.devices[0].commands[i] - kBodyQ[i];
    EXPECT_NEAR(step, -kBodyQ[i] * kDt, 1e-9) << "body channel " << i;
  }
}

// ── on_configure: one target subscription per group, and the bound ──────────

class JointThreeGroupsLifecycle : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /// The controller manager's bring-up order: PreConfigure (node + LoadConfig),
  /// device configs, on_configure (see test_joint_tree_primary_group.cpp).
  std::unique_ptr<DemoJointController> PreConfigure(const Rig& rig, const char* node_name) {
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, "", opts);
    auto ctrl = Make(rig);
    EXPECT_EQ(ctrl->PreConfigure(node_, YAML::Load(rig.yaml)),
              DemoJointController::CallbackReturn::SUCCESS)
        << node_name;
    ctrl->SetDeviceNameConfigs(rig.devices);
    return ctrl;
  }

  DemoJointController::CallbackReturn Configure(DemoJointController& ctrl, const Rig& rig) {
    return ctrl.on_configure(rclcpp_lifecycle::State{}, node_, YAML::Load(rig.yaml));
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

TEST_F(JointThreeGroupsLifecycle, ConfigureSubscribesATargetForEveryGroup) {
  const Rig rig = ThreeGroupRig();
  auto ctrl = PreConfigure(rig, "three_groups_targets");
  ASSERT_EQ(Configure(*ctrl, rig), DemoJointController::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl->OwnedTargetSubscriptionCountForTesting(), 3U);

  // The configured controller drives all three devices too — the subscription
  // count alone would pass with the output lanes still two wide.
  ControllerState state = MakeState();
  const ControllerOutput out = RunTicks(*ctrl, state, 40);
  ExpectHeldAt(out.devices[0], kBodyQ, "body after on_configure");
  ExpectHeldAt(out.devices[1], kRightQ, "right hand after on_configure");
  ExpectHeldAt(out.devices[2], kLeftQ, "left hand after on_configure");
}

// One body and kDemoJointMaxHands hands is the whole contract: a fourth group
// would be subscribed and then never commanded, which is exactly the silent
// failure this file exists to close. Refused — and the control case right
// above shows the refusal is about the count, not about this fixture.
TEST_F(JointThreeGroupsLifecycle, AFourthGroupRefusesConfigure) {
  static_assert(integrated_bringup::kDemoJointMaxHands == 2,
                "this case names one hand more than the controller has lanes for");
  const Rig rig = FourGroupRig();
  auto ctrl = PreConfigure(rig, "four_groups_refused");
  EXPECT_EQ(Configure(*ctrl, rig), DemoJointController::CallbackReturn::FAILURE);
  EXPECT_EQ(ctrl->OwnedTargetSubscriptionCountForTesting(), 0U)
      << "a refused configure must not leave target subscriptions behind";
}

}  // namespace
