// ── DemoJointController on a primary group that is a kinematic TREE ──────────
//
// The controller was written for "one serial arm + one hand": the arm model is
// a `urdf.sub_models` chain, and its tip is the frame the hand hangs off. A
// humanoid upper body driven as ONE device group breaks both halves — waist +
// two arms is a tree with two ends and no tip, and the hand is mounted on one
// of the branches through a fixed transform that need not be the identity.
//
// What the controller does about it, and what this file pins:
//   - the primary group's model is the `urdf.tree_models` entry named after the
//     device (no sub_models at all here);
//   - the "arm tip" is the link the hand is mounted on, read off the secondary
//     tree's root_link — NOT the last link of the branch, and not a tip of the
//     primary tree;
//   - fingertip poses compose through that link:
//       T_root_fingertip = T_root_tip · T_tip_fingertip
//   - the E-STOP tick, which feeds the arm handle the device's positions in
//     arrival order, reads the same pose as the normal tick;
//   - the TF slots are labelled with those same two links.
//
// The oracle is forward kinematics on the FULL model, addressed by joint NAME.
// It shares no joint-order assumption, no reduced model and no frame id with
// the code under test.
//
// Two things the tree work made true are checked on a CHAIN group as well
// (JointChainPrimaryGroup): the E-STOP tick maps a device joint order that is
// not the model's, and a joint name the model does not carry refuses the
// configure. A chain used to be fed positionally with no check at all.
//
// The fixture (rtc_urdf_bridge/test/urdf/dual_arm_tree_hand.urdf) is synthetic
// and owned by this repository. Each of its asymmetries is what makes one of
// the assertions below able to fail — see the comment at the top of that file,
// and FixtureCanTellTheCasesApart, which checks them rather than trusting it.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "test_urdf_path.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;
using integrated_bringup::DemoJointController;
using rtc::ControllerOutput;
using rtc::ControllerState;

constexpr int kBodyDof = 5;
constexpr int kHandDof = 5;
constexpr double kDt = 0.002;

// The BODY's device order is deliberately not the model's (waist, left arm,
// right arm): a path that feeds the model positions "as they arrive" lands on
// the wrong joints, and only a name-based mapping gets this right.
const std::vector<std::string> kBodyJoints = {"waist_yaw", "right_shoulder", "right_elbow",
                                              "left_shoulder", "left_elbow"};
// The HAND's device order is the model's (fingers a..d) in the rigs that pin the
// primary group: what those tests can fail on is then the body's order alone.
// The hand's own order is the secondary group's concern, and has a rig of its
// own below (kShuffledHandJoints).
const std::vector<std::string> kHandJoints = {"finger_a_1", "finger_a_2", "finger_b_1",
                                              "finger_c_1", "finger_d_1"};
// Every joint off zero, and the two arms at different angles: at a symmetric
// or neutral pose, reading the left arm's joints as the right arm's would
// change nothing.
const std::vector<double> kBodyQ = {0.35, -0.50, 0.70, 0.40, -0.60};
const std::vector<double> kHandQ = {0.30, -0.40, 0.50, 0.20, -0.25};
// The same hand as a device that lists its joints in another order, no joint in
// its model slot. Each joint keeps the value it has above, so the fingertips
// are where they were — unless the values are read off the wrong joints.
const std::vector<std::string> kShuffledHandJoints = {"finger_c_1", "finger_a_1", "finger_d_1",
                                                      "finger_b_1", "finger_a_2"};
const std::vector<double> kShuffledHandQ = {0.20, 0.30, -0.25, 0.50, -0.40};

// The same robot with its primary group declared as ONE CHAIN: pelvis →
// hand_base, three joints (the left arm is on no device). The model orders
// them waist_yaw, right_shoulder, right_elbow; the device does not. Each joint
// keeps the value it has in the tree rig, and no two are equal — read
// positionally, every one of them lands on another joint.
const std::vector<std::string> kChainJoints = {"right_elbow", "waist_yaw", "right_shoulder"};
const std::vector<double> kChainQ = {0.70, 0.35, -0.50};

const std::vector<std::string> kTips = {"tip_a", "tip_b", "tip_c", "tip_d"};
const char* const kRoot = "pelvis";
const char* const kHandMount = "hand_base";    // what the hand is built on
const char* const kBranchEnd = "right_wrist";  // the link BEFORE the mount

// ── A rig: one way of declaring the robot, and the devices that go with it ──

struct Rig {
  const rub::ModelConfig* model{nullptr};
  std::shared_ptr<rub::PinocchioModelBuilder> builder;
  std::map<std::string, rtc::DeviceNameConfig> devices;
  std::string yaml;
  std::vector<std::string> body_joints;                // the primary device's order
  std::vector<double> body_q;                          // ... and the position of each
  std::vector<std::string> hand_joints = kHandJoints;  // the secondary device's order
  std::vector<double> hand_q = kHandQ;                 // ... and the position of each
};

std::string Yaml(std::size_t arm_dof, bool with_hand) {
  std::string safe;
  for (std::size_t i = 0; i < arm_dof; ++i) {
    safe += (i == 0 ? "0.0" : ", 0.0");
  }
  std::string yaml = "arm_dof: " + std::to_string(arm_dof) + R"(
robot_trajectory_speed: 2.0
hand_trajectory_speed: 3.0
robot_max_traj_velocity: 3.14
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [)" +
                     safe + R"(]
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
  if (with_hand) {
    yaml += R"(  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
)";
  }
  return yaml;
}

rtc::DeviceNameConfig BodyDevice(const std::vector<std::string>& joints, const char* tip_link) {
  rtc::DeviceNameConfig body;
  body.device_name = "body";
  body.joint_state_names = joints;
  rtc::DeviceUrdfConfig urdf;
  urdf.root_link = kRoot;
  urdf.tip_link = tip_link;
  body.urdf = urdf;
  rtc::DeviceJointLimits limits;
  limits.max_velocity.assign(joints.size(), 5.0);
  limits.position_lower.assign(joints.size(), -2.0);
  limits.position_upper.assign(joints.size(), 2.0);
  body.joint_limits = limits;
  return body;
}

rtc::DeviceNameConfig HandDevice(const std::vector<std::string>& joints = kHandJoints) {
  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  hand.joint_state_names = joints;
  rtc::DeviceUrdfConfig urdf;
  urdf.root_link = kHandMount;
  hand.urdf = urdf;
  return hand;
}

rub::ModelConfig MakeTreeModelConfig() {
  rub::ModelConfig cfg;
  cfg.urdf_path = rtc::test::TestUrdfPath("dual_arm_tree_hand.urdf");
  cfg.root_joint_type = "fixed";
  // No sub_models. The body tree's own tips stop at the branch ends — the
  // right one at the link BEFORE the hand mount — so "a tip of the primary
  // tree" and "the link the hand is mounted on" are different links here.
  cfg.tree_models.push_back({"body", kRoot, {"left_tip", kBranchEnd}});
  cfg.tree_models.push_back({"hand", kHandMount, kTips});
  return cfg;
}

rub::ModelConfig MakeChainModelConfig() {
  rub::ModelConfig cfg;
  cfg.urdf_path = rtc::test::TestUrdfPath("dual_arm_tree_hand.urdf");
  cfg.root_joint_type = "fixed";
  cfg.sub_models.push_back({"body", kRoot, kHandMount});
  cfg.tree_models.push_back({"hand", kHandMount, kTips});
  return cfg;
}

// The body is a TREE (the case this file is named after). Device configs are
// what the controller manager resolves: a tree group gets its root_link from
// the tree model and an EMPTY tip_link.
Rig TreeRig(const std::vector<std::string>& body_joints = kBodyJoints) {
  static const rub::ModelConfig cfg = MakeTreeModelConfig();
  static const auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);
  Rig rig;
  rig.model = &cfg;
  rig.builder = builder;
  rig.devices["body"] = BodyDevice(body_joints, "");
  rig.devices["hand"] = HandDevice();
  rig.yaml = Yaml(body_joints.size(), /*with_hand=*/true);
  rig.body_joints = kBodyJoints;
  rig.body_q = kBodyQ;
  return rig;
}

// The tree rig with the hand device in an order that is not the hand model's.
Rig ShuffledHandTreeRig() {
  Rig rig = TreeRig();
  rig.devices["hand"] = HandDevice(kShuffledHandJoints);
  rig.hand_joints = kShuffledHandJoints;
  rig.hand_q = kShuffledHandQ;
  return rig;
}

// The same tree with NO hand group at all — nothing says where the arm ends.
// `tip_link` is the per-device override the controller manager passes through.
Rig HandlessTreeRig(const char* tip_link) {
  Rig rig = TreeRig();
  rig.devices.erase("hand");
  rig.devices["body"] = BodyDevice(kBodyJoints, tip_link);
  rig.yaml = Yaml(kBodyJoints.size(), /*with_hand=*/false);
  return rig;
}

// The body is one CHAIN, pelvis → hand mount.
Rig ChainRig(const std::vector<std::string>& chain_joints = kChainJoints) {
  static const rub::ModelConfig cfg = MakeChainModelConfig();
  static const auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);
  Rig rig;
  rig.model = &cfg;
  rig.builder = builder;
  rig.devices["body"] = BodyDevice(chain_joints, kHandMount);
  rig.devices["hand"] = HandDevice();
  rig.yaml = Yaml(chain_joints.size(), /*with_hand=*/true);
  rig.body_joints = kChainJoints;
  rig.body_q = kChainQ;
  return rig;
}

ControllerState MakeState(const Rig& rig) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = kDt;
  state.iteration = 1;
  auto& dev0 = state.devices[0];
  dev0.num_channels = static_cast<int>(rig.body_q.size());
  dev0.valid = true;
  dev0.hole_mask = 0;
  for (std::size_t i = 0; i < rig.body_q.size(); ++i) {
    dev0.positions[i] = rig.body_q[i];
  }
  auto& dev1 = state.devices[1];
  dev1.num_channels = kHandDof;
  dev1.valid = true;
  dev1.hole_mask = 0;
  for (std::size_t i = 0; i < rig.hand_q.size(); ++i) {
    dev1.positions[i] = rig.hand_q[i];
  }
  return state;
}

// ── The oracle: full-model FK, by joint name ────────────────────────────────

Eigen::VectorXd FullModelConfiguration(const pinocchio::Model& model, const Rig& rig) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  auto set = [&](const std::string& name, double value) {
    ASSERT_TRUE(model.existJointName(name)) << name;
    q[model.joints[model.getJointId(name)].idx_q()] = value;
  };
  for (std::size_t i = 0; i < rig.body_joints.size(); ++i) {
    set(rig.body_joints[i], rig.body_q[i]);
  }
  for (std::size_t i = 0; i < rig.hand_joints.size(); ++i) {
    set(rig.hand_joints[i], rig.hand_q[i]);
  }
  return q;
}

/// Pose of `frame` expressed in the root link, at the rig's configuration.
pinocchio::SE3 RootRelative(const Rig& rig, const std::string& frame) {
  const pinocchio::Model& model = *rig.builder->GetFullModel();
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, FullModelConfiguration(model, rig));
  EXPECT_TRUE(model.existFrame(frame)) << frame;
  return data.oMf[model.getFrameId(kRoot)].actInv(data.oMf[model.getFrameId(frame)]);
}

pinocchio::SE3 ToSe3(const rtc::Pose& pose) {
  const Eigen::Quaterniond q(pose.quaternion[0], pose.quaternion[1], pose.quaternion[2],
                             pose.quaternion[3]);
  return pinocchio::SE3(q.toRotationMatrix(),
                        Eigen::Vector3d(pose.position[0], pose.position[1], pose.position[2]));
}

void ExpectSamePose(const pinocchio::SE3& got, const pinocchio::SE3& want, double tol,
                    const std::string& what) {
  EXPECT_LE((got.translation() - want.translation()).norm(), tol)
      << what << ": position off by " << (got.translation() - want.translation()).norm()
      << " m\n  got  " << got.translation().transpose() << "\n  want "
      << want.translation().transpose();
  EXPECT_LE((got.rotation() - want.rotation()).norm(), tol)
      << what << ": rotation off by " << (got.rotation() - want.rotation()).norm();
}

/// A controller with what the controller manager injects before any config is
/// read: the system model, the shared builder, the control rate.
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

/// The arm tip the E-STOP tick reports, after a few normal ticks.
pinocchio::SE3 EstopArmTip(DemoJointController& ctrl, const Rig& rig) {
  ControllerState state = MakeState(rig);
  ControllerOutput out = ctrl.Compute(state);
  ctrl.TriggerEstop();
  state.iteration += 1;
  out = ctrl.Compute(state);
  EXPECT_TRUE(ctrl.IsEstopped());
  EXPECT_TRUE(out.arm_tip_pose_valid);
  return ToSe3(out.arm_tip_pose);
}

/// Self-init tick plus a few holds, at the test configuration.
ControllerOutput RunNormalTicks(DemoJointController& ctrl, ControllerState& state, int n = 3) {
  ControllerOutput out = ctrl.Compute(state);
  for (int i = 0; i < n; ++i) {
    state.iteration += 1;
    out = ctrl.Compute(state);
  }
  return out;
}

// ── Canary: the fixture can actually fail these tests ───────────────────────

TEST(JointTreePrimaryGroup, FixtureCanTellTheCasesApart) {
  const Rig rig = TreeRig();
  const pinocchio::Model& full = *rig.builder->GetFullModel();
  ASSERT_EQ(full.nq, kBodyDof + kHandDof);
  ASSERT_TRUE(rig.model->sub_models.empty());
  EXPECT_EQ(rig.builder->GetTreeModel("body")->nv, kBodyDof);
  EXPECT_EQ(rig.builder->GetActuatedModel(), nullptr) << "the fixture's hand must stay serial";

  // The body's device order is not the model's order (the hand's is — see
  // kHandJoints).
  auto model_order = [&](const std::vector<std::string>& names) {
    std::vector<int> idx;
    for (const auto& n : names) {
      idx.push_back(static_cast<int>(full.joints[full.getJointId(n)].idx_q()));
    }
    return idx;
  };
  const auto body_idx = model_order(kBodyJoints);
  const auto hand_idx = model_order(kHandJoints);
  EXPECT_FALSE(std::is_sorted(body_idx.begin(), body_idx.end()));
  EXPECT_TRUE(std::is_sorted(hand_idx.begin(), hand_idx.end()));

  // The hand mount is not the identity, and the tree root is not the world.
  const pinocchio::SE3 mount = RootRelative(rig, kBranchEnd).actInv(RootRelative(rig, kHandMount));
  EXPECT_GT(mount.translation().norm(), 0.03);
  EXPECT_GT((mount.rotation() - Eigen::Matrix3d::Identity()).norm(), 1.0);
  pinocchio::Data data(full);
  pinocchio::framesForwardKinematics(full, data, pinocchio::neutral(full));
  EXPECT_GT(data.oMf[full.getFrameId(kRoot)].translation().norm(), 0.5);
}

// ── The arm tip is the hand mount, in the tree root ─────────────────────────

TEST(JointTreePrimaryGroup, ArmTipIsTheHandMountInTheTreeRoot) {
  const Rig rig = TreeRig();
  auto ctrl = BringUp(rig);
  ControllerState state = MakeState(rig);
  const ControllerOutput out = RunNormalTicks(*ctrl, state);

  ASSERT_TRUE(out.valid);
  ASSERT_TRUE(out.arm_tip_pose_valid);
  ExpectSamePose(ToSe3(out.arm_tip_pose), RootRelative(rig, kHandMount), 1e-9, "arm tip");
}

TEST(JointTreePrimaryGroup, FingertipsComposeThroughTheHandMount) {
  const Rig rig = TreeRig();
  auto ctrl = BringUp(rig);
  ControllerState state = MakeState(rig);
  const ControllerOutput out = RunNormalTicks(*ctrl, state);

  for (std::size_t f = 0; f < kTips.size(); ++f) {
    ASSERT_TRUE(out.task_link_pose_valid[f]) << kTips[f];
    ExpectSamePose(ToSe3(out.task_link_poses[f]), RootRelative(rig, kTips[f]), 1e-9, kTips[f]);
  }
}

// The hand's FK runs on the hand's own tree model, fed the hand device's
// positions as they arrive. A device order that is not that model's has to be
// mapped onto it, or every value lands on another joint (#685).
TEST(JointTreePrimaryGroup, FingertipsFollowAHandDeviceOrderThatIsNotTheModels) {
  const Rig rig = ShuffledHandTreeRig();
  const pinocchio::Model& hand = *rig.builder->GetTreeModel("hand");
  std::vector<int> idx;
  for (const auto& n : kShuffledHandJoints) {
    ASSERT_TRUE(hand.existJointName(n)) << n;
    idx.push_back(static_cast<int>(hand.joints[hand.getJointId(n)].idx_q()));
  }
  ASSERT_FALSE(std::is_sorted(idx.begin(), idx.end()));

  auto ctrl = BringUp(rig);
  ControllerState state = MakeState(rig);
  const ControllerOutput out = RunNormalTicks(*ctrl, state);

  for (std::size_t f = 0; f < kTips.size(); ++f) {
    ASSERT_TRUE(out.task_link_pose_valid[f]) << kTips[f];
    ExpectSamePose(ToSe3(out.task_link_poses[f]), RootRelative(rig, kTips[f]), 1e-9, kTips[f]);
  }
}

// The E-STOP tick computes the arm tip on a different path (the arm handle, fed
// the device's positions as they arrive) from the normal tick (the combined
// model cache, mapped by name). On a tree group those agree only if the handle
// was given the device's joint order.
TEST(JointTreePrimaryGroup, EstopTickReportsTheSameArmTip) {
  const Rig rig = TreeRig();
  auto ctrl = BringUp(rig);
  ControllerState state = MakeState(rig);
  const ControllerOutput normal = RunNormalTicks(*ctrl, state);
  ASSERT_TRUE(normal.arm_tip_pose_valid);

  ctrl->TriggerEstop();
  state.iteration += 1;
  const ControllerOutput estop = ctrl->Compute(state);
  ASSERT_TRUE(ctrl->IsEstopped());
  ASSERT_TRUE(estop.arm_tip_pose_valid);

  ExpectSamePose(ToSe3(estop.arm_tip_pose), ToSe3(normal.arm_tip_pose), 1e-12,
                 "E-STOP arm tip vs normal tick");
}

// ── on_configure: frame names, and the refusal ──────────────────────────────

class JointTreePrimaryGroupLifecycle : public ::testing::Test {
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

  /// The controller manager's bring-up order (rt_controller_node_params.cpp):
  /// PreConfigure (node + LoadConfig), device configs, on_configure. Going
  /// through PreConfigure is what keeps on_configure from loading the config a
  /// second time — which would rebuild the arm handle and the combined cache
  /// AFTER the device configs wired them, a state the node never reaches.
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

TEST_F(JointTreePrimaryGroupLifecycle, TfSlotsAreNamedAfterTheTreeRootAndTheHandMount) {
  const Rig rig = TreeRig();
  auto ctrl = PreConfigure(rig, "tree_primary_tf_slots");
  ASSERT_EQ(Configure(*ctrl, rig), DemoJointController::CallbackReturn::SUCCESS);

  const std::vector<std::pair<std::string, std::string>> expected = {
      {"pelvis", "hand_base_actual"}, {"pelvis", "tip_a_actual"}, {"pelvis", "tip_b_actual"},
      {"pelvis", "tip_c_actual"},     {"pelvis", "tip_d_actual"}, {"pelvis", "virtual_tcp_actual"},
  };
  EXPECT_EQ(ctrl->TfSlotFramesForTesting(), expected);
  EXPECT_EQ(ctrl->OwnedStateFrameIdForTesting(), "pelvis");
}

// What OnDeviceConfigsSet wired has to survive on_configure: the joint-order
// map lives on the arm handle, and the handle is rebuilt by any later
// LoadConfig. Read off the E-STOP tick, the one lane that uses it.
TEST_F(JointTreePrimaryGroupLifecycle, TheConfiguredControllerStillMapsTheJointOrder) {
  const Rig rig = TreeRig();
  auto ctrl = PreConfigure(rig, "tree_primary_configured_estop");
  ASSERT_EQ(Configure(*ctrl, rig), DemoJointController::CallbackReturn::SUCCESS);

  ExpectSamePose(EstopArmTip(*ctrl, rig), RootRelative(rig, kHandMount), 1e-9,
                 "E-STOP arm tip after on_configure");
}

// The same for the hand handle's joint order, on the controller manager's
// bring-up order — where the hand handle is built before the device configs
// exist, and only OnDeviceConfigsSet can give it the device's order.
TEST_F(JointTreePrimaryGroupLifecycle, TheConfiguredControllerMapsTheHandOrder) {
  const Rig rig = ShuffledHandTreeRig();
  auto ctrl = PreConfigure(rig, "tree_primary_configured_hand_order");
  ASSERT_EQ(Configure(*ctrl, rig), DemoJointController::CallbackReturn::SUCCESS);

  ControllerState state = MakeState(rig);
  const ControllerOutput out = RunNormalTicks(*ctrl, state);
  for (std::size_t f = 0; f < kTips.size(); ++f) {
    ASSERT_TRUE(out.task_link_pose_valid[f]) << kTips[f];
    ExpectSamePose(ToSe3(out.task_link_poses[f]), RootRelative(rig, kTips[f]), 1e-9, kTips[f]);
  }
}

// A tree group whose joint names do not all resolve on its model has no joint
// order to give the arm handle, so the E-STOP tick's pose would be read off the
// wrong joints with nothing else out of place. That is refused at configure —
// and the control case next to it shows the refusal is about the name, not
// about this fixture.
TEST_F(JointTreePrimaryGroupLifecycle, AJointNameTheTreeDoesNotCarryRefusesConfigure) {
  const Rig good_rig = TreeRig();
  auto good = PreConfigure(good_rig, "tree_primary_names_ok");
  EXPECT_EQ(Configure(*good, good_rig), DemoJointController::CallbackReturn::SUCCESS);

  std::vector<std::string> names = kBodyJoints;
  names[1] = "finger_a_1";  // a real joint of the robot, but not one of this tree's
  const Rig bad_rig = TreeRig(names);
  auto bad = PreConfigure(bad_rig, "tree_primary_names_bad");
  EXPECT_EQ(Configure(*bad, bad_rig), DemoJointController::CallbackReturn::FAILURE);
}

// A tree has no tip of its own; the arm ends where the hand is mounted. With
// no hand group nothing says where that is, and both tick lanes would report
// the tree ROOT's pose as the arm tip, flagged valid. Refused — and the control
// case, the same hand-less rig with the end named on the device, configures:
// the refusal is about the missing end, not about the missing hand.
TEST_F(JointTreePrimaryGroupLifecycle, ATreeWithNoArmTipRefusesConfigure) {
  const Rig named_rig = HandlessTreeRig(kHandMount);
  auto named = PreConfigure(named_rig, "tree_primary_tip_named");
  EXPECT_EQ(Configure(*named, named_rig), DemoJointController::CallbackReturn::SUCCESS);

  const Rig bare_rig = HandlessTreeRig("");
  auto bare = PreConfigure(bare_rig, "tree_primary_tip_missing");
  EXPECT_EQ(Configure(*bare, bare_rig), DemoJointController::CallbackReturn::FAILURE);

  const Rig wrong_rig = HandlessTreeRig("no_such_link");
  auto wrong = PreConfigure(wrong_rig, "tree_primary_tip_unknown");
  EXPECT_EQ(Configure(*wrong, wrong_rig), DemoJointController::CallbackReturn::FAILURE);
}

// ── The same two guarantees on a CHAIN primary group ────────────────────────

TEST(JointChainPrimaryGroup, FixtureDeviceOrderIsNotTheChainModels) {
  const Rig rig = ChainRig();
  const auto chain = rig.builder->GetReducedModel("body");
  ASSERT_EQ(chain->nv, static_cast<int>(kChainJoints.size()));
  std::vector<int> idx;
  for (const auto& n : kChainJoints) {
    ASSERT_TRUE(chain->existJointName(n)) << n;
    idx.push_back(static_cast<int>(chain->joints[chain->getJointId(n)].idx_q()));
  }
  EXPECT_FALSE(std::is_sorted(idx.begin(), idx.end()));
}

// The E-STOP tick feeds the arm handle the device's positions as they arrive.
// On a chain that was simply assumed to be the model's order.
TEST(JointChainPrimaryGroup, EstopTickMapsADeviceOrderThatIsNotTheModels) {
  const Rig rig = ChainRig();
  auto ctrl = BringUp(rig);

  ExpectSamePose(EstopArmTip(*ctrl, rig), RootRelative(rig, kHandMount), 1e-9,
                 "E-STOP arm tip, chain group");
}

TEST_F(JointTreePrimaryGroupLifecycle, AJointNameTheChainDoesNotCarryRefusesConfigure) {
  const Rig good_rig = ChainRig();
  auto good = PreConfigure(good_rig, "chain_primary_names_ok");
  EXPECT_EQ(Configure(*good, good_rig), DemoJointController::CallbackReturn::SUCCESS);

  std::vector<std::string> names = kChainJoints;
  names[0] = "left_elbow";  // a real joint of the robot, but not on this chain
  const Rig bad_rig = ChainRig(names);
  auto bad = PreConfigure(bad_rig, "chain_primary_names_bad");
  EXPECT_EQ(Configure(*bad, bad_rig), DemoJointController::CallbackReturn::FAILURE);
}

}  // namespace
