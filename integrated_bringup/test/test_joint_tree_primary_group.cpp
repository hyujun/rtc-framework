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
#include <array>
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
// The HAND's device order IS the model's (fingers a..d), and that is not an
// oversight. A serial hand's FK is fed positionally: InitHandModel's
// SetJointOrder runs at LoadConfig, before the device configs exist, and never
// takes effect (#685). That defect is the secondary group's, it predates the
// tree work, and it is not what this file pins — a permuted hand here would
// fail these tests for a reason that has nothing to do with the primary group.
const std::vector<std::string> kHandJoints = {"finger_a_1", "finger_a_2", "finger_b_1",
                                              "finger_c_1", "finger_d_1"};
// Every joint off zero, and the two arms at different angles: at a symmetric
// or neutral pose, reading the left arm's joints as the right arm's would
// change nothing.
const std::array<double, kBodyDof> kBodyQ = {0.35, -0.50, 0.70, 0.40, -0.60};
const std::array<double, kHandDof> kHandQ = {0.30, -0.40, 0.50, 0.20, -0.25};

const std::vector<std::string> kTips = {"tip_a", "tip_b", "tip_c", "tip_d"};
const char* const kRoot = "pelvis";
const char* const kHandMount = "hand_base";    // what the hand is built on
const char* const kBranchEnd = "right_wrist";  // the link BEFORE the mount

rub::ModelConfig MakeModelConfig() {
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

const rub::ModelConfig& ModelConfig() {
  static const rub::ModelConfig cfg = MakeModelConfig();
  return cfg;
}

std::shared_ptr<rub::PinocchioModelBuilder> Builder() {
  static auto builder = std::make_shared<rub::PinocchioModelBuilder>(ModelConfig());
  return builder;
}

// Device configs as the controller manager resolves them: a tree group gets
// its root_link from the tree model and an EMPTY tip_link.
std::map<std::string, rtc::DeviceNameConfig> MakeDeviceConfigs(
    const std::vector<std::string>& body_joints = kBodyJoints) {
  std::map<std::string, rtc::DeviceNameConfig> configs;

  rtc::DeviceNameConfig body;
  body.device_name = "body";
  body.joint_state_names = body_joints;
  rtc::DeviceUrdfConfig body_urdf;
  body_urdf.root_link = kRoot;
  body.urdf = body_urdf;
  rtc::DeviceJointLimits limits;
  limits.max_velocity.assign(static_cast<std::size_t>(kBodyDof), 5.0);
  limits.position_lower.assign(static_cast<std::size_t>(kBodyDof), -2.0);
  limits.position_upper.assign(static_cast<std::size_t>(kBodyDof), 2.0);
  body.joint_limits = limits;
  configs["body"] = std::move(body);

  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  hand.joint_state_names = kHandJoints;
  rtc::DeviceUrdfConfig hand_urdf;
  hand_urdf.root_link = kHandMount;
  hand.urdf = hand_urdf;
  configs["hand"] = std::move(hand);

  return configs;
}

const char* const kYaml = R"(
arm_dof: 5
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
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
)";

ControllerState MakeState() {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = kDt;
  state.iteration = 1;
  auto& dev0 = state.devices[0];
  dev0.num_channels = kBodyDof;
  dev0.valid = true;
  dev0.hole_mask = 0;
  for (std::size_t i = 0; i < kBodyQ.size(); ++i) {
    dev0.positions[i] = kBodyQ[i];
  }
  auto& dev1 = state.devices[1];
  dev1.num_channels = kHandDof;
  dev1.valid = true;
  dev1.hole_mask = 0;
  for (std::size_t i = 0; i < kHandQ.size(); ++i) {
    dev1.positions[i] = kHandQ[i];
  }
  return state;
}

// ── The oracle: full-model FK, by joint name ────────────────────────────────

Eigen::VectorXd FullModelConfiguration(const pinocchio::Model& model) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  auto set = [&](const std::string& name, double value) {
    ASSERT_TRUE(model.existJointName(name)) << name;
    q[model.joints[model.getJointId(name)].idx_q()] = value;
  };
  for (std::size_t i = 0; i < kBodyJoints.size(); ++i) {
    set(kBodyJoints[i], kBodyQ[i]);
  }
  for (std::size_t i = 0; i < kHandJoints.size(); ++i) {
    set(kHandJoints[i], kHandQ[i]);
  }
  return q;
}

/// Pose of `frame` expressed in the tree root, at the test configuration.
pinocchio::SE3 RootRelative(const std::string& frame) {
  const pinocchio::Model& model = *Builder()->GetFullModel();
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, FullModelConfiguration(model));
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

std::unique_ptr<DemoJointController> BringUp(
    const std::map<std::string, rtc::DeviceNameConfig>& devices = MakeDeviceConfigs()) {
  auto ctrl = std::make_unique<DemoJointController>("");
  ctrl->SetSystemModelConfig(ModelConfig());
  ctrl->SetSharedModelBuilder(Builder());
  ctrl->SetControlRate(1.0 / kDt);
  ctrl->LoadConfig(YAML::Load(kYaml));
  ctrl->SetDeviceNameConfigs(devices);
  return ctrl;
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
  const pinocchio::Model& full = *Builder()->GetFullModel();
  ASSERT_EQ(full.nq, kBodyDof + kHandDof);
  ASSERT_TRUE(ModelConfig().sub_models.empty());
  EXPECT_EQ(Builder()->GetTreeModel("body")->nv, kBodyDof);
  EXPECT_EQ(Builder()->GetActuatedModel(), nullptr) << "the fixture's hand must stay serial";

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
  const pinocchio::SE3 mount = RootRelative(kBranchEnd).actInv(RootRelative(kHandMount));
  EXPECT_GT(mount.translation().norm(), 0.03);
  EXPECT_GT((mount.rotation() - Eigen::Matrix3d::Identity()).norm(), 1.0);
  pinocchio::Data data(full);
  pinocchio::framesForwardKinematics(full, data, pinocchio::neutral(full));
  EXPECT_GT(data.oMf[full.getFrameId(kRoot)].translation().norm(), 0.5);
}

// ── The arm tip is the hand mount, in the tree root ─────────────────────────

TEST(JointTreePrimaryGroup, ArmTipIsTheHandMountInTheTreeRoot) {
  auto ctrl = BringUp();
  ControllerState state = MakeState();
  const ControllerOutput out = RunNormalTicks(*ctrl, state);

  ASSERT_TRUE(out.valid);
  ASSERT_TRUE(out.arm_tip_pose_valid);
  ExpectSamePose(ToSe3(out.arm_tip_pose), RootRelative(kHandMount), 1e-9, "arm tip");
}

TEST(JointTreePrimaryGroup, FingertipsComposeThroughTheHandMount) {
  auto ctrl = BringUp();
  ControllerState state = MakeState();
  const ControllerOutput out = RunNormalTicks(*ctrl, state);

  for (std::size_t f = 0; f < kTips.size(); ++f) {
    ASSERT_TRUE(out.task_link_pose_valid[f]) << kTips[f];
    ExpectSamePose(ToSe3(out.task_link_poses[f]), RootRelative(kTips[f]), 1e-9, kTips[f]);
  }
}

// The E-STOP tick computes the arm tip on a different path (the arm handle, fed
// the device's positions as they arrive) from the normal tick (the combined
// model cache, mapped by name). On a tree group those agree only if the handle
// was given the device's joint order.
TEST(JointTreePrimaryGroup, EstopTickReportsTheSameArmTip) {
  auto ctrl = BringUp();
  ControllerState state = MakeState();
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

  DemoJointController::CallbackReturn Configure(DemoJointController& ctrl, const char* node_name) {
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, "", opts);
    return ctrl.on_configure(rclcpp_lifecycle::State{}, node_, YAML::Load(kYaml));
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

TEST_F(JointTreePrimaryGroupLifecycle, TfSlotsAreNamedAfterTheTreeRootAndTheHandMount) {
  auto ctrl = BringUp();
  ASSERT_EQ(Configure(*ctrl, "tree_primary_tf_slots"),
            DemoJointController::CallbackReturn::SUCCESS);

  const std::vector<std::pair<std::string, std::string>> expected = {
      {"pelvis", "hand_base_actual"}, {"pelvis", "tip_a_actual"}, {"pelvis", "tip_b_actual"},
      {"pelvis", "tip_c_actual"},     {"pelvis", "tip_d_actual"}, {"pelvis", "virtual_tcp_actual"},
  };
  EXPECT_EQ(ctrl->TfSlotFramesForTesting(), expected);
  EXPECT_EQ(ctrl->OwnedStateFrameIdForTesting(), "pelvis");
}

// A tree group whose joint names do not all resolve on its model has no joint
// order to give the arm handle, so the E-STOP tick's pose would be read off the
// wrong joints with nothing else out of place. That is refused at configure —
// and the control case next to it shows the refusal is about the name, not
// about this fixture.
TEST_F(JointTreePrimaryGroupLifecycle, AJointNameTheTreeDoesNotCarryRefusesConfigure) {
  auto good = BringUp();
  EXPECT_EQ(Configure(*good, "tree_primary_names_ok"),
            DemoJointController::CallbackReturn::SUCCESS);

  std::vector<std::string> names = kBodyJoints;
  names[1] = "finger_a_1";  // a real joint of the robot, but not one of this tree's
  auto bad = BringUp(MakeDeviceConfigs(names));
  EXPECT_EQ(Configure(*bad, "tree_primary_names_bad"),
            DemoJointController::CallbackReturn::FAILURE);
}

}  // namespace
