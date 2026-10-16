// ── The target-frame contract on the controllers that hold it (#779) ─────────
//
// WBC, Task and Compliance take a task goal for the arm only in the arm's
// root_link or in scene_world, and refuse — counted, through the base's
// DeliverTargetMessage — an empty frame_id, any other name, and scene_world
// on a bring-up that does not define it. A root_link that is named but is not
// a frame, and a scene_world that is not rigid to it, fail the configure.
//
// The rig is iiwa7_leap (root link_0). Its world is added here as an
// urdf.extra_frames entry, offset and turned about two axes, so the WBC value
// check below can tell a correct conversion from one that skips or inverts it.

#include "iiwa7_leap_controller_yamls.hpp"
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include <rtc_math/se3/so3.hpp>
#include <rtc_msgs/msg/robot_target.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <map>
#include <memory>
#include <string>

namespace {

using integrated_bringup::DemoComplianceController;
using integrated_bringup::DemoJointController;
using integrated_bringup::DemoTaskController;
using integrated_bringup::DemoWbcController;
namespace fx = integrated_bringup::testfx;

constexpr const char* kRoot = "link_0";
constexpr const char* kWorld = "scene_world";
constexpr const char* kArmGroup = "iiwa7";

// scene_world in link_0 (p_root = R·p_world + t).
const Eigen::Vector3d kWorldXyz(0.12, -0.08, -0.7);
const Eigen::Vector3d kWorldRpy(0.3, -0.2, 0.7);

std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> BuilderWithWorld(const char* parent) {
  rtc_urdf_bridge::ModelConfig cfg = fx::MakeIiwa7LeapModelConfig();
  rtc_urdf_bridge::ExtraFrameConfig world;
  world.name = kWorld;
  world.parent = parent;
  world.xyz = kWorldXyz;
  world.rpy = kWorldRpy;
  world.provisional = false;
  cfg.extra_frames.push_back(world);
  return std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(cfg);
}

template <class Ctrl>
struct Up {
  std::unique_ptr<Ctrl> ctrl;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  typename Ctrl::CallbackReturn rc{Ctrl::CallbackReturn::ERROR};
};

/// `builder` null → the shipped model (no world).
template <class Ctrl>
Up<Ctrl> Configure(
    const std::string& tag, const std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder>& builder,
    std::map<std::string, rtc::DeviceNameConfig> devices = fx::MakeIiwa7LeapDeviceConfigs()) {
  using Fx = fx::ControllerFixture<Ctrl>;
  Up<Ctrl> up;
  rclcpp::NodeOptions opts;
  opts.use_global_arguments(false);
  up.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      std::string("goal_frames_") + Fx::kName + "_" + tag, "", opts);
  up.ctrl = Fx::Make();
  if (builder) {
    rtc_urdf_bridge::ModelConfig cfg = fx::MakeIiwa7LeapModelConfig();
    cfg.extra_frames = builder->GetConfig().extra_frames;
    up.ctrl->SetSystemModelConfig(cfg);
    up.ctrl->SetSharedModelBuilder(builder);
  } else {
    up.ctrl->SetSystemModelConfig(fx::SharedIiwa7LeapModelConfig());
    up.ctrl->SetSharedModelBuilder(fx::SharedIiwa7LeapBuilder());
  }
  up.ctrl->SetControlRate(1.0 / fx::kDt);
  const YAML::Node yaml = YAML::Load(Fx::Yaml());
  EXPECT_EQ(up.ctrl->PreConfigure(up.node, yaml), Ctrl::CallbackReturn::SUCCESS);
  up.ctrl->SetDeviceNameConfigs(std::move(devices));
  up.rc = up.ctrl->on_configure(rclcpp_lifecycle::State{}, up.node, yaml);
  return up;
}

rtc_msgs::msg::RobotTarget TaskGoal(const std::string& frame, const std::array<double, 6>& pose) {
  rtc_msgs::msg::RobotTarget msg;
  msg.goal_type = "task";
  msg.header.frame_id = frame;
  for (std::size_t i = 0; i < pose.size(); ++i) {
    msg.task_target[i] = pose[i];
  }
  return msg;
}

const std::array<double, 6> kPose{0.5, 0.1, 0.6, 3.0, 0.2, -0.4};

class TaskGoalFrames : public ::testing::Test {
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

  /// Delivers `msg` and returns how many rejects it added.
  template <class Ctrl>
  static std::uint64_t Rejects(Ctrl& ctrl, const rtc_msgs::msg::RobotTarget& msg) {
    const auto before = ctrl.GetTargetRejectCount();
    ctrl.DeliverTargetMessage(kArmGroup, 0, msg);
    return ctrl.GetTargetRejectCount() - before;
  }

  template <class Ctrl>
  static void ExpectTheContract() {
    const std::string who = fx::ControllerFixture<Ctrl>::kName;
    auto up = Configure<Ctrl>("world", BuilderWithWorld(kRoot));
    ASSERT_EQ(up.rc, Ctrl::CallbackReturn::SUCCESS) << who;
    const auto& frames = up.ctrl->TaskTargetFramesForTesting();
    ASSERT_TRUE(frames.WorldDefined()) << who;
    EXPECT_EQ(frames.RootLink(), kRoot) << who;
    EXPECT_LT((frames.RootFromWorld().translation() - kWorldXyz).norm(), 1e-12) << who;

    EXPECT_EQ(Rejects(*up.ctrl, TaskGoal("", kPose)), 1U) << who << ": empty frame_id";
    EXPECT_EQ(Rejects(*up.ctrl, TaskGoal("world", kPose)), 1U) << who << ": not a name taken";
    EXPECT_EQ(Rejects(*up.ctrl, TaskGoal("link_3", kPose)), 1U) << who << ": a model frame";
    EXPECT_EQ(Rejects(*up.ctrl, TaskGoal(kRoot, kPose)), 0U) << who;
    EXPECT_EQ(Rejects(*up.ctrl, TaskGoal(kWorld, kPose)), 0U) << who;

    // The same controller on the shipped model: no world, so world goals go.
    auto bare = Configure<Ctrl>("bare", nullptr);
    ASSERT_EQ(bare.rc, Ctrl::CallbackReturn::SUCCESS) << who;
    EXPECT_FALSE(bare.ctrl->TaskTargetFramesForTesting().WorldDefined()) << who;
    EXPECT_EQ(Rejects(*bare.ctrl, TaskGoal(kWorld, kPose)), 1U) << who;
    EXPECT_EQ(Rejects(*bare.ctrl, TaskGoal(kRoot, kPose)), 0U) << who;
  }

  template <class Ctrl>
  static void ExpectTheConfigureRefusals() {
    const std::string who = fx::ControllerFixture<Ctrl>::kName;
    // A world hung off a moving link would move with the arm.
    auto moving = Configure<Ctrl>("moving_world", BuilderWithWorld("link_3"));
    EXPECT_EQ(moving.rc, Ctrl::CallbackReturn::FAILURE) << who;

    // A root link that is named and is not a frame: it used to fall back to
    // the world in silence.
    auto devices = fx::MakeIiwa7LeapDeviceConfigs();
    devices.at(kArmGroup).urdf->root_link = "no_such_link";
    auto bad_root = Configure<Ctrl>("bad_root", nullptr, devices);
    EXPECT_EQ(bad_root.rc, Ctrl::CallbackReturn::FAILURE) << who;
    EXPECT_NE(bad_root.ctrl->ArmRootConfigError().find("'no_such_link' is not a frame"),
              std::string::npos)
        << who << ": '" << bad_root.ctrl->ArmRootConfigError() << "'";
    EXPECT_EQ(bad_root.ctrl->ArmTipConfigError(), "") << who;
  }
};

TEST_F(TaskGoalFrames, WbcTakesTheRootAndTheWorldOnly) {
  ExpectTheContract<DemoWbcController>();
}

TEST_F(TaskGoalFrames, TaskTakesTheRootAndTheWorldOnly) {
  ExpectTheContract<DemoTaskController>();
}

TEST_F(TaskGoalFrames, ComplianceTakesTheRootAndTheWorldOnly) {
  ExpectTheContract<DemoComplianceController>();
}

TEST_F(TaskGoalFrames, WbcRefusesAMovingWorldAndAnUnresolvedRoot) {
  ExpectTheConfigureRefusals<DemoWbcController>();
}

TEST_F(TaskGoalFrames, TaskRefusesAMovingWorldAndAnUnresolvedRoot) {
  ExpectTheConfigureRefusals<DemoTaskController>();
}

TEST_F(TaskGoalFrames, ComplianceRefusesAMovingWorldAndAnUnresolvedRoot) {
  ExpectTheConfigureRefusals<DemoComplianceController>();
}

// ── Goal kinds (V5) ─────────────────────────────────────────────────────────
//
// A joint-space controller would command a task goal's x, y, z, r, p, y as
// joint radians; a task-space one would read a joint goal's radians as a pose.
// Each refuses — counted — the kind it would misread.

rtc_msgs::msg::RobotTarget JointGoal(const rtc::DeviceNameConfig& device) {
  rtc_msgs::msg::RobotTarget msg;
  msg.goal_type = "joint";
  msg.joint_names = device.joint_state_names;
  msg.joint_target.assign(device.joint_state_names.size(), 0.1);
  return msg;
}

template <class Ctrl>
std::uint64_t RejectsOn(Ctrl& ctrl, const std::string& group, int idx,
                        const rtc_msgs::msg::RobotTarget& msg) {
  const auto before = ctrl.GetTargetRejectCount();
  ctrl.DeliverTargetMessage(group, idx, msg);
  return ctrl.GetTargetRejectCount() - before;
}

template <class Ctrl>
void ExpectTaskSpaceArmKinds() {
  const std::string who = fx::ControllerFixture<Ctrl>::kName;
  const auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  auto up = Configure<Ctrl>("kinds", nullptr);
  ASSERT_EQ(up.rc, Ctrl::CallbackReturn::SUCCESS) << who;
  const std::string hand = up.ctrl->GetSecondaryDeviceName();
  ASSERT_FALSE(hand.empty()) << who;
  EXPECT_EQ(RejectsOn(*up.ctrl, kArmGroup, 0, JointGoal(devices.at(kArmGroup))), 1U)
      << who << ": a joint goal for the arm";
  EXPECT_EQ(RejectsOn(*up.ctrl, kArmGroup, 0, TaskGoal(kRoot, kPose)), 0U) << who;
  EXPECT_EQ(RejectsOn(*up.ctrl, hand, 1, TaskGoal(kRoot, kPose)), 1U)
      << who << ": a task goal for the hand";
  EXPECT_EQ(RejectsOn(*up.ctrl, hand, 1, JointGoal(devices.at(hand))), 0U) << who;
}

TEST_F(TaskGoalFrames, TaskTakesNoJointGoalForTheArm) {
  ExpectTaskSpaceArmKinds<DemoTaskController>();
}

TEST_F(TaskGoalFrames, ComplianceTakesNoJointGoalForTheArm) {
  ExpectTaskSpaceArmKinds<DemoComplianceController>();
}

TEST_F(TaskGoalFrames, JointTakesNoTaskGoal) {
  const auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  auto up = Configure<DemoJointController>("kinds", nullptr);
  ASSERT_EQ(up.rc, DemoJointController::CallbackReturn::SUCCESS);
  EXPECT_EQ(RejectsOn(*up.ctrl, kArmGroup, 0, TaskGoal(kRoot, kPose)), 1U);
  EXPECT_EQ(RejectsOn(*up.ctrl, kArmGroup, 0, JointGoal(devices.at(kArmGroup))), 0U);
}

// The WBC end to end: a goal stated in the world reaches the commanded SE3 as
// the same pose in link_0. The world goal is built from a root goal through
// the inverse placement; the controller must undo it exactly.
TEST_F(TaskGoalFrames, AWorldGoalReachesTheWbcAsTheSamePoseInTheRoot) {
  auto up = Configure<DemoWbcController>("value", BuilderWithWorld(kRoot));
  ASSERT_EQ(up.rc, DemoWbcController::CallbackReturn::SUCCESS);
  rtc::ControllerState state = fx::MakeIiwa7LeapState();
  (void)up.ctrl->Compute(state);  // first tick: seed

  const pinocchio::SE3 root_goal(
      rtc::math::se3::RpyToRotationZyx(Eigen::Vector3d(kPose[3], kPose[4], kPose[5])),
      Eigen::Vector3d(kPose[0], kPose[1], kPose[2]));
  const pinocchio::SE3 root_T_world = up.ctrl->TaskTargetFramesForTesting().RootFromWorld();
  const pinocchio::SE3 world_goal = root_T_world.actInv(root_goal);
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(world_goal.rotation());
  const std::array<double, 6> world6{world_goal.translation().x(),
                                     world_goal.translation().y(),
                                     world_goal.translation().z(),
                                     rpy.x(),
                                     rpy.y(),
                                     rpy.z()};
  ASSERT_GT((world_goal.translation() - root_goal.translation()).norm(), 0.1)
      << "the world must be away from the root for the check to mean anything";

  ASSERT_EQ(Rejects(*up.ctrl, TaskGoal(kWorld, world6)), 0U);
  for (int i = 0; i < 3; ++i) {
    state.iteration += 1;
    (void)up.ctrl->Compute(state);
  }
  ASSERT_TRUE(up.ctrl->IsTcpGoalValidForTesting());
  const pinocchio::SE3& got = up.ctrl->GetTcpGoalForTesting();
  EXPECT_LT((got.translation() - root_goal.translation()).norm(), 1e-9);
  EXPECT_LT((got.rotation() - root_goal.rotation()).norm(), 1e-9);
}

}  // namespace
