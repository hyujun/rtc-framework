// ── Hand fingertip FK: the device's joint order, and the mount on the arm tip ──
//
// A fingertip pose is a product of three things that come from three places:
//
//   T_root_fingertip = T_root_armtip · T_tip_mount · T_handroot_fingertip
//
// the arm tip from the arm's model, the fingertip from the hand's tree model
// (expressed in that tree's root link), and the constant transform between the
// two links. Two of them can be wrong with nothing else out of place:
//
//   - the hand model is fed the device's positions as they arrive. A device
//     that lists its joints in another order than the model's puts every value
//     on another joint;
//   - the hand's root link is not the arm's tip link. Composing the hand FK
//     directly on the arm tip is right only where the two coincide.
//
// Either one gives a pose that is finite, smooth and wrong by centimetres, on
// every tick, and nothing downstream can tell. What this file pins:
//
//   Part A — support/hand_fk_wiring: the map it installs, the mount it resolves
//            and what it refuses, on a synthetic model.
//   Part B — the controllers that publish fingertip poses, on a real
//            arm + hand whose device order is not the model's and whose hand
//            root is not the arm tip: the fingertips they report, the control
//            point a fingertip-based virtual TCP lands on, and the configure
//            they refuse.
//
// The oracle everywhere is forward kinematics on the FULL model, addressed by
// joint NAME. It shares no joint order, no reduced model and no frame id with
// the code under test.
//
// Every hand joint is given its own value. With the hand at zero, or all joints
// at one value, a permuted read is the same read.
//
// Part B runs both bring-up orders. They differ in where the hand handle is
// built relative to the device configs, and a joint order installed in one
// place only survives one of them:
//   - the controller manager's: PreConfigure (node + LoadConfig), device
//     configs, on_configure;
//   - a caller that skips PreConfigure: LoadConfig, device configs, then an
//     on_configure that loads the config a second time and rebuilds the handle.

#include "iiwa7_leap_controller_yamls.hpp"
#include "iiwa7_leap_hand_fk_oracle.hpp"
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/support/hand_fk_wiring.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
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
#include <span>
#include <string>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;
namespace fx = integrated_bringup::testfx;
using fx::ExpectSamePose;
using fx::FullModelPose;
using fx::kLeapTips;
using fx::ToSe3;
using integrated_bringup::DemoComplianceController;
using integrated_bringup::DemoJointController;
using integrated_bringup::DemoTaskController;
using integrated_bringup::DemoWbcController;
using integrated_bringup::HandFkWiring;
using integrated_bringup::InstallHandJointOrder;
using integrated_bringup::VirtualTcpMode;
using integrated_bringup::WireHandFk;

// ═══════════════════════════════════════════════════════════════════════════
// Part A — support/hand_fk_wiring on the synthetic tree fixture
// ═══════════════════════════════════════════════════════════════════════════
//
// rtc_urdf_bridge/test/urdf/dual_arm_tree_hand.urdf: a five-joint hand built on
// `hand_base`, which is fixed to `right_wrist` through a mount that is not the
// identity. The model orders the fingers a..d.

const std::vector<std::string> kSynTips = {"tip_a", "tip_b", "tip_c", "tip_d"};
// Not the model's order (finger_a_1, finger_a_2, finger_b_1, finger_c_1,
// finger_d_1), and no joint in its model slot.
const std::vector<std::string> kSynDeviceOrder = {"finger_c_1", "finger_a_1", "finger_d_1",
                                                  "finger_b_1", "finger_a_2"};
const std::vector<double> kSynDeviceQ = {0.50, 0.30, -0.25, 0.20, -0.40};

const rub::ModelConfig& SynModelConfig() {
  static const rub::ModelConfig cfg = [] {
    rub::ModelConfig c;
    c.urdf_path = rtc::test::TestUrdfPath("dual_arm_tree_hand.urdf");
    c.root_joint_type = "fixed";
    // The arm chain ends on the link BEFORE the hand mount.
    c.sub_models.push_back({"body", "pelvis", "right_wrist"});
    c.tree_models.push_back({"hand", "hand_base", kSynTips});
    return c;
  }();
  return cfg;
}

std::shared_ptr<rub::PinocchioModelBuilder> SynBuilder() {
  static const auto builder = std::make_shared<rub::PinocchioModelBuilder>(SynModelConfig());
  return builder;
}

rtc::DeviceNameConfig SynHandDevice(const std::vector<std::string>& names) {
  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  hand.joint_state_names = names;
  return hand;
}

std::map<std::string, double> ByName(const std::vector<std::string>& names,
                                     const std::vector<double>& q) {
  std::map<std::string, double> out;
  for (std::size_t i = 0; i < names.size(); ++i) {
    out[names[i]] = q[i];
  }
  return out;
}

/// Fingertip `tip` in the hand root, as the handle computes it from the
/// device-ordered positions.
pinocchio::SE3 HandleTipInRoot(rub::RtModelHandle& handle, const std::vector<double>& q_dev,
                               const std::string& tip) {
  handle.ComputeForwardKinematics(std::span<const double>(q_dev.data(), q_dev.size()));
  return handle.GetFramePlacement(handle.GetFrameId("hand_base"))
      .actInv(handle.GetFramePlacement(handle.GetFrameId(tip)));
}

TEST(HandFkWiringSupport, InstallsADeviceOrderThatIsNotTheModels) {
  const pinocchio::Model& full = *SynBuilder()->GetFullModel();
  const auto want = [&](const std::string& tip) {
    return FullModelPose(full, ByName(kSynDeviceOrder, kSynDeviceQ), "hand_base", tip);
  };

  // Canary: fed positionally, this order and these values put a fingertip
  // somewhere else — the assertion below can fail.
  rub::RtModelHandle positional(SynBuilder()->GetTreeModel("hand"));
  ASSERT_EQ(positional.nq(), static_cast<int>(kSynDeviceOrder.size()));
  for (const auto& tip : kSynTips) {
    EXPECT_GT(
        (HandleTipInRoot(positional, kSynDeviceQ, tip).translation() - want(tip).translation())
            .norm(),
        1e-3)
        << tip << ": the fixture does not tell a positional read from a mapped one";
  }

  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const auto device = SynHandDevice(kSynDeviceOrder);
  EXPECT_EQ(InstallHandJointOrder(&handle, &device, /*closed_chain_fk_active=*/false), "");
  EXPECT_TRUE(handle.HasJointReorder());
  for (const auto& tip : kSynTips) {
    ExpectSamePose(HandleTipInRoot(handle, kSynDeviceQ, tip), want(tip), 1e-12, tip);
  }
}

TEST(HandFkWiringSupport, ANameTheHandModelDoesNotCarryIsRefused) {
  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  auto names = kSynDeviceOrder;
  names[1] = "right_elbow";  // a real joint of the robot, but not one of the hand's
  const auto device = SynHandDevice(names);

  const std::string error = InstallHandJointOrder(&handle, &device, false);
  EXPECT_NE(error.find("right_elbow"), std::string::npos) << error;
  EXPECT_NE(error.find("not on the hand model"), std::string::npos) << error;
  EXPECT_FALSE(handle.HasJointReorder()) << "a refused order must not leave a map behind";
}

// RtModelHandle::SetJointOrder asks only that every name exists, so a list that
// covers a strict subset of the hand's joints maps — and leaves the rest of q
// at whatever the buffer holds.
TEST(HandFkWiringSupport, AListThatLeavesAHandJointUnfedIsRefused) {
  auto names = kSynDeviceOrder;
  names.erase(std::find(names.begin(), names.end(), "finger_b_1"));
  const auto device = SynHandDevice(names);

  rub::RtModelHandle bare(SynBuilder()->GetTreeModel("hand"));
  ASSERT_TRUE(bare.SetJointOrder(names)) << "the handle itself is expected to accept a subset";

  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const std::string error = InstallHandJointOrder(&handle, &device, false);
  EXPECT_NE(error.find("finger_b_1"), std::string::npos) << error;
  EXPECT_NE(error.find("no slot"), std::string::npos) << error;
  EXPECT_FALSE(handle.HasJointReorder());
}

TEST(HandFkWiringSupport, AJointListedTwiceIsRefused) {
  auto names = kSynDeviceOrder;
  names.push_back("finger_c_1");  // every joint covered, one of them twice
  const auto device = SynHandDevice(names);

  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const std::string error = InstallHandJointOrder(&handle, &device, false);
  EXPECT_NE(error.find("finger_c_1"), std::string::npos) << error;
  EXPECT_NE(error.find("more than once"), std::string::npos) << error;
  EXPECT_FALSE(handle.HasJointReorder());
}

// A device slot carries one position. A joint that takes another number of them
// would leave the handle's map out of step with the device list from there on;
// the model's own root joint (no position at all) is the one every model has.
TEST(HandFkWiringSupport, AJointThatDoesNotTakeOnePositionIsRefused) {
  auto names = kSynDeviceOrder;
  names.push_back("universe");
  const auto device = SynHandDevice(names);

  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  ASSERT_TRUE(handle.GetModel().existJointName("universe"));
  const std::string error = InstallHandJointOrder(&handle, &device, false);
  EXPECT_NE(error.find("universe"), std::string::npos) << error;
  EXPECT_NE(error.find("takes 0 position values"), std::string::npos) << error;
  EXPECT_FALSE(handle.HasJointReorder());
}

// No hand, no device config, no names yet, or a closed-chain hand whose serial
// handle is never read: nothing to install and nothing to refuse.
TEST(HandFkWiringSupport, NothingToInstallIsNotAnError) {
  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const auto good = SynHandDevice(kSynDeviceOrder);
  const auto empty = SynHandDevice({});
  auto bad_names = kSynDeviceOrder;
  bad_names[0] = "no_such_joint";
  const auto bad = SynHandDevice(bad_names);

  EXPECT_EQ(InstallHandJointOrder(nullptr, &good, false), "");
  EXPECT_EQ(InstallHandJointOrder(&handle, nullptr, false), "");
  EXPECT_EQ(InstallHandJointOrder(&handle, &empty, false), "");
  EXPECT_EQ(InstallHandJointOrder(&handle, &bad, /*closed_chain_fk_active=*/true), "");
  EXPECT_FALSE(handle.HasJointReorder());

  // ... and the same bad list IS refused once the serial handle is the one read.
  EXPECT_NE(InstallHandJointOrder(&handle, &bad, false), "");
}

/// `hand_root` empty stands for "no tree model declared for the hand".
HandFkWiring WireSyn(rub::RtModelHandle* handle, const rtc::DeviceNameConfig* device,
                     const std::string& arm_tip, const std::string& hand_root) {
  // The arm chain's model keeps every link of the robot as a frame, so any of
  // the links used below resolves on it — as it does on a controller's.
  const rub::RtModelHandle arm(SynBuilder()->GetReducedModel("body"));
  const rub::TreeModelConfig hand_tree{"hand", hand_root, kSynTips};
  return WireHandFk({
      .hand_handle = handle,
      .hand_device = device,
      .closed_chain_fk_active = false,
      .model = SynBuilder()->GetFullModel().get(),
      .arm_handle = &arm,
      .arm_tip_link = arm_tip,
      .hand_tree = &hand_tree,
  });
}

TEST(HandFkWiringSupport, TheMountIsThePlacementOfTheHandRootInTheArmTip) {
  const pinocchio::Model& full = *SynBuilder()->GetFullModel();
  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const auto device = SynHandDevice(kSynDeviceOrder);

  // Read off an FK at a configuration that moves every joint: a constant has to
  // come out the same wherever it is measured.
  std::map<std::string, double> q = ByName(kSynDeviceOrder, kSynDeviceQ);
  q["waist_yaw"] = 0.35;
  q["right_shoulder"] = -0.50;
  q["right_elbow"] = 0.70;
  const pinocchio::SE3 want = FullModelPose(full, q, "right_wrist", "hand_base");
  ASSERT_GT(want.translation().norm(), 0.03) << "the fixture's mount must not be the identity";
  ASSERT_GT((want.rotation() - Eigen::Matrix3d::Identity()).norm(), 1.0);

  const HandFkWiring wiring = WireSyn(&handle, &device, "right_wrist", "hand_base");
  EXPECT_EQ(wiring.Error(), "");
  ExpectSamePose(wiring.T_tip_mount, want, 1e-12, "mount");
  EXPECT_TRUE(handle.HasJointReorder()) << "wiring the mount also installs the joint order";

  // A hand built directly on the arm tip mounts through the identity.
  const HandFkWiring same = WireSyn(&handle, &device, "hand_base", "hand_base");
  EXPECT_EQ(same.Error(), "");
  ExpectSamePose(same.T_tip_mount, pinocchio::SE3::Identity(), 0.0, "self mount");
}

// The mount is a constant only when no joint moves between the two links. The
// check has to be made on the full model: the arm's own model has every hand
// joint locked, and there even a fingertip hangs off the arm's last joint.
TEST(HandFkWiringSupport, AHandRootAJointSeparatesFromTheArmTipIsRefused) {
  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const auto device = SynHandDevice(kSynDeviceOrder);

  // Canary for the comment above: on the arm chain's reduced model the
  // fingertip and the arm tip share a parent joint.
  const auto arm = SynBuilder()->GetReducedModel("body");
  ASSERT_TRUE(arm->existFrame("tip_a"));
  ASSERT_EQ(arm->frames[arm->getFrameId("tip_a")].parentJoint,
            arm->frames[arm->getFrameId("right_wrist")].parentJoint);

  // Downstream of a hand joint.
  const HandFkWiring past_a_finger = WireSyn(&handle, &device, "right_wrist", "tip_a");
  EXPECT_NE(past_a_finger.mount_error.find("not rigidly attached"), std::string::npos)
      << past_a_finger.mount_error;
  // Upstream of an arm joint.
  const HandFkWiring before_the_elbow = WireSyn(&handle, &device, "torso", "hand_base");
  EXPECT_NE(before_the_elbow.mount_error.find("not rigidly attached"), std::string::npos)
      << before_the_elbow.mount_error;
  ExpectSamePose(before_the_elbow.T_tip_mount, pinocchio::SE3::Identity(), 0.0,
                 "a refused mount stays at the identity");
}

TEST(HandFkWiringSupport, AHandRootThatIsNotOnTheModelIsRefused) {
  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const auto device = SynHandDevice(kSynDeviceOrder);

  const HandFkWiring missing = WireSyn(&handle, &device, "right_wrist", "no_such_link");
  EXPECT_NE(missing.mount_error.find("no_such_link"), std::string::npos) << missing.mount_error;
  const HandFkWiring undeclared = WireSyn(&handle, &device, "right_wrist", "");
  EXPECT_NE(undeclared.mount_error.find("no root_link"), std::string::npos)
      << undeclared.mount_error;
}

// What is not this wiring's to refuse: a config with no hand model at all, and
// an arm tip that did not resolve (there is then nothing to mount on).
TEST(HandFkWiringSupport, NoHandModelOrNoArmTipLeavesTheMountAlone) {
  const auto device = SynHandDevice(kSynDeviceOrder);
  const HandFkWiring no_hand = WireSyn(nullptr, &device, "right_wrist", "no_such_link");
  EXPECT_EQ(no_hand.Error(), "");

  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  const HandFkWiring no_tip = WireSyn(&handle, &device, "", "no_such_link");
  EXPECT_EQ(no_tip.Error(), "");
  ExpectSamePose(no_tip.T_tip_mount, pinocchio::SE3::Identity(), 0.0, "no arm tip");
  EXPECT_TRUE(handle.HasJointReorder()) << "the joint order does not depend on the arm tip";

  // A tip link the arm model does not carry is the same case: nothing to mount on.
  const HandFkWiring unknown_tip = WireSyn(&handle, &device, "no_such_tip", "no_such_link");
  EXPECT_EQ(unknown_tip.Error(), "");

  // ... and so is a controller with no arm model at all.
  const rub::TreeModelConfig hand_tree{"hand", "no_such_link", kSynTips};
  const HandFkWiring no_arm = WireHandFk({
      .hand_handle = &handle,
      .hand_device = &device,
      .closed_chain_fk_active = false,
      .model = SynBuilder()->GetFullModel().get(),
      .arm_handle = nullptr,
      .arm_tip_link = "right_wrist",
      .hand_tree = &hand_tree,
  });
  EXPECT_EQ(no_arm.Error(), "");
}

// The joint-order verdict wins when both fail: it is the one a config author
// can act on first, and Error() is what on_configure prints.
TEST(HandFkWiringSupport, ErrorReportsTheJointOrderBeforeTheMount) {
  rub::RtModelHandle handle(SynBuilder()->GetTreeModel("hand"));
  auto names = kSynDeviceOrder;
  names[0] = "no_such_joint";
  const auto device = SynHandDevice(names);
  const HandFkWiring both = WireSyn(&handle, &device, "torso", "hand_base");
  ASSERT_FALSE(both.joint_order_error.empty());
  ASSERT_FALSE(both.mount_error.empty());
  EXPECT_EQ(both.Error(), both.joint_order_error);
}

// ═══════════════════════════════════════════════════════════════════════════
// Part B — the four controllers on iiwa7 + LEAP
// ═══════════════════════════════════════════════════════════════════════════

/// Where the full model puts `frame`, in the arm root, at the test state.
pinocchio::SE3 Oracle(const std::string& frame) {
  return fx::Iiwa7LeapOracle(frame);
}

// ── Device configs a controller has to refuse ──

std::map<std::string, rtc::DeviceNameConfig> WithAHandJointTheModelDoesNotCarry() {
  auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  devices.at("leap").joint_state_names[3] = "A1";  // a real joint of the robot — the arm's
  return devices;
}

std::map<std::string, rtc::DeviceNameConfig> WithAHandJointLeftOut() {
  auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  devices.at("leap").joint_state_names.pop_back();
  return devices;
}

std::map<std::string, rtc::DeviceNameConfig> WithTheArmTipOneJointBeforeTheHand() {
  auto devices = fx::MakeIiwa7LeapDeviceConfigs();
  devices.at("iiwa7").urdf->tip_link = "link_6";  // A7 turns between it and the hand
  return devices;
}

enum class BringUp {
  kControllerManager,  ///< PreConfigure → device configs → on_configure
  kConfigLoadedTwice,  ///< LoadConfig → device configs → on_configure (reloads)
};

const char* Name(BringUp order) {
  return order == BringUp::kControllerManager ? "cm" : "reload";
}

template <class Ctrl>
struct Configured {
  std::unique_ptr<Ctrl> ctrl;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;  // owns the publishers' lifetime
  typename Ctrl::CallbackReturn rc{Ctrl::CallbackReturn::ERROR};
};

class HandFkWiringControllers : public ::testing::Test {
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

  template <class Ctrl>
  static Configured<Ctrl> Configure(BringUp order,
                                    const std::map<std::string, rtc::DeviceNameConfig>& devices,
                                    const std::string& tag) {
    using Fx = fx::ControllerFixture<Ctrl>;
    Configured<Ctrl> out;
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    out.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        std::string("hand_fk_") + Fx::kName + "_" + Name(order) + "_" + tag, "", opts);

    out.ctrl = Fx::Make();
    out.ctrl->SetSystemModelConfig(fx::SharedIiwa7LeapModelConfig());
    out.ctrl->SetSharedModelBuilder(fx::SharedIiwa7LeapBuilder());
    out.ctrl->SetControlRate(1.0 / fx::kDt);
    const YAML::Node yaml = YAML::Load(Fx::Yaml());
    if (order == BringUp::kControllerManager) {
      EXPECT_EQ(out.ctrl->PreConfigure(out.node, yaml), Ctrl::CallbackReturn::SUCCESS) << Fx::kName;
    } else {
      out.ctrl->LoadConfig(yaml);
    }
    out.ctrl->SetDeviceNameConfigs(devices);
    out.rc = out.ctrl->on_configure(rclcpp_lifecycle::State{}, out.node, yaml);
    return out;
  }

  /// Self-init tick plus a few holds at the test state.
  template <class Ctrl>
  static rtc::ControllerOutput Run(Ctrl& ctrl, int ticks = 4) {
    rtc::ControllerState state = fx::MakeIiwa7LeapStateWithHandPose();
    rtc::ControllerOutput out = ctrl.Compute(state);
    for (int i = 0; i < ticks; ++i) {
      state.iteration += 1;
      out = ctrl.Compute(state);
    }
    return out;
  }

  template <class Ctrl>
  static void ExpectFingertipsAreTheFullModels(BringUp order) {
    const std::string who = std::string(fx::ControllerFixture<Ctrl>::kName) + "/" + Name(order);
    auto up = Configure<Ctrl>(order, fx::MakeIiwa7LeapDeviceConfigs(), "tips");
    ASSERT_EQ(up.rc, Ctrl::CallbackReturn::SUCCESS)
        << who << ": " << up.ctrl->HandFkWiringErrorForTesting();
    const rtc::ControllerOutput out = Run(*up.ctrl);

    ASSERT_TRUE(out.arm_tip_pose_valid) << who;
    ExpectSamePose(ToSe3(out.arm_tip_pose), Oracle("ee_link"), 1e-9, who + " arm tip");
    for (std::size_t f = 0; f < kLeapTips.size(); ++f) {
      ASSERT_TRUE(out.task_link_pose_valid[f]) << who << " " << kLeapTips[f];
      ExpectSamePose(ToSe3(out.task_link_poses[f]), Oracle(kLeapTips[f]), 1e-9,
                     who + " " + kLeapTips[f]);
    }
  }

  // A fingertip-based virtual TCP is built from the same hand FK: its centroid
  // mode is the mean of the fingertip positions. (The constant-offset mode
  // reads no fingertip and is measured from the arm tip.)
  template <class Ctrl>
  static void ExpectCentroidVirtualTcpIsTheFullModels(BringUp order) {
    const std::string who = std::string(fx::ControllerFixture<Ctrl>::kName) + "/" + Name(order);
    auto up = Configure<Ctrl>(order, fx::MakeIiwa7LeapDeviceConfigs(), "vtcp");
    ASSERT_EQ(up.rc, Ctrl::CallbackReturn::SUCCESS)
        << who << ": " << up.ctrl->HandFkWiringErrorForTesting();
    auto gains = up.ctrl->get_gains();
    gains.vtcp.mode = VirtualTcpMode::kCentroid;
    up.ctrl->set_gains(gains);
    const rtc::ControllerOutput out = Run(*up.ctrl);

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& tip : kLeapTips) {
      centroid += Oracle(tip).translation();
    }
    centroid /= static_cast<double>(kLeapTips.size());

    ASSERT_TRUE(out.virtual_tcp_pose_valid) << who;
    const Eigen::Vector3d got(out.virtual_tcp_pose.position[0], out.virtual_tcp_pose.position[1],
                              out.virtual_tcp_pose.position[2]);
    EXPECT_LE((got - centroid).norm(), 1e-9)
        << who << ": centroid virtual TCP off by " << (got - centroid).norm() << " m";
  }

  /// One refused device config next to the accepted one, on the same rig: the
  /// refusal is about that config, and for the stated reason.
  template <class Ctrl>
  static void ExpectRefused(const std::map<std::string, rtc::DeviceNameConfig>& devices,
                            const std::string& tag, const std::string& reason) {
    for (const BringUp order : {BringUp::kControllerManager, BringUp::kConfigLoadedTwice}) {
      const std::string who = std::string(fx::ControllerFixture<Ctrl>::kName) + "/" + Name(order);
      const auto good = Configure<Ctrl>(order, fx::MakeIiwa7LeapDeviceConfigs(), tag + "_ok");
      EXPECT_EQ(good.rc, Ctrl::CallbackReturn::SUCCESS) << who;
      EXPECT_EQ(good.ctrl->HandFkWiringErrorForTesting(), "") << who;

      const auto bad = Configure<Ctrl>(order, devices, tag);
      EXPECT_EQ(bad.rc, Ctrl::CallbackReturn::FAILURE) << who;
      EXPECT_NE(bad.ctrl->HandFkWiringErrorForTesting().find(reason), std::string::npos)
          << who << ": refused for another reason, or not at all — '"
          << bad.ctrl->HandFkWiringErrorForTesting() << "'";
    }
  }

  template <class Ctrl>
  static void ExpectAllThreeRefusals() {
    ExpectRefused<Ctrl>(WithAHandJointTheModelDoesNotCarry(), "name", "not on the hand model");
    ExpectRefused<Ctrl>(WithAHandJointLeftOut(), "width", "no slot in joint_state_names");
    ExpectRefused<Ctrl>(WithTheArmTipOneJointBeforeTheHand(), "mount", "not rigidly attached");
  }
};

// ── Canary: this rig can fail the assertions below ──────────────────────────

TEST_F(HandFkWiringControllers, FixtureCanTellTheCasesApart) {
  fx::ExpectIiwa7LeapRigTellsTheCasesApart();
}

// ── Fingertips ──────────────────────────────────────────────────────────────

TEST_F(HandFkWiringControllers, JointFingertipsAreTheFullModels) {
  ExpectFingertipsAreTheFullModels<DemoJointController>(BringUp::kControllerManager);
  ExpectFingertipsAreTheFullModels<DemoJointController>(BringUp::kConfigLoadedTwice);
}

TEST_F(HandFkWiringControllers, TaskFingertipsAreTheFullModels) {
  ExpectFingertipsAreTheFullModels<DemoTaskController>(BringUp::kControllerManager);
  ExpectFingertipsAreTheFullModels<DemoTaskController>(BringUp::kConfigLoadedTwice);
}

TEST_F(HandFkWiringControllers, ComplianceFingertipsAreTheFullModels) {
  ExpectFingertipsAreTheFullModels<DemoComplianceController>(BringUp::kControllerManager);
  ExpectFingertipsAreTheFullModels<DemoComplianceController>(BringUp::kConfigLoadedTwice);
}

// DemoWbcController publishes poses only with TSID up; its fingertip cases run
// on the TSID rig, in test_demo_wbc_tsid_path.cpp.

// ── Fingertip-based virtual TCP ─────────────────────────────────────────────

TEST_F(HandFkWiringControllers, JointCentroidVirtualTcpIsTheFullModels) {
  ExpectCentroidVirtualTcpIsTheFullModels<DemoJointController>(BringUp::kControllerManager);
  ExpectCentroidVirtualTcpIsTheFullModels<DemoJointController>(BringUp::kConfigLoadedTwice);
}

TEST_F(HandFkWiringControllers, TaskCentroidVirtualTcpIsTheFullModels) {
  ExpectCentroidVirtualTcpIsTheFullModels<DemoTaskController>(BringUp::kControllerManager);
  ExpectCentroidVirtualTcpIsTheFullModels<DemoTaskController>(BringUp::kConfigLoadedTwice);
}

TEST_F(HandFkWiringControllers, ComplianceCentroidVirtualTcpIsTheFullModels) {
  ExpectCentroidVirtualTcpIsTheFullModels<DemoComplianceController>(BringUp::kControllerManager);
  ExpectCentroidVirtualTcpIsTheFullModels<DemoComplianceController>(BringUp::kConfigLoadedTwice);
}

// ── Refusals ────────────────────────────────────────────────────────────────

TEST_F(HandFkWiringControllers, JointRefusesAHandItCannotWire) {
  ExpectAllThreeRefusals<DemoJointController>();
}

TEST_F(HandFkWiringControllers, TaskRefusesAHandItCannotWire) {
  ExpectAllThreeRefusals<DemoTaskController>();
}

TEST_F(HandFkWiringControllers, ComplianceRefusesAHandItCannotWire) {
  ExpectAllThreeRefusals<DemoComplianceController>();
}

TEST_F(HandFkWiringControllers, WbcRefusesAHandItCannotWire) {
  ExpectAllThreeRefusals<DemoWbcController>();
}

}  // namespace
