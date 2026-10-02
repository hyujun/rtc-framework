// ── DemoJointController: the frame NAMES on_configure registers ──────────────
//
// The TF slot table and the vector-payload frame id are the only places this
// controller spells the arm root and the arm tip as names. The poses that fill
// those slots are resolved by frame id on a different path (OnDeviceConfigsSet),
// so the two can disagree without any pose assertion noticing: a slot labelled
// after the wrong link carries a correct pose under the wrong frame.
//
// This file pins the names for the two robots whose model this repository can
// load in a test (iiwa7_leap, and the ur5e_p1b fixture). It was written against
// the code that read them off `urdf.sub_models[0]`, before the controller
// learned to drive a robot whose first device group is a kinematic tree — so a
// change in where the names come from has to leave these exactly as they are.
//
// on_configure runs on a real LifecycleNode: the slot table is only built when
// the TF publisher exists, and the fixtures that stop at SetDeviceNameConfigs
// never get that far.

#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using integrated_bringup::DemoJointController;
using FramePairs = std::vector<std::pair<std::string, std::string>>;

// The publish entry is what makes on_configure build the TF publisher; without
// it the slot table stays empty and every assertion below would be vacuous.
const char* const kIiwa7LeapYaml = R"(
arm_dof: 7
robot_trajectory_speed: 2.0
hand_trajectory_speed: 3.0
robot_max_traj_velocity: 3.14
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
    publish:
      - topic: "transforms"
        role: "robot_transforms"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

const char* const kUr5eP1bYaml = R"(
arm_dof: 6
robot_trajectory_speed: 2.0
hand_trajectory_speed: 3.0
robot_max_traj_velocity: 3.14
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
fsm:
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  ur5e:
    subscribe:
      - topic: "ur5e/joint_goal"
        role: "target"
    publish:
      - topic: "transforms"
        role: "robot_transforms"
  p1b:
    subscribe:
      - topic: "p1b/joint_goal"
        role: "target"
)";

class JointTfSlotFrames : public ::testing::Test {
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

  /// The production bring-up order (rt_controller_node_params.cpp): model
  /// config + shared builder, LoadConfig, device configs, then on_configure.
  std::unique_ptr<DemoJointController> Configure(
      const char* yaml, const char* node_name, const rtc_urdf_bridge::ModelConfig& model_cfg,
      std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder,
      const std::map<std::string, rtc::DeviceNameConfig>& devices) {
    auto ctrl = std::make_unique<DemoJointController>("");
    ctrl->SetSystemModelConfig(model_cfg);
    ctrl->SetSharedModelBuilder(std::move(builder));
    ctrl->SetControlRate(500.0);
    const YAML::Node cfg = YAML::Load(yaml);
    ctrl->LoadConfig(cfg);
    ctrl->SetDeviceNameConfigs(devices);

    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, "", opts);
    const auto rc = ctrl->on_configure(rclcpp_lifecycle::State{}, node_, cfg);
    EXPECT_EQ(rc, DemoJointController::CallbackReturn::SUCCESS) << node_name;
    return ctrl;
  }

  // The publisher's lifetime is tied to the node, so it outlives Configure().
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

TEST_F(JointTfSlotFrames, Iiwa7LeapSlotsAreNamedAfterTheArmRootAndTip) {
  namespace fx = integrated_bringup::testfx;
  const auto ctrl =
      Configure(kIiwa7LeapYaml, "tf_slots_iiwa7_leap", fx::SharedIiwa7LeapModelConfig(),
                fx::SharedIiwa7LeapBuilder(), fx::MakeIiwa7LeapDeviceConfigs());

  const FramePairs expected = {
      {"link_0", "ee_link_actual"},        {"link_0", "thumb_tip_head_actual"},
      {"link_0", "index_tip_head_actual"}, {"link_0", "middle_tip_head_actual"},
      {"link_0", "ring_tip_head_actual"},  {"link_0", "virtual_tcp_actual"},
  };
  EXPECT_EQ(ctrl->TfSlotFramesForTesting(), expected);
  EXPECT_EQ(ctrl->OwnedStateFrameIdForTesting(), "link_0");
}

// This fixture declares TWO chains from the same root (`ur5e`: base → tool0,
// `ur5e_catch`: base → l_palm_link) and its hand tree hangs off `base_adapter`,
// not off the arm root. So it tells apart three things iiwa7_leap cannot: the
// arm tip from the other chain's tip, and the fingertip parent (the ARM root)
// from the hand tree's own root.
TEST_F(JointTfSlotFrames, Ur5eP1bSlotsAreNamedAfterTheArmRootAndTip) {
  namespace fx = integrated_bringup::testfx;
  const auto ctrl = Configure(kUr5eP1bYaml, "tf_slots_ur5e_p1b", fx::SharedUr5eP1bModelConfig(),
                              fx::SharedUr5eP1bBuilder(), fx::MakeUr5eP1bDeviceConfigs());

  const FramePairs expected = {
      {"base", "tool0_actual"},
      {"base", "l_thumb_tip_bracket_actual"},
      {"base", "l_index_tip_bracket_actual"},
      {"base", "l_middle_tip_bracket_actual"},
      {"base", "l_ring_tip_bracket_actual"},
      {"base", "virtual_tcp_actual"},
  };
  EXPECT_EQ(ctrl->TfSlotFramesForTesting(), expected);
  EXPECT_EQ(ctrl->OwnedStateFrameIdForTesting(), "base");
}

}  // namespace
