/// Target-frame contract of the bridge (#779): every arm task goal carries the
/// frame its pose is in, and that frame is the one the TCP is read in.
///
/// The controllers take a task goal only in their root_link or the scene's
/// world, and refuse one with no frame. The bridge builds its goals from TCP
/// poses read in RobotProfile::base_frame, so it stamps that same name — a
/// root_link the coordinator was not told about then shows up as refused goals
/// instead of goals read in the wrong frame.

#include "inject_fixture.hpp"
#include "ur5e_bt_coordinator/robot_profile.hpp"
#include <rtc_msgs/msg/robot_target.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace rtc_bt::test {
namespace {

constexpr const char* kController = "demo_task_controller";

class TargetFrameTest : public ::testing::Test {
 protected:
  void Make(RobotProfile profile) {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "bt_target_frame_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    bridge_ = std::make_shared<BtRosBridge>(node_, std::move(profile));
    injector_ = std::make_unique<BridgeStateInjector>(bridge_);
    // Lifecycle publishers drop what an inactive node publishes; the real
    // coordinator is active when it sends.
    node_->configure();
    node_->activate();
    injector_->ActiveController(kController);  // creates the target publishers
    sub_ = node_->create_subscription<rtc_msgs::msg::RobotTarget>(
        std::string("/") + kController + "/ur5e/joint_goal", rclcpp::QoS{10},
        [this](rtc_msgs::msg::RobotTarget::SharedPtr msg) { seen_.push_back(*msg); });
    exec_.add_node(node_->get_node_base_interface());
  }

  void TearDown() override {
    if (node_) {
      exec_.remove_node(node_->get_node_base_interface());
    }
    sub_.reset();
    injector_.reset();
    bridge_.reset();
    node_.reset();
  }

  /// Intra-process delivery still needs the subscription's executor to run.
  void Pump() {
    for (int i = 0; i < 20 && seen_.empty(); ++i) {
      exec_.spin_some(std::chrono::milliseconds(5));
    }
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<BtRosBridge> bridge_;
  std::unique_ptr<BridgeStateInjector> injector_;
  rclcpp::Subscription<rtc_msgs::msg::RobotTarget>::SharedPtr sub_;
  std::vector<rtc_msgs::msg::RobotTarget> seen_;
};

geometry_msgs::msg::Polygon OnePoint(float x, float y, float z) {
  geometry_msgs::msg::Polygon poly;
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  poly.points.push_back(p);
  return poly;
}

TEST_F(TargetFrameTest, ATaskGoalCarriesTheBaseFrameByDefault) {
  Make(RobotProfile{});
  bridge_->PublishArmTarget(Pose6D{0.1, 0.2, 0.3, 0.0, 0.0, 0.0});
  Pump();
  ASSERT_EQ(seen_.size(), 1U);
  EXPECT_EQ(seen_[0].goal_type, "task");
  EXPECT_EQ(seen_[0].header.frame_id, "base");
}

TEST_F(TargetFrameTest, ATaskGoalCarriesTheConfiguredBaseFrame) {
  RobotProfile profile;
  profile.base_frame = "link_0";
  Make(std::move(profile));
  bridge_->PublishArmTarget(Pose6D{0.1, 0.2, 0.3, 0.0, 0.0, 0.0});
  Pump();
  ASSERT_EQ(seen_.size(), 1U);
  EXPECT_EQ(seen_[0].header.frame_id, "link_0");
}

TEST_F(TargetFrameTest, AWorldTargetInTheBaseFrameIsUsed) {
  RobotProfile profile;
  profile.world_target_frame = "base";  // explicit, same as base_frame
  Make(std::move(profile));
  injector_->WorldTarget(OnePoint(0.4F, 0.1F, 0.2F));
  Pose6D pose;
  ASSERT_TRUE(bridge_->GetWorldTargetPose(pose));
  EXPECT_NEAR(pose.x, 0.4, 1e-6);
}

TEST_F(TargetFrameTest, AWorldTargetInAnotherFrameIsIgnored) {
  // The tree pairs the position with the TCP orientation (base axes); a
  // position in another frame cannot be sent as one goal.
  RobotProfile profile;
  profile.world_target_frame = "scene_world";
  Make(std::move(profile));
  injector_->WorldTarget(OnePoint(0.4F, 0.1F, 0.2F));
  Pose6D pose;
  EXPECT_FALSE(bridge_->GetWorldTargetPose(pose));
}

}  // namespace
}  // namespace rtc_bt::test
