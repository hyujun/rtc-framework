/// Unit tests for MoveToJoints action node.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class MoveToJointsTest : public RosTestFixture {
protected:
  void SetUp() override
  {
    RosTestFixture::SetUp();
    factory_.registerNodeType<MoveToJoints>("MoveToJoints", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(MoveToJointsTest, StartsRunning)
{
  auto tree = CreateTree(
      R"(<MoveToJoints target="0.0;0.0;0.0;0.0;0.0;0.0"
                       tolerance="0.01" timeout_s="5.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(MoveToJointsTest, SucceedsWhenConverged)
{
  auto tree = CreateTree(
      R"(<MoveToJoints target="0.1;0.2;0.3;0.4;0.5;0.6"
                       tolerance="0.05" timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish joints at target
  Pose6D dummy{};
  PublishArmState(dummy, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(MoveToJointsTest, FailsOnTimeout)
{
  auto tree = CreateTree(
      R"(<MoveToJoints target="1.0;1.0;1.0;1.0;1.0;1.0"
                       tolerance="0.001" timeout_s="0.05"/>)");

  Pose6D dummy{};
  PublishArmState(dummy, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(MoveToJointsTest, PoseNameLookup)
{
  // "home_pose" is a compile-time default in kUR5ePoses (all zeros)
  auto tree = CreateTree(
      R"(<MoveToJoints pose_name="home_pose"
                       tolerance="0.05" timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish joints at home (all zeros)
  Pose6D dummy{};
  PublishArmState(dummy, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(MoveToJointsTest, PoseNameOverridesTarget)
{
  // Even if target is set, pose_name takes precedence
  auto tree = CreateTree(
      R"(<MoveToJoints pose_name="home_pose"
                       target="99.0;99.0;99.0;99.0;99.0;99.0"
                       tolerance="0.05" timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish zeros — should match home_pose, not the nonsense target
  Pose6D dummy{};
  PublishArmState(dummy, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(MoveToJointsTest, WithinToleranceBoundary)
{
  auto tree = CreateTree(
      R"(<MoveToJoints target="1.0;1.0;1.0;1.0;1.0;1.0"
                       tolerance="0.02" timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Max error = 0.015 < tolerance 0.02 → SUCCESS
  Pose6D dummy{};
  PublishArmState(dummy, {1.015, 1.01, 0.99, 1.005, 0.995, 1.0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}
