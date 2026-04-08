/// Unit tests for MoveToPose action node.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class MoveToPoseTest : public RosTestFixture {
protected:
  void SetUp() override
  {
    RosTestFixture::SetUp();
    factory_.registerNodeType<MoveToPose>("MoveToPose", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(MoveToPoseTest, StartsRunning)
{
  auto tree = CreateTree(
      R"(<MoveToPose target="0.3;-0.3;0.15;3.14;0.0;0.0"
                     position_tolerance="0.01" timeout_s="5.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(MoveToPoseTest, SucceedsWhenAtTarget)
{
  auto tree = CreateTree(
      R"(<MoveToPose target="0.3;-0.3;0.15;3.14;0.0;0.0"
                     position_tolerance="0.01"
                     orientation_tolerance="0.1"
                     timeout_s="5.0"/>)");

  // First tick starts movement
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish TCP pose at target
  Pose6D at_target{0.3, -0.3, 0.15, 3.14, 0.0, 0.0};
  PublishArmState(at_target, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(MoveToPoseTest, StillRunningWhenFar)
{
  auto tree = CreateTree(
      R"(<MoveToPose target="0.3;-0.3;0.15;3.14;0.0;0.0"
                     position_tolerance="0.001"
                     timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish pose far from target
  Pose6D far{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  PublishArmState(far, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(MoveToPoseTest, FailsOnTimeout)
{
  auto tree = CreateTree(
      R"(<MoveToPose target="0.3;-0.3;0.15;3.14;0.0;0.0"
                     position_tolerance="0.001"
                     timeout_s="0.05"/>)");

  // Publish far-away pose
  Pose6D far{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  PublishArmState(far, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(MoveToPoseTest, WithinPositionButNotOrientation)
{
  auto tree = CreateTree(
      R"(<MoveToPose target="0.3;-0.3;0.15;3.14;0.0;0.0"
                     position_tolerance="0.01"
                     orientation_tolerance="0.01"
                     timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Position close, but orientation differs
  Pose6D close_pos{0.3, -0.3, 0.15, 0.0, 0.0, 0.0};
  PublishArmState(close_pos, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}
