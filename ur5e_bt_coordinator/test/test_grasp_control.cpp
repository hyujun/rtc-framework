/// Unit tests for GraspControl action node.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/grasp_control.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class GraspControlTest : public RosTestFixture {
protected:
  void SetUp() override
  {
    RosTestFixture::SetUp();
    factory_.registerNodeType<GraspControl>("GraspControl", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(GraspControlTest, OpenModeStartsRunning)
{
  auto tree = CreateTree(
      R"(<GraspControl mode="open"
                       target_positions="0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0"
                       timeout_s="5.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(GraspControlTest, OpenModeSucceedsAtTarget)
{
  auto tree = CreateTree(
      R"(<GraspControl mode="open"
                       target_positions="0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0"
                       timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish hand at target (all zeros)
  PublishHandState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(GraspControlTest, CloseModeStartsRunning)
{
  // Provide hand state so close mode can read current positions
  PublishHandState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  Spin();

  auto tree = CreateTree(
      R"(<GraspControl mode="close"
                       close_speed="0.3" max_position="1.4"
                       timeout_s="5.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(GraspControlTest, CloseModeFailsAtMaxPosition)
{
  // Start near max_position so all motors hit max quickly
  PublishHandState({1.39, 1.39, 1.39, 1.39, 1.39, 1.39, 1.39, 1.39, 1.39, 1.39});
  Spin();

  auto tree = CreateTree(
      R"(<GraspControl mode="close"
                       close_speed="100.0" max_position="1.4"
                       timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  // After one increment, all should be at max → FAILURE
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(GraspControlTest, CloseModeTimeout)
{
  PublishHandState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  Spin();

  auto tree = CreateTree(
      R"(<GraspControl mode="close"
                       close_speed="0.001" max_position="999.0"
                       timeout_s="0.05"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(GraspControlTest, PinchModeOnlyAffectsSpecifiedMotors)
{
  PublishHandState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  Spin();

  auto tree = CreateTree(
      R"(<GraspControl mode="pinch"
                       pinch_motors="0,1"
                       close_speed="0.3" max_position="1.4"
                       timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  // Should be running since motors 0,1 haven't reached max
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(GraspControlTest, PresetModeSucceeds)
{
  auto tree = CreateTree(
      R"(<GraspControl mode="preset"
                       target_positions="0.5;0.5;0.5;0.5;0.5;0.5;0.5;0.5;0.5;0.5"
                       timeout_s="5.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Hand reaches target
  PublishHandState({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}
