/// Unit tests for ComputeOffsetPose action node.

#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <cmath>

using namespace rtc_bt;

class ComputeOffsetPoseTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    factory_.registerNodeType<ComputeOffsetPose>("ComputeOffsetPose");
  }

  /// Create a single-node tree with the given XML snippet.
  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full_xml =
        R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)" + xml +
        R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full_xml);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(ComputeOffsetPoseTest, ZeroOffset)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="1.0;2.0;3.0;0.1;0.2;0.3"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_DOUBLE_EQ(out.x, 1.0);
  EXPECT_DOUBLE_EQ(out.y, 2.0);
  EXPECT_DOUBLE_EQ(out.z, 3.0);
  // Orientation preserved
  EXPECT_DOUBLE_EQ(out.roll, 0.1);
  EXPECT_DOUBLE_EQ(out.pitch, 0.2);
  EXPECT_DOUBLE_EQ(out.yaw, 0.3);
}

TEST_F(ComputeOffsetPoseTest, ApplyOffsets)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="1.0;2.0;3.0;0.1;0.2;0.3"
                            offset_x="0.5" offset_y="-0.3" offset_z="0.1"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.x, 1.5, 1e-12);
  EXPECT_NEAR(out.y, 1.7, 1e-12);
  EXPECT_NEAR(out.z, 3.1, 1e-12);
  // Orientation unchanged
  EXPECT_DOUBLE_EQ(out.roll, 0.1);
  EXPECT_DOUBLE_EQ(out.pitch, 0.2);
  EXPECT_DOUBLE_EQ(out.yaw, 0.3);
}

TEST_F(ComputeOffsetPoseTest, NegativeOffsets)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                            offset_x="-1.0" offset_y="-2.0" offset_z="-3.0"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_DOUBLE_EQ(out.x, -1.0);
  EXPECT_DOUBLE_EQ(out.y, -2.0);
  EXPECT_DOUBLE_EQ(out.z, -3.0);
}

TEST_F(ComputeOffsetPoseTest, OnlyZOffset)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="1.0;2.0;3.0;0.0;0.0;0.0"
                            offset_z="0.05"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_DOUBLE_EQ(out.x, 1.0);
  EXPECT_DOUBLE_EQ(out.y, 2.0);
  EXPECT_NEAR(out.z, 3.05, 1e-12);
}

TEST_F(ComputeOffsetPoseTest, BlackboardInputPose)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="{inp}"
                            offset_x="0.1"
                            output_pose="{out}"/>)");

  Pose6D inp{5.0, 6.0, 7.0, 1.0, 2.0, 3.0};
  tree.rootBlackboard()->set("inp", inp);

  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.x, 5.1, 1e-12);
}
