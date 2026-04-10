/// Unit tests for SetPoseZ action node.

#include "ur5e_bt_coordinator/action_nodes/set_pose_z.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace rtc_bt;

class SetPoseZTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    factory_.registerNodeType<SetPoseZ>("SetPoseZ");
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

TEST_F(SetPoseZTest, OverrideZ)
{
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="1.0;2.0;3.0;0.1;0.2;0.3"
                   z="0.085"
                   output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  // X, Y preserved
  EXPECT_DOUBLE_EQ(out.x, 1.0);
  EXPECT_DOUBLE_EQ(out.y, 2.0);
  // Z replaced with constant
  EXPECT_NEAR(out.z, 0.085, 1e-12);
  // Orientation preserved
  EXPECT_DOUBLE_EQ(out.roll, 0.1);
  EXPECT_DOUBLE_EQ(out.pitch, 0.2);
  EXPECT_DOUBLE_EQ(out.yaw, 0.3);
}

TEST_F(SetPoseZTest, DefaultNaNPassThrough)
{
  // No `z` port provided → default NaN → pass-through.
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="1.0;2.0;3.0;0.1;0.2;0.3"
                   output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_DOUBLE_EQ(out.x, 1.0);
  EXPECT_DOUBLE_EQ(out.y, 2.0);
  EXPECT_DOUBLE_EQ(out.z, 3.0);
  EXPECT_DOUBLE_EQ(out.roll, 0.1);
  EXPECT_DOUBLE_EQ(out.pitch, 0.2);
  EXPECT_DOUBLE_EQ(out.yaw, 0.3);
}

TEST_F(SetPoseZTest, ExplicitNaNPassThrough)
{
  // Explicit NaN on the port → pass-through.
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="5.0;6.0;7.0;0.0;0.0;0.0"
                   z="nan"
                   output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_DOUBLE_EQ(out.z, 7.0);
}

TEST_F(SetPoseZTest, NegativeZ)
{
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="0.0;0.0;0.5;0.0;0.0;0.0"
                   z="-0.25"
                   output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.z, -0.25, 1e-12);
}

TEST_F(SetPoseZTest, ZeroZ)
{
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="1.0;2.0;3.0;0.0;0.0;0.0"
                   z="0.0"
                   output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_DOUBLE_EQ(out.z, 0.0);
  // X, Y unchanged
  EXPECT_DOUBLE_EQ(out.x, 1.0);
  EXPECT_DOUBLE_EQ(out.y, 2.0);
}

TEST_F(SetPoseZTest, BlackboardInputPose)
{
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="{inp}"
                   z="0.1"
                   output_pose="{out}"/>)");

  Pose6D inp{5.0, 6.0, 7.0, 1.0, 2.0, 3.0};
  tree.rootBlackboard()->set("inp", inp);

  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.z, 0.1, 1e-12);
  // Other fields unchanged
  EXPECT_DOUBLE_EQ(out.x, 5.0);
  EXPECT_DOUBLE_EQ(out.y, 6.0);
  EXPECT_DOUBLE_EQ(out.roll, 1.0);
  EXPECT_DOUBLE_EQ(out.pitch, 2.0);
  EXPECT_DOUBLE_EQ(out.yaw, 3.0);
}

TEST_F(SetPoseZTest, BlackboardZValue)
{
  // Verify that the `z` port can itself come from the blackboard
  // (mirrors real usage: SubTree SlowDescend z="{constant_z}").
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="1.0;2.0;3.0;0.0;0.0;0.0"
                   z="{final_z}"
                   output_pose="{out}"/>)");

  tree.rootBlackboard()->set("final_z", 0.123);

  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.z, 0.123, 1e-12);
}

TEST_F(SetPoseZTest, BlackboardZNaNPassThrough)
{
  auto tree = CreateTree(
      R"(<SetPoseZ input_pose="1.0;2.0;3.0;0.0;0.0;0.0"
                   z="{final_z}"
                   output_pose="{out}"/>)");

  tree.rootBlackboard()->set(
      "final_z", std::numeric_limits<double>::quiet_NaN());

  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  // NaN → pass-through, original Z preserved
  EXPECT_DOUBLE_EQ(out.z, 3.0);
}
