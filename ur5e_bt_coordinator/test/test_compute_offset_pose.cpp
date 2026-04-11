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

// ─── Rotation: "add" mode (component-wise RPY sum) ──────────────────────────

TEST_F(ComputeOffsetPoseTest, AddModeRpyOnly)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="0.0;0.0;0.0;0.10;0.20;0.30"
                            offset_roll="0.05"
                            offset_pitch="-0.10"
                            offset_yaw="0.40"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  // Translation untouched
  EXPECT_DOUBLE_EQ(out.x, 0.0);
  EXPECT_DOUBLE_EQ(out.y, 0.0);
  EXPECT_DOUBLE_EQ(out.z, 0.0);
  // RPY = component-wise sum
  EXPECT_NEAR(out.roll,  0.15, 1e-12);
  EXPECT_NEAR(out.pitch, 0.10, 1e-12);
  EXPECT_NEAR(out.yaw,   0.70, 1e-12);
}

TEST_F(ComputeOffsetPoseTest, AddModeFullSixDof)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="1.0;2.0;3.0;0.10;0.20;0.30"
                            offset_x="0.5" offset_y="-0.3" offset_z="0.1"
                            offset_roll="0.01" offset_pitch="0.02" offset_yaw="0.03"
                            rotation_mode="add"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.x, 1.5, 1e-12);
  EXPECT_NEAR(out.y, 1.7, 1e-12);
  EXPECT_NEAR(out.z, 3.1, 1e-12);
  EXPECT_NEAR(out.roll,  0.11, 1e-12);
  EXPECT_NEAR(out.pitch, 0.22, 1e-12);
  EXPECT_NEAR(out.yaw,   0.33, 1e-12);
}

// ─── Rotation: "quat_body" mode (intrinsic / TCP-frame composition) ─────────

TEST_F(ComputeOffsetPoseTest, QuatBodyIdentityIsNoOp)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="1.0;2.0;3.0;0.10;0.20;0.30"
                            rotation_mode="quat_body"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.x, 1.0, 1e-12);
  EXPECT_NEAR(out.y, 2.0, 1e-12);
  EXPECT_NEAR(out.z, 3.0, 1e-12);
  // Round-trip rpy → quat → rpy must reproduce input within numeric tolerance
  EXPECT_NEAR(out.roll,  0.10, 1e-9);
  EXPECT_NEAR(out.pitch, 0.20, 1e-9);
  EXPECT_NEAR(out.yaw,   0.30, 1e-9);
}

TEST_F(ComputeOffsetPoseTest, QuatBodyYawFromIdentity)
{
  // From identity, applying yaw=π/2 in any frame should produce yaw=π/2,
  // pitch=0, roll=0 — the two compositions agree at identity.
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                            offset_yaw="1.5707963267948966"
                            rotation_mode="quat_body"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(out.roll,  0.0, 1e-9);
  EXPECT_NEAR(out.pitch, 0.0, 1e-9);
  EXPECT_NEAR(out.yaw,   M_PI_2, 1e-9);
}

// ─── Rotation: "quat_body" vs "quat_world" must differ for non-commuting ────

TEST_F(ComputeOffsetPoseTest, QuatBodyAndWorldDifferForNonCommuting)
{
  // Start with a non-trivial pose, apply a non-commuting offset (mostly roll),
  // and verify body-frame ≠ world-frame composition (otherwise the modes
  // would be indistinguishable in this BT).
  const std::string base = "0.0;0.0;0.0;0.10;0.30;0.50";

  auto body_tree = CreateTree(
      R"(<ComputeOffsetPose input_pose=")" + base +
      R"(" offset_roll="0.4" offset_pitch="0.0" offset_yaw="0.0"
              rotation_mode="quat_body"
              output_pose="{out}"/>)");
  auto world_tree = CreateTree(
      R"(<ComputeOffsetPose input_pose=")" + base +
      R"(" offset_roll="0.4" offset_pitch="0.0" offset_yaw="0.0"
              rotation_mode="quat_world"
              output_pose="{out}"/>)");

  ASSERT_EQ(body_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(world_tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out_body  = body_tree.rootBlackboard()->get<Pose6D>("out");
  auto out_world = world_tree.rootBlackboard()->get<Pose6D>("out");

  // At least one RPY component should differ noticeably.
  const double diff = std::abs(out_body.roll  - out_world.roll)
                    + std::abs(out_body.pitch - out_world.pitch)
                    + std::abs(out_body.yaw   - out_world.yaw);
  EXPECT_GT(diff, 1e-3) << "body and world quat composition produced "
                           "indistinguishable results";
}

// ─── Gimbal lock: result is returned (no FAILURE), pitch ≈ ±π/2 ─────────────

TEST_F(ComputeOffsetPoseTest, QuatBodyNearGimbalLockSucceeds)
{
  // Start near +π/2 pitch and add a tiny pitch nudge to land in the warn band.
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="0.0;0.0;0.0;0.0;1.5697963267948966;0.0"
                            offset_pitch="0.0005"
                            rotation_mode="quat_body"
                            output_pose="{out}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto out = tree.rootBlackboard()->get<Pose6D>("out");
  EXPECT_NEAR(std::abs(out.pitch), M_PI_2, 2e-3);
}

// ─── Invalid rotation_mode → FAILURE ───────────────────────────────────────

TEST_F(ComputeOffsetPoseTest, UnknownRotationModeFails)
{
  auto tree = CreateTree(
      R"(<ComputeOffsetPose input_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                            offset_yaw="0.1"
                            rotation_mode="bogus"
                            output_pose="{out}"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
