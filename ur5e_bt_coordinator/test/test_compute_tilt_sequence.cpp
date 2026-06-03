/// Unit tests for ComputeTiltSequence action node.
///
/// Pure-logic (Tier 1) node: from a base pose it generates a sinusoidal tilt
/// waypoint sequence alternating pitch (even steps) / roll (odd steps), plus a
/// final return-to-base waypoint. Previously only providedPorts() was covered.

#include "ur5e_bt_coordinator/action_nodes/compute_tilt_sequence.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace rtc_bt;

class ComputeTiltSequenceTest : public ::testing::Test {
 protected:
  void SetUp() override { factory_.registerNodeType<ComputeTiltSequence>("ComputeTiltSequence"); }

  BT::Tree CreateTree(const std::string& xml) {
    const std::string full =
        R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" + xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(ComputeTiltSequenceTest, ProducesNumStepsPlusOneWaypoints) {
  auto tree = CreateTree(
      R"(<ComputeTiltSequence base_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                              amplitude_deg="15.0" num_steps="4" waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  ASSERT_EQ(wp.size(), 5u);  // num_steps (4) + final return-to-base
  // i=0 has angle sin(0)=0 → waypoint equals base; last waypoint is base too.
  EXPECT_DOUBLE_EQ(wp.front().pitch, 0.0);
  EXPECT_DOUBLE_EQ(wp.front().roll, 0.0);
  EXPECT_DOUBLE_EQ(wp.back().roll, 0.0);
  EXPECT_DOUBLE_EQ(wp.back().pitch, 0.0);
}

TEST_F(ComputeTiltSequenceTest, AlternatesPitchAndRoll) {
  // amplitude 90° = π/2 rad, num_steps 4: i=1 (odd→roll) at t=0.25 →
  // angle = (π/2)·sin(π/2) = π/2; i=3 (odd) at t=0.75 → angle = -π/2.
  auto tree = CreateTree(
      R"(<ComputeTiltSequence base_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                              amplitude_deg="90.0" num_steps="4" waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  ASSERT_EQ(wp.size(), 5u);
  // i=1: roll varies, pitch stays at base.
  EXPECT_NEAR(wp[1].roll, M_PI_2, 1e-9);
  EXPECT_DOUBLE_EQ(wp[1].pitch, 0.0);
  // i=2: even → pitch, angle sin(π)=0 → back to base.
  EXPECT_NEAR(wp[2].pitch, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(wp[2].roll, 0.0);
  // i=3: roll = -π/2.
  EXPECT_NEAR(wp[3].roll, -M_PI_2, 1e-9);
}

TEST_F(ComputeTiltSequenceTest, ClampsNumStepsToMinimumTwo) {
  // num_steps=1 is clamped up to 2 → 2 loop waypoints + 1 final = 3.
  auto tree = CreateTree(
      R"(<ComputeTiltSequence base_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                              amplitude_deg="15.0" num_steps="1" waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.rootBlackboard()->get<std::vector<Pose6D>>("wp").size(), 3u);
}

TEST_F(ComputeTiltSequenceTest, UsesPortDefaultsWhenOmitted) {
  // amplitude_deg / num_steps omitted → defaults 15.0 / 6 → 7 waypoints.
  auto tree = CreateTree(
      R"(<ComputeTiltSequence base_pose="0.0;0.0;0.0;0.0;0.0;0.0" waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.rootBlackboard()->get<std::vector<Pose6D>>("wp").size(), 7u);
}

TEST_F(ComputeTiltSequenceTest, PreservesBaseTranslation) {
  auto tree = CreateTree(
      R"(<ComputeTiltSequence base_pose="0.3;-0.2;0.5;0.1;0.2;0.3"
                              amplitude_deg="30.0" num_steps="6" waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  for (const auto& p : wp) {
    EXPECT_DOUBLE_EQ(p.x, 0.3);
    EXPECT_DOUBLE_EQ(p.y, -0.2);
    EXPECT_DOUBLE_EQ(p.z, 0.5);
    EXPECT_DOUBLE_EQ(p.yaw, 0.3);  // yaw never varied
  }
}

TEST_F(ComputeTiltSequenceTest, BlackboardBasePose) {
  auto tree =
      CreateTree(R"(<ComputeTiltSequence base_pose="{base}" num_steps="2" waypoints="{wp}"/>)");
  Pose6D base{1.0, 2.0, 3.0, 0.4, 0.5, 0.6};
  tree.rootBlackboard()->set("base", base);

  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  ASSERT_FALSE(wp.empty());
  EXPECT_DOUBLE_EQ(wp.back().x, 1.0);
  EXPECT_DOUBLE_EQ(wp.back().pitch, 0.5);  // final waypoint == base
}

TEST_F(ComputeTiltSequenceTest, MissingBasePoseThrows) {
  auto tree = CreateTree(R"(<ComputeTiltSequence waypoints="{wp}"/>)");
  EXPECT_THROW((void)tree.tickOnce(), BT::RuntimeError);
}
