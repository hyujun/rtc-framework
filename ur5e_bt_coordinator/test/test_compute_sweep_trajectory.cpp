/// Unit tests for ComputeSweepTrajectory action node.

#include "ur5e_bt_coordinator/action_nodes/compute_sweep_trajectory.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <cmath>

using namespace rtc_bt;

class ComputeSweepTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    factory_.registerNodeType<ComputeSweepTrajectory>("ComputeSweepTrajectory");
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full_xml =
        R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)" + xml +
        R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full_xml);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(ComputeSweepTest, DefaultParameters)
{
  auto tree = CreateTree(
      R"(<ComputeSweepTrajectory start_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                                 waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  EXPECT_EQ(wp.size(), 8u);  // default num_waypoints=8
}

TEST_F(ComputeSweepTest, FirstAndLastWaypoint)
{
  auto tree = CreateTree(
      R"(<ComputeSweepTrajectory start_pose="1.0;2.0;3.0;0.1;0.2;0.3"
                                 direction_x="1.0" direction_y="0.0"
                                 distance="0.5" arc_height="0.1"
                                 num_waypoints="5"
                                 waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  ASSERT_EQ(wp.size(), 5u);

  // First waypoint = start (t=0, sin(0)=0)
  EXPECT_NEAR(wp[0].x, 1.0, 1e-12);
  EXPECT_NEAR(wp[0].y, 2.0, 1e-12);
  EXPECT_NEAR(wp[0].z, 3.0, 1e-12);  // arc_height * sin(0) = 0

  // Last waypoint = start + distance along dir (t=1, sin(pi)≈0)
  EXPECT_NEAR(wp[4].x, 1.5, 1e-12);  // 1.0 + 0.5*1.0
  EXPECT_NEAR(wp[4].y, 2.0, 1e-12);
  EXPECT_NEAR(wp[4].z, 3.0, 1e-6);   // sin(pi)≈0
}

TEST_F(ComputeSweepTest, ArcPeakAtMiddle)
{
  auto tree = CreateTree(
      R"(<ComputeSweepTrajectory start_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                                 direction_x="1.0" direction_y="0.0"
                                 distance="1.0" arc_height="0.1"
                                 num_waypoints="3"
                                 waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  ASSERT_EQ(wp.size(), 3u);

  // Middle waypoint (t=0.5): z = 0.1 * sin(pi*0.5) = 0.1
  EXPECT_NEAR(wp[1].z, 0.1, 1e-12);
  EXPECT_NEAR(wp[1].x, 0.5, 1e-12);
}

TEST_F(ComputeSweepTest, DirectionNormalization)
{
  auto tree = CreateTree(
      R"(<ComputeSweepTrajectory start_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                                 direction_x="3.0" direction_y="4.0"
                                 distance="5.0" arc_height="0.0"
                                 num_waypoints="2"
                                 waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  ASSERT_EQ(wp.size(), 2u);

  // Normalized direction: (0.6, 0.8), distance=5.0
  // End: x=0.6*5=3.0, y=0.8*5=4.0
  EXPECT_NEAR(wp[1].x, 3.0, 1e-10);
  EXPECT_NEAR(wp[1].y, 4.0, 1e-10);
}

TEST_F(ComputeSweepTest, MinimumTwoWaypoints)
{
  auto tree = CreateTree(
      R"(<ComputeSweepTrajectory start_pose="0.0;0.0;0.0;0.0;0.0;0.0"
                                 num_waypoints="1"
                                 waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  EXPECT_GE(wp.size(), 2u);  // n < 2 becomes n = 2
}

TEST_F(ComputeSweepTest, OrientationPreserved)
{
  auto tree = CreateTree(
      R"(<ComputeSweepTrajectory start_pose="0.0;0.0;0.0;1.0;2.0;3.0"
                                 num_waypoints="4"
                                 waypoints="{wp}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto wp = tree.rootBlackboard()->get<std::vector<Pose6D>>("wp");
  for (const auto& w : wp) {
    EXPECT_DOUBLE_EQ(w.roll, 1.0);
    EXPECT_DOUBLE_EQ(w.pitch, 2.0);
    EXPECT_DOUBLE_EQ(w.yaw, 3.0);
  }
}
