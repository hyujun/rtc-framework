/// Unit tests for hand-related action nodes:
/// SetHandPose, MoveFinger, FlexExtendFinger, MoveOpposition, UR5eHoldPose,
/// TrackTrajectory.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_opposition.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/ur5e_hold_pose.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class HandNodeTest : public RosTestFixture {
protected:
  void SetUp() override {
    RosTestFixture::SetUp();
    factory_.registerNodeType<SetHandPose>("SetHandPose", bridge_);
    factory_.registerNodeType<MoveFinger>("MoveFinger", bridge_);
    factory_.registerNodeType<FlexExtendFinger>("FlexExtendFinger", bridge_);
    factory_.registerNodeType<MoveOpposition>("MoveOpposition", bridge_);
    factory_.registerNodeType<UR5eHoldPose>("UR5eHoldPose", bridge_);
    factory_.registerNodeType<TrackTrajectory>("TrackTrajectory", bridge_);
  }

  BT::Tree CreateTree(const std::string &xml) {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

// ══════════════════════════════════════════════════════════════════════════
// SetHandPose
// ══════════════════════════════════════════════════════════════════════════

TEST_F(HandNodeTest, SetHandPose_StartsRunning) {
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(R"(<SetHandPose pose="home"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(HandNodeTest, SetHandPose_CompletesAfterDuration) {
  // Start at home (all zeros) → target home → distance 0 → min duration
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(R"(<SetHandPose pose="home"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Min duration is 0.01 * margin(1.1) = 0.011s
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(HandNodeTest, SetHandPose_UnknownPoseThrows) {
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(R"(<SetHandPose pose="nonexistent_pose"/>)");
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

// ══════════════════════════════════════════════════════════════════════════
// MoveFinger
// ══════════════════════════════════════════════════════════════════════════

TEST_F(HandNodeTest, MoveFinger_StartsRunning) {
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree =
      CreateTree(R"(<MoveFinger finger_name="thumb" pose="thumb_flex"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(HandNodeTest, MoveFinger_InvalidFingerThrows) {
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(R"(<MoveFinger finger_name="pinky" pose="home"/>)");
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

TEST_F(HandNodeTest, MoveFinger_ZeroDistanceQuickComplete) {
  // Already at home → target home → zero distance → quick
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(R"(<MoveFinger finger_name="thumb" pose="home"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ══════════════════════════════════════════════════════════════════════════
// FlexExtendFinger
// ══════════════════════════════════════════════════════════════════════════

TEST_F(HandNodeTest, FlexExtendFinger_StartsRunning) {
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(R"(<FlexExtendFinger finger_name="index"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(HandNodeTest, FlexExtendFinger_CompletesFlexExtendCycle) {
  // At home → flex → extend (home)
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  // High hand_trajectory_speed so duration_ is dominated by max_vel limit
  // (kDefaultHandMaxTrajVelocity = 4.0 rad/s) and finishes within the
  // 100-tick budget below.
  auto tree = CreateTree(
      R"(<FlexExtendFinger finger_name="thumb"
                           hand_trajectory_speed="100.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // hand_trajectory_speed=100 saturates speed term; duration is then bounded
  // by max_vel=kDefaultHandMaxTrajVelocity (4.0). Flex (0..0.6 thumb_flex)
  // costs ~0.28s × 1.1 margin ≈ 0.31s, extend back to 0 ≈ same. Allow 600
  // ticks × 5 ms = 3 s budget for safety.
  for (int i = 0; i < 600; ++i) {
    auto status = tree.tickOnce();
    if (status == BT::NodeStatus::SUCCESS)
      return;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  FAIL() << "FlexExtendFinger did not complete within timeout";
}

// ══════════════════════════════════════════════════════════════════════════
// MoveOpposition
// ══════════════════════════════════════════════════════════════════════════

TEST_F(HandNodeTest, MoveOpposition_StartsRunning) {
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(
      R"(<MoveOpposition thumb_pose="thumb_index_oppose"
                         target_finger="index"
                         target_pose="index_oppose"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(HandNodeTest, MoveOpposition_InvalidFingerThrows) {
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Spin();

  auto tree = CreateTree(
      R"(<MoveOpposition thumb_pose="thumb_index_oppose"
                         target_finger="pinky"
                         target_pose="index_oppose"/>)");
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

// ══════════════════════════════════════════════════════════════════════════
// UR5eHoldPose
// ══════════════════════════════════════════════════════════════════════════

TEST_F(HandNodeTest, UR5eHoldPose_AlwaysRunning) {
  auto tree = CreateTree(R"(<UR5eHoldPose pose="demo_pose"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(HandNodeTest, UR5eHoldPose_UnknownPoseThrows) {
  auto tree = CreateTree(R"(<UR5eHoldPose pose="nonexistent"/>)");
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

// ══════════════════════════════════════════════════════════════════════════
// TrackTrajectory
// ══════════════════════════════════════════════════════════════════════════

TEST_F(HandNodeTest, TrackTrajectory_StartsRunning) {
  auto tree = CreateTree(
      R"(<TrackTrajectory waypoints="{wp}"
                          position_tolerance="0.01" timeout_s="5.0"/>)");

  std::vector<Pose6D> wp = {{0.1, 0.2, 0.3, 0, 0, 0}, {0.4, 0.5, 0.6, 0, 0, 0}};
  tree.rootBlackboard()->set("wp", wp);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(HandNodeTest, TrackTrajectory_SucceedsAllWaypoints) {
  auto tree = CreateTree(
      R"(<TrackTrajectory waypoints="{wp}"
                          position_tolerance="0.05" timeout_s="5.0"/>)");

  std::vector<Pose6D> wp = {{0.1, 0.0, 0.0, 0, 0, 0}, {0.2, 0.0, 0.0, 0, 0, 0}};
  tree.rootBlackboard()->set("wp", wp);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Move to first waypoint
  Pose6D at_wp0{0.1, 0.0, 0.0, 0, 0, 0};
  PublishArmState(at_wp0, {0, 0, 0, 0, 0, 0});
  Spin();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING); // Advances to wp[1]

  // Move to second waypoint
  Pose6D at_wp1{0.2, 0.0, 0.0, 0, 0, 0};
  PublishArmState(at_wp1, {0, 0, 0, 0, 0, 0});
  Spin();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(HandNodeTest, TrackTrajectory_FailsOnTimeout) {
  auto tree = CreateTree(
      R"(<TrackTrajectory waypoints="{wp}"
                          position_tolerance="0.001" timeout_s="0.05"/>)");

  std::vector<Pose6D> wp = {{10.0, 10.0, 10.0, 0, 0, 0}};
  tree.rootBlackboard()->set("wp", wp);

  Pose6D far{0, 0, 0, 0, 0, 0};
  PublishArmState(far, {0, 0, 0, 0, 0, 0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
