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
  void SetUp() override {
    RosTestFixture::SetUp();
    factory_.registerNodeType<GraspControl>("GraspControl", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml) {
    const std::string full =
        R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" + xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(GraspControlTest, OpenModeStartsRunning) {
  auto tree = CreateTree(
      R"(<GraspControl mode="open"
                       target_positions="0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0"
                       timeout_s="5.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(GraspControlTest, OpenModeSucceedsAtTarget) {
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

TEST_F(GraspControlTest, CloseModeStartsRunning) {
  // Provide hand state so close mode can read current positions
  PublishHandState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  Spin();

  auto tree = CreateTree(
      R"(<GraspControl mode="close"
                       close_speed="0.3" max_position="1.4"
                       timeout_s="5.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(GraspControlTest, CloseModeFailsAtMaxPosition) {
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

TEST_F(GraspControlTest, CloseModeTimeout) {
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

TEST_F(GraspControlTest, PinchModeOnlyAffectsSpecifiedMotors) {
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

TEST_F(GraspControlTest, CloseIncrementScalesWithMeasuredTickDt) {
  // Regression (Phase 2a): the closing increment must scale with the MEASURED
  // tick dt so the effective closing rate equals close_speed regardless of BT
  // tick_rate_hz. The old code used a hardcoded 0.05 s dt, so a slow/fast tick
  // closed slower/faster than close_speed advertised. Proof: two ticks with a
  // ~2x sleep ratio must yield a ~2x increment ratio, and each increment must
  // track close_speed * elapsed (far above the old fixed 0.05 * close_speed).
  PublishHandState({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  Spin();

  constexpr double kCloseSpeed = 1.0;  // rad/s
  auto tree = CreateTree(
      R"(<GraspControl mode="close"
                       close_speed="1.0" max_position="999.0"
                       timeout_s="30.0"/>)");

  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);  // onStart: target starts at 0

  // First interval (~150ms) → increment ≈ close_speed * 0.15.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  const double inc1 = bridge_->GetLastHandTarget().at(0);  // from 0

  // Second interval (~2x longer) → increment ≈ 2x larger.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  const double inc2 = bridge_->GetLastHandTarget().at(0) - inc1;

  // Each increment tracks close_speed * elapsed; generous bounds absorb
  // scheduler overrun (sleep_for is a lower bound on real elapsed).
  EXPECT_GT(inc1, kCloseSpeed * 0.15 * 0.8);
  EXPECT_LT(inc1, kCloseSpeed * 0.15 * 3.0);
  // Proportionality: the ~2x-longer interval yields a clearly larger increment.
  EXPECT_GT(inc2, inc1 * 1.4);
}

TEST_F(GraspControlTest, PresetModeSucceeds) {
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
