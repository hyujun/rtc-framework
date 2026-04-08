/// Unit tests for shape estimation nodes:
/// TriggerShapeEstimation, WaitShapeResult.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/trigger_shape_estimation.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_shape_result.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class ShapeNodeTest : public RosTestFixture {
protected:
  void SetUp() override
  {
    RosTestFixture::SetUp();
    factory_.registerNodeType<TriggerShapeEstimation>("TriggerShapeEstimation", bridge_);
    factory_.registerNodeType<WaitShapeResult>("WaitShapeResult", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

// ══════════════════════════════════════════════════════════════════════════
// TriggerShapeEstimation
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ShapeNodeTest, TriggerStart_Succeeds)
{
  auto tree = CreateTree(
      R"(<TriggerShapeEstimation command="start"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ShapeNodeTest, TriggerStop_Succeeds)
{
  auto tree = CreateTree(
      R"(<TriggerShapeEstimation command="stop"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ShapeNodeTest, TriggerDefaultCommand)
{
  // Default command is "start"
  auto tree = CreateTree(R"(<TriggerShapeEstimation/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ShapeNodeTest, TriggerStart_ClearsEstimate)
{
  // First inject a shape estimate
  PublishShapeEstimate(2, 0.9, 100);
  Spin();

  // Verify it's available
  shape_estimation_msgs::msg::ShapeEstimate est;
  EXPECT_TRUE(bridge_->GetShapeEstimate(est));

  // Trigger "start" should clear cached estimate
  auto tree = CreateTree(
      R"(<TriggerShapeEstimation command="start"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Cached estimate should be cleared
  EXPECT_FALSE(bridge_->GetShapeEstimate(est));
}

// ══════════════════════════════════════════════════════════════════════════
// WaitShapeResult
// ══════════════════════════════════════════════════════════════════════════

TEST_F(ShapeNodeTest, WaitShapeResult_StartsRunning)
{
  auto tree = CreateTree(
      R"(<WaitShapeResult confidence_threshold="0.7"
                          timeout_s="5.0" estimate="{est}"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(ShapeNodeTest, WaitShapeResult_SucceedsOnGoodEstimate)
{
  auto tree = CreateTree(
      R"(<WaitShapeResult confidence_threshold="0.7"
                          timeout_s="5.0" estimate="{est}"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish a confident, non-UNKNOWN estimate
  PublishShapeEstimate(2, 0.95, 500);  // SPHERE
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto est = tree.rootBlackboard()->get<shape_estimation_msgs::msg::ShapeEstimate>("est");
  EXPECT_EQ(est.shape_type, 2u);
  EXPECT_GE(est.confidence, 0.7);
}

TEST_F(ShapeNodeTest, WaitShapeResult_StillRunningBelowThreshold)
{
  auto tree = CreateTree(
      R"(<WaitShapeResult confidence_threshold="0.8"
                          timeout_s="5.0" estimate="{est}"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish low confidence estimate
  PublishShapeEstimate(2, 0.5, 50);
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(ShapeNodeTest, WaitShapeResult_RejectsUnknownType)
{
  auto tree = CreateTree(
      R"(<WaitShapeResult confidence_threshold="0.1"
                          timeout_s="5.0" estimate="{est}"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // shape_type=0 (UNKNOWN) even with high confidence → still RUNNING
  PublishShapeEstimate(0, 0.99, 500);
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(ShapeNodeTest, WaitShapeResult_FailsOnTimeout)
{
  auto tree = CreateTree(
      R"(<WaitShapeResult confidence_threshold="0.99"
                          timeout_s="0.05" estimate="{est}"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish low-confidence estimate (won't meet threshold)
  PublishShapeEstimate(2, 0.5, 50);
  Spin();

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ShapeNodeTest, WaitShapeResult_TimeoutWithPartialResult)
{
  auto tree = CreateTree(
      R"(<WaitShapeResult confidence_threshold="0.99"
                          timeout_s="0.05" estimate="{est}"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish a partial result
  PublishShapeEstimate(3, 0.6, 200);  // CYLINDER but low confidence
  Spin();

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Should still output partial estimate for diagnostics
  auto est = tree.rootBlackboard()->get<shape_estimation_msgs::msg::ShapeEstimate>("est");
  EXPECT_EQ(est.shape_type, 3u);
}

TEST_F(ShapeNodeTest, WaitShapeResult_TimeoutNoEstimate)
{
  auto tree = CreateTree(
      R"(<WaitShapeResult confidence_threshold="0.7"
                          timeout_s="0.05" estimate="{est}"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // No estimate published at all
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
