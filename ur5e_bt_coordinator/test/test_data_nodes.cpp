/// Unit tests for the bridge-backed data nodes (Tier 2 inject, InjectTestFixture):
///   StartToFCollection, StopToFCollection, ProcessSearchData, GetCurrentPose.
///
/// These nodes read/write BtRosBridge state (ToF buffer, TCP pose). No ToF
/// snapshots are published by the fixture, so the collection buffer stays empty
/// (count == 0) — that is the expected, asserted state. TCP pose is injected
/// via PublishArmState() (tf2 base→tool0_actual) and read back through the
/// quaternion↔RPY round-trip GetTcpPose() performs.

#include "inject_fixture.hpp"
#include "ur5e_bt_coordinator/action_nodes/get_current_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/process_search_data.hpp"
#include "ur5e_bt_coordinator/action_nodes/start_tof_collection.hpp"
#include "ur5e_bt_coordinator/action_nodes/stop_tof_collection.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

using namespace rtc_bt;
using namespace rtc_bt::test;

class DataNodeTest : public InjectTestFixture {
 protected:
  void SetUp() override {
    InjectTestFixture::SetUp();
    factory_.registerNodeType<StartToFCollection>("StartToFCollection", bridge_);
    factory_.registerNodeType<StopToFCollection>("StopToFCollection", bridge_);
    factory_.registerNodeType<ProcessSearchData>("ProcessSearchData", bridge_);
    factory_.registerNodeType<GetCurrentPose>("GetCurrentPose", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml) {
    const std::string full =
        R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" + xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

// ── StartToFCollection ─────────────────────────────────────────────────────

TEST_F(DataNodeTest, StartToFCollectionSucceeds) {
  auto tree = CreateTree(R"(<StartToFCollection/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ── StopToFCollection ──────────────────────────────────────────────────────

TEST_F(DataNodeTest, StopToFCollectionOutputsZeroCountWhenEmpty) {
  bridge_->StartToFCollection();
  auto tree = CreateTree(R"(<StopToFCollection count="{n}"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.rootBlackboard()->get<int>("n"), 0);
}

TEST_F(DataNodeTest, StartStopCycleViaNodes) {
  auto start = CreateTree(R"(<StartToFCollection/>)");
  auto stop = CreateTree(R"(<StopToFCollection count="{n}"/>)");
  EXPECT_EQ(start.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(stop.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(stop.rootBlackboard()->get<int>("n"), 0);
  EXPECT_EQ(bridge_->GetCollectedToFCount(), 0u);
}

// ── GetCurrentPose ─────────────────────────────────────────────────────────

TEST_F(DataNodeTest, GetCurrentPoseReturnsInjectedTcp) {
  Pose6D tcp{0.3, -0.2, 0.5, 0.1, 0.2, 0.3};
  PublishArmState(tcp, {0, 0, 0, 0, 0, 0});
  Spin(std::chrono::milliseconds(100));

  auto tree = CreateTree(R"(<GetCurrentPose pose="{p}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto p = tree.rootBlackboard()->get<Pose6D>("p");
  EXPECT_NEAR(p.x, 0.3, 1e-6);
  EXPECT_NEAR(p.y, -0.2, 1e-6);
  EXPECT_NEAR(p.z, 0.5, 1e-6);
  EXPECT_NEAR(p.roll, 0.1, 1e-6);
  EXPECT_NEAR(p.pitch, 0.2, 1e-6);
  EXPECT_NEAR(p.yaw, 0.3, 1e-6);
}

// ── ProcessSearchData ──────────────────────────────────────────────────────

TEST_F(DataNodeTest, ProcessSearchDataFallsBackToCurrentPose) {
  // With no ToF data collected, the (placeholder) processing falls back to the
  // current TCP pose as the output goal.
  Pose6D tcp{0.4, 0.1, 0.6, -0.1, 0.05, 0.2};
  PublishArmState(tcp, {0, 0, 0, 0, 0, 0});
  Spin(std::chrono::milliseconds(100));

  auto tree = CreateTree(R"(<ProcessSearchData output_pose="{goal}"/>)");
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  auto goal = tree.rootBlackboard()->get<Pose6D>("goal");
  EXPECT_NEAR(goal.x, 0.4, 1e-6);
  EXPECT_NEAR(goal.y, 0.1, 1e-6);
  EXPECT_NEAR(goal.z, 0.6, 1e-6);
  EXPECT_NEAR(goal.roll, -0.1, 1e-6);
  EXPECT_NEAR(goal.pitch, 0.05, 1e-6);
  EXPECT_NEAR(goal.yaw, 0.2, 1e-6);
}

// ── GetTopicHealth staleness arithmetic ─────────────────────────────────────

namespace {
const TopicHealth* FindHealth(const std::vector<TopicHealth>& health, const std::string& substr) {
  for (const auto& h : health) {
    if (h.name.find(substr) != std::string::npos)
      return &h;
  }
  return nullptr;
}
}  // namespace

TEST_F(DataNodeTest, GetTopicHealthReflectsReceiptAndStaleness) {
  // A topic with no message yet: received=false, sentinel age, unhealthy.
  {
    auto health = bridge_->GetTopicHealth(100.0);
    const auto* hand = FindHealth(health, "hand/joint_states");
    ASSERT_NE(hand, nullptr);
    EXPECT_FALSE(hand->received);
    EXPECT_DOUBLE_EQ(hand->seconds_since_last, -1.0);
    EXPECT_FALSE(hand->healthy);
  }

  // After a hand joint_states sample lands, the entry is received and — with a
  // generous timeout — healthy, with a non-negative age.
  PublishHandState({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  {
    auto health = bridge_->GetTopicHealth(100.0);
    const auto* hand = FindHealth(health, "hand/joint_states");
    ASSERT_NE(hand, nullptr);
    EXPECT_TRUE(hand->received);
    EXPECT_GE(hand->seconds_since_last, 0.0);
    EXPECT_TRUE(hand->healthy);
  }

  // Same received sample, but a zero timeout makes any positive age stale:
  // healthy = (age <= timeout) must now be false while received stays true.
  {
    auto health = bridge_->GetTopicHealth(0.0);
    const auto* hand = FindHealth(health, "hand/joint_states");
    ASSERT_NE(hand, nullptr);
    EXPECT_TRUE(hand->received);
    EXPECT_FALSE(hand->healthy);
  }
}
