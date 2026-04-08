/// Unit tests for WaitDuration action node.

#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace rtc_bt;

class WaitDurationTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    factory_.registerNodeType<WaitDuration>("WaitDuration");
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

TEST_F(WaitDurationTest, StartsRunning)
{
  auto tree = CreateTree(R"(<WaitDuration duration_s="1.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(WaitDurationTest, StillRunningBeforeDuration)
{
  auto tree = CreateTree(R"(<WaitDuration duration_s="0.5"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  // Immediately tick again — should still be RUNNING
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(WaitDurationTest, SucceedsAfterDuration)
{
  auto tree = CreateTree(R"(<WaitDuration duration_s="0.05"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitDurationTest, DefaultDuration)
{
  // Default is 0.5s
  auto tree = CreateTree(R"(<WaitDuration/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(WaitDurationTest, VeryShortDuration)
{
  auto tree = CreateTree(R"(<WaitDuration duration_s="0.001"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}
