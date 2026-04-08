/// Unit tests for SwitchController action node.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class SwitchControllerTest : public RosTestFixture {
protected:
  void SetUp() override
  {
    RosTestFixture::SetUp();
    factory_.registerNodeType<SwitchController>("SwitchController", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(SwitchControllerTest, AlreadyActiveNoGains)
{
  PublishActiveController("demo_joint_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          load_gains="false" timeout_s="3.0"/>)");
  // Already active + no gains → immediate SUCCESS
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SwitchControllerTest, AlreadyActiveWithGainsLoading)
{
  PublishActiveController("demo_task_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_task_controller"
                          load_gains="true" timeout_s="3.0"
                          current_gains="{cg}"/>)");

  // Already active + load_gains=true → RUNNING (waiting for gains)
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish gains
  PublishCurrentGains({1.0, 2.0, 3.0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto gains = tree.rootBlackboard()->get<std::vector<double>>("cg");
  ASSERT_EQ(gains.size(), 3u);
  EXPECT_DOUBLE_EQ(gains[0], 1.0);
}

TEST_F(SwitchControllerTest, SwitchAndConfirm)
{
  PublishActiveController("old_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          load_gains="false" timeout_s="3.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Simulate controller manager confirming the switch
  PublishActiveController("demo_joint_controller");
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SwitchControllerTest, NormalizedNameComparison)
{
  // Test that "DemoJointController" matches "demo_joint_controller"
  PublishActiveController("DemoJointController");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          load_gains="false" timeout_s="3.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SwitchControllerTest, FailsOnTimeout)
{
  PublishActiveController("old_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="new_controller"
                          load_gains="false" timeout_s="0.05"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Don't publish the expected controller
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(SwitchControllerTest, GainsTimeoutStillSucceeds)
{
  // If gains timeout, SwitchController still returns SUCCESS (warns but continues)
  PublishActiveController("demo_task_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_task_controller"
                          load_gains="true" timeout_s="0.05"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Don't publish gains, just wait for timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}
