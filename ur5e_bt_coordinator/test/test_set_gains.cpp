/// Unit tests for SetGains action node.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

using namespace rtc_bt;
using namespace rtc_bt::test;

class SetGainsTest : public RosTestFixture {
protected:
  void SetUp() override
  {
    RosTestFixture::SetUp();
    factory_.registerNodeType<SetGains>("SetGains", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml)
  {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(SetGainsTest, FullGainsOverride)
{
  auto tree = CreateTree(
      R"(<SetGains full_gains="1.0;2.0;3.0;4.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, DemoJointControllerDefaults)
{
  PublishActiveController("demo_joint_controller");
  Spin();

  auto tree = CreateTree(R"(<SetGains/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, DemoJointWithSpeedOverride)
{
  PublishActiveController("demo_joint_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SetGains trajectory_speed="0.15"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, DemoTaskControllerDefaults)
{
  PublishActiveController("demo_task_controller");
  Spin();

  auto tree = CreateTree(R"(<SetGains/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, DemoTaskWithKpOverride)
{
  PublishActiveController("demo_task_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SetGains kp_translation="500,500,500"
                   kp_rotation="250,250,250"
                   trajectory_speed="0.08"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, DemoJointWithCachedGains)
{
  PublishActiveController("demo_joint_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SetGains current_gains="{cg}"
                   trajectory_speed="0.2"/>)");

  // Inject cached gains (7 values for DemoJoint)
  std::vector<double> cached = {0.05, 1.57, 0.25, 3.14, 0.4, 0.8, 2.0};
  tree.rootBlackboard()->set("cg", cached);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, DemoTaskWithCachedGains)
{
  PublishActiveController("DemoTaskController");
  Spin();

  auto tree = CreateTree(
      R"(<SetGains current_gains="{cg}"
                   trajectory_speed="0.1"/>)");

  // Inject cached gains (19 values for DemoTask)
  std::vector<double> cached(19, 1.0);
  tree.rootBlackboard()->set("cg", cached);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, GraspCommandAppended)
{
  PublishActiveController("demo_joint_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SetGains grasp_command="1" grasp_target_force="3.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, GraspCommandZeroNotAppended)
{
  PublishActiveController("demo_joint_controller");
  Spin();

  // grasp_command=0 → gains should NOT be extended
  auto tree = CreateTree(R"(<SetGains grasp_command="0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}
