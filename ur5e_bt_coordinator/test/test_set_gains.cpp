/// Unit tests for SetGains action node.
///
/// The fixture spawns three mock controller LifecycleNodes
/// (demo_joint_controller, demo_task_controller, demo_wbc_controller),
/// each declaring the relevant parameter subset and a grasp_command srv.
/// SetActiveAlias() rebinds the bridge to the matching mock; SetGains
/// ticks then call SetActiveControllerGains / SendGraspCommand against
/// that mock and tests verify parameter values or recorded grasp calls.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rtc_msgs/srv/grasp_command.hpp>

using namespace rtc_bt;
using namespace rtc_bt::test;

class SetGainsTest : public RosTestFixture {
protected:
  void SetUp() override {
    RosTestFixture::SetUp();
    factory_.registerNodeType<SetGains>("SetGains", bridge_);
  }

  BT::Tree CreateTree(const std::string &xml) {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(SetGainsTest, NoPortsIsNoop) {
  // Fixture defaults to active="demo_task_controller".
  auto tree = CreateTree(R"(<SetGains/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetGainsTest, DemoJointSetsRobotTrajectorySpeed) {
  SetActiveAlias("demo_joint_controller");
  auto tree = CreateTree(R"(<SetGains trajectory_speed="0.15"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // SetGains maps trajectory_speed → robot_trajectory_speed for DemoJoint.
  EXPECT_DOUBLE_EQ(
      mock_joint_.node->get_parameter("robot_trajectory_speed").as_double(),
      0.15);
}

TEST_F(SetGainsTest, DemoTaskSetsKpAndDamping) {
  // Fixture default = demo_task_controller; no SetActiveAlias needed.
  auto tree = CreateTree(
      R"(<SetGains kp_translation="500,500,500"
                   kp_rotation="250,250,250"
                   damping="0.05"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  const auto kp_t =
      mock_task_.node->get_parameter("kp_translation").as_double_array();
  ASSERT_EQ(kp_t.size(), 3u);
  EXPECT_DOUBLE_EQ(kp_t[0], 500.0);

  const auto kp_r =
      mock_task_.node->get_parameter("kp_rotation").as_double_array();
  ASSERT_EQ(kp_r.size(), 3u);
  EXPECT_DOUBLE_EQ(kp_r[1], 250.0);

  EXPECT_DOUBLE_EQ(mock_task_.node->get_parameter("damping").as_double(), 0.05);
}

TEST_F(SetGainsTest, DemoTaskMapsTrajectorySpeedDirectly) {
  auto tree = CreateTree(R"(<SetGains trajectory_speed="0.08"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_DOUBLE_EQ(
      mock_task_.node->get_parameter("trajectory_speed").as_double(), 0.08);
}

TEST_F(SetGainsTest, DemoWbcMapsTrajectorySpeedToArm) {
  SetActiveAlias("demo_wbc_controller");
  auto tree =
      CreateTree(R"(<SetGains trajectory_speed="0.4" se3_weight="200.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_DOUBLE_EQ(
      mock_wbc_.node->get_parameter("arm_trajectory_speed").as_double(), 0.4);
  EXPECT_DOUBLE_EQ(mock_wbc_.node->get_parameter("se3_weight").as_double(),
                   200.0);
}

TEST_F(SetGainsTest, GraspCommandGraspCallsServer) {
  SetActiveAlias("demo_joint_controller");
  auto tree =
      CreateTree(R"(<SetGains grasp_command="1" grasp_target_force="3.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_GE(mock_joint_.grasp_cmd_calls.load(), 1);
  EXPECT_EQ(mock_joint_.last_grasp_cmd,
            rtc_msgs::srv::GraspCommand::Request::GRASP);
  EXPECT_DOUBLE_EQ(mock_joint_.last_grasp_force, 3.0);
}

TEST_F(SetGainsTest, GraspCommandReleaseCallsServer) {
  SetActiveAlias("demo_joint_controller");
  auto tree = CreateTree(R"(<SetGains grasp_command="2"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_GE(mock_joint_.grasp_cmd_calls.load(), 1);
  EXPECT_EQ(mock_joint_.last_grasp_cmd,
            rtc_msgs::srv::GraspCommand::Request::RELEASE);
}

TEST_F(SetGainsTest, GraspCommandZeroIsSkipped) {
  SetActiveAlias("demo_joint_controller");
  const int before = mock_joint_.grasp_cmd_calls.load();
  auto tree = CreateTree(R"(<SetGains grasp_command="0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(mock_joint_.grasp_cmd_calls.load(), before);
}

TEST_F(SetGainsTest, UnknownActiveControllerFails) {
  SetActiveAlias("totally_unknown_controller");
  auto tree = CreateTree(R"(<SetGains trajectory_speed="0.1"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
