// Tier 1 — DeviceTargetCallback tests for RtControllerNode.
//
// Covers rt_controller_node_callbacks.cpp: goal_type routing (joint vs task),
// empty-joint-target early return, joint_names reorder against the device's
// joint_state_names, kMaxDeviceChannels clipping, and dispatch to the
// currently-active controller's SetDeviceTarget. Previously 0% coverage.

#include "rt_cm_test_access.hpp"

#include "rtc_base/types/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rtc_msgs/msg/robot_target.hpp>

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rtc {
namespace {

class DeviceTargetCallbackTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    node_ = std::make_shared<RtControllerNode>("test_target_cb_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<MockController>("ctrl_a");
    auto b = std::make_unique<MockController>("ctrl_b");
    ctrl_a_ = a.get();
    ctrl_b_ = b.get();
    ctrls.push_back(std::move(a));
    ctrls.push_back(std::move(b));
    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls),
                                                     {"type_a", "type_b"});
    ControllerLifecycleTestAccess::SetActiveIdx(*node_, 0);  // ctrl_a active
  }

  static rtc_msgs::msg::RobotTarget::SharedPtr MakeJointTarget(
      const std::vector<double>& joints, const std::vector<std::string>& names = {}) {
    auto msg = std::make_shared<rtc_msgs::msg::RobotTarget>();
    msg->goal_type = "joint";
    msg->joint_target = joints;
    msg->joint_names = names;
    return msg;
  }

  std::shared_ptr<RtControllerNode> node_;
  MockController* ctrl_a_{nullptr};
  MockController* ctrl_b_{nullptr};
};

// ── goal_type routing ────────────────────────────────────────────────────────

TEST_F(DeviceTargetCallbackTest, JointTargetForwardedToActiveController) {
  auto msg = MakeJointTarget({1.0, 2.0, 3.0});
  ControllerLifecycleTestAccess::CallDeviceTargetCallback(*node_, /*slot=*/0, msg);

  EXPECT_EQ(1, ctrl_a_->SetTargetCount());
  EXPECT_EQ(0, ctrl_b_->SetTargetCount());
  EXPECT_EQ(0, ctrl_a_->LastTargetSlot());
  ASSERT_EQ(3u, ctrl_a_->LastTarget().size());
  EXPECT_DOUBLE_EQ(1.0, ctrl_a_->LastTarget()[0]);
  EXPECT_DOUBLE_EQ(3.0, ctrl_a_->LastTarget()[2]);
}

TEST_F(DeviceTargetCallbackTest, TaskTargetForwardedAsSixElements) {
  auto msg = std::make_shared<rtc_msgs::msg::RobotTarget>();
  msg->goal_type = "task";
  msg->task_target = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  ControllerLifecycleTestAccess::CallDeviceTargetCallback(*node_, /*slot=*/0, msg);

  EXPECT_EQ(1, ctrl_a_->SetTargetCount());
  ASSERT_EQ(6u, ctrl_a_->LastTarget().size());
  EXPECT_DOUBLE_EQ(0.1, ctrl_a_->LastTarget()[0]);
  EXPECT_DOUBLE_EQ(0.6, ctrl_a_->LastTarget()[5]);
}

TEST_F(DeviceTargetCallbackTest, EmptyJointTargetIsIgnored) {
  auto msg = MakeJointTarget({});  // joint goal with no payload → early return
  ControllerLifecycleTestAccess::CallDeviceTargetCallback(*node_, /*slot=*/0, msg);
  EXPECT_EQ(0, ctrl_a_->SetTargetCount());
}

// ── joint_names reorder ──────────────────────────────────────────────────────

TEST_F(DeviceTargetCallbackTest, JointNamesReorderedAgainstDeviceConfig) {
  ControllerLifecycleTestAccess::SetSlotToGroupName(*node_, {"arm"});
  std::map<std::string, DeviceNameConfig> cfgs;
  DeviceNameConfig arm;
  arm.device_name = "arm";
  arm.joint_state_names = {"j1", "j2", "j3"};
  cfgs.emplace("arm", std::move(arm));
  ControllerLifecycleTestAccess::InjectDeviceNameConfigs(*node_, std::move(cfgs));

  // msg lists joints out of device order: j3, j1, j2 → 30, 10, 20.
  auto msg = MakeJointTarget({30.0, 10.0, 20.0}, {"j3", "j1", "j2"});
  ControllerLifecycleTestAccess::CallDeviceTargetCallback(*node_, /*slot=*/0, msg);

  ASSERT_EQ(3u, ctrl_a_->LastTarget().size());
  // Reordered back into device order j1, j2, j3.
  EXPECT_DOUBLE_EQ(10.0, ctrl_a_->LastTarget()[0]);
  EXPECT_DOUBLE_EQ(20.0, ctrl_a_->LastTarget()[1]);
  EXPECT_DOUBLE_EQ(30.0, ctrl_a_->LastTarget()[2]);
}

TEST_F(DeviceTargetCallbackTest, JointNamesUnknownSlotFallsThroughUnreordered) {
  // device_slot beyond slot_to_group_name_ → reorder block skipped, payload
  // forwarded in message order.
  auto msg = MakeJointTarget({7.0, 8.0}, {"jx", "jy"});
  ControllerLifecycleTestAccess::CallDeviceTargetCallback(*node_, /*slot=*/5, msg);

  ASSERT_EQ(2u, ctrl_a_->LastTarget().size());
  EXPECT_DOUBLE_EQ(7.0, ctrl_a_->LastTarget()[0]);
  EXPECT_DOUBLE_EQ(8.0, ctrl_a_->LastTarget()[1]);
}

// ── dispatch target ──────────────────────────────────────────────────────────

TEST_F(DeviceTargetCallbackTest, DispatchesToCurrentlyActiveController) {
  ControllerLifecycleTestAccess::SetActiveIdx(*node_, 1);  // ctrl_b active now
  auto msg = MakeJointTarget({5.0});
  ControllerLifecycleTestAccess::CallDeviceTargetCallback(*node_, /*slot=*/0, msg);

  EXPECT_EQ(0, ctrl_a_->SetTargetCount());
  EXPECT_EQ(1, ctrl_b_->SetTargetCount());
}

TEST_F(DeviceTargetCallbackTest, OverlongJointTargetClippedToMaxChannels) {
  std::vector<double> big(kMaxDeviceChannels + 5, 1.0);
  auto msg = MakeJointTarget(big);  // no joint_names → no reorder, just clip
  ControllerLifecycleTestAccess::CallDeviceTargetCallback(*node_, /*slot=*/0, msg);

  ASSERT_EQ(1, ctrl_a_->SetTargetCount());
  EXPECT_EQ(static_cast<std::size_t>(kMaxDeviceChannels), ctrl_a_->LastTarget().size());
}

}  // namespace
}  // namespace rtc
