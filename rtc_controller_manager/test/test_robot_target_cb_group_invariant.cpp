// Invariant test for the CM-owned RobotTarget subscription callback-group
// binding (spec §0d, thread-layout-v3, Issue #100).
//
// Spec §0d (ground truth):
//   - CM-owned `target_sub_` lives on the non-RT callback group
//     (cb_group_nrt_callback_).
//   - Controller-owned RobotTarget subs also live on the node's default
//     (= non-RT) group.
// Rationale: the RT boundary is controller ↔ hardware/sim only. RobotTarget
// is an external intent input (target generator / BT / teleop) and must not
// run at FIFO 70.
//
// History: PR #97 Phase 3 (`f6a78b6`) introduced cb_group_rt_inbound_
// injection for DeviceBackend state lanes but reused the same SubscriptionOptions
// for the manager-owned RobotTarget sub, accidentally promoting it to RT.
// This test locks the invariant against future regressions.
//
// Test strategy: build an RtControllerNode, inject a single manager-owned
// SubscribeTopicEntry through the ControllerLifecycleTestAccess friend bridge,
// drive CreateSubscriptions() directly, then reverse-lookup the resulting
// subscription via CallbackGroup::find_subscription_ptrs_if (jazzy rclcpp does
// not expose the callback group from SubscriptionBase — see memory
// feedback_rclcpp_subscription_cb_group_reverse_lookup).

#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rtc {

// Friend bridge for this test binary. The class name is shared across all
// rtc_controller_manager test TUs (each gtest binary links independently), so
// keep the surface minimal here.
class ControllerLifecycleTestAccess {
 public:
  static void InjectSingleManagerTargetEntry(RtControllerNode& node) {
    TopicConfig cfg;
    auto& group = cfg["arm"];
    group.subscribe.push_back(SubscribeTopicEntry{"/test/robot_target", TopicOwnership::kManager});
    node.controller_topic_configs_.push_back(std::move(cfg));
    node.active_groups_.insert("arm");
    node.group_slot_map_["arm"] = 0;
  }

  static void EnsureCallbackGroups(RtControllerNode& node) {
    if (!node.cb_group_rt_inbound_) {
      node.cb_group_rt_inbound_ =
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    if (!node.cb_group_nrt_callback_) {
      node.cb_group_nrt_callback_ =
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
  }

  static void RunCreateSubscriptions(RtControllerNode& node) { node.CreateSubscriptions(); }

  static rclcpp::CallbackGroup::SharedPtr RtInboundGroup(const RtControllerNode& node) {
    return node.cb_group_rt_inbound_;
  }

  static rclcpp::CallbackGroup::SharedPtr NrtCallbackGroup(const RtControllerNode& node) {
    return node.cb_group_nrt_callback_;
  }

  static const std::vector<rclcpp::SubscriptionBase::SharedPtr>& TopicSubscriptions(
      const RtControllerNode& node) {
    return node.topic_subscriptions_;
  }
};

}  // namespace rtc

// True iff `sub` is registered in `group`. Reverse-lookup via
// CallbackGroup::find_subscription_ptrs_if — see
// test_device_backend_cb_group_injection.cpp for the same pattern.
static bool GroupContainsSubscription(const rclcpp::CallbackGroup::SharedPtr& group,
                                      const rclcpp::SubscriptionBase::SharedPtr& sub) {
  if (!group || !sub)
    return false;
  auto match =
      group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr& candidate) {
        return candidate.get() == sub.get();
      });
  return match != nullptr;
}

class RobotTargetCbGroupInvariantTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok())
      rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<RtControllerNode>("test_robot_target_cb_group_node");
    rtc::ControllerLifecycleTestAccess::EnsureCallbackGroups(*node_);
    rtc::ControllerLifecycleTestAccess::InjectSingleManagerTargetEntry(*node_);
    rtc::ControllerLifecycleTestAccess::RunCreateSubscriptions(*node_);
  }

  void TearDown() override { node_.reset(); }

  std::shared_ptr<RtControllerNode> node_;
};

// Positive: the manager-owned RobotTarget sub lands on the non-RT callback
// group, matching spec §0d.
TEST_F(RobotTargetCbGroupInvariantTest, ManagerTargetSubAttachesToNrtCallbackGroup) {
  const auto& subs = rtc::ControllerLifecycleTestAccess::TopicSubscriptions(*node_);
  ASSERT_EQ(subs.size(), 1u) << "expected exactly one manager-owned RobotTarget sub";
  const auto sub = subs.front();
  ASSERT_NE(sub, nullptr);

  EXPECT_TRUE(
      GroupContainsSubscription(rtc::ControllerLifecycleTestAccess::NrtCallbackGroup(*node_), sub))
      << "CM-owned RobotTarget sub did not land in cb_group_nrt_callback_ — spec §0d "
         "violated; RobotTarget is an external-intent input, not an RT-boundary lane";
}

// Negative: the manager-owned RobotTarget sub must NOT land on the RT inbound
// callback group. This is the exact regression introduced by PR #97 Phase 3
// (sub_options reused across state lanes and the target lane) and reported as
// Issue #100.
TEST_F(RobotTargetCbGroupInvariantTest, ManagerTargetSubDoesNotAttachToRtInboundGroup) {
  const auto& subs = rtc::ControllerLifecycleTestAccess::TopicSubscriptions(*node_);
  ASSERT_EQ(subs.size(), 1u);
  const auto sub = subs.front();
  ASSERT_NE(sub, nullptr);

  EXPECT_FALSE(
      GroupContainsSubscription(rtc::ControllerLifecycleTestAccess::RtInboundGroup(*node_), sub))
      << "RobotTarget sub leaked into cb_group_rt_inbound_ — external-intent traffic "
         "would execute at FIFO 70 alongside hardware state callbacks";
}
