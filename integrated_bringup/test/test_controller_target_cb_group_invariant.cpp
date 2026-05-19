// Invariant test for controller-owned RobotTarget subscription callback-group
// binding (spec §0d, thread-layout-v3, follow-up to Issue #100).
//
// Spec §0d (ground truth):
//   - CM-owned `target_sub_` lives on cb_group_nrt_callback_ (locked by
//     rtc_controller_manager/test/test_robot_target_cb_group_invariant.cpp).
//   - Controller-owned RobotTarget subs live on the LifecycleNode's default
//     callback group — which the CM later attaches to nrt_callback_executor
//     (see agent_docs/architecture.md cb_group→executor matrix).
//
// What this test locks: CreateOwnedTopics() in
// integrated_bringup/src/support/owned_topics.cpp creates RobotTarget subs
// with `node->create_subscription(topic, qos, cb)` and NO SubscriptionOptions
// argument — so rclcpp routes the sub onto the node's default callback group.
// A future change adding `SubscriptionOptions{.callback_group = ...}` would
// silently route the sub elsewhere; this test catches that regression.
//
// Reverse-lookup uses CallbackGroup::find_subscription_ptrs_if (jazzy ABI
// does not expose the group from SubscriptionBase) — same pattern as
// rtc_controller_manager/test/test_device_backend_cb_group_injection.cpp.

#include "integrated_bringup/support/owned_topics.hpp"
#include <rtc_base/types/types.hpp>
#include <rtc_controller_interface/rt_controller_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <span>
#include <string_view>
#include <utility>

namespace integrated_bringup {

// Minimal RTControllerInterface stub. The base's `node_` and `topic_config_`
// are protected; re-expose via public setters so the test fixture can seed
// them without driving the full PreConfigure/LoadConfig pipeline.
class TargetSubMockController : public rtc::RTControllerInterface {
 public:
  TargetSubMockController() = default;

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    return {};
  }

  void SetDeviceTarget(int, std::span<const double>) noexcept override {}

  [[nodiscard]] std::string_view Name() const noexcept override { return "target_sub_mock"; }

  void SetNodeForTest(rclcpp_lifecycle::LifecycleNode::SharedPtr node) { node_ = std::move(node); }

  void SetTopicConfigForTest(rtc::TopicConfig cfg) { topic_config_ = std::move(cfg); }
};

namespace {

bool GroupContainsSubscription(const rclcpp::CallbackGroup::SharedPtr& group,
                               const rclcpp::SubscriptionBase::SharedPtr& sub) {
  if (!group || !sub) {
    return false;
  }
  auto match =
      group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr& candidate) {
        return candidate.get() == sub.get();
      });
  return match != nullptr;
}

class ControllerTargetCbGroupInvariantTest : public ::testing::Test {
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
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "test_controller_target_cb_group_node", "", opts);
    // Capture an explicit non-default cb_group so the negative assertion can
    // verify the sub is NOT misrouted there.
    explicit_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    ctrl_ = std::make_unique<TargetSubMockController>();
    ctrl_->SetNodeForTest(node_);

    rtc::TopicConfig cfg;
    auto& group = cfg["arm"];
    group.subscribe.push_back(rtc::SubscribeTopicEntry{"/test/controller_owned_robot_target",
                                                       rtc::TopicOwnership::kController});
    ctrl_->SetTopicConfigForTest(std::move(cfg));

    CreateOwnedTopics(*ctrl_, handles_);
  }

  void TearDown() override {
    handles_.target_subs = {};
    ctrl_.reset();
    explicit_cb_group_.reset();
    node_.reset();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr explicit_cb_group_;
  std::unique_ptr<TargetSubMockController> ctrl_;
  ControllerTopicHandles handles_;
};

}  // namespace

// Positive: the controller-owned RobotTarget sub lands on the LifecycleNode's
// default callback group. CM later attaches that group to nrt_callback_executor.
TEST_F(ControllerTargetCbGroupInvariantTest, TargetSubAttachesToNodeDefaultGroup) {
  ASSERT_NE(handles_.target_subs[0], nullptr) << "CreateOwnedTopics did not create a target sub";
  auto default_group = node_->get_node_base_interface()->get_default_callback_group();
  ASSERT_NE(default_group, nullptr);

  EXPECT_TRUE(GroupContainsSubscription(default_group, handles_.target_subs[0]))
      << "controller-owned RobotTarget sub did not land in the LifecycleNode default group — "
         "a SubscriptionOptions.callback_group was likely added in owned_topics.cpp, breaking "
         "spec §0d (RobotTarget is an external-intent input, must not run at RT priority)";
}

// Negative: the sub must NOT land in an explicit non-default cb_group. This
// locks the "no SubscriptionOptions argument" contract — any future change
// that routes the controller-owned target sub through a non-default group
// should fail this assertion.
TEST_F(ControllerTargetCbGroupInvariantTest, TargetSubDoesNotAttachToExplicitGroup) {
  ASSERT_NE(handles_.target_subs[0], nullptr);

  EXPECT_FALSE(GroupContainsSubscription(explicit_cb_group_, handles_.target_subs[0]))
      << "controller-owned RobotTarget sub landed in an explicit non-default cb_group — "
         "this would silently bypass the nrt_callback_executor attachment that CM relies on";
}

}  // namespace integrated_bringup
