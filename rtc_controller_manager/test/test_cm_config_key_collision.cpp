// Tier 2 — a config_key that shadows an EARLIER controller's Name() is refused.
//
// Issue #205 B-1. Tier 2 scans two identifiers per registration:
//
//     for (const std::string& id : {ctrl_name, entry.config_key})
//
// The existing collision test (test_cm_controller_name_collision.cpp) cannot
// distinguish those two arms. Its fixture's Name() is the literal string
// "rtc_cm_cfg_test", which is also registration #0's config_key, so the two map
// keys collapse into one and only the `ctrl_name` arm ever hits. Narrowing the
// loop to {ctrl_name} would leave that test green.
//
// This file covers the arm that is otherwise unreachable: registration #1 whose
// *config_key* equals registration #0's *Name()*, with every other identifier
// distinct. Only the `entry.config_key` arm can catch it.
//
//     #0: config_key "rtc_cm_cfg_test"      Name() "rtc_cm_alias_name_a"
//     #1: config_key "rtc_cm_alias_name_a"  Name() "rtc_cm_alias_name_b"
//                    ^^^^^^^^^^^^^^^^^^^^ collides with #0's Name()
//
// Selecting "rtc_cm_alias_name_a" would be ambiguous — it names #0's class and
// #1's YAML — so the operator cannot know which controller they get.
//
// Own gtest executable, and the polluting case runs LAST, for the same reason as
// the two sibling collision tests: the registry is a process-wide singleton with
// no removal API. The clean case asserts a pristine registry first so a
// reordering (--gtest_shuffle / --gtest_repeat / a TEST_F appended below) fails
// naming itself instead of looking like a guard bug.

#include "rt_cm_pipeline_fixtures.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;

RTC_REGISTER_CONTROLLER(rtc_cm_cfg_test, "", "rtc_controller_manager",
                        std::make_unique<AliasNameTestControllerA>())
RTC_REGISTER_DEVICE_BACKEND(cm_pipe_backend, std::make_unique<PipelineStubBackend>())

rclcpp_lifecycle::State StateUnconfigured() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                 "unconfigured");
}

rclcpp_lifecycle::State StateInactive() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}

class CmConfigKeyCollisionTest : public ::testing::Test {
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
    PipelineTestController::ResetCaptured();
    PipelineStubBackend::ResetCaptured();
  }

  static std::shared_ptr<RtControllerNode> MakeNode(const std::string& name) {
    auto node = std::make_shared<RtControllerNode>(name);
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", 200.0);
    node->declare_parameter("config_variant", std::string("test_fixtures"));
    node->declare_parameter("devices.arm.joint_state_names",
                            std::vector<std::string>{"panda_joint1", "panda_joint2"});
    node->declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));
    node->declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
    node->declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));
    return node;
  }
};

// ── Control: Name() != config_key is legal and configures normally ───────────
//
// Without this the refusal below would pass equally against a guard that
// rejects any registration whose Name() differs from its config_key.
TEST_F(CmConfigKeyCollisionTest, DistinctIdentifiersConfigure) {
  ASSERT_EQ(std::size_t{1}, ControllerRegistry::Instance().GetEntries().size())
      << "registry is already polluted — this test must run BEFORE the "
         "collision-registering case in this file. Check for --gtest_shuffle, "
         "--gtest_repeat, or a TEST_F added after it.";

  auto node = MakeNode("test_cm_cfgkey_collision_clean_node");

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1, PipelineTestController::instances_created.load(std::memory_order_relaxed));
  EXPECT_GT(PipelineStubBackend::configure_calls.load(std::memory_order_relaxed), 0)
      << "control case must get past D1 and actually wire the device backend";

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── A config_key colliding with an earlier Name() refuses configure ──────────
//
// MUST BE LAST: the second registration cannot be undone.
TEST_F(CmConfigKeyCollisionTest, ConfigKeyShadowingEarlierNameRefusesConfigure) {
  // Tier 1's pre-Pass-1 scan compares config_key against config_key only, so
  // "rtc_cm_alias_name_a" vs "rtc_cm_cfg_test" passes it. The controller built
  // here reports "rtc_cm_alias_name_b", which collides with nothing — so the
  // ONLY colliding identifier in this registry is #1's config_key against #0's
  // Name(). If Tier 2 stops checking entry.config_key, this configure succeeds
  // and the test fails.
  ControllerRegistry::Instance().Register(
      {AliasNameTestControllerA::kName, "", "rtc_controller_manager",
       [](const std::string&) { return std::make_unique<AliasNameTestControllerB>(); }});

  auto node = MakeNode("test_cm_cfgkey_collision_dup_node");

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  // Both factories ran: Tier 2 is deliberately post-instantiation because
  // Name() is not knowable before factory().
  EXPECT_EQ(2, PipelineTestController::instances_created.load(std::memory_order_relaxed));

  // What D1 is worth — refused before CreateDeviceBackends(), so no publisher,
  // subscription, log channel, service, timer or RT thread exists.
  EXPECT_EQ(0, PipelineStubBackend::configure_calls.load(std::memory_order_relaxed))
      << "refusal must land before any device wiring";
}

}  // namespace
}  // namespace rtc
