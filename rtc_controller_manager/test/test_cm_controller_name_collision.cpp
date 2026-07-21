// Tier 2 — a Name() collision refuses configure before any device wiring.
//
// Issue #196 Phase 5 (review follow-up). CM resolves switch_controller and
// initial_controller through controller_name_to_idx_, a SINGLE namespace that
// holds both a controller's Name() and its registration config_key. The
// pre-Pass-1 duplicate scan (test_cm_duplicate_controller_key.cpp) can only
// see config_key, because ControllerEntry carries no name and Name() is a
// virtual on the instance. So two registrations with distinct config_keys but
// a colliding Name() — the copy-pasted-controller-class mistake — slip past
// Tier 1 and are caught in Pass 1 instead, at the D1 checkpoint.
//
// Own gtest executable for the same reason as the duplicate-key test: the
// registration below is permanent for the binary and must not reach any other
// test's bring-up. Test order within the file is load-bearing — clean case
// first, polluting case last — and the clean case asserts a pristine registry
// so a reordering fails naming itself rather than looking like a guard bug.

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
                        std::make_unique<PipelineTestController>())
RTC_REGISTER_DEVICE_BACKEND(cm_pipe_backend, std::make_unique<PipelineStubBackend>())

rclcpp_lifecycle::State StateUnconfigured() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                 "unconfigured");
}

rclcpp_lifecycle::State StateInactive() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}

class CmNameCollisionTest : public ::testing::Test {
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
    PipelineStubBackend::configure_calls.store(0, std::memory_order_relaxed);
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

// ── Control: a collision-free registry configures and wires its backend ─────
TEST_F(CmNameCollisionTest, DistinctIdentifiersConfigure) {
  ASSERT_EQ(std::size_t{1}, ControllerRegistry::Instance().GetEntries().size())
      << "registry is already polluted — this test must run BEFORE the "
         "collision-registering case in this file. Check for --gtest_shuffle, "
         "--gtest_repeat, or a TEST_F added after it.";

  auto node = MakeNode("test_cm_name_collision_clean_node");

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1, PipelineTestController::instances_created.load(std::memory_order_relaxed));
  EXPECT_GT(PipelineStubBackend::configure_calls.load(std::memory_order_relaxed), 0)
      << "control case must get past D1 and actually wire the device backend";

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Colliding Name() under a distinct config_key refuses configure ──────────
//
// MUST BE LAST: the second registration cannot be undone.
TEST_F(CmNameCollisionTest, CollidingNameRefusesConfigureBeforeDeviceWiring) {
  // Distinct config_key, so Tier 1's duplicate scan passes. The factory builds
  // another PipelineTestController, whose Name() is "rtc_cm_cfg_test" — which
  // collides with BOTH the first registration's Name() and its config_key,
  // the two ways one namespace can be shadowed.
  ControllerRegistry::Instance().Register(
      {"rtc_cm_cfg_test_v2", "", "rtc_controller_manager",
       [](const std::string&) { return std::make_unique<PipelineTestController>(); }});

  auto node = MakeNode("test_cm_name_collision_dup_node");

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  // Both factories ran: Tier 2 is deliberately post-instantiation because
  // Name() is not knowable before factory(). This pins that asymmetry so a
  // future "move it earlier" refactor has to confront it.
  EXPECT_EQ(2, PipelineTestController::instances_created.load(std::memory_order_relaxed));

  // What the D1 checkpoint is worth: refused before CreateDeviceBackends(),
  // so no publisher, subscription, log channel, service, timer or RT thread
  // exists for a bring-up that could never be correct.
  EXPECT_EQ(0, PipelineStubBackend::configure_calls.load(std::memory_order_relaxed));
}

}  // namespace
}  // namespace rtc
