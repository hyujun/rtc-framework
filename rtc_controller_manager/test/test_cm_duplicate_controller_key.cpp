// Tier 2 — duplicate controller config_key is refused before instantiation.
//
// Issue #196 Phase 5. ControllerRegistry::Register() only warns on a duplicate
// key and lets the second registration shadow the first: it cannot throw,
// because registration happens during static init where ordering across TUs
// makes a throwing registrar fragile (see controller_registry.cpp). CM
// therefore scans the registry when it enters bring-up, before Pass 1 calls
// any factory.
//
// This lives in its own gtest executable because the registry is a process-wide
// singleton with no removal API — the duplicate registered below is permanent
// for the binary, so it must not be able to reach any other test's bring-up.
// Test order within the file is load-bearing for the same reason: the clean
// case runs first and the polluting case last. That ordering is a gtest
// declaration-order guarantee, which --gtest_shuffle / --gtest_repeat / a new
// TEST_F appended below would break — so the clean case asserts the registry
// is still pristine first, and fails naming the real cause instead of looking
// like a scan bug.

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

class CmDuplicateKeyTest : public ::testing::Test {
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

  void SetUp() override { PipelineTestController::ResetCaptured(); }

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

// ── Control: a unique registry configures and does build the controller ─────
//
// Without this the duplicate case below would pass just as well against a scan
// that refuses every bring-up.
TEST_F(CmDuplicateKeyTest, UniqueKeysConfigureAndInstantiate) {
  ASSERT_EQ(std::size_t{1}, ControllerRegistry::Instance().GetEntries().size())
      << "registry is already polluted — this test must run BEFORE the "
         "duplicate-registering case in this file. Check for --gtest_shuffle, "
         "--gtest_repeat, or a TEST_F added after it.";

  auto node = MakeNode("test_cm_duplicate_key_clean_node");

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1, PipelineTestController::instances_created.load(std::memory_order_relaxed));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Duplicate config_key refuses configure, having built nothing ────────────
//
// MUST BE LAST: the second registration cannot be undone.
TEST_F(CmDuplicateKeyTest, DuplicateKeyRefusesConfigureBeforeInstantiating) {
  // A second package registering the same key — the packaging mistake the scan
  // exists for. Shadowing means the operator silently runs the second factory.
  ControllerRegistry::Instance().Register(
      {"rtc_cm_cfg_test", "", "rtc_controller_manager",
       [](const std::string&) { return std::make_unique<PipelineTestController>(); }});

  auto node = MakeNode("test_cm_duplicate_key_dup_node");

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  // The whole point of scanning before Pass 1: no controller and no
  // per-controller LifecycleNode was constructed for a bring-up that could
  // never be correct.
  EXPECT_EQ(0, PipelineTestController::instances_created.load(std::memory_order_relaxed));
}

}  // namespace
}  // namespace rtc
