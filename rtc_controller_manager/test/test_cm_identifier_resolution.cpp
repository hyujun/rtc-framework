// initial_controller resolves through BOTH halves of the identifier namespace.
//
// Issue #205 B-2. controller_name_to_idx_ holds a controller's Name() *and* its
// registration config_key, and CM inserts both (rt_controller_node_params.cpp).
// Nothing tested the config_key half: every fixture-driven test injected only
// Name(), and the one bring-up that resolved a real string used
// "rtc_cm_cfg_test", which is simultaneously that fixture's Name() and its
// config_key. Deleting the production config_key insert therefore left the
// entire suite green while breaking the operator-facing path — you select a
// controller by the key you wrote in YAML, not by its C++ class label.
//
// Two things make the assertions here discriminating:
//
//   1. TWO controllers are registered, so the selected index is 1. The
//      not-found fallback stores 0 (rt_controller_node_params.cpp), so an
//      EXPECT on index 1 cannot be satisfied by a lookup that silently missed —
//      which is exactly how the pre-existing `EXPECT_EQ(0, GetActiveIdx)`
//      assertions could not tell resolution from fallback.
//   2. The controllers' Name() differs from their config_key
//      (AliasNameTestController{A,B}), so the two halves of the namespace are
//      separately addressable.
//
// No registration is added at runtime here, so this file does not pollute the
// registry and its test order is not load-bearing.

#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"
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

// Entry 0 keeps the config_key that owns the fixture YAML. Entry 1 has no YAML
// of its own, which is deliberately harmless: a MISSING controller config falls
// through to built-in defaults by design (issue #196 decision D2), so it cannot
// manufacture the bring-up failure or success this file asserts on.
RTC_REGISTER_CONTROLLER(rtc_cm_cfg_test, "", "rtc_controller_manager",
                        std::make_unique<AliasNameTestControllerA>())
RTC_REGISTER_CONTROLLER(rtc_cm_cfg_test_b, "", "rtc_controller_manager",
                        std::make_unique<AliasNameTestControllerB>())
RTC_REGISTER_DEVICE_BACKEND(cm_pipe_backend, std::make_unique<PipelineStubBackend>())

rclcpp_lifecycle::State StateUnconfigured() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                 "unconfigured");
}

rclcpp_lifecycle::State StateInactive() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}

class CmIdentifierResolutionTest : public ::testing::Test {
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

  static std::shared_ptr<RtControllerNode> MakeNode(const std::string& name,
                                                    const std::string& initial_controller) {
    auto node = std::make_shared<RtControllerNode>(name);
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", 200.0);
    node->declare_parameter("config_variant", std::string("test_fixtures"));
    node->declare_parameter("initial_controller", initial_controller);
    node->declare_parameter("devices.arm.joint_state_names",
                            std::vector<std::string>{"panda_joint1", "panda_joint2"});
    node->declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));
    node->declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
    node->declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));
    return node;
  }
};

// ── The operator-facing lane: select by the key written in YAML ──────────────
TEST_F(CmIdentifierResolutionTest, InitialControllerResolvesByConfigKey) {
  ASSERT_EQ(std::size_t{2}, ControllerRegistry::Instance().GetEntries().size());

  auto node = MakeNode("test_cm_ident_cfgkey_node", "rtc_cm_cfg_test_b");
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetActiveIdx(*node))
      << "initial_controller='rtc_cm_cfg_test_b' is entry #1's config_key. Index 0 "
         "here means the lookup MISSED and fell back to the first controller — the "
         "config_key half of controller_name_to_idx_ is not being populated.";

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── The class-label lane, kept covered so a fix to one does not drop the other ─
TEST_F(CmIdentifierResolutionTest, InitialControllerResolvesByName) {
  auto node = MakeNode("test_cm_ident_name_node", AliasNameTestControllerB::kName);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetActiveIdx(*node))
      << "initial_controller='" << AliasNameTestControllerB::kName
      << "' is entry #1's Name(); index 0 means the Name() half missed.";

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Negative control: an unknown identifier must NOT look like a hit ─────────
//
// Without this, both cases above could be satisfied by a resolver that returns
// index 1 for anything. It also pins the documented fallback (index 0) rather
// than leaving it to be inferred.
TEST_F(CmIdentifierResolutionTest, UnknownInitialControllerFallsBackToFirst) {
  auto node = MakeNode("test_cm_ident_unknown_node", "no_such_controller");
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetActiveIdx(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

}  // namespace
}  // namespace rtc
