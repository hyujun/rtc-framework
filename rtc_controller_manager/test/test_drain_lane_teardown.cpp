// DrainLog() against the lifecycle teardown (#935).
//
// After RtControllerMain's Phase 3 the drain timer fires on the nrt_logging
// executor while on_cleanup / on_error run on nrt_callback. Dropping the timer
// does not stop a pass the executor has already picked up, so a pass can run
// after the teardown released what it touches — estop_pub_ first among them.
// These cases stand in for that pass by calling DrainLog() (or holding its
// lock) from the test thread.
//
// Its own binary: before the fix the first case dereferences a null publisher,
// and a crash here should not take the rest of a suite down with it.

#include "rclcpp_suite_fixture.hpp"
#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;
using Access = ControllerLifecycleTestAccess;
using namespace std::chrono_literals;

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

class DrainLaneTeardownTest : public RclcppSuiteTest {
 protected:
  void SetUp() override {
    PipelineTestController::ResetCaptured();
    PipelineStubBackend::ResetCaptured();
  }

  static std::shared_ptr<RtControllerNode> MakeConfiguredNode() {
    auto node = std::make_shared<RtControllerNode>("test_drain_lane_teardown_node");
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
    EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
    return node;
  }
};

// A pass that starts after on_cleanup finds a status transition queued — here
// a trigger, as a late one would leave — and must not act on it: the publisher
// it would report through is gone.
TEST_F(DrainLaneTeardownTest, ADrainPassAfterCleanupDoesNothing) {
  auto node = MakeConfiguredNode();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));

  Access::CallTriggerEstop(*node, "after_cleanup");
  ASSERT_TRUE(Access::GetEstopStatusPending(*node));

  Access::CallDrainLog(*node);

  EXPECT_TRUE(Access::GetEstopStatusPending(*node)) << "the pass acted on a torn-down node";
  EXPECT_TRUE(Access::GetEstopLogPending(*node));
}

// Same for on_error, the teardown that itself raises the E-STOP.
TEST_F(DrainLaneTeardownTest, ADrainPassAfterOnErrorDoesNothing) {
  auto node = MakeConfiguredNode();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_error(StateInactive()));
  ASSERT_TRUE(Access::IsEstopped(*node));
  ASSERT_FALSE(Access::GetEstopStatusPending(*node)) << "on_error must flush its own trigger";

  (void)Access::CallClearEstop(*node);
  ASSERT_TRUE(Access::GetEstopStatusPending(*node));

  Access::CallDrainLog(*node);

  EXPECT_TRUE(Access::GetEstopStatusPending(*node)) << "the pass acted on a torn-down node";
}

// A pass in flight holds the drain lock. on_cleanup must wait for it before it
// touches anything the pass reads — the drain timer included, since dropping
// it first is what let a picked-up pass outlive the teardown.
TEST_F(DrainLaneTeardownTest, CleanupWaitsForADrainPassInFlight) {
  auto node = MakeConfiguredNode();
  const auto timer = Access::DrainTimer(*node);
  ASSERT_FALSE(timer.expired());

  auto pass = Access::LockDrainLane(*node);
  std::atomic<bool> returned{false};
  CallbackReturn rc = CallbackReturn::ERROR;
  std::thread lifecycle([&] {
    rc = node->on_cleanup(StateInactive());
    returned.store(true);
  });
  std::this_thread::sleep_for(200ms);

  EXPECT_FALSE(timer.expired()) << "on_cleanup dropped the drain timer while a pass held the lane";
  EXPECT_FALSE(returned.load());

  pass.unlock();
  lifecycle.join();
  EXPECT_EQ(CallbackReturn::SUCCESS, rc);
  EXPECT_TRUE(timer.expired());
}

}  // namespace
}  // namespace rtc
