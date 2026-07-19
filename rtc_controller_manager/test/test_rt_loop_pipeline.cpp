// Tier 2 — RT-loop pipeline tests for RtControllerNode (rt_loop.cpp).
//
// Drives the real lifecycle with the pipeline fixtures (controller with an
// "arm" group + backend stub providing joint/motor/sensor/inference lanes) so
// ControlLoop's per-capability copy blocks, the publish-snapshot fill, and the
// inline WriteCommand hand-off run on a live RT tick. Also covers the
// boundary/abort branches the issue-#149 scope calls out:
//   - init-timeout FATAL → E-STOP ("init_timeout")
//   - sim-sync mode: CV proceed path + timeout abort → E-STOP ("sim_sync_timeout")
//   - consecutive-overrun → E-STOP ("consecutive_overrun")
//
// The abort tests call rclcpp::shutdown() from inside the node (by design), so
// every test re-inits rclcpp in SetUp and the shutdown-triggering tests run
// LAST (gtest executes in declaration order within a TU).

#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;
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

rclcpp_lifecycle::State StateActive() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
}

// Poll until `pred` holds or `budget` elapses. The abort branches are driven
// by wall-clock deadlines inside the RT thread, so tests wait generously and
// assert the outcome, never the latency.
template <typename Pred>
bool WaitFor(Pred pred, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return pred();
}

class RtLoopPipelineTest : public ::testing::Test {
 protected:
  // Abort tests shut the context down; re-init per test.
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    PipelineTestController::ResetCaptured();
  }

  static std::shared_ptr<RtControllerNode> MakeNode(double control_rate) {
    auto node = std::make_shared<RtControllerNode>("test_rt_loop_pipeline_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", control_rate);
    node->declare_parameter("config_variant", std::string("test_fixtures"));
    node->declare_parameter("initial_controller", std::string("rtc_cm_cfg_test"));
    // Full arm device: backend binding + sensor layout with an inference lane
    // so the kMotorState/kSensorData/kInference copy blocks all run.
    node->declare_parameter("devices.arm.joint_state_names", std::vector<std::string>{"j1", "j2"});
    node->declare_parameter("devices.arm.backend.type", std::string("cm_pipe_backend"));
    node->declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
    node->declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));
    node->declare_parameter("devices.arm.backend.motor_topic", std::string("/arm/motor"));
    node->declare_parameter("devices.arm.backend.sensor_topic", std::string("/arm/sensor"));
    node->declare_parameter("devices.arm.sensor_layout.primary_count_per_group", 1);
    node->declare_parameter("devices.arm.sensor_layout.secondary_count_per_group", 1);
    node->declare_parameter("devices.arm.sensor_layout.inference_values_per_group", 1);
    // Watchdog entry so state_received_ starts false (init gate armed).
    node->declare_parameter("device_timeout_names", std::vector<std::string>{"arm"});
    node->declare_parameter("device_timeout_values", std::vector<double>{60000.0});
    return node;
  }

  static PipelineStubBackend* Backend(RtControllerNode& node) {
    return dynamic_cast<PipelineStubBackend*>(ControllerLifecycleTestAccess::GetBackend(node, 0));
  }
};

// ── Happy path: full tick pipeline over a live device group ──────────────────

TEST_F(RtLoopPipelineTest, TickCopiesStateAndWritesCommandInline) {
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  // Release the init gate — the watchdog entry keeps ticks idle until the
  // backend reports its first state.
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return backend->WriteCount() > 0; }, 2000ms));

  // Phase 2 → Phase 3 → WriteCommand: the controller's fixed command vector
  // arrives at the backend verbatim, alongside the state read in Phase 1.
  EXPECT_EQ(2, backend->LastNumChannels());
  EXPECT_DOUBLE_EQ(PipelineTestController::kCmd0, backend->LastCommands()[0]);
  EXPECT_DOUBLE_EQ(PipelineTestController::kCmd1, backend->LastCommands()[1]);
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos0, backend->LastActualPositions()[0]);
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos1, backend->LastActualPositions()[1]);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Consecutive deadline overruns → E-STOP ───────────────────────────────────

TEST_F(RtLoopPipelineTest, ConsecutiveOverrunsTriggerEstop) {
  auto node = MakeNode(/*control_rate=*/500.0);  // 2 ms budget
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  // Every Compute sleeps 5× the budget → guaranteed overrun each tick,
  // regardless of host load (load only makes the overrun larger).
  PipelineTestController::compute_sleep_us.store(10000, std::memory_order_relaxed);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 5000ms));
  EXPECT_EQ("consecutive_overrun", ControllerLifecycleTestAccess::GetEstopReason(*node));

  PipelineTestController::compute_sleep_us.store(0, std::memory_order_relaxed);
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Sim-sync mode: CV proceed path, then timeout abort ───────────────────────
// NOTE: aborts call rclcpp::shutdown() → keep the shutdown-triggering tests
// (this one and InitTimeout below) after every non-aborting test.

TEST_F(RtLoopPipelineTest, SimSyncProceedsOnStateThenAbortsOnTimeout) {
  auto node = MakeNode(/*control_rate=*/250.0);
  node->declare_parameter("use_sim_time_sync", true);
  node->declare_parameter("sim_sync_timeout_sec", 0.3);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));

  // Feed the CV a few times — WaitForNextTick's kProceed path runs a tick per
  // notification (lock-step with the "simulator").
  for (int i = 0; i < 5; ++i) {
    backend->FireStateReady();
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_GT(backend->WriteCount(), 0);

  // Stop feeding → sim-sync timeout → OnLoopAborted → E-STOP + shutdown.
  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 5000ms));
  EXPECT_EQ("sim_sync_timeout", ControllerLifecycleTestAccess::GetEstopReason(*node));
  EXPECT_TRUE(WaitFor([] { return !rclcpp::ok(); }, 2000ms));
}

// ── Init timeout: no device state ever arrives → FATAL + E-STOP ──────────────

TEST_F(RtLoopPipelineTest, InitTimeoutTriggersEstopAndShutdown) {
  auto node = MakeNode(/*control_rate=*/200.0);
  node->declare_parameter("init_timeout_sec", 0.2);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  // Never fire the backend's state-ready hook: state_received_ stays false,
  // ticks idle in the init gate until init_timeout_ticks_ elapse.

  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 5000ms));
  EXPECT_EQ("init_timeout", ControllerLifecycleTestAccess::GetEstopReason(*node));
  EXPECT_TRUE(WaitFor([] { return !rclcpp::ok(); }, 2000ms));
}

}  // namespace
}  // namespace rtc
