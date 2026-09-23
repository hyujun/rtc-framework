// Tier 2 — sim-sync tick barrier over TWO device groups (issue #566).
//
// The defect: in lock-step the simulator publishes each device group's joint
// state as its own message, and every one of them woke the RT loop. With an
// arm and a hand that was up to two ticks per simulator step — measured 838 Hz
// against a 500 Hz step on ur5e_p1b — and every law that integrates
// ControllerState::dt ran ~1.67× faster than the simulated world.
//
// The partner message is deliberately fired a few milliseconds after the
// first: that is the interleaving that split a step in two (the loop woke on
// the arm alone and ticked before the hand's state arrived). Firing both back
// to back would mostly collapse into one eventfd read and pass either way.
//
// Own gtest binary for the same reason as test_device_readiness_gate: the
// controller registry is process-global, and a two-group controller would
// reshape every node built in the single-group pipeline TU.

#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;
using namespace std::chrono_literals;

class TwoDeviceTestController : public PipelineTestController {
 public:
  static constexpr const char* kName = "rtc_cm_two_device_test";

  std::string_view Name() const noexcept override { return kName; }

  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    ControllerOutput out{};
    out.num_devices = 2;
    for (std::size_t d = 0; d < 2; ++d) {
      out.devices[d].num_channels = 2;
      out.devices[d].commands[0] = kCmd0;
      out.devices[d].commands[1] = kCmd1;
    }
    return out;
  }
};

RTC_REGISTER_CONTROLLER(rtc_cm_two_device_test, "", "rtc_controller_manager",
                        std::make_unique<TwoDeviceTestController>())
RTC_REGISTER_DEVICE_BACKEND(cm_sim_sync_backend, std::make_unique<PipelineStubBackend>())

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

template <typename Pred>
bool WaitFor(Pred pred, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return pred();
}

// Long enough to separate "the loop looked and declined" from "the loop has
// not looked yet" on a loaded CI host, short against sim_sync_timeout_sec.
constexpr auto kSettle = 50ms;
// The gap between a step's two messages — well past one RT wake-up, so the
// loop has certainly seen the first message alone before the second lands.
constexpr auto kPartnerGap = 5ms;

class SimSyncBarrierTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    PipelineTestController::ResetCaptured();
    PipelineStubBackend::ResetCaptured();
  }

  // Slots follow group_slot_map_'s ordering: arm=0, hand=1.
  static std::shared_ptr<RtControllerNode> MakeNode(
      const std::vector<std::string>& tick_devices = {}) {
    auto node = std::make_shared<RtControllerNode>("test_sim_sync_barrier_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", 500.0);
    node->declare_parameter("config_variant", std::string("test_fixtures"));
    node->declare_parameter("initial_controller", std::string(TwoDeviceTestController::kName));
    node->declare_parameter("use_sim_time_sync", true);
    node->declare_parameter("sim_sync_timeout_sec", 5.0);
    if (!tick_devices.empty()) {
      node->declare_parameter("sim_sync_tick_devices", tick_devices);
    }
    for (const auto* group : {"arm", "hand"}) {
      const std::string g(group);
      node->declare_parameter("devices." + g + ".joint_state_names",
                              std::vector<std::string>{"j1", "j2"});
      node->declare_parameter("devices." + g + ".backend.type", std::string("cm_sim_sync_backend"));
      node->declare_parameter("devices." + g + ".backend.state_topic", "/" + g + "/state");
      node->declare_parameter("devices." + g + ".backend.command_topic", "/" + g + "/cmd");
    }
    node->declare_parameter("device_timeout_names", std::vector<std::string>{"arm", "hand"});
    node->declare_parameter("device_timeout_values", std::vector<double>{60000.0, 60000.0});
    return node;
  }

  static PipelineStubBackend* BackendAt(RtControllerNode& node, std::size_t slot) {
    return dynamic_cast<PipelineStubBackend*>(
        ControllerLifecycleTestAccess::GetBackend(node, slot));
  }
};

TEST_F(SimSyncBarrierTest, OneTickPerStepWhenEveryDeviceCompletesTheStep) {
  auto node = MakeNode();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* arm = BackendAt(*node, 0);
  auto* hand = BackendAt(*node, 1);
  ASSERT_NE(nullptr, arm);
  ASSERT_NE(nullptr, hand);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));

  // Half a step: the loop must not run.
  arm->FireStateReady();
  std::this_thread::sleep_for(kSettle);
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::RtTickCount(*node))
      << "ticked on the arm's state alone";

  // The step completes: exactly one tick.
  hand->FireStateReady();
  ASSERT_TRUE(
      WaitFor([&] { return ControllerLifecycleTestAccess::RtTickCount(*node) >= 1; }, 2000ms));
  std::this_thread::sleep_for(kSettle);
  EXPECT_EQ(1U, ControllerLifecycleTestAccess::RtTickCount(*node));

  // N more steps, each split by kPartnerGap — the interleaving that used to
  // double-tick. Also cover the reverse order and a device reporting twice.
  constexpr std::uint64_t kSteps = 20;
  for (std::uint64_t i = 0; i < kSteps; ++i) {
    auto* first = (i % 2 == 0) ? arm : hand;
    auto* second = (i % 2 == 0) ? hand : arm;
    first->FireStateReady();
    if (i % 5 == 0) {
      first->FireStateReady();  // a repeat is still one step, not two
    }
    std::this_thread::sleep_for(kPartnerGap);
    second->FireStateReady();
    ASSERT_TRUE(
        WaitFor([&] { return ControllerLifecycleTestAccess::RtTickCount(*node) >= 2 + i; }, 2000ms))
        << "step " << i;
    // Settle before the next step: otherwise its first message lands on the
    // heels of this step's second and the two collapse into one eventfd read,
    // which hides a split step (measured: a barrier-less mutant then gained
    // only one tick over 20 steps instead of one per step).
    std::this_thread::sleep_for(kPartnerGap);
    ASSERT_EQ(2 + i, ControllerLifecycleTestAccess::RtTickCount(*node))
        << "step " << i << " was split into more than one tick";
  }
  std::this_thread::sleep_for(kSettle);
  EXPECT_EQ(1U + kSteps, ControllerLifecycleTestAccess::RtTickCount(*node))
      << "a step was split into more than one tick";

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(SimSyncBarrierTest, ADeviceLeftOutOfTheBarrierIsReadLatestValue) {
  // The decimated-lane shape: the hand refreshes every 5th step, the arm
  // every step. Only the arm completes a step.
  auto node = MakeNode({"arm"});
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* arm = BackendAt(*node, 0);
  auto* hand = BackendAt(*node, 1);
  ASSERT_NE(nullptr, arm);
  ASSERT_NE(nullptr, hand);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));

  constexpr std::uint64_t kSteps = 10;
  for (std::uint64_t i = 0; i < kSteps; ++i) {
    if (i % 5 == 0) {
      hand->FireStateReady();  // readiness gate needs one; after that it only refreshes
      std::this_thread::sleep_for(kPartnerGap);
    }
    arm->FireStateReady();
    ASSERT_TRUE(
        WaitFor([&] { return ControllerLifecycleTestAccess::RtTickCount(*node) >= 1 + i; }, 2000ms))
        << "step " << i;
    std::this_thread::sleep_for(kPartnerGap);  // see the settle note above
    ASSERT_EQ(1 + i, ControllerLifecycleTestAccess::RtTickCount(*node)) << "step " << i;
  }
  // A hand refresh on its own is not a step.
  hand->FireStateReady();
  std::this_thread::sleep_for(kSettle);
  EXPECT_EQ(kSteps, ControllerLifecycleTestAccess::RtTickCount(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(SimSyncBarrierTest, StopIsNotHeldByAnIncompleteStep) {
  auto node = MakeNode();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* arm = BackendAt(*node, 0);
  ASSERT_NE(nullptr, arm);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));

  // The loop is waiting for the hand. The stop nudge is a single wake that
  // completes no step; the barrier must not swallow it and wait out the
  // 5 s sim_sync_timeout_sec.
  arm->FireStateReady();
  std::this_thread::sleep_for(kSettle);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(SimSyncBarrierTest, AnUnknownTickDeviceRefusesToConfigure) {
  auto node = MakeNode({"arm", "hnad"});
  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
}

}  // namespace
}  // namespace rtc
