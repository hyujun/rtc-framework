// Tier 2 — startup readiness gate over TWO device groups (issue #198 §1).
//
// The defect: every backend set the same global `state_received_` flag and the
// RT loop's init gate read only that flag, so one device's first packet
// released the gate for the whole robot. A device that was silent from the
// start stayed `received=false`, which the watchdog skipped forever — an arm
// that runs while its hand never came up, with no E-STOP. The shipped configs
// are exactly this shape (`device_timeout_names: ["ur5e", "p1a"]`).
//
// This lives in its own gtest binary on purpose. Controllers are discovered
// through a process-global registry, so registering a two-group controller in
// the single-group pipeline TU would add a `hand` group to every node built
// there — and every one of those tests would then sit in the readiness gate
// waiting for a device the test never wires. One TU, one robot shape.

#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;
using namespace std::chrono_literals;

// Two-group controller. Emits a well-formed output for both devices so the
// actuator-boundary validation (issue #196) forwards it unchanged and
// WriteCount() means "the control law ran", not "a hold was substituted".
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
RTC_REGISTER_DEVICE_BACKEND(cm_gate_backend, std::make_unique<PipelineStubBackend>())

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
    std::this_thread::sleep_for(5ms);
  }
  return pred();
}

class DeviceReadinessGateTest : public ::testing::Test {
 protected:
  // The deadline test shuts the context down from inside the node; re-init.
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    PipelineTestController::ResetCaptured();
    PipelineStubBackend::ResetCaptured();
  }

  // Slots follow group_slot_map_'s ordering: arm=0, hand=1.
  static std::shared_ptr<RtControllerNode> MakeNode(double control_rate) {
    auto node = std::make_shared<RtControllerNode>("test_device_readiness_gate_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", control_rate);
    node->declare_parameter("config_variant", std::string("test_fixtures"));
    node->declare_parameter("initial_controller", std::string(TwoDeviceTestController::kName));
    for (const auto* group : {"arm", "hand"}) {
      const std::string g(group);
      node->declare_parameter("devices." + g + ".joint_state_names",
                              std::vector<std::string>{"j1", "j2"});
      node->declare_parameter("devices." + g + ".backend.type", std::string("cm_gate_backend"));
      node->declare_parameter("devices." + g + ".backend.state_topic", "/" + g + "/state");
      node->declare_parameter("devices." + g + ".backend.command_topic", "/" + g + "/cmd");
    }
    // Long enough that nothing trips on staleness — these tests are about
    // first-report readiness, not about the watchdog.
    node->declare_parameter("device_timeout_names", std::vector<std::string>{"arm", "hand"});
    node->declare_parameter("device_timeout_values", std::vector<double>{60000.0, 60000.0});
    return node;
  }

  static PipelineStubBackend* BackendAt(RtControllerNode& node, std::size_t slot) {
    return dynamic_cast<PipelineStubBackend*>(
        ControllerLifecycleTestAccess::GetBackend(node, slot));
  }
};

TEST_F(DeviceReadinessGateTest, OneReadyDeviceDoesNotReleaseTheGateForTheOther) {
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  auto* arm = BackendAt(*node, 0);
  auto* hand = BackendAt(*node, 1);
  ASSERT_NE(nullptr, arm);
  ASSERT_NE(nullptr, hand);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));

  // Only the arm reports. At 250 Hz that is ~25 ticks, and every one of them
  // must idle in the gate — no Compute(), no WriteCommand(), on either device.
  arm->FireStateReady();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(0, arm->WriteCount()) << "control law ran while a device had never reported";
  EXPECT_EQ(0, hand->WriteCount());

  // The hand reports → the gate opens and both devices are driven.
  hand->FireStateReady();
  EXPECT_TRUE(WaitFor([&] { return arm->WriteCount() > 0 && hand->WriteCount() > 0; }, 2000ms));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// Declared last: shuts the context down from inside the node (gtest runs in
// declaration order within a TU).
TEST_F(DeviceReadinessGateTest, SilentDeviceEstopsAtTheInitDeadlineAndNamesItself) {
  auto node = MakeNode(/*control_rate=*/200.0);
  node->declare_parameter("init_timeout_sec", 0.2);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  auto* arm = BackendAt(*node, 0);
  ASSERT_NE(nullptr, arm);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));

  // The arm streams normally; the hand never does.
  arm->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 5000ms));
  // Names the device that held the gate shut — the operator-facing payoff of
  // tracking readiness per device rather than globally.
  EXPECT_EQ("hand_init_timeout", ControllerLifecycleTestAccess::GetEstopReason(*node));
  EXPECT_EQ(0, arm->WriteCount());
  EXPECT_TRUE(WaitFor([] { return !rclcpp::ok(); }, 2000ms));
}

}  // namespace
}  // namespace rtc
