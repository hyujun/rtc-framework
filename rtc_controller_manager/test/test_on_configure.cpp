// Tier 2 — on_configure bring-up integration tests for RtControllerNode.
//
// Drives the real lifecycle pipeline (on_configure -> on_activate ->
// on_deactivate -> on_cleanup -> on_shutdown) end to end, exercising
// rt_controller_node_params.cpp (DeclareAndLoadParameters + controller
// loading), rt_controller_node_device_config.cpp, rt_controller_node_
// publishers.cpp, rt_controller_node_subscriptions.cpp, rt_controller_node.cpp
// lifecycle callbacks, and (via on_activate's RT loop) parts of
// rt_controller_node_rt_loop.cpp.
//
// A minimal controller is registered through the controller registry so the
// bring-up has something to instantiate. Parameters are injected by declaring
// them on the node before on_configure — DeclareAndLoadParameters uses
// safe_declare (declare-if-absent), so pre-declared values win. URDF resolution
// uses the repo-vendored panda model via the installed robot_descriptions
// share dir (ament runtime lookup, ARCH-5 compliant — see <test_depend>).

#include "rtc_controller_manager/rt_controller_node.hpp"

#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <span>
#include <string>
#include <thread>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;
using namespace std::chrono_literals;

// Minimal registry-loadable controller. No groups in its TopicConfig, so the
// device-backend pipeline takes its empty-group path. Ignores the URDF.
class OnConfigureTestController : public RTControllerInterface {
 public:
  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    return ControllerOutput{};
  }
  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}
  std::string_view Name() const noexcept override { return "OnConfigureTestController"; }
};

// File-scope registration. config_package = rtc_controller_manager (its share
// dir exists; the per-controller YAML does not, so LoadConfig falls through to
// defaults — the WARN path is intentional and harmless here).
RTC_REGISTER_CONTROLLER(rtc_cm_test_ctrl, "", "rtc_controller_manager",
                        std::make_unique<OnConfigureTestController>())

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

class OnConfigureTest : public ::testing::Test {
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

  // Builds a node and pre-declares low-side-effect parameters so the bring-up
  // does not write logs / CSVs. control_rate kept low so the RT loop period is
  // long if on_activate is exercised.
  static std::shared_ptr<RtControllerNode> MakeNode(double control_rate = 200.0) {
    auto node = std::make_shared<RtControllerNode>("test_on_configure_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("enable_estop", true);
    node->declare_parameter("control_rate", control_rate);
    node->declare_parameter("initial_controller", std::string(""));
    // Exercise the device-timeout parsing loop in DeclareAndLoadParameters.
    // The test controller declares no groups, so "arm" matches no active group
    // and hits the "no matching topic group" warn-and-skip branch.
    node->declare_parameter("device_timeout_names", std::vector<std::string>{"arm"});
    node->declare_parameter("device_timeout_values", std::vector<double>{100.0});
    return node;
  }
};

// ── T1: no-URDF smoke ────────────────────────────────────────────────────────

TEST_F(OnConfigureTest, ConfigureNoUrdfSucceeds) {
  auto node = MakeNode();
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  // CM exposes its registered controller via /rtc_cm/list_controllers; here we
  // just confirm the bring-up reached the end (cleanup must also succeed).
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── T2: with vendored panda URDF (kinematics branch in params.cpp) ──────────

TEST_F(OnConfigureTest, ConfigureWithPandaUrdfSucceeds) {
  auto node = MakeNode();
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── T3: full lifecycle cycle (drives on_activate + RT loop + on_deactivate) ──

TEST_F(OnConfigureTest, FullLifecycleCycle) {
  auto node = MakeNode(/*control_rate=*/100.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  // on_activate starts the RT loop jthread (SCHED_OTHER fallback in the test
  // sandbox — ApplyThreadConfig's return is intentionally discarded).
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));

  // Let the RT loop tick a few times (period = 10 ms at 100 Hz).
  std::this_thread::sleep_for(60ms);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_shutdown(StateUnconfigured()));
}

// ── T4: control_rate outside design range hits the WARN branch ──────────────

TEST_F(OnConfigureTest, ConfigureOutOfRangeControlRateStillSucceeds) {
  auto node = MakeNode(/*control_rate=*/10.0);  // below kMinControlRateHz
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── T5: on_error / on_shutdown from inactive ────────────────────────────────

TEST_F(OnConfigureTest, ErrorAndShutdownCallbacks) {
  auto node = MakeNode();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  // on_error transition handler.
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_error(StateInactive()));
  // on_shutdown from a non-active state.
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_shutdown(StateInactive()));
}

}  // namespace
}  // namespace rtc
