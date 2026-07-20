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

#include "rt_cm_test_access.hpp"
#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"

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
  // When set, LoadConfig throws — the CM instantiates this controller itself,
  // so a static knob is the only handle a test has before on_configure. Used
  // to prove CM refuses the whole configure when a controller fails to
  // configure (issue #196 §1, decision D1).
  static inline std::atomic<bool> fail_load_config{false};

  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    return ControllerOutput{};
  }

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  std::string_view Name() const noexcept override { return "OnConfigureTestController"; }

  void LoadConfig(const YAML::Node& cfg) override {
    if (fail_load_config.load(std::memory_order_relaxed)) {
      throw std::runtime_error("synthetic LoadConfig failure");
    }
    RTControllerInterface::LoadConfig(cfg);
  }
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

  void SetUp() override {
    OnConfigureTestController::fail_load_config.store(false, std::memory_order_relaxed);
  }

  void TearDown() override {
    OnConfigureTestController::fail_load_config.store(false, std::memory_order_relaxed);
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

// ── T4b/T4c: Extended-URDF closure resolution reaches BOTH URDF branches ─────
//    #14 — the closure-sidecar resolution used to be nested inside the top-level
//    urdf.package/urdf.path branch, so a devices-fallback URDF with
//    urdf.extended=true silently skipped it. It now runs after either branch
//    settles. panda ships no <stem>.closure.yaml, so both cases hit the missing-
//    sidecar ERROR path — which is non-fatal (bring-up still SUCCEEDs) and
//    leaves closure_yaml_path empty. Equivalence of the two branches is the
//    contract (#14 "top-level 과 동치").

TEST_F(OnConfigureTest, ExtendedUrdfTopLevelMissingSidecarIsNonFatal) {
  auto node = MakeNode();
  node->declare_parameter("urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.extended", true);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  // Missing sidecar → closure path left unset (ERROR logged, not bound).
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetResolvedClosureYamlPath(*node).empty());
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OnConfigureTest, ExtendedUrdfDevicesFallbackMissingSidecarIsNonFatal) {
  auto node = MakeNode();
  // No top-level urdf.* — force the devices-fallback URDF branch.
  node->declare_parameter("devices.arm.urdf.package", std::string("robot_descriptions"));
  node->declare_parameter("devices.arm.urdf.path", std::string("robots/panda/urdf/panda.urdf"));
  node->declare_parameter("urdf.extended", true);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  // Same as the top-level case: extended=true is honoured on the fallback path,
  // missing sidecar leaves closure_yaml_path unset without aborting bring-up.
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetResolvedClosureYamlPath(*node).empty());
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

// ── T6: fail-closed bring-up (issue #196 §1) ────────────────────────────────
//
// Before this, CM logged a warning and kept a controller whose PreConfigure or
// on_configure had failed, so a partially configured controller stayed in the
// active/switch candidate set and could be dispatched by the RT loop. CM now
// refuses the whole configure (decision D1).

TEST_F(OnConfigureTest, ControllerConfigureFailureRefusesWholeConfigure) {
  OnConfigureTestController::fail_load_config.store(true, std::memory_order_relaxed);

  auto node = MakeNode();
  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  // Nothing was brought up: no controller registered means no active/switch
  // candidate, and the refusal happens before backends/RT thread exist.
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetControllerCount(*node));
  EXPECT_EQ(nullptr, ControllerLifecycleTestAccess::GetBackend(*node, 0));
}

TEST_F(OnConfigureTest, ConfigureRecoversAfterAFailedAttempt) {
  // A refused configure must not poison the next one — the bring-up clears the
  // state it accumulated, so a retry registers each controller exactly once.
  OnConfigureTestController::fail_load_config.store(true, std::memory_order_relaxed);
  auto node = MakeNode();
  ASSERT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  OnConfigureTestController::fail_load_config.store(false, std::memory_order_relaxed);
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U, ControllerLifecycleTestAccess::GetControllerCount(*node));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── T7: config file absent vs. present-but-broken (decision D2) ─────────────

TEST_F(OnConfigureTest, MissingControllerConfigFileFallsBackToDefaults) {
  // The default variant has no rtc_cm_test_ctrl.yaml anywhere, so LoadFile
  // raises YAML::BadFile. That is the supported "controller runs on built-in
  // defaults" path and must stay permissive.
  auto node = MakeNode();
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1U, ControllerLifecycleTestAccess::GetControllerCount(*node));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OnConfigureTest, MalformedControllerConfigFileRefusesConfigure) {
  // config/test_bad_yaml/controllers/rtc_cm_test_ctrl.yaml exists but does not
  // parse. Falling back to defaults here would run the controller with values
  // the operator never authored — refuse instead.
  auto node = MakeNode();
  node->declare_parameter("config_variant", std::string("test_bad_yaml"));

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetControllerCount(*node));
}

}  // namespace
}  // namespace rtc
