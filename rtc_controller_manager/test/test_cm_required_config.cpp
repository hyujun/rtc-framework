// Tier 2 — ControllerEntry::config_required, the third case of issue #196 D2.
//
// D2 had two cases: config file absent → run on built-in defaults; file present
// but unreadable → refuse the configure. A controller with no defensible
// defaults (a policy controller: no model path, no IO schema) fits neither. As
// a plain registration it threw out of LoadConfig on the absent-file path and
// the D1 checkpoint refused EVERY controller — so a robot that simply does not
// use that controller could not bring up the four that it does use.
//
// config_required makes the absent-file case mean "not on this robot". Two
// properties are pinned here, and the second is the one that bites silently:
//
//   1. the skip happens and the rest of the bring-up survives it;
//   2. a name still resolves to the RIGHT controller afterwards.
//
// (2) exists because controller_name_to_idx_ maps a name to an index into
// controllers_, while the Pass 1 loop walks registry entries. Those two indices
// coincide only while nothing is skipped. Storing the entry index would make
// switch_controller("x") activate a different controller and report success —
// the bounds check in SwitchActiveController cannot catch it, because the wrong
// index is still in range.

#include "rt_cm_test_access.hpp"
#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <span>
#include <string>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;

// Registered config_required with no YAML anywhere → must be skipped.
// LoadConfig throws exactly like DemoInferenceController's does, so if the skip
// regresses this test fails loudly rather than silently changing shape.
class RequiredConfigController : public RTControllerInterface {
 public:
  static inline std::atomic<int> load_config_calls{0};

  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    return ControllerOutput{};
  }

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  std::string_view Name() const noexcept override { return "RequiredConfigController"; }

  void LoadConfig(const YAML::Node& cfg) override {
    load_config_calls.fetch_add(1, std::memory_order_relaxed);
    if (!cfg || !cfg.IsMap()) {
      throw std::invalid_argument("rtc_cm_required_cfg_test: config node is missing or not a map");
    }
    RTControllerInterface::LoadConfig(cfg);
  }
};

// Tolerates defaults — the survivor.
class PlainController : public RTControllerInterface {
 public:
  ControllerOutput Compute(const ControllerState& /*state*/) noexcept override {
    return ControllerOutput{};
  }

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  std::string_view Name() const noexcept override { return "PlainController"; }
};

// Registration ORDER is load-bearing: the skipped entry must come FIRST so the
// survivor's registry index (1) differs from its controllers_ index (0). With
// the survivor first, both are 0 and the index-alignment assertion below would
// pass even against the stale-index bug it exists to catch.
RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG(rtc_cm_required_cfg_test, "", "rtc_controller_manager",
                                         std::make_unique<RequiredConfigController>())

RTC_REGISTER_CONTROLLER(rtc_cm_plain_cfg_test, "", "rtc_controller_manager",
                        std::make_unique<PlainController>())

rclcpp_lifecycle::State StateUnconfigured() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                 "unconfigured");
}

class RequiredConfigTest : public ::testing::Test {
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

  void SetUp() override { RequiredConfigController::load_config_calls.store(0); }

  static std::shared_ptr<RtControllerNode> MakeNode() {
    auto node = std::make_shared<RtControllerNode>("test_cm_required_config_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("enable_estop", true);
    node->declare_parameter("control_rate", 200.0);
    node->declare_parameter("initial_controller", std::string(""));
    return node;
  }
};

TEST_F(RequiredConfigTest, ControllerRequiringConfigIsSkippedNotFatal) {
  // The whole point: configure SUCCEEDS. Before config_required this returned
  // FAILURE and no controller at all was registered.
  auto node = MakeNode();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  // Exactly the tolerant controller survives.
  EXPECT_EQ(1U, ControllerLifecycleTestAccess::GetControllerCount(*node));

  // Skipped BEFORE instantiation reaches LoadConfig — not instantiated and then
  // swallowed. A skip implemented by catching the throw would leave this at 1
  // and would also swallow genuine config errors.
  EXPECT_EQ(0, RequiredConfigController::load_config_calls.load());

  EXPECT_EQ(CallbackReturn::SUCCESS,
            node->on_cleanup(rclcpp_lifecycle::State(
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")));
}

TEST_F(RequiredConfigTest, NameResolutionSurvivesASkippedRegistration) {
  // The silent half. The survivor is registry entry #1 but controllers_[0];
  // every identifier it owns must resolve to 0, or switch_controller sends the
  // robot to a controller nobody selected and reports success.
  auto node = MakeNode();
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  const auto& idx_map = ControllerLifecycleTestAccess::GetNameToIdx(*node);
  ASSERT_TRUE(idx_map.contains("PlainController"));
  ASSERT_TRUE(idx_map.contains("rtc_cm_plain_cfg_test"));
  EXPECT_EQ(0, idx_map.at("PlainController"));
  EXPECT_EQ(0, idx_map.at("rtc_cm_plain_cfg_test"));

  // The skipped controller must not be addressable at all — a leftover entry
  // would index a controllers_ slot belonging to someone else.
  EXPECT_FALSE(idx_map.contains("RequiredConfigController"));
  EXPECT_FALSE(idx_map.contains("rtc_cm_required_cfg_test"));

  EXPECT_EQ(CallbackReturn::SUCCESS,
            node->on_cleanup(rclcpp_lifecycle::State(
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")));
}

TEST_F(RequiredConfigTest, APresentButMisspelledConfigStillRefuses) {
  // config_required relaxes ONLY the absent-file case. config/test_bad_yaml/
  // controllers/rtc_cm_required_cfg_test.yaml exists and holds the wrong
  // top-level key, so the node is empty for the same reason a typo produces —
  // and that must still refuse, or a fat-fingered key silently drops the
  // policy controller on the robot that does run it.
  auto node = MakeNode();
  node->declare_parameter("config_variant", std::string("test_required_cfg_typo"));

  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(1, RequiredConfigController::load_config_calls.load())
      << "the file was found, so LoadConfig must have been reached and thrown";
}

}  // namespace
}  // namespace rtc
