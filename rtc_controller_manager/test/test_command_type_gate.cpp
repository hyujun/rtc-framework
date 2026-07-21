// Tier 2 — controller command type vs backend capability (issue #198).
//
// The defect: WriteCommand's `command_type` argument was advisory.
// ur_driver_native ignores it and publishes every value it is handed into a
// forward_position_controller Float64MultiArray, so a torque-mode controller
// bound to it would have put newton-metres on the wire as joint angles — and
// CM's own hold command, which emits 0.0 for kTorque because it carries no
// dynamic model, would have commanded the arm to its zero configuration
// rather than releasing it. That backend's source asserted the pairing was
// "validated at YAML time"; nothing validated it anywhere.
//
// This lives in its own gtest binary on purpose. Controllers are discovered
// through a process-global registry and CM instantiates every registered one,
// so a torque controller registered in the pipeline TU would be checked
// against the position-only stub backend in EVERY node built there — and
// every one of those configures would start failing. One TU, one command-type
// shape.

#include "rt_cm_pipeline_fixtures.hpp"
#include "rt_cm_test_access.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace rtc {
namespace {

using CallbackReturn = RtControllerNode::CallbackReturn;

// Torque-mode controller. Everything else matches the pipeline fixture — the
// command type is the only variable under test.
class TorqueTestController : public PipelineTestController {
 public:
  static constexpr const char* kName = "rtc_cm_torque_test";

  std::string_view Name() const noexcept override { return kName; }

  CommandType GetCommandType() const noexcept override { return CommandType::kTorque; }
};

// Position-only backend: keeps DeviceBackend::AcceptsCommandType's default,
// which is what every hardware backend in this tree relies on.
class PositionOnlyBackend : public PipelineStubBackend {};

// Declares that it honours the mode on the wire, like mujoco_native.
class AnyCommandTypeBackend : public PipelineStubBackend {
 public:
  bool AcceptsCommandType(CommandType /*ct*/) const noexcept override { return true; }
};

RTC_REGISTER_CONTROLLER(rtc_cm_torque_test, "", "rtc_controller_manager",
                        std::make_unique<TorqueTestController>())
RTC_REGISTER_DEVICE_BACKEND(cm_position_only_backend, std::make_unique<PositionOnlyBackend>())
RTC_REGISTER_DEVICE_BACKEND(cm_any_type_backend, std::make_unique<AnyCommandTypeBackend>())

rclcpp_lifecycle::State StateUnconfigured() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                 "unconfigured");
}

class CommandTypeGateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    PipelineTestController::ResetCaptured();
    PipelineStubBackend::ResetCaptured();
  }

  static std::shared_ptr<RtControllerNode> MakeNode(const std::string& backend_type) {
    auto node = std::make_shared<RtControllerNode>("test_command_type_gate_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", 250.0);
    node->declare_parameter("config_variant", std::string("test_fixtures"));
    node->declare_parameter("initial_controller", std::string(TorqueTestController::kName));
    node->declare_parameter("devices.arm.joint_state_names", std::vector<std::string>{"j1", "j2"});
    node->declare_parameter("devices.arm.backend.type", backend_type);
    node->declare_parameter("devices.arm.backend.state_topic", std::string("/arm/state"));
    node->declare_parameter("devices.arm.backend.command_topic", std::string("/arm/cmd"));
    node->declare_parameter("device_timeout_names", std::vector<std::string>{"arm"});
    node->declare_parameter("device_timeout_values", std::vector<double>{60000.0});
    return node;
  }
};

TEST_F(CommandTypeGateTest, RefusesATorqueControllerBoundToAPositionOnlyBackend) {
  // The mismatch is silent at every other layer: the backend accepts the enum
  // and reinterprets the numbers, so nothing downstream can tell newton-metres
  // from radians. Configure is the last place it is still a question.
  auto node = MakeNode("cm_position_only_backend");
  EXPECT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));
}

TEST_F(CommandTypeGateTest, AcceptsATorqueControllerWhenTheBackendDeclaresTheMode) {
  // Same controller, same wiring — only the backend's declaration differs, so
  // the refusal above cannot be an artefact of the torque fixture itself.
  auto node = MakeNode("cm_any_type_backend");
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  EXPECT_EQ(CallbackReturn::SUCCESS,
            node->on_cleanup(rclcpp_lifecycle::State(
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")));
}

}  // namespace
}  // namespace rtc
