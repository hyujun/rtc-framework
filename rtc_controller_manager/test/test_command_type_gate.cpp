// Tier 2 — controller command type vs backend capability (issue #198), plus
// the torque E-STOP sag advisory carried in the same pass (issue #339).
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
#include <rcutils/logging.h>

#include <array>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
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

// Position-mode sibling, registered alongside the torque one so every node
// built here carries both. The sag advisory has to pick one of them out; a
// suite where the only controller is torque-mode cannot tell "warns about
// torque" from "warns about whatever it is handed".
class PositionTestController : public PipelineTestController {
 public:
  static constexpr const char* kName = "rtc_cm_position_test";

  std::string_view Name() const noexcept override { return kName; }
};

// Mixed-mode sibling. kPdFeedforward's commands[] carries a position target
// and the feedforward overlay is an additive channel a position-only backend
// simply does not transmit — the graceful fallback types.hpp documents and the
// shipped WBC→hand lane rides on. The tick-path guard (#342) passes it for
// that reason, so the configure gate has to as well: the wire behaviour is
// identical whether the mode arrives as a controller-global type or as a
// per-device override, and a policy that depends on which axis it came through
// is not a policy.
class PdFeedforwardTestController : public PipelineTestController {
 public:
  static constexpr const char* kName = "rtc_cm_pd_feedforward_test";

  std::string_view Name() const noexcept override { return kName; }

  CommandType GetCommandType() const noexcept override { return CommandType::kPdFeedforward; }
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
RTC_REGISTER_CONTROLLER(rtc_cm_position_test, "", "rtc_controller_manager",
                        std::make_unique<PositionTestController>())
RTC_REGISTER_CONTROLLER(rtc_cm_pd_feedforward_test, "", "rtc_controller_manager",
                        std::make_unique<PdFeedforwardTestController>())
RTC_REGISTER_DEVICE_BACKEND(cm_position_only_backend, std::make_unique<PositionOnlyBackend>())
RTC_REGISTER_DEVICE_BACKEND(cm_any_type_backend, std::make_unique<AnyCommandTypeBackend>())

// ── Log capture ─────────────────────────────────────────────────────────────
//
// The advisory's only observable is the log line. It deliberately does not
// refuse, does not change the CallbackReturn, and adds no member to the node —
// inventing a counter on RtControllerNode purely so a test could read it would
// put test-only state in the RT node, so the assertion goes to the sink the
// message actually reaches.
//
// No other test in this tree captures logs. The handler is process-global, and
// this binary is already a single-TU island (see the file header) precisely so
// that command-type shapes cannot leak into unrelated fixtures — which makes it
// the one place such a handler can be installed without that reach.
class LogSink {
 public:
  static void Install() {
    {
      const std::lock_guard<std::mutex> lock(Mutex());
      Lines().clear();
    }
    Previous() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&LogSink::Handler);
  }

  static void Restore() {
    if (Previous() != nullptr) {
      rcutils_logging_set_output_handler(Previous());
      Previous() = nullptr;
    }
  }

  // Captured lines at `severity` containing every needle. Matching on
  // substrings rather than the whole message keeps the assertions from pinning
  // wording that carries no contract — but each needle below is a claim the
  // operator has to receive, not decoration.
  static std::vector<std::string> Matching(int severity, const std::vector<std::string>& needles) {
    const std::lock_guard<std::mutex> lock(Mutex());
    std::vector<std::string> hits;
    for (const auto& [sev, text] : Lines()) {
      if (sev != severity) {
        continue;
      }
      bool all = true;
      for (const auto& needle : needles) {
        if (text.find(needle) == std::string::npos) {
          all = false;
          break;
        }
      }
      if (all) {
        hits.push_back(text);
      }
    }
    return hits;
  }

 private:
  // Longer than any message this suite asserts on; vsnprintf truncates and
  // NUL-terminates regardless, so an overlong line degrades to a shorter
  // capture rather than corrupting the sink.
  static constexpr std::size_t kLineBufferSize = 1024;

  static void Handler(const rcutils_log_location_t* /*location*/, int severity,
                      const char* /*name*/, rcutils_time_point_value_t /*timestamp*/,
                      const char* format, va_list* args) {
    std::array<char, kLineBufferSize> buf{};
    va_list copy;
    va_copy(copy, *args);
    (void)std::vsnprintf(buf.data(), buf.size(), format, copy);
    va_end(copy);
    const std::lock_guard<std::mutex> lock(Mutex());
    Lines().emplace_back(severity, std::string(buf.data()));
  }

  // Function-local statics rather than data members: the handler is a plain C
  // function pointer with no user-data slot, so the sink has to be reachable
  // without an instance.
  static std::mutex& Mutex() {
    static std::mutex m;
    return m;
  }

  static std::vector<std::pair<int, std::string>>& Lines() {
    static std::vector<std::pair<int, std::string>> lines;
    return lines;
  }

  static rcutils_logging_output_handler_t& Previous() {
    static rcutils_logging_output_handler_t handler{nullptr};
    return handler;
  }
};

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
    LogSink::Install();
  }

  void TearDown() override { LogSink::Restore(); }

  // The control rate MakeNode configures. The sag advisory reports the
  // exposure window it derives from this, so the expected value is computed
  // here from the same inputs rather than pasted as a literal.
  static constexpr double kControlRateHz = 250.0;

  static std::shared_ptr<RtControllerNode> MakeNode(const std::string& backend_type) {
    auto node = std::make_shared<RtControllerNode>("test_command_type_gate_node");
    node->declare_parameter("enable_logging", false);
    node->declare_parameter("enable_timing_log", false);
    node->declare_parameter("enable_device_log", false);
    node->declare_parameter("control_rate", kControlRateHz);
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

// ── Torque E-STOP sag advisory (issue #339) ─────────────────────────────────

TEST_F(CommandTypeGateTest, WarnsThatAnAcceptedTorquePairingHasNoSafeHold) {
  // Accepting the mode answers the transport question only. CM's E-STOP
  // substitution for torque is 0 N·m — it carries no dynamic model — so the
  // arm sags instead of stopping, and nothing else in the bring-up says so.
  auto node = MakeNode("cm_any_type_backend");
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  const auto warnings = LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN,
                                          {TorqueTestController::kName, "torque", "0 N.m"});
  ASSERT_EQ(1U, warnings.size()) << "expected exactly one sag advisory for the torque controller";

  // The exposure window is the actionable part: an advisory that says "this
  // sags" without saying for how long leaves the operator no way to judge it.
  // Derived from the node's own rate-dependent value rather than a literal, so
  // the assertion follows a rate change instead of pinning 250 Hz.
  const auto ticks = ControllerLifecycleTestAccess::GetOutputRejectEstopTicks(*node);
  ASSERT_GT(ticks, 0U);
  const auto expected_ms =
      std::to_string(static_cast<long>(1000.0 * static_cast<double>(ticks) / kControlRateHz));
  EXPECT_NE(std::string::npos, warnings[0].find(expected_ms + " ms"))
      << "advisory did not report the measured exposure window: " << warnings[0];

  // And that the window cannot be tuned away — kOutputRejectEstopSeconds is a
  // compile-time constant. An earlier comment in BuildHoldOutput told torque
  // configurations to shorten it; there has never been a knob to shorten.
  EXPECT_NE(std::string::npos, warnings[0].find("cannot be shortened"))
      << "advisory implied the window is tunable: " << warnings[0];

  EXPECT_EQ(CallbackReturn::SUCCESS,
            node->on_cleanup(rclcpp_lifecycle::State(
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")));
}

TEST_F(CommandTypeGateTest, DoesNotWarnAboutTheSameNodesPositionModeController) {
  // The position controller is configured in the very same pass, on the very
  // same backend, and must not draw the advisory: kPosition holds to the
  // measured position, which is a true stop. Without this the suite could not
  // distinguish "warns about torque" from "warns about every pairing".
  auto node = MakeNode("cm_any_type_backend");
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  // Guard against a vacuous pass on two axes: the position controller must
  // exist in this node at all, and the pass must have reached the point where
  // an advisory is possible — which the sibling torque advisory demonstrates.
  // Three since the pd_feedforward fixture joined this TU; the number is the
  // registry's contents, not a claim about this test.
  ASSERT_EQ(3U, ControllerLifecycleTestAccess::GetControllerCount(*node));
  ASSERT_EQ(1U, LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, {"0 N.m"}).size());
  EXPECT_TRUE(LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, {PositionTestController::kName}).empty())
      << "position-mode controller drew an E-STOP sag advisory";

  EXPECT_EQ(CallbackReturn::SUCCESS,
            node->on_cleanup(rclcpp_lifecycle::State(
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive")));
}

TEST_F(CommandTypeGateTest, DoesNotAdviseOnAPairingItAlreadyRefused) {
  // Position-only backend: the torque pairing is refused outright, so the
  // configure never reaches a state where the sag window means anything.
  // Advising there would read as "this is allowed but risky" about a
  // combination that is not allowed at all.
  auto node = MakeNode("cm_position_only_backend");
  ASSERT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(1U, LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR,
                                  {TorqueTestController::kName, "cannot honour"})
                    .size());
  EXPECT_TRUE(LogSink::Matching(RCUTILS_LOG_SEVERITY_WARN, {"0 N.m"}).empty())
      << "refused pairing also emitted the sag advisory";
}

// ── One predicate for both gates (issue #342 review) ────────────────────────

TEST_F(CommandTypeGateTest, DoesNotRefuseAPdFeedforwardControllerOnAPositionOnlyBackend) {
  // The configure gate used to ask the backend directly while the tick gate
  // asked CommandTypeIsHonoured, so this pairing was REFUSED when the mode came
  // from GetCommandType() and PERMITTED when the identical mode came from a
  // per-device override on the same slot. Both now read the cached mask through
  // the same predicate.
  //
  // The node still fails to configure — the torque fixture in this TU is
  // refused, which is the point of the sibling test above. What is asserted is
  // WHICH pairing drew the refusal.
  auto node = MakeNode("cm_position_only_backend");
  ASSERT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  // Vacuity guard AND the regression itself: two refusals here would mean the
  // pd_feedforward pairing was judged by a second, stricter rule.
  ASSERT_EQ(1U, LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, {"cannot honour"}).size());
  EXPECT_TRUE(
      LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR, {PdFeedforwardTestController::kName}).empty())
      << "the documented position/pd_feedforward fallback was refused at configure";
}

TEST_F(CommandTypeGateTest, StillRefusesTorqueWhenTheGatesShareAPredicate) {
  // The tolerance is kPdFeedforward-shaped, not a general loosening: kTorque is
  // the one mode whose commands[] carries a different physical quantity, which
  // is the substitution #198 named as the hazard. Pinned separately so a fix
  // that widened CommandTypeIsHonoured to "anything the mask does not name"
  // could not pass the test above.
  auto node = MakeNode("cm_position_only_backend");
  ASSERT_EQ(CallbackReturn::FAILURE, node->on_configure(StateUnconfigured()));

  EXPECT_EQ(1U, LogSink::Matching(RCUTILS_LOG_SEVERITY_ERROR,
                                  {TorqueTestController::kName, "torque", "cannot honour"})
                    .size());
}

}  // namespace
}  // namespace rtc
