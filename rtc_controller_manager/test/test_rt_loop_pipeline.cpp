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

// A channel count comfortably past every fixed array capacity in the tick path.
constexpr int kOverCapacityChannels = 999;

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
    PipelineStubBackend::ResetCaptured();
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
    // Device group entry so the startup readiness gate is armed.
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

// ── Out-of-contract channel counts are bounded before any copy (issue #196) ──
//
// ControllerOutput::num_channels and DeviceStateCache::num_channels are filled
// by out-of-tree implementations. Before the fix both were cast straight to
// size_t and handed to copy_n, so a negative count wrapped to a huge length and
// an over-capacity count walked past the fixed array — an out-of-bounds copy on
// the RT thread.
//
// Two layers now cover that, and the split matters for reading these tests:
//
//   - BoundedCount (this issue's memory-safety hotfix) still clamps every
//     count used as a copy length. It is unconditional and remains the
//     guarantee that no length can ever run off an array.
//   - ValidateControllerOutput (Phase 4) sits in front of it and *rejects* an
//     output whose counts are out of contract, so it never reaches the wire
//     at all.
//
// The two output-side cases below therefore assert the reject contract, not
// the clamp: an out-of-contract count means the tick is held, not truncated
// and forwarded. That is a deliberate contract change — forwarding a
// silently-shortened command vector to an actuator was never safe, it was
// only memory-safe. The state-side case keeps asserting the clamp, because a
// faulty *backend* count is not grounds for rejecting the controller.

TEST_F(RtLoopPipelineTest, OverlargeOutputChannelCountIsRejectedAndHeld) {
  PipelineTestController::output_num_channels.store(kOverCapacityChannels,
                                                    std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();
  ASSERT_TRUE(WaitFor([&] { return backend->WriteCount() > 0; }, 2000ms));

  // The count on the wire comes from the device state, not from the rejected
  // output — and the controller's commands never arrive.
  EXPECT_EQ(2, backend->LastNumChannels());
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos0, backend->LastCommands()[0]);
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos1, backend->LastCommands()[1]);
  EXPECT_GT(ControllerLifecycleTestAccess::GetRejectedOutputCount(*node), 0U);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(RtLoopPipelineTest, NegativeOutputChannelCountIsRejectedAndHeld) {
  PipelineTestController::output_num_channels.store(-1, std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();
  ASSERT_TRUE(WaitFor([&] { return backend->WriteCount() > 0; }, 2000ms));

  // Previously this asserted a zero-length command reached the backend. A
  // zero-length write is a well-formed message that commands nothing, which
  // some backends read as "hold" and others as "no update" — the hold makes
  // the intent explicit instead of leaving it to the backend.
  EXPECT_EQ(2, backend->LastNumChannels());
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos0, backend->LastCommands()[0]);
  EXPECT_GT(ControllerLifecycleTestAccess::GetRejectedOutputCount(*node), 0U);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(RtLoopPipelineTest, OverlargeBackendStateChannelCountIsClampedToCapacity) {
  PipelineStubBackend::state_num_channels.store(kOverCapacityChannels, std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();
  ASSERT_TRUE(WaitFor([&] { return backend->WriteCount() > 0; }, 2000ms));

  // Bounded on the read path, so the state count mirrored into the snapshot
  // (and the DeviceState the controller sees) never exceeds capacity.
  EXPECT_EQ(kMaxDeviceChannels, backend->LastActualNumChannels());
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos0, backend->LastActualPositions()[0]);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── Invalid ControllerOutput → hold + reject counter + E-STOP (issue #196) ───
//
// Nothing an in-tree controller emits reaches the validator's reject branches
// (the pre-audit for this phase found zero call sites that set valid=false or
// disagree with state.num_devices), so every case below is driven by the
// fixture's fault-injection knobs. Without them these tests would be green
// against an empty implementation.
//
// The hold value is what makes the assertions load-bearing: the stub backend
// reports positions kPos0/kPos1 (0.11/0.22) while the controller commands
// kCmd0/kCmd1 (1.5/-2.5). The two never coincide, so "hold was written"
// cannot be satisfied by the un-validated output slipping through.

using OutputFault = PipelineTestController::OutputFault;

// Brings a node up to the first backend write with `fault` active on every
// tick, and returns it with the backend already holding an observation.
class OutputValidationTest : public RtLoopPipelineTest {
 protected:
  static_assert(PipelineStubBackend::kPos0 != PipelineTestController::kCmd0,
                "hold and normal command must differ or the assertions prove nothing");
  static_assert(PipelineStubBackend::kPos1 != PipelineTestController::kCmd1,
                "hold and normal command must differ or the assertions prove nothing");
};

void ExpectHeldAtMeasuredPosition(PipelineStubBackend* backend) {
  // Channel count comes from the device state, not the rejected output.
  EXPECT_EQ(2, backend->LastNumChannels());
  // kPosition hold → servo to the measured position.
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos0, backend->LastCommands()[0]);
  EXPECT_DOUBLE_EQ(PipelineStubBackend::kPos1, backend->LastCommands()[1]);
  // Explicitly: the controller's own command never reached the wire.
  EXPECT_NE(PipelineTestController::kCmd0, backend->LastCommands()[0]);
  EXPECT_NE(PipelineTestController::kCmd1, backend->LastCommands()[1]);
}

class OutputFaultParamTest : public OutputValidationTest,
                             public ::testing::WithParamInterface<OutputFault> {};

TEST_P(OutputFaultParamTest, RejectedOutputIsReplacedByHoldAtTheBackend) {
  PipelineTestController::output_fault.store(GetParam(), std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();
  ASSERT_TRUE(WaitFor([&] { return backend->WriteCount() > 0; }, 2000ms));

  ExpectHeldAtMeasuredPosition(backend);
  EXPECT_GT(ControllerLifecycleTestAccess::GetRejectedOutputCount(*node), 0U);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

INSTANTIATE_TEST_SUITE_P(AllRejectShapes, OutputFaultParamTest,
                         ::testing::Values(OutputFault::kInvalidFlag, OutputFault::kDeviceCount,
                                           OutputFault::kChannelCount,
                                           OutputFault::kNonFiniteCommand),
                         [](const ::testing::TestParamInfo<OutputFault>& info) {
                           switch (info.param) {
                             case OutputFault::kInvalidFlag:
                               return "InvalidFlag";
                             case OutputFault::kDeviceCount:
                               return "DeviceCountMismatch";
                             case OutputFault::kChannelCount:
                               return "ChannelCountPastDeviceReport";
                             case OutputFault::kNonFiniteCommand:
                               return "NonFiniteCommand";
                             case OutputFault::kNone:
                               return "None";
                           }
                           return "Unknown";
                         });

TEST_F(OutputValidationTest, ValidOutputIsForwardedUnchangedAfterAFaultClears) {
  // The hold must not latch: once the controller recovers, its own command
  // reaches the backend again on the very next tick.
  PipelineTestController::output_fault.store(OutputFault::kNonFiniteCommand,
                                             std::memory_order_relaxed);
  PipelineTestController::output_fault_ticks.store(3, std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(
      WaitFor([&] { return backend->LastCommands()[0] == PipelineTestController::kCmd0; }, 2000ms));

  EXPECT_DOUBLE_EQ(PipelineTestController::kCmd1, backend->LastCommands()[1]);
  // Exactly the injected ticks were rejected, and the consecutive counter
  // cleared when the good output landed.
  EXPECT_EQ(3U, ControllerLifecycleTestAccess::GetRejectedOutputCount(*node));
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetConsecutiveRejectedOutputCount(*node));
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, RejectCounterMatchesInjectedFaultTicks) {
  constexpr int kFaultTicks = 7;
  PipelineTestController::output_fault.store(OutputFault::kInvalidFlag, std::memory_order_relaxed);
  PipelineTestController::output_fault_ticks.store(kFaultTicks, std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor(
      [&] {
        return PipelineTestController::injected_fault_ticks.load(std::memory_order_relaxed) ==
                   kFaultTicks &&
               ControllerLifecycleTestAccess::GetConsecutiveRejectedOutputCount(*node) == 0U;
      },
      2000ms));

  // Ground truth (what the controller emitted) against the node's own tally —
  // neither over- nor under-counts.
  EXPECT_EQ(static_cast<uint64_t>(kFaultTicks),
            ControllerLifecycleTestAccess::GetRejectedOutputCount(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── E-STOP escalation threshold (N = max(10, lround(0.1 * control_rate))) ────

TEST_F(OutputValidationTest, EstopThresholdIsDerivedFromControlRate) {
  // A fixed tick count would mean 100 ms at 100 Hz and 2 ms at 5 kHz. The
  // floor takes over at the low end of the supported rate range.
  auto slow = MakeNode(/*control_rate=*/100.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, slow->on_configure(StateUnconfigured()));
  EXPECT_EQ(10U, ControllerLifecycleTestAccess::GetOutputRejectEstopTicks(
                     *slow));  // 0.1 * 100 = 10, at the floor
  EXPECT_EQ(CallbackReturn::SUCCESS, slow->on_cleanup(StateInactive()));

  auto mid = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, mid->on_configure(StateUnconfigured()));
  EXPECT_EQ(25U, ControllerLifecycleTestAccess::GetOutputRejectEstopTicks(*mid));
  EXPECT_EQ(CallbackReturn::SUCCESS, mid->on_cleanup(StateInactive()));

  auto fast = MakeNode(/*control_rate=*/1000.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, fast->on_configure(StateUnconfigured()));
  EXPECT_EQ(100U, ControllerLifecycleTestAccess::GetOutputRejectEstopTicks(*fast));
  EXPECT_EQ(CallbackReturn::SUCCESS, fast->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, OneTickBelowThresholdDoesNotEstop) {
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  const uint64_t threshold = ControllerLifecycleTestAccess::GetOutputRejectEstopTicks(*node);
  ASSERT_EQ(25U, threshold);

  // The fault budget is what pins "exactly N-1": the RT loop free-runs, so the
  // bad-tick count has to be bounded by the producer, not by trying to catch
  // the consumer at the right moment.
  PipelineTestController::output_fault.store(OutputFault::kInvalidFlag, std::memory_order_relaxed);
  PipelineTestController::output_fault_ticks.store(static_cast<int>(threshold) - 1,
                                                   std::memory_order_relaxed);

  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(
      WaitFor([&] { return backend->LastCommands()[0] == PipelineTestController::kCmd0; }, 2000ms));

  EXPECT_EQ(threshold - 1, ControllerLifecycleTestAccess::GetRejectedOutputCount(*node));
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, ThresholdConsecutiveRejectsTriggerEstop) {
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  const uint64_t threshold = ControllerLifecycleTestAccess::GetOutputRejectEstopTicks(*node);

  PipelineTestController::output_fault.store(OutputFault::kInvalidFlag, std::memory_order_relaxed);
  PipelineTestController::output_fault_ticks.store(static_cast<int>(threshold),
                                                   std::memory_order_relaxed);

  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 2000ms));
  EXPECT_EQ(threshold, ControllerLifecycleTestAccess::GetRejectedOutputCount(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, ValidationKeepsHoldingAfterEstop) {
  // Fail-closed must survive into the terminal state: a controller whose
  // E-STOP path also emits bad output still gets held, not forwarded.
  PipelineTestController::output_fault.store(OutputFault::kNonFiniteCommand,
                                             std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 3000ms));

  const uint64_t after_estop = ControllerLifecycleTestAccess::GetRejectedOutputCount(*node);
  ASSERT_TRUE(WaitFor(
      [&] { return ControllerLifecycleTestAccess::GetRejectedOutputCount(*node) > after_estop; },
      2000ms));
  ExpectHeldAtMeasuredPosition(backend);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

// ── E-STOP command substitution (issue #198 Phase 3) ─────────────────────────
//
// PipelineTestController never overrides TriggerEstop / ClearEstop, so the
// base no-op runs and Compute() keeps emitting its normal command under
// E-STOP. That is the fixture the acceptance criterion asks for: a controller
// whose E-STOP hook does nothing must still not reach the actuator.

TEST_F(OutputValidationTest, EstopSubstitutesHoldEvenWhenTheControllerOutputIsValid) {
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  // Baseline: with no fault and no E-STOP, the controller's own command is
  // what reaches the wire — so the substitution below is the only thing that
  // can change it.
  ASSERT_TRUE(
      WaitFor([&] { return backend->LastCommands()[0] == PipelineTestController::kCmd0; }, 2000ms));
  ASSERT_EQ(0U, ControllerLifecycleTestAccess::GetEstopSubstitutedOutputCount(*node));

  ControllerLifecycleTestAccess::CallTriggerEstop(*node, "test_estop");

  ASSERT_TRUE(
      WaitFor([&] { return backend->LastCommands()[0] == PipelineStubBackend::kPos0; }, 2000ms));
  ExpectHeldAtMeasuredPosition(backend);
  EXPECT_GT(ControllerLifecycleTestAccess::GetEstopSubstitutedOutputCount(*node), 0U);
  // The output was never invalid — this is the E-STOP guard acting, not the
  // validator. Without a separate counter the two are indistinguishable at
  // the backend, since both emit the same hold.
  EXPECT_EQ(0U, ControllerLifecycleTestAccess::GetRejectedOutputCount(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, EstopSubstitutionComposesAfterValidationRatherThanReplacingIt) {
  // The Phase 2b block owns "terminal state is still validated" (#196). The
  // E-STOP substitution is layered behind it, so the reject counter must keep
  // climbing while estopped — an implementation that short-circuits the
  // validator once the latch is set would leave it frozen.
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();
  ASSERT_TRUE(WaitFor([&] { return backend->WriteCount() > 0; }, 2000ms));

  ControllerLifecycleTestAccess::CallTriggerEstop(*node, "test_estop");
  ASSERT_TRUE(WaitFor(
      [&] { return ControllerLifecycleTestAccess::GetEstopSubstitutedOutputCount(*node) > 0U; },
      2000ms));

  // Now make the controller's E-STOP-time output invalid too.
  const uint64_t rejects_before = ControllerLifecycleTestAccess::GetRejectedOutputCount(*node);
  PipelineTestController::output_fault.store(OutputFault::kNonFiniteCommand,
                                             std::memory_order_relaxed);
  EXPECT_TRUE(WaitFor(
      [&] { return ControllerLifecycleTestAccess::GetRejectedOutputCount(*node) > rejects_before; },
      2000ms));
  // Both guards agree on the command, which is why the counters have to
  // disagree for either to be testable.
  ExpectHeldAtMeasuredPosition(backend);

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, EstopStatusPublishIsDeferredOffTheRtThread) {
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));

  ASSERT_FALSE(ControllerLifecycleTestAccess::GetEstopStatusPending(*node));
  ControllerLifecycleTestAccess::CallTriggerEstop(*node, "test_estop");
  // TriggerGlobalEstop is reachable from the RT loop, so it must hand the
  // /system/estop_status publish to the aux lane instead of doing it inline.
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetEstopStatusPending(*node));

  ControllerLifecycleTestAccess::CallDrainLog(*node);
  EXPECT_FALSE(ControllerLifecycleTestAccess::GetEstopStatusPending(*node));

  // The clear takes the same lane, and the lifecycle teardown flush is what
  // keeps the final transition from dying with the drain timer.
  ControllerLifecycleTestAccess::CallClearEstop(*node);
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetEstopStatusPending(*node));
  ControllerLifecycleTestAccess::CallFlushEstopStatus(*node);
  EXPECT_FALSE(ControllerLifecycleTestAccess::GetEstopStatusPending(*node));

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, UnbuildableHoldHandsTheDecisionToTheBackend) {
  // The zero-length command CM emits when the hold cannot be built was treated
  // as fail-closed, but every backend early-returns on it — so what reached
  // the hardware was silence, which reads as "carry on, motors still driven"
  // (issue #198 Phase 4). The device now gets an explicit safe-output call
  // instead. At the wire on today's backends this leaves the actuator where
  // the silent path left it; what changes is that the intent is stated rather
  // than inferred, and that a backend which can do better has somewhere to
  // put it.
  PipelineTestController::output_fault.store(OutputFault::kInvalidFlag, std::memory_order_relaxed);
  PipelineStubBackend::state_position_nan.store(true, std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return backend->SafeWriteCount() > 0; }, 2000ms));
  // And it is the safe path, not the normal one: a zero-length WriteCommand
  // would have been indistinguishable at the backend, which is exactly why
  // the counter is separate.
  EXPECT_EQ(0, backend->LastNumChannels());

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, HealthyTicksNeverReachTheSafeOutputPath) {
  // Guards the inverse: the safe path must stay reserved for the unbuildable
  // case, or the test above would pass on a backend that got a safe write
  // every tick.
  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return backend->WriteCount() > 5; }, 2000ms));
  EXPECT_EQ(0, backend->SafeWriteCount());

  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_deactivate(StateActive()));
  EXPECT_EQ(CallbackReturn::SUCCESS, node->on_cleanup(StateInactive()));
}

TEST_F(OutputValidationTest, NonFiniteStateMakesTheHoldUnbuildableAndEstopsImmediately) {
  // The hold is only as good as the state it is rebuilt from. A backend
  // reporting NaN would otherwise make the replacement command carry the very
  // value the validator just rejected — and 0.0 is no substitute, since for a
  // position command it means "go to the origin". The device is dropped to a
  // zero-length command and the escalation fires on the first tick instead of
  // riding out the window, because the window only makes sense while the hold
  // is actually holding.
  PipelineTestController::output_fault.store(OutputFault::kInvalidFlag, std::memory_order_relaxed);
  PipelineStubBackend::state_position_nan.store(true, std::memory_order_relaxed);

  auto node = MakeNode(/*control_rate=*/250.0);
  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_configure(StateUnconfigured()));
  const uint64_t threshold = ControllerLifecycleTestAccess::GetOutputRejectEstopTicks(*node);
  auto* backend = Backend(*node);
  ASSERT_NE(nullptr, backend);

  ASSERT_EQ(CallbackReturn::SUCCESS, node->on_activate(StateInactive()));
  backend->FireStateReady();

  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 2000ms));
  // Escalated well before the consecutive-reject window would have expired.
  EXPECT_LT(ControllerLifecycleTestAccess::GetRejectedOutputCount(*node), threshold);
  EXPECT_EQ("unholdable_controller_output_valid_flag_false",
            ControllerLifecycleTestAccess::GetEstopReason(*node));
  // Nothing commanded: a zero-length write, not a NaN and not a zero command.
  EXPECT_EQ(0, backend->LastNumChannels());

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
  // Never fire the backend's state-ready hook: the device never reports, so
  // ticks idle in the startup gate until init_timeout_ticks_ elapse.

  ASSERT_TRUE(WaitFor([&] { return ControllerLifecycleTestAccess::IsEstopped(*node); }, 5000ms));
  // The reason names the device that held the gate shut. It was the bare
  // "init_timeout" while readiness was a single global flag — there was no
  // device to name. Per-device readiness (issue #198 §1) makes the offender
  // identifiable, and an E-STOP that says which device is silent is the point
  // of tracking them separately; a fleet operator reading only "init_timeout"
  // has to guess. Deliberate contract change, not a weakened assertion.
  EXPECT_EQ("arm_init_timeout", ControllerLifecycleTestAccess::GetEstopReason(*node));
  EXPECT_TRUE(WaitFor([] { return !rclcpp::ok(); }, 2000ms));
}

}  // namespace
}  // namespace rtc
