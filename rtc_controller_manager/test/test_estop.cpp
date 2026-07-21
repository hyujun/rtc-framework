// Tier 1 — E-STOP path + device-timeout watchdog tests for RtControllerNode.
//
// Covers rt_controller_node_estop.cpp (TriggerGlobalEstop / ClearGlobalEstop:
// CAS idempotency, controller propagation, reason capture) and the watchdog
// helpers in rt_controller_node_rt_loop.cpp (CheckTimeouts /
// AllTimeoutDevicesReceived). These are safety-critical paths (escalation
// trigger E-8) that previously had 0% coverage.

#include "rt_cm_test_access.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rtc {
namespace {

using namespace std::chrono_literals;

class EstopTest : public ::testing::Test {
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
    node_ = std::make_shared<RtControllerNode>("test_estop_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<MockController>("ctrl_a");
    auto b = std::make_unique<MockController>("ctrl_b");
    ctrl_a_ = a.get();
    ctrl_b_ = b.get();
    ctrls.push_back(std::move(a));
    ctrls.push_back(std::move(b));
    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls),
                                                     {"type_a", "type_b"});
    ControllerLifecycleTestAccess::EnsureEstopPublisher(*node_);
  }

  std::shared_ptr<RtControllerNode> node_;
  MockController* ctrl_a_{nullptr};
  MockController* ctrl_b_{nullptr};
};

// ── TriggerGlobalEstop ───────────────────────────────────────────────────────

TEST_F(EstopTest, TriggerSetsFlagAndPropagatesToAllControllers) {
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));

  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "unit_test");

  EXPECT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  // Each controller is told to E-STOP and to assert the hand E-STOP exactly once.
  EXPECT_EQ(1, ctrl_a_->TriggerEstopCount());
  EXPECT_EQ(1, ctrl_b_->TriggerEstopCount());
  EXPECT_EQ(1, ctrl_a_->SetHandEstopCount());
  EXPECT_EQ(1, ctrl_b_->SetHandEstopCount());
  EXPECT_TRUE(ctrl_a_->HandEstop());
  EXPECT_TRUE(ctrl_b_->HandEstop());
  // Deferred log flag raised for the non-RT log thread.
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetEstopLogPending(*node_));
}

TEST_F(EstopTest, TriggerCapturesReason) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "arm_timeout");
  EXPECT_EQ("arm_timeout", ControllerLifecycleTestAccess::GetEstopReason(*node_));
}

TEST_F(EstopTest, TriggerIsIdempotent) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "first");
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "second");
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "third");

  // Only the first trigger propagates (CAS guard) — counts stay at 1 and the
  // reason reflects the first call, not later ones.
  EXPECT_EQ(1, ctrl_a_->TriggerEstopCount());
  EXPECT_EQ(1, ctrl_b_->TriggerEstopCount());
  EXPECT_EQ("first", ControllerLifecycleTestAccess::GetEstopReason(*node_));
}

TEST_F(EstopTest, TriggerTruncatesOverlongReason) {
  // estop_reason_ is a fixed 128-byte buffer — an over-length reason must be
  // truncated (and NUL-terminated) without overflowing.
  const std::string long_reason(300, 'x');
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, long_reason);
  const std::string captured = ControllerLifecycleTestAccess::GetEstopReason(*node_);
  EXPECT_LT(captured.size(), long_reason.size());
  EXPECT_LE(captured.size(), 127u);  // 128-byte buffer incl. NUL
}

// ── ClearGlobalEstop ─────────────────────────────────────────────────────────

TEST_F(EstopTest, ClearAfterTriggerReenablesControllers) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "unit_test");
  ControllerLifecycleTestAccess::ClearEstopLogPending(*node_);

  ControllerLifecycleTestAccess::CallClearEstop(*node_);

  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  EXPECT_EQ(1, ctrl_a_->ClearEstopCount());
  EXPECT_EQ(1, ctrl_b_->ClearEstopCount());
  EXPECT_FALSE(ctrl_a_->HandEstop());
  EXPECT_FALSE(ctrl_b_->HandEstop());
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetEstopLogPending(*node_));
}

TEST_F(EstopTest, ClearWithoutTriggerIsNoOp) {
  // Not estopped → ClearGlobalEstop returns early, touches no controller.
  ControllerLifecycleTestAccess::CallClearEstop(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  EXPECT_EQ(0, ctrl_a_->ClearEstopCount());
  EXPECT_EQ(0, ctrl_b_->ClearEstopCount());
}

// ── CheckTimeouts / AllTimeoutDevicesReceived (watchdog) ─────────────────────

TEST_F(EstopTest, CheckTimeoutsEmptyIsNoOp) {
  // No device timeouts registered → early return, no E-STOP.
  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
}

TEST_F(EstopTest, CheckTimeoutsExpiredTriggersEstop) {
  // received=true and last_update far in the past → exceeds the 10 ms budget.
  const auto stale = std::chrono::steady_clock::now() - 1s;
  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, "arm", 10ms, /*received=*/true, stale);

  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);

  EXPECT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  EXPECT_EQ("arm_timeout", ControllerLifecycleTestAccess::GetEstopReason(*node_));
}

TEST_F(EstopTest, CheckTimeoutsTruncatesOverlongGroupNameWithoutAllocating) {
  // The reason used to be `group_name + "_timeout"` — the only heap allocation
  // left on the RT path (issue #198 §4). It is now formatted into a fixed
  // buffer, so an arbitrarily long group name must truncate rather than
  // allocate or overflow. The prefix still identifies the device.
  const std::string long_name(200, 'g');
  const auto stale = std::chrono::steady_clock::now() - 1s;
  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, long_name, 10ms, /*received=*/true,
                                                  stale);

  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);

  ASSERT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  const std::string reason = ControllerLifecycleTestAccess::GetEstopReason(*node_);
  EXPECT_LT(reason.size(), long_name.size());
  EXPECT_EQ(std::string::npos, reason.find_first_not_of('g'));
}

TEST_F(EstopTest, CheckTimeoutsIgnoresDeviceThatNeverReported) {
  // received=false → device is skipped even though last_update is stale.
  const auto stale = std::chrono::steady_clock::now() - 1s;
  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, "arm", 10ms, /*received=*/false, stale);

  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
}

TEST_F(EstopTest, CheckTimeoutsFreshDeviceDoesNotTrip) {
  // received=true but last_update is recent → still within budget.
  const auto fresh = std::chrono::steady_clock::now();
  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, "arm", 500ms, /*received=*/true, fresh);

  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
}

TEST_F(EstopTest, AllTimeoutDevicesReceivedReflectsReceivedFlags) {
  const auto now = std::chrono::steady_clock::now();
  EXPECT_TRUE(
      ControllerLifecycleTestAccess::CallAllTimeoutDevicesReceived(*node_));  // empty → true

  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, "arm", 100ms, /*received=*/true, now);
  EXPECT_TRUE(ControllerLifecycleTestAccess::CallAllTimeoutDevicesReceived(*node_));

  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, "hand", 100ms, /*received=*/false, now);
  EXPECT_FALSE(ControllerLifecycleTestAccess::CallAllTimeoutDevicesReceived(*node_));
}

// ── DrainLog (deferred E-STOP log + timing summary) ──────────────────────────

TEST_F(EstopTest, DrainLogEmitsDeferredEstopMessages) {
  // Trigger sets estop_log_pending_; DrainLog consumes it via the ERROR branch.
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "drain_test");
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetEstopLogPending(*node_));
  ControllerLifecycleTestAccess::CallDrainLog(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::GetEstopLogPending(*node_));

  // Clear sets it again; DrainLog consumes it via the INFO (cleared) branch.
  ControllerLifecycleTestAccess::CallClearEstop(*node_);
  EXPECT_TRUE(ControllerLifecycleTestAccess::GetEstopLogPending(*node_));
  ControllerLifecycleTestAccess::CallDrainLog(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::GetEstopLogPending(*node_));
}

TEST_F(EstopTest, DrainLogPrintsTimingSummaryWhenSignalled) {
  // print_timing_summary_ gates the summary block; controllers_ are injected so
  // the active-controller name lookup is in bounds.
  ControllerLifecycleTestAccess::SetActiveIdx(*node_, 0);
  ControllerLifecycleTestAccess::SetPrintTimingSummary(*node_, true);
  ControllerLifecycleTestAccess::CallDrainLog(*node_);  // emits summary, resets profiler
  // A second drain with the flag cleared takes the no-summary path.
  ControllerLifecycleTestAccess::CallDrainLog(*node_);
  SUCCEED();
}

}  // namespace
}  // namespace rtc
