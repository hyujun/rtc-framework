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

// ── Latch/propagation ordering (issue #299) ──────────────────────────────────
//
// The RT loop decides substitution from the global latch alone
// (rt_controller_node_rt_loop.cpp Phase 2c: `const bool estopped =
// IsGlobalEstopped();`), so the order in which the latch and the per-controller
// state change is itself a safety property, not an implementation detail:
//
//   trigger — latch first, then propagate. A tick in between substitutes
//             hold_output_ for a controller that has not yet estopped. Safe.
//   clear   — propagate first, then latch. A tick in between substitutes
//             hold_output_ for a controller that is already cleared. Safe.
//
// Lowering the latch first inverts the clear window into a fail-open one: the
// tick forwards the output of a controller still holding its E-STOP state.
// Neither ordering is visible from the final state, which is all the two tests
// above look at — they pass under both. Observing it requires reading the latch
// from *inside* the propagation callback, which is what this controller does.
class LatchObservingController : public MockController {
 public:
  LatchObservingController(std::string name, const RtControllerNode& node)
      : MockController(std::move(name)), node_(node) {}

  void TriggerEstop() noexcept override {
    latch_during_trigger_ = ControllerLifecycleTestAccess::IsEstopped(node_);
    MockController::TriggerEstop();
  }

  void ClearEstop() noexcept override {
    latch_during_clear_ = ControllerLifecycleTestAccess::IsEstopped(node_);
    MockController::ClearEstop();
  }

  bool LatchDuringTrigger() const { return latch_during_trigger_; }

  bool LatchDuringClear() const { return latch_during_clear_; }

 private:
  const RtControllerNode& node_;
  bool latch_during_trigger_{false};
  bool latch_during_clear_{false};
};

class EstopOrderTest : public EstopTest {
 protected:
  // Deliberately does not chain to EstopTest::SetUp(): the base fixture's plain
  // MockControllers cannot read the latch, and injecting over them would leave
  // the base's ctrl_a_ / ctrl_b_ dangling. Those stay null here.
  void SetUp() override {
    node_ = std::make_shared<RtControllerNode>("test_estop_order_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<LatchObservingController>("ctrl_a", *node_);
    auto b = std::make_unique<LatchObservingController>("ctrl_b", *node_);
    obs_a_ = a.get();
    obs_b_ = b.get();
    ctrls.push_back(std::move(a));
    ctrls.push_back(std::move(b));
    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls),
                                                     {"type_a", "type_b"});
    ControllerLifecycleTestAccess::EnsureEstopPublisher(*node_);
  }

  LatchObservingController* obs_a_{nullptr};
  LatchObservingController* obs_b_{nullptr};
};

TEST_F(EstopOrderTest, ClearPropagatesToControllersBeforeLoweringTheLatch) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "unit_test");
  ASSERT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_));

  ControllerLifecycleTestAccess::CallClearEstop(*node_);

  // Guard the observation itself: a latch value recorded by a callback that
  // never ran would read as the member's initial value, not as evidence.
  ASSERT_EQ(1, obs_a_->ClearEstopCount());
  ASSERT_EQ(1, obs_b_->ClearEstopCount());

  EXPECT_TRUE(obs_a_->LatchDuringClear());
  EXPECT_TRUE(obs_b_->LatchDuringClear());
  // …and down only once every controller has been told.
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
}

TEST_F(EstopOrderTest, TriggerRaisesTheLatchBeforePropagating) {
  // The mirror of the above, and the reason "propagate first" is the correct
  // repair direction for clear rather than the reverse. Already holds today —
  // this pins it so a later symmetry edit cannot silently move the trigger side
  // to match a still-broken clear side.
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "unit_test");

  ASSERT_EQ(1, obs_a_->TriggerEstopCount());
  ASSERT_EQ(1, obs_b_->TriggerEstopCount());

  EXPECT_TRUE(obs_a_->LatchDuringTrigger());
  EXPECT_TRUE(obs_b_->LatchDuringTrigger());
}

// ── Swallowed-trigger refusal (issue #288) ───────────────────────────────────
//
// An external entry point onto ClearGlobalEstop makes it concurrent with the RT
// thread, which is where every trigger comes from. Two of those call sites are
// unguarded (ControlLoopThread::OnOverrun, OnLoopAborted), so one can land while
// a clear holds the latch up (#299 ordering): its CAS fails, it returns without
// propagating or recording a reason, and the clear then lowers the latch — the
// safety event leaves no trace at all. Counting requests at TriggerGlobalEstop's
// entry rather than at its CAS is what makes that visible.

// Calls TriggerGlobalEstop from INSIDE its own ClearEstop(), which is exactly
// where an unguarded RT-side trigger would land. One-shot, so the re-assert
// loop that follows the refusal cannot re-enter it.
class TriggeringOnClearController : public MockController {
 public:
  TriggeringOnClearController(std::string name, RtControllerNode& node)
      : MockController(std::move(name)), node_(node) {}

  void ClearEstop() noexcept override {
    MockController::ClearEstop();
    if (!fired_) {
      fired_ = true;
      ControllerLifecycleTestAccess::CallTriggerEstop(node_, "consecutive_overrun");
    }
  }

 private:
  RtControllerNode& node_;
  bool fired_{false};
};

TEST_F(EstopTest, TriggerCountsTheRequestsItsCasTurnsAway) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "first");
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "second");
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "third");

  // Propagation still happens once (the CAS guard), but all three ASKED — and
  // the two the CAS turned away are the ones a concurrent clear must notice.
  EXPECT_EQ(1, ctrl_a_->TriggerEstopCount());
  EXPECT_EQ(3U, ControllerLifecycleTestAccess::GetEstopTriggerRequestCount(*node_));
}

TEST_F(EstopTest, ClearReportsNotLatchedRatherThanSilentlyDoingNothing) {
  EXPECT_EQ(ControllerLifecycleTestAccess::EstopClearOutcome::kNotLatched,
            ControllerLifecycleTestAccess::CallClearEstop(*node_));
}

TEST_F(EstopTest, ClearReportsClearedOnTheNormalPath) {
  ControllerLifecycleTestAccess::CallTriggerEstop(*node_, "unit_test");
  EXPECT_EQ(ControllerLifecycleTestAccess::EstopClearOutcome::kCleared,
            ControllerLifecycleTestAccess::CallClearEstop(*node_));
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
}

TEST_F(EstopTest, ClearRefusesAndReassertsWhenATriggerLandsMidPropagation) {
  auto node = std::make_shared<RtControllerNode>("test_estop_retrigger_node");
  std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
  auto a = std::make_unique<TriggeringOnClearController>("ctrl_a", *node);
  auto b = std::make_unique<MockController>("ctrl_b");
  auto* obs_a = a.get();
  auto* obs_b = b.get();
  ctrls.push_back(std::move(a));
  ctrls.push_back(std::move(b));
  ControllerLifecycleTestAccess::InjectControllers(*node, std::move(ctrls), {"type_a", "type_b"});
  ControllerLifecycleTestAccess::EnsureEstopPublisher(*node);

  ControllerLifecycleTestAccess::CallTriggerEstop(*node, "device_timeout");
  ControllerLifecycleTestAccess::ClearEstopLogPending(*node);

  EXPECT_EQ(ControllerLifecycleTestAccess::EstopClearOutcome::kRetriggered,
            ControllerLifecycleTestAccess::CallClearEstop(*node));

  // The latch stays UP, so the RT loop keeps substituting hold_output_.
  EXPECT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node));
  // Propagation ran once and was then undone, so the controllers agree with the
  // latch again rather than resting in "latch up, controllers cleared" — safe
  // as a transient inside the loop, not as a state nothing will leave.
  EXPECT_EQ(1, obs_a->ClearEstopCount());
  EXPECT_EQ(1, obs_b->ClearEstopCount());
  EXPECT_EQ(2, obs_a->TriggerEstopCount());  // the original + the re-assert
  EXPECT_EQ(2, obs_b->TriggerEstopCount());
  EXPECT_TRUE(obs_a->HandEstop());
  EXPECT_TRUE(obs_b->HandEstop());
  // No deferred log — announcing a clear that was abandoned is worse than
  // saying nothing.
  EXPECT_FALSE(ControllerLifecycleTestAccess::GetEstopLogPending(*node));
}

// ── Device liveness: startup gate + watchdog (issue #198 §1/§2) ──────────────
//
// Liveness now comes from the backend's own LastStateStamp(), so these tests
// stage a stub backend per slot instead of poking a CM-side timestamp. That is
// the point of the change: there is no second copy of the stamp to poke.

// Minimal backend whose only interesting behaviour is the liveness stamp.
class StampOnlyBackend : public DeviceBackend {
 public:
  void Configure(rclcpp_lifecycle::LifecycleNode* /*node*/, const DeviceBackendConfig& /*config*/,
                 rclcpp::CallbackGroup::SharedPtr /*cb*/) override {}

  void Activate() override {}

  void Deactivate() override {}

  bool ReadState(DeviceStateCache& /*cache*/) noexcept override { return false; }

  void WriteCommand(const PublishSnapshot::GroupCommandSlot& /*slot*/,
                    CommandType /*type*/) noexcept override {}

  void WriteSafeCommand() noexcept override {}

  std::chrono::steady_clock::time_point LastStateStamp() const noexcept override { return stamp_; }

  void SetStamp(std::chrono::steady_clock::time_point stamp) { stamp_ = stamp; }

 private:
  std::chrono::steady_clock::time_point stamp_{};  // never reported
};

// Registers group `name` at `slot` with a stub backend already stamped at
// `stamp` (default-constructed = never reported).
StampOnlyBackend* StageDevice(RtControllerNode& node, const std::string& name, int slot,
                              std::chrono::milliseconds timeout,
                              std::chrono::steady_clock::time_point stamp) {
  auto backend = std::make_unique<StampOnlyBackend>();
  auto* raw = backend.get();
  raw->SetStamp(stamp);
  ControllerLifecycleTestAccess::InjectBackend(node, static_cast<std::size_t>(slot),
                                               std::move(backend));
  ControllerLifecycleTestAccess::AddDeviceTimeout(node, name, timeout, slot);
  return raw;
}

TEST_F(EstopTest, CheckTimeoutsEmptyIsNoOp) {
  // No device groups registered → early return, no E-STOP.
  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
}

TEST_F(EstopTest, CheckTimeoutsExpiredTriggersEstop) {
  // Reported once, a second ago → past the 10 ms budget.
  StageDevice(*node_, "arm", 0, 10ms, std::chrono::steady_clock::now() - 1s);

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
  StageDevice(*node_, long_name, 0, 10ms, std::chrono::steady_clock::now() - 1s);

  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);

  ASSERT_TRUE(ControllerLifecycleTestAccess::IsEstopped(*node_));
  const std::string reason = ControllerLifecycleTestAccess::GetEstopReason(*node_);
  EXPECT_LT(reason.size(), long_name.size());
  EXPECT_EQ(std::string::npos, reason.find_first_not_of('g'));
}

TEST_F(EstopTest, CheckTimeoutsDefersNeverReportedDeviceToTheStartupGate) {
  // A device that has never reported is NOT a watchdog trip: pre-init it is
  // expected, and post-init it cannot happen, because FirstUnreadyDevice()
  // refuses to open the gate until every device has reported. What changed in
  // #198 is that skipping it here is no longer the end of the story — the
  // assertion below pins the other half.
  StageDevice(*node_, "arm", 0, 10ms, {});

  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));

  // ...and the device is not ready, so the control law never starts.
  EXPECT_EQ(0, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(
                   *node_, std::chrono::steady_clock::now()));
}

TEST_F(EstopTest, CheckTimeoutsFreshDeviceDoesNotTrip) {
  StageDevice(*node_, "arm", 0, 500ms, std::chrono::steady_clock::now());

  ControllerLifecycleTestAccess::CallCheckTimeouts(*node_);
  EXPECT_FALSE(ControllerLifecycleTestAccess::IsEstopped(*node_));
}

TEST_F(EstopTest, ReadinessRequiresEveryDeviceNotJustOne) {
  // The defect this closes: one device's first packet used to release the
  // startup gate for all of them (a single global state_received_ flag), so a
  // device that was silent from the start never blocked the control law and
  // was skipped by the watchdog forever.
  const auto now = std::chrono::steady_clock::now();
  EXPECT_EQ(-1, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(*node_, now));  // empty

  StageDevice(*node_, "arm", 0, 100ms, now);
  EXPECT_EQ(-1, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(*node_, now));

  StageDevice(*node_, "hand", 1, 100ms, {});  // never reported
  EXPECT_EQ(1, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(*node_, now));
}

TEST_F(EstopTest, ReadinessAlsoRefusesADeviceThatWentQuiet) {
  // Readiness and staleness are one predicate, so a re-activate after the
  // device stopped reporting is refused for the same reason a running loop
  // E-STOPs — a stale stamp is not "ready", it is just old.
  const auto now = std::chrono::steady_clock::now();
  StageDevice(*node_, "arm", 0, 100ms, now - 1s);

  EXPECT_FALSE(ControllerLifecycleTestAccess::CallDeviceLive(*node_, 0, now));
  EXPECT_EQ(0, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(*node_, now));
}

TEST_F(EstopTest, ReadinessRefusesAGroupWhoseBackendNeverCameUp) {
  // A configured group with no backend at its slot is not live. It used to be
  // absent from device_timeouts_ entirely — invisible to both the gate and the
  // watchdog (#198 §1, the third fail-open).
  ControllerLifecycleTestAccess::AddDeviceTimeout(*node_, "arm", 100ms, /*slot=*/0);

  EXPECT_EQ(0, ControllerLifecycleTestAccess::CallFirstUnreadyDevice(
                   *node_, std::chrono::steady_clock::now()));
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

// ── Slot-mapping build guards (issue #198 comment 16) ────────────────────────
//
// BuildControllerSlotMappings translates each controller's declared device
// groups into the fixed slot array the RT loop indexes. Both refusals below
// defend inputs the controller-interface parser makes impossible today — it
// rejects a group that declares neither `subscribe:` nor `publish:`, which is
// what would otherwise leave a group out of group_slot_map_ or let a
// controller's group count outrun the upstream ceiling. That invariant lives
// in a different package with nothing binding the two together, so the guards
// are driven directly here; through YAML they are unreachable and would be
// decorative.

class SlotMappingTest : public EstopTest {
 protected:
  // Builds a TopicConfig with `n` groups named g0..g{n-1}, each carrying one
  // subscribe entry so it looks exactly like a parser-accepted group.
  static TopicConfig MakeGroups(int n) {
    TopicConfig tc;
    for (int i = 0; i < n; ++i) {
      tc["g" + std::to_string(i)].subscribe.push_back({"topic" + std::to_string(i)});
    }
    return tc;
  }
};

TEST_F(SlotMappingTest, MapsEveryDeclaredGroupToItsConfiguredSlot) {
  ControllerLifecycleTestAccess::ClearGroupSlots(*node_);
  ControllerLifecycleTestAccess::SetGroupSlot(*node_, "g0", 3);
  ControllerLifecycleTestAccess::SetGroupSlot(*node_, "g1", 1);
  ControllerLifecycleTestAccess::SetTopicConfigForController(*node_, 0, MakeGroups(2));
  ControllerLifecycleTestAccess::SetTopicConfigForController(*node_, 1, MakeGroups(2));

  ASSERT_TRUE(ControllerLifecycleTestAccess::CallBuildControllerSlotMappings(*node_));
  EXPECT_EQ(2, ControllerLifecycleTestAccess::GetMappedGroupCount(*node_, 0));
  EXPECT_EQ(3, ControllerLifecycleTestAccess::GetMappedSlot(*node_, 0, 0));
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetMappedSlot(*node_, 0, 1));
}

TEST_F(SlotMappingTest, RefusesAGroupThatHasNoDeviceSlot) {
  // g1 is deliberately absent from group_slot_map_. operator[] would have
  // default-inserted 0, silently routing g1's commands to whichever device
  // owns slot 0 — and, since the map is re-read afterwards, minting a
  // duplicate watchdog entry and digital twin for that slot too.
  ControllerLifecycleTestAccess::ClearGroupSlots(*node_);
  ControllerLifecycleTestAccess::SetGroupSlot(*node_, "g0", 0);
  ControllerLifecycleTestAccess::SetTopicConfigForController(*node_, 0, MakeGroups(2));
  ControllerLifecycleTestAccess::SetTopicConfigForController(*node_, 1, MakeGroups(1));

  EXPECT_FALSE(ControllerLifecycleTestAccess::CallBuildControllerSlotMappings(*node_));
}

TEST_F(SlotMappingTest, RefusesAControllerDeclaringMoreGroupsThanTheRtArraysHold) {
  // The upstream ceiling counts groups that survived the topics filter; the
  // RT read loop iterates the declared groups. This guard closes that gap on
  // the count the loop actually uses.
  const int over = ControllerLifecycleTestAccess::MaxSlots() + 1;
  ControllerLifecycleTestAccess::ClearGroupSlots(*node_);
  for (int i = 0; i < over; ++i) {
    ControllerLifecycleTestAccess::SetGroupSlot(*node_, "g" + std::to_string(i), 0);
  }
  ControllerLifecycleTestAccess::SetTopicConfigForController(*node_, 0, MakeGroups(over));
  ControllerLifecycleTestAccess::SetTopicConfigForController(*node_, 1, MakeGroups(1));

  EXPECT_FALSE(ControllerLifecycleTestAccess::CallBuildControllerSlotMappings(*node_));
}

}  // namespace
}  // namespace rtc
