// ── test_reset_fault_contract.cpp ────────────────────────────────────────────
// Base-level contract for the controller-local fault latch (issue #260).
//
// ResetFault() / HasLatchedFault() are the pair the controller manager's
// /rtc_cm/reset_fault service drives, and the CM only ever holds an
// RTControllerInterface&. A controller that implements the pair on its concrete
// class alone compiles, passes its own suite, and is still unreachable from the
// CM — which is the defect #260 reports. So every check here goes through a
// base reference: if the virtuals leave the base, this file stops compiling.
//
// The binding below is the REFERENCE SHAPE the base's doc comments describe: an
// off-RT atomic request, consumed once on the RT tick ahead of the E-STOP early
// return, with the observable latch published per tick through a SeqLock. It
// latches on command rather than by provoking a real fault, because the latch
// CAUSE belongs to whichever controller owns it — what the base owns is the
// recovery path around it.
//
// The four contract items:
//   1. both calls dispatch through the base interface;
//   2. HasLatchedFault() reports what actually happened, so the service answers
//      "cleared" rather than "delivered";
//   3. E-8 separation in BOTH directions — ClearEstop() does not release the
//      controller fault, ResetFault() does not release the global E-STOP;
//   4. an unconsumed reset request does not survive an activation boundary,
//      which would otherwise launder a fault nobody re-authorised.
//
// Coverage note: post-migration there is no production binding that overrides
// this pair, so "real latch → CM service → cleared" end to end lives in
// rtc_controller_manager/test/test_reset_fault_service.cpp against a fixture
// controller. This file pins the interface half.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/threading/seqlock.hpp>
#include <rtc_controller_interface/rt_controller_interface.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <span>
#include <string_view>

namespace {

const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive");

class FaultLatchBinding : public rtc::RTControllerInterface {
 public:
  // Diagnostic snapshot stored on every Compute() exit path — including the
  // E-STOP early return, so an E-STOPPED tick publishes the real latch state
  // instead of a default-constructed "clean".
  struct Diagnostics {
    bool latched{false};
    bool estopped{false};
  };

  // ── The imperative latch (test-only) ─────────────────────────────────────
  // Stands in for whatever fault a real controller detects. Consumed by the
  // next tick, like a real detection would be.
  void LatchFaultForTesting() noexcept { latch_request_.store(true, std::memory_order_release); }

  // ── #260 pair ────────────────────────────────────────────────────────────
  void ResetFault() noexcept override { reset_requested_.store(true, std::memory_order_release); }

  [[nodiscard]] bool HasLatchedFault() const noexcept override { return diag_lock_.Load().latched; }

  // ── E-STOP (E-8: a separate latch, on purpose) ───────────────────────────
  void TriggerEstop() noexcept override { estopped_.store(true, std::memory_order_release); }

  void ClearEstop() noexcept override { estopped_.store(false, std::memory_order_release); }

  [[nodiscard]] bool IsEstopped() const noexcept override {
    return estopped_.load(std::memory_order_acquire);
  }

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    Diagnostics diag;
    diag.estopped = estopped_.load(std::memory_order_acquire);

    // Consume the reset edge BEFORE the E-STOP early return: a controller fault
    // must be clearable while the global latch is up, and the two latches are
    // deliberately independent.
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      latched_ = false;
      ++resets_consumed_;
    }
    if (latch_request_.exchange(false, std::memory_order_acq_rel)) {
      latched_ = true;
    }

    diag.latched = latched_;
    diag_lock_.Store(diag);

    rtc::ControllerOutput out{};
    out.valid = !diag.estopped;
    return out;
  }

  void SetDeviceTarget(int, std::span<const double>) noexcept override {}

  [[nodiscard]] std::string_view Name() const noexcept override { return "FaultLatchBinding"; }

  [[nodiscard]] int resets_consumed() const noexcept { return resets_consumed_; }

 protected:
  void ResetTargetInitialization() noexcept override {
    // An UNCONSUMED reset request must not cross an activation boundary (#260).
    // Compute() runs only while active, so a request delivered to an inactive
    // controller would otherwise be honoured on the first tick after the next
    // activation — unlatching a fault nobody re-authorised then.
    reset_requested_.store(false, std::memory_order_release);
  }

 private:
  std::atomic<bool> reset_requested_{false};
  std::atomic<bool> latch_request_{false};
  std::atomic<bool> estopped_{false};
  rtc::SeqLock<Diagnostics> diag_lock_;
  bool latched_{false};  // RT-thread-owned
  int resets_consumed_{0};
};

// A binding that keeps no fault latch of its own: it must read as "never
// latched" rather than inheriting anything, so the service reports a no-op for
// it instead of claiming a clear.
class NoLatchBinding : public rtc::RTControllerInterface {
 public:
  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    rtc::ControllerOutput out{};
    out.valid = true;
    return out;
  }

  void SetDeviceTarget(int, std::span<const double>) noexcept override {}

  [[nodiscard]] std::string_view Name() const noexcept override { return "NoLatchBinding"; }
};

rtc::ControllerState MakeState() {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = 0.002;
  state.devices[0].num_channels = 6;
  state.devices[0].valid = true;
  return state;
}

// Fixture that drives the binding only through the base reference, so nothing
// below can accidentally reach a concrete-class-only implementation.
class ResetFaultContract : public ::testing::Test {
 protected:
  void SetUp() override {
    state_ = MakeState();
    ctrl_.LatchFaultForTesting();
    Tick();
    ASSERT_TRUE(base().HasLatchedFault()) << "precondition: the binding failed to latch";
  }

  void Tick() { (void)ctrl_.Compute(state_); }

  void Reactivate() { (void)base().on_activate(kInactive); }

  [[nodiscard]] rtc::RTControllerInterface& base() noexcept { return ctrl_; }

  FaultLatchBinding ctrl_;
  rtc::ControllerState state_{};
};

}  // namespace

// (1) + (2) — the pair dispatches through the base, and the observable half
// reports the clear rather than merely the delivery.
TEST_F(ResetFaultContract, ResetThroughTheBaseInterfaceClearsTheLatchAndIsReported) {
  base().ResetFault();
  EXPECT_TRUE(base().HasLatchedFault()) << "the request cleared the latch off the RT thread";

  Tick();
  EXPECT_FALSE(base().HasLatchedFault())
      << "ResetFault() through the base interface did not clear the latch";
  EXPECT_EQ(ctrl_.resets_consumed(), 1);
}

// The request is an edge, not a level: one delivery clears one latch. A sticky
// request would silently unlatch the next fault the controller detects.
TEST_F(ResetFaultContract, OneRequestClearsOneLatchAndIsNotReplayed) {
  base().ResetFault();
  Tick();
  ASSERT_FALSE(base().HasLatchedFault());

  ctrl_.LatchFaultForTesting();
  Tick();
  EXPECT_TRUE(base().HasLatchedFault()) << "a consumed reset request unlatched a later fault";
  EXPECT_EQ(ctrl_.resets_consumed(), 1);
}

// (3a) E-8, first direction. The E-STOP is run for real — trigger, tick through
// the early return, clear — because that early return is a different Compute()
// exit with its own diagnostic store, and clearing an E-STOP that was never set
// would exercise none of it.
TEST_F(ResetFaultContract, ClearEstopDoesNotReleaseTheControllerFault) {
  base().TriggerEstop();
  Tick();
  ASSERT_TRUE(base().HasLatchedFault())
      << "the E-STOP tick published a state that hides the SAFE_STOP latch";

  base().ClearEstop();
  Tick();
  EXPECT_TRUE(base().HasLatchedFault())
      << "ClearEstop() released a controller-local fault latch (E-8)";
}

// (3b) E-8, other direction.
TEST_F(ResetFaultContract, ResetFaultDoesNotReleaseTheGlobalEstop) {
  base().TriggerEstop();
  base().ResetFault();
  Tick();

  EXPECT_FALSE(base().HasLatchedFault()) << "the controller fault should have cleared";
  EXPECT_TRUE(base().IsEstopped()) << "ResetFault() cleared the global E-STOP latch (E-8)";
}

// The mechanism that makes (3b) possible: the request is consumed ahead of the
// E-STOP early return, so a controller fault is recoverable while the robot is
// still held. Consuming it after would strand the fault until the E-STOP is
// released, which is the opposite of the intended operator sequence.
TEST_F(ResetFaultContract, AResetIsConsumedEvenOnAnEstoppedTick) {
  base().TriggerEstop();
  base().ResetFault();
  Tick();

  EXPECT_EQ(ctrl_.resets_consumed(), 1) << "the E-STOP early return swallowed the reset request";
}

// (4) An unconsumed request must not cross an activation boundary.
TEST_F(ResetFaultContract, UnconsumedResetDoesNotSurviveAnActivationBoundary) {
  base().ResetFault();  // delivered while the controller is not ticking
  Reactivate();
  Tick();

  EXPECT_TRUE(base().HasLatchedFault())
      << "activation consumed a stale reset request and laundered the fault (E-8)";
  EXPECT_EQ(ctrl_.resets_consumed(), 0);

  // …and the operator can still recover by asking again, now that the
  // controller is the one running.
  base().ResetFault();
  Tick();
  EXPECT_FALSE(base().HasLatchedFault());
}

// The latch itself is what must NOT be cleared by activation: only the pending
// request is dropped. A re-activated controller that faulted stays faulted.
TEST_F(ResetFaultContract, ActivationDoesNotClearTheLatchItself) {
  Reactivate();
  Tick();
  EXPECT_TRUE(base().HasLatchedFault()) << "activation auto-recovered a latched fault (§10.6)";
}

// ── Base defaults ───────────────────────────────────────────────────────────

// "No latch" must mean no latch, not "recovered": a controller that keeps none
// reads as clean before and after a reset, and the reset must not fabricate one.
TEST(ResetFaultDefaults, ABindingWithNoLatchReadsCleanAndResetIsANoOp) {
  NoLatchBinding ctrl;
  rtc::RTControllerInterface& base = ctrl;
  auto state = MakeState();

  EXPECT_FALSE(base.HasLatchedFault()) << "an un-ticked controller reported a latched fault";

  (void)ctrl.Compute(state);
  EXPECT_FALSE(base.HasLatchedFault()) << "a nominal tick reported a latched fault";

  base.ResetFault();
  (void)ctrl.Compute(state);
  EXPECT_FALSE(base.HasLatchedFault()) << "ResetFault() fabricated a latch on a clean controller";
}

// A controller that has never ticked has never stored a diagnostic snapshot;
// the answer must be "not latched", not whatever a default-constructed snapshot
// happens to hold.
TEST(ResetFaultDefaults, AnUntickedBindingWithALatchStillReadsClean) {
  FaultLatchBinding ctrl;
  rtc::RTControllerInterface& base = ctrl;

  EXPECT_FALSE(base.HasLatchedFault());

  ctrl.LatchFaultForTesting();
  EXPECT_FALSE(base.HasLatchedFault())
      << "the latch became observable before the tick that publishes it";
}
