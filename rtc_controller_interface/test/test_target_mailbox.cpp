// ── test_target_mailbox.cpp ──────────────────────────────────────────────────
// Base-level contract for the target mailbox (issue #206).
//
// The off-RT → RT marshal used to be reimplemented once per controller: ten
// copies of the same push/drain skeleton, which is how a fix could land in one
// and be missed in the others (PR #256 F5/F7/F8/F9, PR #263's missing
// device-invalid drain). The skeleton now lives in RTControllerInterface and
// this file is its single pinning point.
//
// The binding below is deliberately minimal — a TargetSlot, a SeqLock, and an
// ApplyPendingTarget override — because that is exactly what the three-tier
// split leaves to a derived controller. Everything else the tests touch
// (bounds, the generation gate, saturation policy, drop accounting) is base
// behaviour, so a regression in it fails here rather than in one robot's suite.
//
// Not covered here: DeliverTargetMessage's message-level validation
// (test_rt_controller_interface.cpp) and the generation counter's own
// arithmetic (test_activation_generation.cpp — that file drives a hand-rolled
// stand-in for the queue; this one drives the real one end to end).
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/threading/seqlock.hpp>
#include <rtc_controller_interface/rt_controller_interface.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <span>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

namespace {

const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive");

// A binding that owns only what the three-tier split says it should: the slot
// layout, the SeqLock it publishes through, and the meaning of `is_task`.
// Joint and task targets land in independent slots, mirroring the one real
// controller (DemoWbc) that needs both — a binding that keeps a single slot is
// the degenerate case of this one.
class MailboxBinding : public rtc::RTControllerInterface {
 public:
  struct TargetSlot {
    std::array<std::array<double, rtc::kMaxDeviceChannels>, rtc::ControllerState::kMaxDevices>
        joint{};
    std::array<double, rtc::kTaskSpaceDim> task{};
  };

  static_assert(std::is_trivially_copyable_v<TargetSlot>,
                "TargetSlot must be trivially copyable for SeqLock<TargetSlot>");

  // ── Ingress (off-RT) ─────────────────────────────────────────────────────
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override {
    PushPendingTarget(device_idx, target, /*is_task=*/false);
  }

  void SetDeviceTaskTarget(int device_idx, std::span<const double> task6) noexcept override {
    PushPendingTarget(device_idx, task6, /*is_task=*/true);
  }

  // ── RT tick ──────────────────────────────────────────────────────────────
  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    if (estopped_.load(std::memory_order_acquire)) {
      DiscardPendingTargets();
      estop_ticks_.fetch_add(1, std::memory_order_relaxed);
      rtc::ControllerOutput out{};
      out.valid = false;
      return out;
    }
    applied_last_tick_ = DrainPendingTargets();
    if (applied_last_tick_ > 0) {
      slot_seqlock_.Store(slot_);
    }
    rtc::ControllerOutput out{};
    out.valid = true;
    return out;
  }

  [[nodiscard]] std::string_view Name() const noexcept override { return "MailboxBinding"; }

  void TriggerEstop() noexcept override { estopped_.store(true, std::memory_order_release); }

  void ClearEstop() noexcept override { estopped_.store(false, std::memory_order_release); }

  [[nodiscard]] bool IsEstopped() const noexcept override {
    return estopped_.load(std::memory_order_acquire);
  }

  // ── Observation ──────────────────────────────────────────────────────────
  [[nodiscard]] int applied_last_tick() const noexcept { return applied_last_tick_; }

  [[nodiscard]] const TargetSlot& rt_slot() const noexcept { return slot_; }

  [[nodiscard]] TargetSlot published_slot() const noexcept { return slot_seqlock_.Load(); }

  [[nodiscard]] int joint_applies() const noexcept { return joint_applies_; }

  [[nodiscard]] int task_applies() const noexcept { return task_applies_; }

  // Every span length ApplyPendingTarget was handed, in order.
  [[nodiscard]] const std::vector<std::size_t>& applied_widths() const noexcept {
    return applied_widths_;
  }

 protected:
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override {
    applied_widths_.push_back(values.size());
    if (is_task) {
      ++task_applies_;
      for (std::size_t i = 0; i < values.size() && i < slot_.task.size(); ++i) {
        slot_.task[i] = values[i];
      }
      return;
    }
    ++joint_applies_;
    auto& row = slot_.joint[static_cast<std::size_t>(device_idx)];
    for (std::size_t i = 0; i < values.size() && i < row.size(); ++i) {
      row[i] = values[i];
    }
  }

 private:
  TargetSlot slot_{};
  rtc::SeqLock<TargetSlot> slot_seqlock_;
  std::atomic<bool> estopped_{false};
  std::atomic<int> estop_ticks_{0};
  int applied_last_tick_{0};
  int joint_applies_{0};
  int task_applies_{0};
  // push_back is legal here: the test's "RT" thread is a plain std::thread and
  // this vector exists only to observe the base, never in a shipped tick.
  std::vector<std::size_t> applied_widths_;
};

rtc::ControllerState MakeState() {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = 0.002;
  state.devices[0].num_channels = 6;
  state.devices[0].valid = true;
  return state;
}

std::vector<double> Uniform(std::size_t n, double v) {
  return std::vector<double>(n, v);
}

}  // namespace

// ── Push → drain → apply ────────────────────────────────────────────────────

// The defining property of the marshal: the off-RT caller never touches the
// slot. Nothing is applied until the RT tick runs.
TEST(TargetMailbox, PushedTargetIsAppliedOnTheNextTickAndNotBefore) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, Uniform(6, 0.5));
  EXPECT_EQ(ctrl.joint_applies(), 0) << "the off-RT push reached the slot without an RT tick";

  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 1);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 0.5);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][5], 0.5);
}

// One drain consumes everything queued since the last one, in FIFO order, and
// reports how many entries it applied — the count derived controllers use to
// decide whether to publish their slot.
TEST(TargetMailbox, OneDrainConsumesEveryQueuedEntryInOrder) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, Uniform(6, 1.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 2.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 3.0));

  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 3);
  EXPECT_EQ(ctrl.joint_applies(), 3);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 3.0) << "entries were not applied oldest-first";

  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 0) << "the queue was not emptied by the previous drain";
}

// The applied count is what a derived controller uses as its "slot changed"
// predicate, so an empty tick must not publish.
TEST(TargetMailbox, SlotIsPublishedOnlyOnTicksThatAppliedSomething) {
  MailboxBinding ctrl;
  auto state = MakeState();

  (void)ctrl.Compute(state);
  EXPECT_DOUBLE_EQ(ctrl.published_slot().joint[0][0], 0.0);

  ctrl.SetDeviceTarget(0, Uniform(6, 0.75));
  (void)ctrl.Compute(state);
  EXPECT_DOUBLE_EQ(ctrl.published_slot().joint[0][0], 0.75);
}

// ── Bounds ──────────────────────────────────────────────────────────────────

// A goal wider than the device capacity is truncated at ingress, so the RT side
// can index the span without re-checking.
TEST(TargetMailbox, ValuesAreBoundedToTheChannelCapacityAtIngress) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, Uniform(static_cast<std::size_t>(rtc::kMaxDeviceChannels) + 32, 0.25));
  (void)ctrl.Compute(state);

  ASSERT_EQ(ctrl.applied_widths().size(), 1U);
  EXPECT_EQ(ctrl.applied_widths()[0], static_cast<std::size_t>(rtc::kMaxDeviceChannels));
}

// An empty goal still counts as an application: the pre-existing controllers
// all marked their slot dirty for it, and a derived re-arm flag (trajectory
// restart) keys off the count, not off the width.
TEST(TargetMailbox, AnEmptyGoalStillCountsAsAnApplication) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, std::span<const double>{});
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.applied_last_tick(), 1);
  ASSERT_EQ(ctrl.applied_widths().size(), 1U);
  EXPECT_EQ(ctrl.applied_widths()[0], 0U);
}

// An out-of-range device index is refused before the queue, so it consumes no
// depth and is not counted as a saturation drop.
TEST(TargetMailbox, OutOfRangeDeviceIndexIsRefusedAtIngress) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(-1, Uniform(6, 1.0));
  ctrl.SetDeviceTarget(rtc::ControllerState::kMaxDevices, Uniform(6, 1.0));
  ctrl.SetDeviceTarget(rtc::ControllerState::kMaxDevices + 7, Uniform(6, 1.0));

  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 0);
  EXPECT_EQ(ctrl.GetTargetDropCount(), 0U) << "a rejected index must not read as queue saturation";
}

// The last legal index is accepted — the bound is exclusive, and an
// off-by-one the other way would silently disable the highest device.
TEST(TargetMailbox, TheHighestLegalDeviceIndexIsAccepted) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(rtc::ControllerState::kMaxDevices - 1, Uniform(4, 0.125));
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.applied_last_tick(), 1);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[rtc::ControllerState::kMaxDevices - 1][0], 0.125);
}

// ── is_task routing ─────────────────────────────────────────────────────────

// The base carries the tag but never reads it; routing is the binding's call.
TEST(TargetMailbox, TaskAndJointTargetsReachIndependentSlots) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, Uniform(6, 1.5));
  ctrl.SetDeviceTaskTarget(0, Uniform(rtc::kTaskSpaceDim, 2.5));
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.joint_applies(), 1);
  EXPECT_EQ(ctrl.task_applies(), 1);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 1.5) << "a task goal clobbered the joint slot";
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().task[0], 2.5);
}

namespace {

// A binding that does NOT override SetDeviceTaskTarget, to pin the base's
// default forwarding: a task goal must arrive as an ordinary joint-tagged push.
class SingleSlotBinding : public MailboxBinding {
 public:
  using MailboxBinding::SetDeviceTarget;
};

}  // namespace

TEST(TargetMailbox, DefaultTaskIngressForwardsToTheJointPath) {
  SingleSlotBinding ctrl;
  auto state = MakeState();

  // Reach the base default explicitly rather than through the derived override.
  ctrl.rtc::RTControllerInterface::SetDeviceTaskTarget(0, Uniform(rtc::kTaskSpaceDim, 3.5));
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.task_applies(), 0) << "the base default must not tag the entry as a task goal";
  EXPECT_EQ(ctrl.joint_applies(), 1);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 3.5);
}

// ── Activation generation gate, through the real queue ──────────────────────

TEST(TargetMailbox, EntryQueuedBeforeReactivationIsDiscardedByTheDrain) {
  MailboxBinding ctrl;
  auto state = MakeState();
  (void)ctrl.on_activate(kInactive);

  ctrl.SetDeviceTarget(0, Uniform(6, 1.0));
  (void)ctrl.Compute(state);
  ASSERT_EQ(ctrl.applied_last_tick(), 1) << "a live target was dropped";

  // The subscription stays alive while Inactive, so this still lands on the queue.
  (void)ctrl.on_deactivate(kInactive);
  ctrl.SetDeviceTarget(0, Uniform(6, 9.0));
  (void)ctrl.on_activate(kInactive);

  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 0)
      << "a target queued before this activation survived the cycle";
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 1.0) << "the stale target reached the slot";
}

// A stale entry must be POPPED, not skipped: the drain has to keep going past
// one. Were it to stop instead, dead entries would sit in front of every later
// goal and the controller would go deaf after one deactivation. Two stale
// entries, so the fresh one still fits in the usable depth — the interaction
// when it does NOT fit is the next test.
TEST(TargetMailbox, StaleEntriesAreConsumedSoLaterOnesStillArrive) {
  MailboxBinding ctrl;
  auto state = MakeState();
  (void)ctrl.on_activate(kInactive);

  ctrl.SetDeviceTarget(0, Uniform(6, 7.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 7.0));
  (void)ctrl.on_activate(kInactive);

  ctrl.SetDeviceTarget(0, Uniform(6, 4.0));
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.applied_last_tick(), 1) << "the drain stopped at the first stale entry";
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 4.0);
  EXPECT_EQ(ctrl.GetTargetDropCount(), 0U) << "three entries must fit in the usable depth";
}

// Stale entries still OCCUPY depth until an RT tick drains them, because the
// gate is applied on the drain and nothing may pop from the lifecycle thread
// (that would give the SPSC queue a second consumer). So a producer that
// saturates the queue while the controller is Inactive costs the first goal
// after re-activation. Pinned because it is the visible cost of the design, and
// because it is now observable: the drop counter names it instead of the goal
// vanishing without trace.
TEST(TargetMailbox, StaleEntriesStillOccupyDepthUntilTheNextTick) {
  MailboxBinding ctrl;
  auto state = MakeState();
  (void)ctrl.on_activate(kInactive);

  ctrl.SetDeviceTarget(0, Uniform(6, 7.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 7.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 7.0));
  (void)ctrl.on_activate(kInactive);

  ctrl.SetDeviceTarget(0, Uniform(6, 4.0));
  EXPECT_EQ(ctrl.GetTargetDropCount(), 1U) << "the fresh goal was lost without being counted";

  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 0) << "a stale entry was applied";
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 0.0);

  // One tick later the depth is back and the operator's next goal lands.
  ctrl.SetDeviceTarget(0, Uniform(6, 4.0));
  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 1);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 4.0);
}

// ── Saturation policy: FIFO + newest-drop, three usable slots ───────────────

TEST(TargetMailbox, OverflowDropsTheNewestAndIsCountedNotSilent) {
  MailboxBinding ctrl;
  auto state = MakeState();

  // Capacity is four with one slot reserved to distinguish full from empty, so
  // three entries fit between two ticks.
  ctrl.SetDeviceTarget(0, Uniform(6, 1.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 2.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 3.0));
  EXPECT_EQ(ctrl.GetTargetDropCount(), 0U) << "the usable depth is smaller than three";

  ctrl.SetDeviceTarget(0, Uniform(6, 4.0));
  EXPECT_EQ(ctrl.GetTargetDropCount(), 1U) << "the fourth push was neither queued nor counted";

  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.applied_last_tick(), 3);
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 3.0)
      << "saturation must drop the arriving goal, not overwrite an accepted one";
}

TEST(TargetMailbox, DropCountIsCumulativeAndCapacityRecoversAfterADrain) {
  MailboxBinding ctrl;
  auto state = MakeState();

  for (int i = 0; i < 6; ++i) {
    ctrl.SetDeviceTarget(0, Uniform(6, static_cast<double>(i)));
  }
  EXPECT_EQ(ctrl.GetTargetDropCount(), 3U);

  (void)ctrl.Compute(state);
  ctrl.SetDeviceTarget(0, Uniform(6, 42.0));
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.applied_last_tick(), 1) << "the queue did not free the drained slots";
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 42.0);
  EXPECT_EQ(ctrl.GetTargetDropCount(), 3U) << "the drop counter is not monotonic";
}

// Saturation and malformed-message rejection are different failures and must
// stay separately countable — one blames the producer's rate, the other its
// payload.
TEST(TargetMailbox, SaturationDoesNotMoveTheRejectCounter) {
  MailboxBinding ctrl;

  for (int i = 0; i < 8; ++i) {
    ctrl.SetDeviceTarget(0, Uniform(6, 1.0));
  }

  EXPECT_GT(ctrl.GetTargetDropCount(), 0U);
  EXPECT_EQ(ctrl.GetTargetRejectCount(), 0U);
}

// ── Discard ─────────────────────────────────────────────────────────────────

// The E-STOP / device-invalid primitive: entries are dropped without reaching
// the slot, and the queue is usable again immediately afterwards.
TEST(TargetMailbox, DiscardEmptiesTheQueueWithoutApplyingAnything) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, Uniform(6, 5.0));
  ctrl.SetDeviceTarget(0, Uniform(6, 6.0));
  ctrl.TriggerEstop();
  (void)ctrl.Compute(state);  // takes the E-STOP early return → DiscardPendingTargets()

  EXPECT_EQ(ctrl.joint_applies(), 0) << "a goal queued during E-STOP reached the slot";
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 0.0);

  ctrl.ClearEstop();
  ctrl.SetDeviceTarget(0, Uniform(6, 8.0));
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.applied_last_tick(), 1) << "the discard left the queue unusable";
  EXPECT_DOUBLE_EQ(ctrl.rt_slot().joint[0][0], 8.0);
}

// A discard is not a drop: nothing was lost to saturation, the controller chose
// to refuse. Conflating the two would make the diagnostic counter lie about the
// producer's rate every time a robot is E-STOPPED.
TEST(TargetMailbox, DiscardDoesNotMoveTheDropCounter) {
  MailboxBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, Uniform(6, 5.0));
  ctrl.TriggerEstop();
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.GetTargetDropCount(), 0U);
}

namespace {

// Pushes but never overrides ApplyPendingTarget — the R4 trade-off made
// visible. Pinned rather than left to prose because the failure is silent: the
// drain pops the entries, so the drop counter stays clean while every goal is
// lost. If ApplyPendingTarget is ever made pure virtual, this stops compiling,
// which is the intended way to find this test.
class ForgetfulBinding : public rtc::RTControllerInterface {
 public:
  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    last_applied_ = DrainPendingTargets();
    rtc::ControllerOutput out{};
    out.valid = true;
    return out;
  }

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override {
    PushPendingTarget(device_idx, target, /*is_task=*/false);
  }

  [[nodiscard]] std::string_view Name() const noexcept override { return "ForgetfulBinding"; }

  [[nodiscard]] int last_applied() const noexcept { return last_applied_; }

 private:
  int last_applied_{0};
};

}  // namespace

TEST(TargetMailbox, BindingWithoutApplyOverrideLosesTargetsWithoutADiagnostic) {
  ForgetfulBinding ctrl;
  auto state = MakeState();

  ctrl.SetDeviceTarget(0, Uniform(6, 1.0));
  (void)ctrl.Compute(state);

  EXPECT_EQ(ctrl.last_applied(), 1) << "the entry was consumed";
  EXPECT_EQ(ctrl.GetTargetDropCount(), 0U)
      << "documented trade-off (#206 R4): a missing ApplyPendingTarget override is invisible "
         "to the drop counter — the drain popped the entry and nobody stored it";
}

// ── SPSC stress: one off-RT producer against the RT drain ───────────────────

// The mailbox's whole reason to exist is that the producer and the RT thread
// run concurrently. Every pushed goal is uniform across channels, so any
// published slot whose channels disagree can only have come from two different
// entries — a torn read or a drain that applied a partial entry.
TEST(TargetMailbox, ConcurrentProducerAndRtDrainNeverPublishATornSlot) {
  MailboxBinding ctrl;
  auto state = MakeState();

  constexpr int kIters = 100'000;
  std::atomic<bool> producer_done{false};
  std::atomic<int> ticks{0};
  std::atomic<int> torn{0};
  std::atomic<int> checked{0};

  std::thread producer([&]() {
    std::array<double, 6> tgt{};
    for (int i = 0; i < kIters; ++i) {
      const double v = 1.0 + 0.001 * static_cast<double>(i % 1024);
      tgt.fill(v);
      ctrl.SetDeviceTarget(0, tgt);
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    while (!producer_done.load(std::memory_order_acquire) ||
           ticks.load(std::memory_order_relaxed) < kIters / 8) {
      (void)ctrl.Compute(state);
      ticks.fetch_add(1, std::memory_order_relaxed);
    }
  });

  producer.join();
  consumer.join();

  // Both workers have joined, so this read races with nothing.
  const auto published = ctrl.published_slot();
  for (std::size_t i = 1; i < 6; ++i) {
    if (published.joint[0][i] != published.joint[0][0]) {
      torn.fetch_add(1, std::memory_order_relaxed);
    }
    checked.fetch_add(1, std::memory_order_relaxed);
  }

  EXPECT_GT(ticks.load(std::memory_order_relaxed), 0);
  EXPECT_EQ(checked.load(std::memory_order_relaxed), 5);
  EXPECT_EQ(torn.load(std::memory_order_relaxed), 0)
      << "the published slot mixed values from two different goals";
  EXPECT_GE(published.joint[0][0], 1.0) << "no goal was ever applied — the race never opened";
}
