// HandCommLoop — the one comm thread that drives every hand of the process
// (epic #906 D69 · D70 · D72). Scripted members stand in for the controllers so
// the loop's own contract is pinned without a link: tick order, the shared
// E-Stop latch between hands, the abort sequence (every hand stops before any
// zero-write, each zero-write once), the timing-row vote, and that one hand
// behaves exactly like the per-controller loop it replaced.

#include "rtc_base/testing/wait_until.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "udp_hand_driver/hand_comm_loop.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <vector>

namespace {

using udp_hand_driver::HandCommLoop;
using udp_hand_driver::HandCommLoopMember;
using namespace std::chrono_literals;

enum class What { kRun, kStop, kAbort };

struct Event {
  int hand;
  What what;
};

// Shared event log (test code: a mutex is fine off the RT rules). Read after
// the loop thread is joined or has exited.
class EventLog {
 public:
  void Add(int hand, What what) {
    std::lock_guard lock(mutex_);
    events_.push_back({hand, what});
  }

  [[nodiscard]] std::vector<Event> Snapshot() const {
    std::lock_guard lock(mutex_);
    return events_;
  }

 private:
  mutable std::mutex mutex_;
  std::vector<Event> events_;
};

class ScriptedHand final : public HandCommLoopMember {
 public:
  ScriptedHand(int id, EventLog& log, HandCommLoop& loop) : id_(id), log_(log), loop_(loop) {}

  // Raise `latch` during the n-th cycle of this hand (1-based) — an E-Stop
  // that lands inside this hand's receive window.
  void RaiseLatchOnRun(std::atomic<bool>* latch, int n) {
    latch_ = latch;
    raise_on_run_ = n;
  }

  // Check `latch` at the head of every cycle, as UdpHandController does.
  void CheckLatchAtHead(const std::atomic<bool>* latch) { head_latch_ = latch; }

  void VoteSuppress(bool vote) { suppress_.store(vote); }

  [[nodiscard]] int Runs() const { return runs_.load(); }

 private:
  void RunCommCycle() noexcept override {
    if (head_latch_ != nullptr && head_latch_->load()) {
      loop_.AbortForEstop();
      return;
    }
    const int n = runs_.fetch_add(1) + 1;
    log_.Add(id_, What::kRun);
    if (latch_ != nullptr && n == raise_on_run_) {
      latch_->store(true);
    }
    if (suppress_.load()) {
      loop_.SuppressTiming();
    }
  }

  void MarkLoopStopping() noexcept override { log_.Add(id_, What::kStop); }

  void OnCommLoopAborted() noexcept override { log_.Add(id_, What::kAbort); }

  int id_;
  EventLog& log_;
  HandCommLoop& loop_;
  std::atomic<bool>* latch_{nullptr};
  int raise_on_run_{0};
  const std::atomic<bool>* head_latch_{nullptr};
  std::atomic<bool> suppress_{false};
  std::atomic<int> runs_{0};
};

// CFS, no pin: the contract under test is ordering, not latency, and CI has
// no SCHED_FIFO.
rtc::PeriodicRtThreadConfig LoopConfig() {
  rtc::ThreadConfig thread{};
  thread.cpu_core = -1;
  thread.sched_policy = SCHED_OTHER;
  thread.sched_priority = 0;
  thread.name = "hand_comm_test";
  return {.thread_config = thread, .frequency_hz = 1000.0};
}

std::size_t CountOf(const std::vector<Event>& events, int hand, What what) {
  return static_cast<std::size_t>(std::count_if(events.begin(), events.end(), [&](const Event& e) {
    return e.hand == hand && e.what == what;
  }));
}

std::size_t DrainRows(rtc::HandTimingBuffer& buffer) {
  return buffer.Drain([](const rtc::RtTickTimingSample&) {});
}

TEST(HandCommLoop, AddRefusesNullDuplicateOverflowAndARunningLoop) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand a(0, log, loop);
  ScriptedHand b(1, log, loop);
  ScriptedHand c(2, log, loop);

  EXPECT_FALSE(loop.Add(nullptr));
  ASSERT_TRUE(loop.Add(&a));
  EXPECT_FALSE(loop.Add(&a)) << "a hand may not transact twice per tick";
  ASSERT_TRUE(loop.Add(&b));
  EXPECT_FALSE(loop.Add(&c)) << "more hands than one bus serves per tick";
  EXPECT_EQ(loop.Count(), HandCommLoop::kMaxHands);

  loop.Start(LoopConfig());
  ASSERT_TRUE(loop.Running());
  EXPECT_FALSE(loop.Clear());
  loop.Join();
  EXPECT_TRUE(loop.Clear());
  EXPECT_EQ(loop.Count(), 0u);
  EXPECT_TRUE(loop.Add(&c));
}

// D69: one tick = every hand, in Add() order, back to back. A stop only lands
// between ticks, so both hands always have the same number of cycles.
TEST(HandCommLoop, RunsEveryHandInAddOrderEveryTick) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));

  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Runs() >= 10; }));
  loop.Join();

  const auto events = log.Snapshot();
  ASSERT_GE(events.size(), 20u);
  EXPECT_EQ(right.Runs(), left.Runs());
  for (std::size_t i = 0; i < events.size(); ++i) {
    EXPECT_EQ(events[i].what, What::kRun);
    EXPECT_EQ(events[i].hand, static_cast<int>(i % 2)) << "tick order broken at event " << i;
  }
  EXPECT_EQ(CountOf(events, 0, What::kAbort), 0u) << "an external stop is not an E-Stop";
}

// D70 · B1: an E-Stop raised during hand 0's window is caught before hand 1's
// transaction in the same tick — hand 1 never transacts again — and the abort
// is one sequence: both hands stop accepting work, then each zero-writes once,
// in tick order.
TEST(HandCommLoop, EstopDuringTheFirstHandSkipsTheSecondHandThatTick) {
  EventLog log;
  HandCommLoop loop;
  std::atomic<bool> latch{false};
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  right.RaiseLatchOnRun(&latch, 3);
  right.CheckLatchAtHead(&latch);
  left.CheckLatchAtHead(&latch);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  loop.SetEstopFlag(&latch);
  rtc::HandTimingBuffer timing;
  loop.SetTimingProducer(&timing);

  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return !loop.Running(); }))
      << "the loop did not self-exit on the E-Stop";
  loop.Join();

  EXPECT_EQ(right.Runs(), 3);
  EXPECT_EQ(left.Runs(), 2) << "hand 1 transacted after the latch rose in hand 0's window";
  const auto events = log.Snapshot();
  EXPECT_EQ(CountOf(events, 0, What::kAbort), 1u);
  EXPECT_EQ(CountOf(events, 1, What::kAbort), 1u);
  EXPECT_EQ(CountOf(events, 0, What::kStop), 1u);
  EXPECT_EQ(CountOf(events, 1, What::kStop), 1u);
  // Order: …run(0) → stop(0) stop(1) → abort(0) abort(1).
  ASSERT_GE(events.size(), 4u);
  const auto n = events.size();
  EXPECT_EQ(events[n - 4].what, What::kStop);
  EXPECT_EQ(events[n - 3].what, What::kStop);
  EXPECT_EQ(events[n - 2].what, What::kAbort);
  EXPECT_EQ(events[n - 2].hand, 0);
  EXPECT_EQ(events[n - 1].what, What::kAbort);
  EXPECT_EQ(events[n - 1].hand, 1);
  // Two full ticks pushed a row each; the aborted tick did not.
  EXPECT_EQ(DrainRows(timing), 2u);
}

// A hand that finds the latch at its own head (the controller's check) aborts
// the whole loop the same way.
TEST(HandCommLoop, AHeadCheckAbortStopsEveryHand) {
  EventLog log;
  HandCommLoop loop;
  std::atomic<bool> latch{false};
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  right.CheckLatchAtHead(&latch);
  left.CheckLatchAtHead(&latch);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  // No SetEstopFlag: only the members look, as on a one-hand loop.

  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Runs() >= 3; }));
  latch.store(true);
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return !loop.Running(); }));
  loop.Join();

  const auto events = log.Snapshot();
  EXPECT_EQ(CountOf(events, 0, What::kStop), 1u);
  EXPECT_EQ(CountOf(events, 1, What::kStop), 1u);
  EXPECT_EQ(CountOf(events, 0, What::kAbort), 1u);
  EXPECT_EQ(CountOf(events, 1, What::kAbort), 1u);
  const auto first_abort = std::find_if(events.begin(), events.end(),
                                        [](const Event& e) { return e.what == What::kAbort; });
  EXPECT_TRUE(std::none_of(first_abort, events.end(), [](const Event& e) {
    return e.what != What::kAbort;
  })) << "a hand ran or stopped after a zero-write";
}

// D72: a tick's timing row is dropped only when every hand that ran asked.
TEST(HandCommLoop, ATimingRowIsDroppedOnlyWhenEveryHandVotes) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  rtc::HandTimingBuffer timing;
  loop.SetTimingProducer(&timing);

  right.VoteSuppress(true);
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Runs() >= 10; }));
  loop.Join();
  EXPECT_EQ(DrainRows(timing), loop.TickCount()) << "one hand's skip hid the other's work";

  left.VoteSuppress(true);
  const auto ticks_before = loop.TickCount();
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Runs() >= 20; }));
  loop.Join();
  EXPECT_GT(loop.TickCount(), ticks_before);
  EXPECT_EQ(DrainRows(timing), 0u);
}

// N = 1 is the loop the controller used to own: the hand's own vote decides,
// its head check aborts, its zero-write runs once.
TEST(HandCommLoop, OneHandBehavesLikeThePerControllerLoop) {
  EventLog log;
  HandCommLoop loop;
  std::atomic<bool> latch{false};
  ScriptedHand hand(0, log, loop);
  hand.CheckLatchAtHead(&latch);
  ASSERT_TRUE(loop.Add(&hand));
  rtc::HandTimingBuffer timing;
  loop.SetTimingProducer(&timing);

  hand.VoteSuppress(true);
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return hand.Runs() >= 5; }));
  loop.Join();
  EXPECT_EQ(DrainRows(timing), 0u);

  hand.VoteSuppress(false);
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return hand.Runs() >= 10; }));
  latch.store(true);
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return !loop.Running(); }));
  loop.Join();
  const auto events = log.Snapshot();
  EXPECT_EQ(CountOf(events, 0, What::kStop), 1u);
  EXPECT_EQ(CountOf(events, 0, What::kAbort), 1u);
  EXPECT_EQ(events.back().what, What::kAbort);
  EXPECT_GE(DrainRows(timing), 4u);
}

// deactivate → activate after an E-Stop: the loop re-arms, so a second E-Stop
// aborts (and zero-writes) again.
TEST(HandCommLoop, RestartAfterAnAbortAbortsAgain) {
  EventLog log;
  HandCommLoop loop;
  std::atomic<bool> latch{false};
  ScriptedHand hand(0, log, loop);
  hand.CheckLatchAtHead(&latch);
  ASSERT_TRUE(loop.Add(&hand));

  for (int round = 1; round <= 2; ++round) {
    latch.store(false);
    const int runs_before = hand.Runs();
    loop.Start(LoopConfig());
    ASSERT_TRUE(rtc::testing::WaitUntil([&] { return hand.Runs() >= runs_before + 3; }))
        << "round " << round << " did not run";
    latch.store(true);
    ASSERT_TRUE(rtc::testing::WaitUntil([&] { return !loop.Running(); }));
    loop.Join();
    EXPECT_EQ(CountOf(log.Snapshot(), 0, What::kAbort), static_cast<std::size_t>(round));
  }
}

// Stopping one hand of a shared loop stops the thread that ran them all, so
// every hand is marked stopped; a hand that is destroyed takes itself off.
TEST(HandCommLoop, StopAllMarksEveryHandAndRemoveKeepsTheOrder) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Runs() >= 3; }));
  loop.StopAll();
  EXPECT_FALSE(loop.Running());
  const auto events = log.Snapshot();
  EXPECT_EQ(CountOf(events, 0, What::kStop), 1u);
  EXPECT_EQ(CountOf(events, 1, What::kStop), 1u);
  EXPECT_EQ(CountOf(events, 0, What::kAbort), 0u) << "a stop is not an E-Stop";

  EXPECT_FALSE(loop.Remove(nullptr));
  ASSERT_TRUE(loop.Remove(&right));
  EXPECT_FALSE(loop.Remove(&right));
  EXPECT_EQ(loop.Count(), 1u);
  const int left_before = left.Runs();
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Runs() >= left_before + 3; }));
  EXPECT_FALSE(loop.Remove(&left)) << "removed from a running loop";
  loop.Join();
}

}  // namespace
