// HandCommLoop — the one comm thread that drives every hand of the process
// (epic #906 D69 · D70 · D72). Scripted members stand in for the controllers so
// the loop's own contract is pinned without a link: tick order, the shared
// E-Stop latch between hands, the abort sequence (every hand stops before any
// zero-write, each zero-write once), the timing-row vote, and that one hand
// behaves exactly like the per-controller loop it replaced. The shared-request
// tick (#956): Prepare per hand → one REQ from the first hand that takes part →
// Finish per taking-part hand, and the E-Stop re-checks before every Prepare
// after the first, right before the REQ and between Finishes.

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
using udp_hand_driver::SharedReqPrep;
using udp_hand_driver::SharedRequest;
using namespace std::chrono_literals;

enum class What { kRun, kStop, kAbort, kPrepare, kReq, kFinish };

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

  // Shared-request tick script (#956): whether Prepare takes part / wrote,
  // whether SendSharedReq succeeds, and a latch raised on the n-th call of a
  // step (kPrepare, kReq or kFinish).
  void TakesPart(bool v) { takes_part_ = v; }

  void Writes(bool v) { writes_ = v; }

  void ReqSends(bool v) { req_sends_ = v; }

  void RaiseLatchOnStep(std::atomic<bool>* latch, What step, int n) {
    step_latch_ = latch;
    latch_step_ = step;
    latch_step_n_ = n;
  }

  [[nodiscard]] int Runs() const { return runs_.load(); }

  [[nodiscard]] int Prepares() const { return prepares_.load(); }

  [[nodiscard]] int Finishes() const { return finishes_.load(); }

  [[nodiscard]] SharedRequest LastReq() const {
    std::lock_guard lock(req_mutex_);
    return last_req_;
  }

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

  SharedReqPrep PrepareSharedReqCycle() noexcept override {
    if (head_latch_ != nullptr && head_latch_->load()) {
      loop_.AbortForEstop();
      return {};
    }
    const int n = prepares_.fetch_add(1) + 1;
    log_.Add(id_, What::kPrepare);
    MaybeRaise(What::kPrepare, n);
    return {.takes_part = takes_part_, .wrote = writes_};
  }

  bool SendSharedReq(
      udp_hand_driver::HandReadOutcome::Clock::time_point& t_sent) noexcept override {
    const int n = reqs_.fetch_add(1) + 1;
    log_.Add(id_, What::kReq);
    t_sent = udp_hand_driver::HandReadOutcome::Clock::now();
    MaybeRaise(What::kReq, n);
    return req_sends_;
  }

  void FinishSharedReqCycle(const SharedRequest& req) noexcept override {
    const int n = finishes_.fetch_add(1) + 1;
    log_.Add(id_, What::kFinish);
    {
      std::lock_guard lock(req_mutex_);
      last_req_ = req;
    }
    MaybeRaise(What::kFinish, n);
  }

  void MaybeRaise(What step, int n) {
    if (step_latch_ != nullptr && step == latch_step_ && n == latch_step_n_) {
      step_latch_->store(true);
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
  bool takes_part_{true};
  bool writes_{false};
  bool req_sends_{true};
  std::atomic<bool>* step_latch_{nullptr};
  What latch_step_{What::kRun};
  int latch_step_n_{0};
  std::atomic<int> prepares_{0};
  std::atomic<int> reqs_{0};
  std::atomic<int> finishes_{0};
  mutable std::mutex req_mutex_;
  SharedRequest last_req_{};
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

// ═══════════════════ #956 shared-request tick ═══════════════════

// The events of one shared-request tick, in order, as "P0 P1 R0 F0 F1".
std::vector<std::vector<Event>> SplitTicks(const std::vector<Event>& events, int first_hand) {
  std::vector<std::vector<Event>> ticks;
  for (const Event& e : events) {
    if (e.what == What::kPrepare && e.hand == first_hand) {
      ticks.emplace_back();
    }
    if (!ticks.empty()) {
      ticks.back().push_back(e);
    }
  }
  return ticks;
}

bool Is(const Event& e, int hand, What what) {
  return e.hand == hand && e.what == what;
}

// D82: Prepare for every hand in Add() order, then ONE REQ from the first
// hand, then Finish for every hand; the REQ's result reaches every Finish.
TEST(HandCommLoopSharedReq, PrepareEachThenOneReqThenFinishEach) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  ASSERT_TRUE(loop.SetSharedReq(true));
  left.Writes(true);  // one hand's write grows every hand's bound (D83)
  rtc::HandTimingBuffer timing;
  loop.SetTimingProducer(&timing);

  loop.Start(LoopConfig());
  EXPECT_FALSE(loop.SetSharedReq(false)) << "changed under a running loop";
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Finishes() >= 10; }));
  loop.Join();

  const auto events = log.Snapshot();
  EXPECT_EQ(CountOf(events, 0, What::kRun) + CountOf(events, 1, What::kRun), 0u);
  const auto ticks = SplitTicks(events, 0);
  ASSERT_GE(ticks.size(), 10u);
  for (std::size_t t = 0; t < ticks.size(); ++t) {
    const auto& k = ticks[t];
    ASSERT_EQ(k.size(), 5u) << "tick " << t;
    EXPECT_TRUE(Is(k[0], 0, What::kPrepare));
    EXPECT_TRUE(Is(k[1], 1, What::kPrepare));
    EXPECT_TRUE(Is(k[2], 0, What::kReq)) << "the first hand sends the one REQ";
    EXPECT_TRUE(Is(k[3], 0, What::kFinish));
    EXPECT_TRUE(Is(k[4], 1, What::kFinish));
  }
  const SharedRequest req = left.LastReq();
  EXPECT_TRUE(req.sent);
  EXPECT_TRUE(req.after_write);
  EXPECT_NE(req.t_sent, udp_hand_driver::HandReadOutcome::Clock::time_point{});
  EXPECT_EQ(DrainRows(timing), loop.TickCount());
}

// A failed REQ is handed to every Finish (each counts its set error — S4).
TEST(HandCommLoopSharedReq, AFailedReqReachesEveryFinish) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  ASSERT_TRUE(loop.SetSharedReq(true));
  right.ReqSends(false);
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Finishes() >= 3; }));
  loop.Join();
  EXPECT_FALSE(right.LastReq().sent);
  EXPECT_FALSE(left.LastReq().sent);
  EXPECT_FALSE(left.LastReq().after_write);
}

// A hand that does not take part (fake model, decimated skip — S6) gets no
// Finish; the REQ comes from the first hand that does.
TEST(HandCommLoopSharedReq, TheFirstTakingPartHandSendsTheReq) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  ASSERT_TRUE(loop.SetSharedReq(true));
  right.TakesPart(false);
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Finishes() >= 5; }));
  loop.Join();

  const auto events = log.Snapshot();
  EXPECT_EQ(CountOf(events, 0, What::kFinish), 0u);
  EXPECT_EQ(CountOf(events, 0, What::kReq), 0u);
  for (const auto& k : SplitTicks(events, 0)) {
    ASSERT_EQ(k.size(), 4u);
    EXPECT_TRUE(Is(k[2], 1, What::kReq));
    EXPECT_TRUE(Is(k[3], 1, What::kFinish));
  }
}

// Nobody takes part: no REQ goes out at all.
TEST(HandCommLoopSharedReq, NoHandTakingPartSendsNoReq) {
  EventLog log;
  HandCommLoop loop;
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  ASSERT_TRUE(loop.Add(&right));
  ASSERT_TRUE(loop.Add(&left));
  ASSERT_TRUE(loop.SetSharedReq(true));
  right.TakesPart(false);
  left.TakesPart(false);
  loop.Start(LoopConfig());
  ASSERT_TRUE(rtc::testing::WaitUntil([&] { return left.Prepares() >= 5; }));
  loop.Join();
  const auto events = log.Snapshot();
  EXPECT_EQ(CountOf(events, 0, What::kReq) + CountOf(events, 1, What::kReq), 0u);
  EXPECT_EQ(CountOf(events, 0, What::kFinish) + CountOf(events, 1, What::kFinish), 0u);
}

// The abort sequence ends a shared-request tick like a per-hand one: every
// hand stops, then each zero-writes once, and nothing transacts after.
void ExpectOneAbortSequence(const std::vector<Event>& events) {
  for (const int hand : {0, 1}) {
    EXPECT_EQ(CountOf(events, hand, What::kStop), 1u) << "hand " << hand;
    EXPECT_EQ(CountOf(events, hand, What::kAbort), 1u) << "hand " << hand;
  }
  ASSERT_GE(events.size(), 4u);
  const auto n = events.size();
  EXPECT_EQ(events[n - 4].what, What::kStop);
  EXPECT_EQ(events[n - 3].what, What::kStop);
  EXPECT_TRUE(Is(events[n - 2], 0, What::kAbort));
  EXPECT_TRUE(Is(events[n - 1], 1, What::kAbort));
}

struct AbortRun {
  int right_prepares, left_prepares, reqs, right_finishes, left_finishes;
};

// Runs a two-hand shared-request loop until `raiser` raises the latch on the
// third call of `step`; returns what each hand did.
AbortRun RunUntilLatch(int raiser, What step, std::vector<Event>& events) {
  EventLog log;
  HandCommLoop loop;
  std::atomic<bool> latch{false};
  ScriptedHand right(0, log, loop);
  ScriptedHand left(1, log, loop);
  (raiser == 0 ? right : left).RaiseLatchOnStep(&latch, step, 3);
  right.CheckLatchAtHead(&latch);
  left.CheckLatchAtHead(&latch);
  EXPECT_TRUE(loop.Add(&right));
  EXPECT_TRUE(loop.Add(&left));
  EXPECT_TRUE(loop.SetSharedReq(true));
  loop.SetEstopFlag(&latch);
  loop.Start(LoopConfig());
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return !loop.Running(); }))
      << "the loop did not self-exit on the E-Stop";
  loop.Join();
  events = log.Snapshot();
  return {right.Prepares(), left.Prepares(), static_cast<int>(CountOf(events, 0, What::kReq)),
          right.Finishes(), left.Finishes()};
}

// S7 (a): the latch rises in hand 0's Prepare — the check before hand 1's
// Prepare aborts: no REQ, no Finish that tick.
TEST(HandCommLoopSharedReq, LatchInTheFirstPrepareStopsBeforeTheReq) {
  std::vector<Event> events;
  const AbortRun r = RunUntilLatch(0, What::kPrepare, events);
  EXPECT_EQ(r.right_prepares, 3);
  EXPECT_EQ(r.left_prepares, 2);
  EXPECT_EQ(r.reqs, 2);
  EXPECT_EQ(r.right_finishes, 2);
  EXPECT_EQ(r.left_finishes, 2);
  ExpectOneAbortSequence(events);
}

// S7 (b): the latch rises in the last Prepare — the re-check right before the
// REQ aborts, so no hand is made to burst.
TEST(HandCommLoopSharedReq, LatchAfterThePreparesStopsBeforeTheReq) {
  std::vector<Event> events;
  const AbortRun r = RunUntilLatch(1, What::kPrepare, events);
  EXPECT_EQ(r.right_prepares, 3);
  EXPECT_EQ(r.left_prepares, 3);
  EXPECT_EQ(r.reqs, 2) << "the REQ went out after the latch rose";
  EXPECT_EQ(r.right_finishes, 2);
  EXPECT_EQ(r.left_finishes, 2);
  ExpectOneAbortSequence(events);
}

// S7 (c): the latch rises in hand 0's Finish — hand 1's Finish is skipped.
TEST(HandCommLoopSharedReq, LatchInAFinishSkipsTheRemainingFinishes) {
  std::vector<Event> events;
  const AbortRun r = RunUntilLatch(0, What::kFinish, events);
  EXPECT_EQ(r.reqs, 3);
  EXPECT_EQ(r.right_finishes, 3);
  EXPECT_EQ(r.left_finishes, 2);
  ExpectOneAbortSequence(events);
}

}  // namespace
