/// @file test_solve_trigger_channel.cpp
/// @brief The trigger channel between a consumer tick and an MPC thread
///        (solve_trigger_channel.hpp): the ids and fds in both directions,
///        arming, coalescing, cancelling a wait from the other side, and the
///        consumer-side calls allocating nothing (malloc gate: this header is
///        included in exactly one translation unit per binary).

#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/thread/solve_trigger_channel.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <thread>

namespace {

using rtc::mpc::SolveTriggerChannel;
using Status = SolveTriggerChannel::WaitStatus;
using namespace std::chrono_literals;

constexpr std::int64_t kShort = 20'000'000;  // 20 ms
constexpr std::int64_t kLong = 2'000'000'000;

TEST(SolveTriggerChannel, OpensDisarmedAndACloseDisarms) {
  SolveTriggerChannel ch;
  EXPECT_FALSE(ch.Opened());
  EXPECT_FALSE(ch.Armed());
  EXPECT_EQ(ch.WaitIdle(kShort), Status::kCancelled) << "a closed channel waits for nothing";
  ASSERT_TRUE(ch.Open());
  EXPECT_TRUE(ch.Opened());
  EXPECT_FALSE(ch.Armed()) << "open is not armed";
  ch.Arm();
  EXPECT_TRUE(ch.Armed());
  ch.Close();
  EXPECT_FALSE(ch.Opened());
  EXPECT_FALSE(ch.Armed());
}

// A request while disarmed is ignored and counted; armed, it moves the
// requested id and the thread side sees it.
TEST(SolveTriggerChannel, ARequestNeedsTheChannelArmed) {
  SolveTriggerChannel ch;
  ASSERT_TRUE(ch.Open());
  ch.RequestSolve();
  EXPECT_EQ(ch.IgnoredRequests(), 1U);
  EXPECT_EQ(ch.RequestedId(), 0U);
  EXPECT_FALSE(ch.WaitTrigger(0));

  ch.Arm();
  ch.RequestSolve();
  EXPECT_EQ(ch.RequestedId(), 1U);
  EXPECT_FALSE(ch.Idle());
  EXPECT_TRUE(ch.WaitTrigger(0));
  EXPECT_EQ(ch.IgnoredRequests(), 1U);
}

// The round trip: request → the thread side wakes → completes → the
// consumer's wait returns idle; without the completion it times out.
TEST(SolveTriggerChannel, ACompletedRequestEndsTheWaitAndAnUnansweredOneTimesOut) {
  SolveTriggerChannel ch;
  ASSERT_TRUE(ch.Open());
  ch.Arm();
  EXPECT_EQ(ch.WaitIdle(kShort), Status::kIdle) << "nothing requested";
  ch.RequestSolve();
  EXPECT_EQ(ch.WaitIdle(kShort), Status::kTimeout);
  ASSERT_TRUE(ch.WaitTrigger(0));
  ch.MarkCompleted(ch.RequestedId());
  EXPECT_EQ(ch.WaitIdle(kShort), Status::kIdle);
  EXPECT_EQ(ch.CompletedId(), 1U);
  EXPECT_FALSE(ch.WaitTrigger(0)) << "answered: nothing outstanding";
}

// Requests behind an outstanding one coalesce: the thread side wakes once
// and answers the latest id with one completion.
TEST(SolveTriggerChannel, RequestsBehindAnOutstandingOneCoalesce) {
  SolveTriggerChannel ch;
  ASSERT_TRUE(ch.Open());
  ch.Arm();
  ch.RequestSolve();
  ch.RequestSolve();
  ch.RequestSolve();
  EXPECT_EQ(ch.RequestedId(), 3U);
  EXPECT_EQ(ch.CoalescedRequests(), 2U);
  ASSERT_TRUE(ch.WaitTrigger(0));
  ch.MarkCompleted(ch.RequestedId());
  EXPECT_EQ(ch.WaitIdle(kShort), Status::kIdle);
  EXPECT_FALSE(ch.WaitTrigger(0)) << "one drain answered all three";
}

// A wait on either side ends when the other side wakes it: the thread's wait
// on a request, the consumer's wait on a completion — each from another
// thread, well within the timeout.
TEST(SolveTriggerChannel, AWakeFromTheOtherSideEndsTheWait) {
  SolveTriggerChannel ch;
  ASSERT_TRUE(ch.Open());
  ch.Arm();
  std::thread solver([&ch] {
    EXPECT_TRUE(ch.WaitTrigger(2000));
    ch.MarkCompleted(ch.RequestedId());
  });
  std::this_thread::sleep_for(20ms);
  const auto t0 = std::chrono::steady_clock::now();
  ch.RequestSolve();
  EXPECT_EQ(ch.WaitIdle(kLong), Status::kIdle);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
  solver.join();
}

// Disarming cancels a consumer wait in progress and wakes a thread side wait
// without a request.
TEST(SolveTriggerChannel, DisarmingCancelsAWaitFromAnotherThread) {
  SolveTriggerChannel ch;
  ASSERT_TRUE(ch.Open());
  ch.Arm();
  ch.RequestSolve();
  std::thread disarmer([&ch] {
    std::this_thread::sleep_for(20ms);
    ch.Disarm();
  });
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(ch.WaitIdle(kLong), Status::kCancelled);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
  disarmer.join();
  EXPECT_FALSE(ch.WaitTrigger(0)) << "disarmed: the thread side sees no request";

  // Re-arming answers whatever was outstanding.
  ch.Arm();
  EXPECT_TRUE(ch.Idle());
  EXPECT_EQ(ch.WaitIdle(kShort), Status::kIdle);
}

// A nudge wakes a thread side wait without making a request.
TEST(SolveTriggerChannel, ANudgeWakesTheSolverSideWithoutARequest) {
  SolveTriggerChannel ch;
  ASSERT_TRUE(ch.Open());
  ch.Arm();
  std::thread nudger([&ch] {
    std::this_thread::sleep_for(20ms);
    ch.Nudge();
  });
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(ch.WaitTrigger(2000));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
  nudger.join();
}

// The consumer-side and solver-side calls of a tick allocate nothing: the
// request, the wait (idle and timed out), the trigger wait and the completion.
TEST(SolveTriggerChannel, TheTickCallsAllocateNothing) {
  SolveTriggerChannel ch;
  ASSERT_TRUE(ch.Open());
  ch.Arm();
  // A first round outside the gate: lazy statics (chrono, poll) settle.
  ch.RequestSolve();
  static_cast<void>(ch.WaitTrigger(0));
  ch.MarkCompleted(ch.RequestedId());
  static_cast<void>(ch.WaitIdle(kShort));
  {
    rtc::testing::ScopedMallocGate gate;
    ch.RequestSolve();
    EXPECT_EQ(ch.WaitIdle(1'000'000), Status::kTimeout);
    EXPECT_TRUE(ch.WaitTrigger(0));
    ch.MarkCompleted(ch.RequestedId());
    EXPECT_EQ(ch.WaitIdle(kShort), Status::kIdle);
    EXPECT_EQ(gate.count(), 0U);
  }
}

}  // namespace
