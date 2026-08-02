// ── Contract tests for the shared sleep-poll wait (rtc_base/testing) ─────────
// WaitUntil is consumed by four packages' test suites, and every one of those
// call sites only ever asserts the happy path (EXPECT_TRUE / ASSERT_TRUE). That
// leaves the properties the helper actually promises — the poll budget, the
// predicate-before-deadline ordering, and not copying the predicate — pinned by
// nothing: each could be mutated away with the whole 3591-test suite still
// green. This file is the sensor for those promises.
//
// Timing assertions here are deliberately ONE-SIDED. A loaded host makes sleeps
// overshoot, which yields FEWER polls and LONGER elapsed times, never the
// reverse — so upper bounds on the poll count and lower bounds on "did it
// sleep" stay deterministic under CI load, while their opposites would flake.
//
// NOT covered, and deliberately so: the `>=` vs `>` spelling of the deadline
// comparison. Measured — mutating it to `>` leaves all nine tests below green.
// The two differ only when steady_clock reads EXACTLY the deadline, and since
// the clock is sampled after pred() returns, that tick is never observed. Any
// test claiming to pin it would be pinning something else. What matters about
// that line is that an expired budget terminates the loop without sleeping,
// which SpentBudgetReturnsWithoutSleeping does pin; making the boundary itself
// testable would mean templating the helper on a clock, which is a large
// contract change for a sleep-only test utility. Kill results for the three
// mutations that ARE covered are named on the tests below.
#include "rtc_base/testing/wait_until.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace {

using namespace std::chrono_literals;
using rtc::testing::WaitUntil;

// A predicate that is an object, not a lambda, so the test can inspect the
// mutations it made after the wait returns.
struct CountingPredicate {
  int calls = 0;
  int flip_after = -1;  // negative ⇒ never true

  bool operator()() {
    ++calls;
    return flip_after >= 0 && calls >= flip_after;
  }
};

// ── The `poll` parameter ────────────────────────────────────────────────────

// Mutation killed: deleting the `poll` parameter and hardcoding the old 1 ms.
// With a 50 ms poll the predicate can be consulted only a handful of times
// inside a 250 ms budget; at 1 ms it would be consulted ~250 times. The bound
// is loose enough that only a genuine reversion trips it.
TEST(WaitUntil, PollIntervalGovernsHowOftenPredicateRuns) {
  CountingPredicate pred{};  // never true → runs the full budget
  EXPECT_FALSE(WaitUntil(pred, 250ms, 50ms));
  EXPECT_GE(pred.calls, 1);
  EXPECT_LE(pred.calls, 20) << "poll interval ignored — predicate ran " << pred.calls
                            << " times in 250 ms, which is the 1 ms cadence, not 50 ms";
}

// The documented defaults exist and are usable, so a caller may omit both.
TEST(WaitUntil, DefaultsCoverTimeoutAndPoll) {
  CountingPredicate pred{.calls = 0, .flip_after = 1};
  EXPECT_TRUE(WaitUntil(pred));
  EXPECT_EQ(pred.calls, 1);
}

// ── Predicate-before-deadline ordering ──────────────────────────────────────

// Mutation killed: consulting the deadline first (the semantics rtc_communication
// was migrated OFF). With a spent budget and a condition that already holds, a
// deadline-first loop reports a timeout for something that is, in fact, true.
TEST(WaitUntil, SpentBudgetStillReportsAnAlreadySatisfiedPredicate) {
  CountingPredicate pred{.calls = 0, .flip_after = 1};
  EXPECT_TRUE(WaitUntil(pred, 0ms, 50ms))
      << "predicate was true on entry but the expired budget won — deadline is "
         "being consulted before the predicate";
  EXPECT_EQ(pred.calls, 1);
}

// The other half of the ordering: an expired budget with a false predicate
// returns at once and does NOT sleep first. The 500 ms poll is far larger than
// the 150 ms bound, so an implementation that sleeps before testing the
// deadline cannot pass this no matter how lightly the host is loaded.
TEST(WaitUntil, SpentBudgetReturnsWithoutSleeping) {
  CountingPredicate pred{};
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(WaitUntil(pred, 0ms, 500ms));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(pred.calls, 1) << "expired budget should cost exactly one predicate evaluation";
  EXPECT_LT(elapsed, 150ms) << "slept a poll interval before noticing the budget was spent";
}

// ── Predicate is not copied ─────────────────────────────────────────────────

// Mutation killed: taking `Pred` by value. The helper would then poll a copy,
// so the caller's counter would still read 0 here and any stateful predicate
// (call counter, captured latch) would silently lose its mutations.
TEST(WaitUntil, StatefulPredicateMutationsAreVisibleToTheCaller) {
  CountingPredicate pred{.calls = 0, .flip_after = 3};
  EXPECT_TRUE(WaitUntil(pred, 2s, 1ms));
  EXPECT_EQ(pred.calls, 3) << "predicate was polled by copy — the caller's state never advanced";
}

// `mutable` lambdas must remain callable. This also pins the signature against
// `const Pred&`, the other way to avoid the copy: that spelling would not
// compile here, so this test failing to build IS the failure signal.
TEST(WaitUntil, AcceptsMutableLambda) {
  int calls = 0;
  auto pred = [n = 0]() mutable { return ++n >= 2; };
  EXPECT_TRUE(WaitUntil(pred, 2s, 1ms));
  EXPECT_EQ(calls, 0);  // untouched; present so the capture above stays meaningful
}

// A predicate passed as a temporary must work too (the common call shape at
// every existing call site is an inline lambda).
TEST(WaitUntil, AcceptsRvaluePredicate) {
  int calls = 0;
  EXPECT_TRUE(WaitUntil([&] { return ++calls >= 2; }, 2s, 1ms));
  EXPECT_EQ(calls, 2);
}

// ── Return value ────────────────────────────────────────────────────────────

TEST(WaitUntil, ReturnsTrueOncePredicateFlips) {
  CountingPredicate pred{.calls = 0, .flip_after = 4};
  EXPECT_TRUE(WaitUntil(pred, 2s, 1ms));
  EXPECT_EQ(pred.calls, 4);
}

// A never-true predicate must terminate on the budget rather than hang, and
// must have actually waited roughly that long (one-sided: at least the budget).
TEST(WaitUntil, ReturnsFalseAfterExhaustingTheBudget) {
  CountingPredicate pred{};
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(WaitUntil(pred, 120ms, 10ms));
  EXPECT_GE(std::chrono::steady_clock::now() - start, 100ms)
      << "returned before the budget was spent";
}

}  // namespace
