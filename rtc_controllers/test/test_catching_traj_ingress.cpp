// ── L1 ingress judgement (S5.2) ─────────────────────────────────────────────
//
// What this suite pins, and why each case exists rather than being implied by
// the one above it:
//
//   1. Order and validity (L1 §4.4, A-S5-4). The sequence expectation must
//      survive a duplicate and must NOT survive a track change — the second
//      half is the one that fails silently, as blindness after a vision
//      restart rather than as an error.
//   2. The two time axes (plan §3). Staleness is judged on the receive axis
//      and horizon exhaustion on the lead axis, and every case here uses
//      T_arm != 0 so the two cannot be confused: at T_arm = 0 they coincide
//      and a test that mixed them would still pass.
//   3. The activation gate (D-23). A snapshot received while the controller
//      was inactive is refused on the first tick after re-activation.
//   4. The jump diagnostic evaluates both predictions at ONE instant, which is
//      what separates "the two predictions disagree" from "the ball moved".

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

namespace {

using rtc::catching::CheckOrder;
using rtc::catching::HorizonShort;
using rtc::catching::IngressReject;
using rtc::catching::IngressState;
using rtc::catching::JumpBetween;
using rtc::catching::NowLead;
using rtc::catching::NowReal;
using rtc::catching::ReadTraj;
using rtc::catching::TrajectorySnapshot;

constexpr std::int64_t kMs = 1'000'000;
/// Deliberately non-zero everywhere in this suite: see the header note.
constexpr std::int64_t kTArmNs = 50 * kMs;
constexpr std::int64_t kStaleNs = 100 * kMs;
constexpr std::uint64_t kActivation = 7;

/// A well-formed snapshot: `n` samples 50 ms apart starting at `t0`, received
/// at `recv`, moving along +x at 1 m/s so a sampled position is a function of
/// the instant and a mis-sampled one is visible.
TrajectorySnapshot MakeSnapshot(std::int64_t recv, std::int64_t t0, int n = 8,
                                std::uint64_t sequence = 1, std::uint64_t generation = 100,
                                double x_offset = 0.0) {
  TrajectorySnapshot snap{};
  snap.valid = true;
  snap.n = n;
  snap.token.activation_generation = kActivation;
  snap.token.generation = generation;
  snap.token.snapshot_sequence = sequence;
  snap.token.traj_recv_ns = recv;
  for (int i = 0; i < n; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    snap.s[idx].t_ns = t0 + static_cast<std::int64_t>(i) * 50 * kMs;
    const double seconds = static_cast<double>(snap.s[idx].t_ns) * 1e-9;
    snap.s[idx].p = {seconds + x_offset, 0.0, 0.0};
    snap.s[idx].v = {1.0, 0.0, 0.0};
    snap.s[idx].a = {0.0, 0.0, 0.0};
  }
  return snap;
}

// ── 1. Order and validity ───────────────────────────────────────────────────

TEST(CatchingIngressOrder, AcceptsTheFirstMessageAndThenOnlyIncreasingSequences) {
  IngressState state{};
  EXPECT_EQ(CheckOrder({100, 5, true}, state), IngressReject::kNone);
  EXPECT_EQ(CheckOrder({100, 6, true}, state), IngressReject::kNone);
  // Duplicate and overtaken-in-flight are the same verdict: an older
  // prediction must not overwrite a newer one.
  EXPECT_EQ(CheckOrder({100, 6, true}, state), IngressReject::kStaleSequence);
  EXPECT_EQ(CheckOrder({100, 4, true}, state), IngressReject::kStaleSequence);
  EXPECT_EQ(state.last_sequence, 6U) << "a rejected message updated the expectation";
}

TEST(CatchingIngressOrder, ATrackChangeResetsTheSequenceExpectation) {
  // A-S5-4, and the case that fails SILENTLY without the rule: a vision
  // restart numbers its new track from 1, and an expectation carried across
  // the change would refuse every message until the new track climbed past the
  // old high-water mark — blindness with no error anywhere.
  IngressState state{};
  ASSERT_EQ(CheckOrder({100, 9000, true}, state), IngressReject::kNone);

  EXPECT_EQ(CheckOrder({101, 1, true}, state), IngressReject::kNone)
      << "a new track was refused for carrying a lower sequence";
  EXPECT_EQ(state.last_generation, 101U);
  EXPECT_EQ(state.last_sequence, 1U);
  // And the expectation is now the NEW track's: the old one's numbers no
  // longer authorise anything.
  EXPECT_EQ(CheckOrder({101, 1, true}, state), IngressReject::kStaleSequence);
}

TEST(CatchingIngressOrder, AnInvalidMessageIsRefusedAndRecordsNothing) {
  // C-1 (one invalid point refuses the message) plus the ordering of the two
  // checks: a message we have decided not to believe must not move the
  // sequence expectation, or the NEXT good message inherits its number.
  IngressState state{};
  ASSERT_EQ(CheckOrder({100, 5, true}, state), IngressReject::kNone);

  EXPECT_EQ(CheckOrder({100, 6, false}, state), IngressReject::kNotEvaluated);
  EXPECT_EQ(state.last_sequence, 5U);
  EXPECT_EQ(CheckOrder({100, 6, true}, state), IngressReject::kNone)
      << "the refused message's sequence was recorded and blocked the good one";
}

// ── 2. The two axes ─────────────────────────────────────────────────────────

TEST(CatchingIngressRead, FreshSnapshotIsUsableAndCountsAsNewExactlyOnce) {
  const TrajectorySnapshot snap = MakeSnapshot(/*recv=*/1000 * kMs, /*t0=*/1000 * kMs);
  std::uint64_t last_seq = 0;
  const NowReal now{1010 * kMs};
  const NowLead lead{now.ns + kTArmNs};

  const auto first = ReadTraj(snap, now, lead, kStaleNs, kActivation, last_seq);
  EXPECT_FALSE(first.stale);
  EXPECT_FALSE(first.expired);
  EXPECT_TRUE(first.is_new);
  EXPECT_EQ(first.age_ns, 10 * kMs);

  // Same snapshot, next tick: still usable, no longer new. `is_new` drives
  // work that must happen once per snapshot (planner wake, hint reset), so a
  // second true here would repeat it every tick.
  const auto second =
      ReadTraj(snap, NowReal{now.ns + 2 * kMs}, lead, kStaleNs, kActivation, last_seq);
  EXPECT_FALSE(second.stale);
  EXPECT_FALSE(second.is_new);
}

TEST(CatchingIngressRead, StalenessIsJudgedOnTheRECEIVEAxis) {
  // The snapshot's PREDICTION still covers the present — only its arrival is
  // old. A staleness check that looked at the sample instants (or at a header
  // stamp) would call this fresh, which is the failure the repo's clock rule
  // exists to prevent: vision stopped publishing and nobody noticed.
  const TrajectorySnapshot snap = MakeSnapshot(/*recv=*/1000 * kMs, /*t0=*/1000 * kMs);
  std::uint64_t last_seq = 0;

  const NowReal now{1000 * kMs + kStaleNs + 1};
  const auto view = ReadTraj(snap, now, NowLead{now.ns + kTArmNs}, kStaleNs, kActivation, last_seq);
  EXPECT_TRUE(view.stale);
  EXPECT_GT(view.age_ns, kStaleNs);
  EXPECT_FALSE(view.expired) << "the prediction window itself had not run out";
}

TEST(CatchingIngressRead, HorizonExhaustionIsJudgedOnTheLEADAxis) {
  // The T_arm != 0 case that the two axes exist for. The real instant is still
  // inside the window; the instant this tick's command REALISES is not. A
  // check on the real axis would let the controller plan against a window it
  // will have flown out of by the time the arm gets there.
  // Received recently (age 40 ms, well inside the staleness budget) so that
  // `expired` is the only thing this case can be measuring.
  const TrajectorySnapshot snap = MakeSnapshot(/*recv=*/1300 * kMs, /*t0=*/1000 * kMs, /*n=*/8);
  const std::int64_t last_sample = snap.s[7].t_ns;  // t0 + 350 ms
  std::uint64_t last_seq = 0;

  const NowReal now{last_sample - 10 * kMs};
  const auto view = ReadTraj(snap, now, NowLead{now.ns + kTArmNs}, kStaleNs, kActivation, last_seq);
  EXPECT_FALSE(view.stale);
  EXPECT_TRUE(view.expired);

  // Same snapshot judged on the real axis alone would NOT be expired — that is
  // the mistake being excluded, stated as an assertion rather than a comment.
  const auto on_real_axis = ReadTraj(snap, now, NowLead{now.ns}, kStaleNs, kActivation, last_seq);
  EXPECT_FALSE(on_real_axis.expired);
}

TEST(CatchingIngressRead, AnUnfilledOrNonPositiveThresholdFailsClosed) {
  std::uint64_t last_seq = 0;
  const NowReal now{1000 * kMs};

  // Never filled: valid=false, and a zero receive instant means "no claim".
  const TrajectorySnapshot empty{};
  EXPECT_TRUE(
      ReadTraj(empty, now, NowLead{now.ns + kTArmNs}, kStaleNs, kActivation, last_seq).stale);

  // A misconfigured threshold withholds the lane rather than disabling the
  // check — "no sample can satisfy this" is the safe reading of <= 0.
  const TrajectorySnapshot snap = MakeSnapshot(now.ns, now.ns);
  EXPECT_TRUE(
      ReadTraj(snap, now, NowLead{now.ns + kTArmNs}, /*t_stale_ns=*/0, kActivation, last_seq)
          .stale);
}

// ── 3. The activation gate (D-23) ───────────────────────────────────────────

TEST(CatchingIngressRead, ASnapshotFromAnEarlierActivationIsRefused) {
  // The subscription outlives deactivation (lifecycle gates publishers, not
  // subscriptions), so without this the first tick after re-activation
  // consumes a trajectory received while another controller had the arm.
  TrajectorySnapshot snap = MakeSnapshot(/*recv=*/1000 * kMs, /*t0=*/1000 * kMs);
  snap.token.activation_generation = kActivation - 1;
  std::uint64_t last_seq = 0;

  const NowReal now{1001 * kMs};
  const auto view = ReadTraj(snap, now, NowLead{now.ns + kTArmNs}, kStaleNs, kActivation, last_seq);
  EXPECT_TRUE(view.stale) << "a trajectory from the previous activation was accepted";
  // Freshness alone would have passed it — the age is 1 ms.
  EXPECT_LT(view.age_ns, kStaleNs);
}

// ── 4. Horizon requirement and the jump diagnostic ──────────────────────────

TEST(CatchingIngressHorizon, ShortWindowsAreDiagnosedNotRejected) {
  // 8 points 50 ms apart span 350 ms.
  const TrajectorySnapshot snap = MakeSnapshot(1000 * kMs, 1000 * kMs, /*n=*/8);
  EXPECT_FALSE(HorizonShort(snap, 300 * kMs));
  EXPECT_TRUE(HorizonShort(snap, 400 * kMs));
  // A single point has no window at all, which is short by any requirement.
  EXPECT_TRUE(HorizonShort(MakeSnapshot(1000 * kMs, 1000 * kMs, /*n=*/1), 1));
}

TEST(CatchingIngressJump, ComparesTwoPredictionsAtOneInstant) {
  // Two predictions of the same track, the second offset by 0.25 m in x. The
  // jump is that offset and nothing else — in particular NOT the distance the
  // ball travelled between the two messages, which is what a comparison at
  // each snapshot's own first sample would have measured (the second snapshot
  // starts 100 ms later, i.e. 0.1 m further along at 1 m/s).
  const TrajectorySnapshot older = MakeSnapshot(1000 * kMs, 1000 * kMs, 8, 1, 100, 0.0);
  const TrajectorySnapshot newer = MakeSnapshot(1100 * kMs, 1100 * kMs, 8, 2, 100, 0.25);

  const double jump = JumpBetween(older, newer, /*eval_offset_ns=*/50 * kMs);
  ASSERT_GE(jump, 0.0);
  EXPECT_NEAR(jump, 0.25, 1e-9);
}

TEST(CatchingIngressJump, RefusesToCompareAcrossTracksOrEmptySnapshots) {
  // A negative answer, not zero: "not compared" and "no jump" must not look
  // alike to a caller deciding whether the prediction is behaving.
  const TrajectorySnapshot a = MakeSnapshot(1000 * kMs, 1000 * kMs, 8, 1, /*generation=*/100);
  const TrajectorySnapshot b = MakeSnapshot(1100 * kMs, 1100 * kMs, 8, 2, /*generation=*/101);
  EXPECT_LT(JumpBetween(a, b, 50 * kMs), 0.0);
  EXPECT_LT(JumpBetween(a, TrajectorySnapshot{}, 50 * kMs), 0.0);
}

// ── 5. The SeqLock hand-off (G1-C, G1-H) ────────────────────────────────────
//
// The trajectory crosses from the non-RT callback to the RT tick through
// `rtc::SeqLock`, and two properties of that crossing are worth pinning here
// rather than in the lock's own tests: the payload is large (a 40-sample
// snapshot), which is what makes tearing observable at all, and the "is this
// new" decision is made from INSIDE the payload (D-21) rather than from the
// lock's counter.

TEST(CatchingIngressSeqLock, AConcurrentWriterNeverProducesATornSnapshot) {
  // Every sample of a stored snapshot carries the same marker as its token, so
  // a snapshot assembled from two different stores is visible as a sample that
  // disagrees with its own token. A checksum field would do the same job; this
  // needs no change to the payload.
  rtc::SeqLock<TrajectorySnapshot> box{};
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> torn{0};
  std::atomic<std::uint64_t> reads{0};

  std::thread writer([&] {
    std::uint64_t seq = 1;
    while (!stop.load(std::memory_order_relaxed)) {
      TrajectorySnapshot snap =
          MakeSnapshot(static_cast<std::int64_t>(seq) * kMs, static_cast<std::int64_t>(seq) * kMs,
                       /*n=*/rtc::catching::kCap, seq);
      for (int i = 0; i < snap.n; ++i) {
        snap.s[static_cast<std::size_t>(i)].p[2] = static_cast<double>(seq);
      }
      box.Store(snap);
      ++seq;
    }
  });

  std::int64_t worst_load_ns = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto t0 = std::chrono::steady_clock::now();
    const TrajectorySnapshot snap = box.Load();
    const auto t1 = std::chrono::steady_clock::now();
    worst_load_ns = std::max<std::int64_t>(
        worst_load_ns, std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    reads.fetch_add(1, std::memory_order_relaxed);
    if (snap.n == 0) {
      continue;  // before the first store
    }
    const double marker = static_cast<double>(snap.token.snapshot_sequence);
    for (int i = 0; i < snap.n; ++i) {
      if (snap.s[static_cast<std::size_t>(i)].p[2] != marker) {
        torn.fetch_add(1, std::memory_order_relaxed);
        break;
      }
    }
  }
  stop.store(true, std::memory_order_relaxed);
  writer.join();

  EXPECT_EQ(torn.load(), 0U);
  EXPECT_GT(reads.load(), 1000U) << "too few reads to mean anything";
  // The retry cost itself, recorded rather than asserted: it is a property of
  // the machine and the payload size, and a threshold here would be a
  // measurement of the CI host. G1-C asks for the number, not for a bound.
  ::testing::Test::RecordProperty("worst_load_ns", static_cast<int>(worst_load_ns));
}

TEST(CatchingIngressSeqLock, EveryStoreIsSeenExactlyOnceByAReaderThatLoadsEveryTick) {
  // D-21 / G1-H. The hazard this excludes is an implementation that asks the
  // LOCK whether anything is new (`sequence()`) and only then loads: a store
  // landing between those two calls pairs a new counter with an old payload,
  // and the newest snapshot is then invisible for as long as the pattern
  // repeats. ReadTraj cannot make that mistake — it is handed the payload —
  // and this pins the consequence rather than the mechanism.
  rtc::SeqLock<TrajectorySnapshot> box{};
  std::uint64_t last_seq = 0;
  const NowReal now{10'000 * kMs};

  int seen_new = 0;
  for (std::uint64_t seq = 1; seq <= 20; ++seq) {
    box.Store(MakeSnapshot(now.ns, now.ns, /*n=*/8, seq));
    // Two ticks per store: the first must see it as new, the second must not.
    for (int tick = 0; tick < 2; ++tick) {
      const auto view =
          ReadTraj(box.Load(), now, NowLead{now.ns + kTArmNs}, kStaleNs, kActivation, last_seq);
      if (view.is_new) {
        ++seen_new;
        EXPECT_EQ(tick, 0) << "a snapshot read as new twice";
      }
    }
  }
  EXPECT_EQ(seen_new, 20) << "a stored snapshot was never seen";
}

}  // namespace
