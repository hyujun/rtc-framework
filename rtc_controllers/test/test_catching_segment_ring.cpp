// The ring of published segments both segment planners keep (E1-F16 #742,
// segment_ring.hpp). Header-only pure logic: no model, no solver.
//
// What is tested: the ring holds the segments of ONE plan (a segment of
// another plan empties it); it holds eight, and a full ring drops the oldest
// segment the RT did not last report, so the followed and the pending segment
// survive a burst of pushes; a payload moves with its segment; and the three
// questions the cycle asks — which segments the RT reports (Reported), which
// one a solve at an instant starts from (SourceSeq), whose track the plan is
// (FollowedTrack) — are answered from what the RT reports, never inferred.
//
// A segment here carries only what the ring reads: plan id, catch instant,
// seq, node-0 instant and the track generation. Its payload is an int that
// names it (10 × seq), so a payload that did not move with its segment shows.
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/segment_ring.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <type_traits>

namespace {

using rtc::catching::NoSegmentPayload;
using rtc::catching::PlannerRtState;
using rtc::catching::ReportedSegments;
using rtc::catching::SegmentRing;
using rtc::catching::SegmentSnapshot;

constexpr std::uint32_t kPlan = 4;
constexpr std::int64_t kCatch = 5'000'000'000;
constexpr std::uint64_t kTrack = 9;
constexpr std::uint32_t kFull = SegmentRing<int>::kSize;

[[nodiscard]] SegmentSnapshot Seg(std::uint32_t seq, std::int64_t t0_ns = 0,
                                  std::uint32_t plan = kPlan, std::int64_t t_c = kCatch,
                                  std::uint64_t track = kTrack) {
  SegmentSnapshot s{};
  s.valid = true;
  s.plan_id = plan;
  s.t_c_ns = t_c;
  s.segment_seq = seq;
  s.t0_ns = t0_ns;
  s.token.generation = track;
  return s;
}

[[nodiscard]] int PayloadFor(std::uint32_t seq) {
  return 10 * static_cast<int>(seq);
}

// The RT following plan (`plan`, `t_c`), reporting `following` as the followed
// segment and `pending` as the admitted one that has not started (0 = none).
[[nodiscard]] PlannerRtState Rt(std::uint32_t following, std::uint32_t pending = 0,
                                std::uint32_t plan = kPlan, std::int64_t t_c = kCatch) {
  PlannerRtState rt{};
  rt.valid = true;
  rt.plan_active = true;
  rt.plan_id = plan;
  rt.plan_t_c_ns = t_c;
  rt.segment_active = following != 0;
  rt.segment_seq = following;
  rt.segment_pending = pending != 0;
  rt.segment_pending_seq = pending;
  return rt;
}

void PushSeqs(SegmentRing<int>& ring, std::uint32_t first, std::uint32_t last) {
  for (std::uint32_t seq = first; seq <= last; ++seq) {
    ring.Push(Seg(seq), PayloadFor(seq));
  }
}

// Whether `seq` is in the ring with its own payload.
[[nodiscard]] bool HoldsWithPayload(const SegmentRing<int>& ring, std::uint32_t seq) {
  const SegmentSnapshot* s = ring.Find(seq);
  return s != nullptr && s->segment_seq == seq && ring.PayloadOf(s) == PayloadFor(seq);
}

// ── Empty ────────────────────────────────────────────────────────────────────

TEST(SegmentRingEmpty, AnEmptyRingFindsAndAnswersNothing) {
  const SegmentRing<int> ring{};
  const PlannerRtState rt = Rt(/*following=*/1, /*pending=*/2);
  EXPECT_TRUE(ring.Empty());
  EXPECT_EQ(ring.Find(0), nullptr);
  EXPECT_EQ(ring.Find(1), nullptr);
  EXPECT_FALSE(ring.IsOf(rt));
  const SegmentRing<int>::Report r = ring.ReportedIn(rt);
  EXPECT_EQ(r.pending, nullptr);
  EXPECT_EQ(r.following, nullptr);
  EXPECT_EQ(ring.SourceSeq(rt, 0), 0U);
  EXPECT_EQ(ring.SourceSeq(rt, kCatch), 0U);
  std::uint64_t generation = 77;
  EXPECT_FALSE(ring.FollowedTrack(rt, generation));
  EXPECT_EQ(generation, 77U) << "a false answer leaves the output alone";
  ReportedSegments out;
  out.has_pending = true;
  out.has_following = true;
  ring.Reported(rt, out);
  EXPECT_FALSE(out.has_pending);
  EXPECT_FALSE(out.has_following);
}

// ── Push, Find, PayloadOf, IsOf ──────────────────────────────────────────────

TEST(SegmentRingStore, SegmentsOfOnePlanAreFoundBySeqWithTheirOwnPayload) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 3);
  EXPECT_FALSE(ring.Empty());
  for (std::uint32_t seq = 1; seq <= 3; ++seq) {
    ASSERT_TRUE(HoldsWithPayload(ring, seq)) << seq;
    EXPECT_EQ(ring.Find(seq)->plan_id, kPlan);
    EXPECT_EQ(ring.Find(seq)->t_c_ns, kCatch);
  }
  EXPECT_EQ(ring.Find(0), nullptr);
  EXPECT_EQ(ring.Find(4), nullptr);
  // The entries are distinct slots.
  EXPECT_NE(ring.Find(1), ring.Find(2));
  EXPECT_NE(ring.Find(2), ring.Find(3));
}

TEST(SegmentRingStore, IsOfNeedsAnActivePlanItsIdAndItsCatchInstant) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 2);
  EXPECT_TRUE(ring.IsOf(Rt(1)));
  // Each of the three conditions alone.
  PlannerRtState inactive = Rt(1);
  inactive.plan_active = false;
  EXPECT_FALSE(ring.IsOf(inactive));
  EXPECT_FALSE(ring.IsOf(Rt(1, 0, kPlan + 1, kCatch)));
  EXPECT_FALSE(ring.IsOf(Rt(1, 0, kPlan, kCatch + 1)));
  EXPECT_FALSE(ring.IsOf(Rt(1, 0, kPlan, kCatch - 1)));
}

// ── One plan ─────────────────────────────────────────────────────────────────

TEST(SegmentRingPlan, ASegmentOfAnotherPlanIdEmptiesTheRingFirst) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 3);
  ring.Push(Seg(1, 0, kPlan + 1, kCatch), 111);
  // Nothing of the old plan survives, though seq 1 exists in both.
  EXPECT_EQ(ring.Find(2), nullptr);
  EXPECT_EQ(ring.Find(3), nullptr);
  ASSERT_NE(ring.Find(1), nullptr);
  EXPECT_EQ(ring.Find(1)->plan_id, kPlan + 1);
  EXPECT_EQ(ring.PayloadOf(ring.Find(1)), 111);
  EXPECT_FALSE(ring.IsOf(Rt(1)));
  EXPECT_TRUE(ring.IsOf(Rt(1, 0, kPlan + 1, kCatch)));
}

TEST(SegmentRingPlan, ASegmentOfTheSameIdAndAnotherCatchInstantEmptiesItToo) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 3);
  ring.Push(Seg(7, 0, kPlan, kCatch + 1'000'000), 777);
  EXPECT_EQ(ring.Find(1), nullptr);
  EXPECT_EQ(ring.Find(2), nullptr);
  EXPECT_EQ(ring.Find(3), nullptr);
  ASSERT_NE(ring.Find(7), nullptr);
  EXPECT_EQ(ring.PayloadOf(ring.Find(7)), 777);
  EXPECT_TRUE(ring.IsOf(Rt(7, 0, kPlan, kCatch + 1'000'000)));
  EXPECT_FALSE(ring.IsOf(Rt(7)));
  // The plan the ring has moved to is the one it keeps filling.
  ring.Push(Seg(8, 0, kPlan, kCatch + 1'000'000), 888);
  EXPECT_NE(ring.Find(7), nullptr);
  EXPECT_NE(ring.Find(8), nullptr);
}

// ── Eviction ─────────────────────────────────────────────────────────────────

TEST(SegmentRingEviction, WithNothingReportedTheNinthPushDropsTheOldest) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, kFull);
  for (std::uint32_t seq = 1; seq <= 8; ++seq) {
    ASSERT_NE(ring.Find(seq), nullptr) << "a full ring holds all eight: " << seq;
  }
  ring.Push(Seg(9), PayloadFor(9));
  EXPECT_EQ(ring.Find(1), nullptr);
  // The survivors moved down one slot and kept their payloads.
  for (std::uint32_t seq = 2; seq <= 9; ++seq) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
  ring.Push(Seg(10), PayloadFor(10));
  EXPECT_EQ(ring.Find(2), nullptr);
  for (std::uint32_t seq = 3; seq <= 10; ++seq) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
}

TEST(SegmentRingEviction, TheOldestSegmentTheRtDidNotReportIsTheOneDropped) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, kFull);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(Seg(9), PayloadFor(9));
  // 1 and 2 are the oldest but are reported: the third is the oldest that is
  // not.
  EXPECT_EQ(ring.Find(3), nullptr);
  for (const std::uint32_t seq : {1U, 2U, 4U, 5U, 6U, 7U, 8U, 9U}) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
}

TEST(SegmentRingEviction, AReportedSegmentSurvivesABurstOfPushes) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, kFull);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  PushSeqs(ring, 9, 29);
  // The two reported ones and the six newest.
  for (const std::uint32_t seq : {1U, 2U, 24U, 25U, 26U, 27U, 28U, 29U}) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
  for (std::uint32_t seq = 3; seq <= 23; ++seq) {
    EXPECT_EQ(ring.Find(seq), nullptr) << seq;
  }
}

TEST(SegmentRingEviction, OnlyTheFollowedOneReportedProtectsOnlyIt) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, kFull);
  // A pending report names seq 2 only when the RT says it has one.
  PlannerRtState rt = Rt(/*following=*/1);
  rt.segment_pending = false;
  rt.segment_pending_seq = 2;  // stale field: not reported
  ring.NoteReported(rt);
  ring.Push(Seg(9), PayloadFor(9));
  EXPECT_NE(ring.Find(1), nullptr);
  EXPECT_EQ(ring.Find(2), nullptr);
  // And an inactive segment's seq protects nothing.
  SegmentRing<int> other;
  PushSeqs(other, 1, kFull);
  PlannerRtState idle = Rt(/*following=*/1);
  idle.segment_active = false;
  other.NoteReported(idle);
  other.Push(Seg(9), PayloadFor(9));
  EXPECT_EQ(other.Find(1), nullptr);
}

TEST(SegmentRingEviction, ClearReportedMakesTheOldestEvictableAgain) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, kFull);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.ClearReported();
  ring.Push(Seg(9), PayloadFor(9));
  EXPECT_EQ(ring.Find(1), nullptr);
  EXPECT_NE(ring.Find(2), nullptr);
}

TEST(SegmentRingEviction, ClearForgetsEverythingTheSegmentsAndWhatWasReported) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, kFull);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Clear();
  EXPECT_TRUE(ring.Empty());
  for (std::uint32_t seq = 1; seq <= 9; ++seq) {
    EXPECT_EQ(ring.Find(seq), nullptr) << seq;
  }
  EXPECT_FALSE(ring.IsOf(Rt(1)));
  std::uint64_t generation = 0;
  EXPECT_FALSE(ring.FollowedTrack(Rt(1), generation));
  // The ring is usable again, and the report it was holding is gone: seq 1 is
  // the oldest, not protected.
  PushSeqs(ring, 1, kFull + 1);
  EXPECT_EQ(ring.Find(1), nullptr);
  EXPECT_TRUE(HoldsWithPayload(ring, 2));
}

// ── What the RT reports ──────────────────────────────────────────────────────

TEST(SegmentRingReport, ReportedInPointsAtTheEntriesTheRtNames) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 3);
  const SegmentRing<int>::Report both = ring.ReportedIn(Rt(/*following=*/1, /*pending=*/2));
  EXPECT_EQ(both.following, ring.Find(1));
  EXPECT_EQ(both.pending, ring.Find(2));
  const SegmentRing<int>::Report only_following = ring.ReportedIn(Rt(1));
  EXPECT_EQ(only_following.following, ring.Find(1));
  EXPECT_EQ(only_following.pending, nullptr);
  // A seq the ring does not hold is not found, the other side still is.
  const SegmentRing<int>::Report unknown = ring.ReportedIn(Rt(/*following=*/1, /*pending=*/9));
  EXPECT_EQ(unknown.following, ring.Find(1));
  EXPECT_EQ(unknown.pending, nullptr);
  // The RT reporting a plan the ring is not of finds nothing in it.
  const SegmentRing<int>::Report other_plan =
      ring.ReportedIn(Rt(/*following=*/1, /*pending=*/2, kPlan + 1));
  EXPECT_EQ(other_plan.following, nullptr);
  EXPECT_EQ(other_plan.pending, nullptr);
  const SegmentRing<int>::Report other_catch =
      ring.ReportedIn(Rt(/*following=*/1, /*pending=*/2, kPlan, kCatch + 1));
  EXPECT_EQ(other_catch.following, nullptr);
  EXPECT_EQ(other_catch.pending, nullptr);
}

TEST(SegmentRingReport, ReportedCopiesOutExactlyWhatReportedInPointsAt) {
  SegmentRing<int> ring;
  for (std::uint32_t seq = 1; seq <= 3; ++seq) {
    SegmentSnapshot s = Seg(seq, /*t0_ns=*/100 * seq);
    s.q[0] = 0.5 * seq;
    s.qd[7] = -0.25 * seq;
    ring.Push(s, PayloadFor(seq));
  }
  const PlannerRtState rt = Rt(/*following=*/1, /*pending=*/3);
  ReportedSegments out;
  ring.Reported(rt, out);
  ASSERT_TRUE(out.has_pending);
  ASSERT_TRUE(out.has_following);
  EXPECT_EQ(out.pending.segment_seq, 3U);
  EXPECT_EQ(out.pending.t0_ns, 300);
  EXPECT_EQ(out.pending.q[0], 1.5);
  EXPECT_EQ(out.pending.qd[7], -0.75);
  EXPECT_EQ(out.following.segment_seq, 1U);
  EXPECT_EQ(out.following.t0_ns, 100);
  EXPECT_EQ(out.following.q[0], 0.5);
  EXPECT_EQ(out.following.qd[7], -0.25);

  // Both flags are always written; a snapshot only when its flag is true.
  ReportedSegments none;
  none.has_pending = true;
  none.has_following = true;
  ring.Reported(Rt(/*following=*/9), none);
  EXPECT_FALSE(none.has_pending);
  EXPECT_FALSE(none.has_following);
  ReportedSegments just_pending;
  just_pending.has_following = true;
  just_pending.following.segment_seq = 555;
  PlannerRtState pending_only = Rt(/*following=*/0, /*pending=*/2);
  ring.Reported(pending_only, just_pending);
  EXPECT_TRUE(just_pending.has_pending);
  EXPECT_FALSE(just_pending.has_following);
  EXPECT_EQ(just_pending.pending.segment_seq, 2U);
  EXPECT_EQ(just_pending.following.segment_seq, 555U) << "unflagged: not written";
}

TEST(SegmentRingReport, SourceSeqIsThePendingOneFromItsOwnNodeZeroOnElseTheFollowing) {
  SegmentRing<int> ring;
  ring.Push(Seg(1, /*t0_ns=*/1'000), 10);
  ring.Push(Seg(2, /*t0_ns=*/2'000), 20);
  const PlannerRtState rt = Rt(/*following=*/1, /*pending=*/2);
  // Both sides of the pending segment's node 0.
  EXPECT_EQ(ring.SourceSeq(rt, 1'999), 1U);
  EXPECT_EQ(ring.SourceSeq(rt, 2'000), 2U);
  EXPECT_EQ(ring.SourceSeq(rt, 2'001), 2U);
  EXPECT_EQ(ring.SourceSeq(rt, 0), 1U);
  // It is SourceSegmentAt on what Reported answers.
  ReportedSegments reported;
  ring.Reported(rt, reported);
  for (const std::int64_t t : {0LL, 1'999LL, 2'000LL, 2'001LL}) {
    const SegmentSnapshot* src = rtc::catching::SourceSegmentAt(reported, t);
    ASSERT_NE(src, nullptr) << t;
    EXPECT_EQ(ring.SourceSeq(rt, t), src->segment_seq) << t;
  }
  // No pending: the followed one at every instant.
  EXPECT_EQ(ring.SourceSeq(Rt(1), 0), 1U);
  EXPECT_EQ(ring.SourceSeq(Rt(1), 5'000), 1U);
  // A pending one with nothing followed has no source before its node 0.
  EXPECT_EQ(ring.SourceSeq(Rt(/*following=*/0, /*pending=*/2), 1'999), 0U);
  EXPECT_EQ(ring.SourceSeq(Rt(/*following=*/0, /*pending=*/2), 2'000), 2U);
  // A report of another plan, or of a seq the ring lacks: no source.
  EXPECT_EQ(ring.SourceSeq(Rt(1, 2, kPlan + 1), 5'000), 0U);
  EXPECT_EQ(ring.SourceSeq(Rt(/*following=*/9), 5'000), 0U);
}

TEST(SegmentRingReport, FollowedTrackIsThePlansTrackWhenTheRtFollowsThePlan) {
  SegmentRing<int> ring;
  ring.Push(Seg(1, 0, kPlan, kCatch, /*track=*/kTrack), 10);
  ring.Push(Seg(2, 0, kPlan, kCatch, /*track=*/kTrack), 20);
  std::uint64_t generation = 0;
  ASSERT_TRUE(ring.FollowedTrack(Rt(1), generation));
  EXPECT_EQ(generation, kTrack);
  // It answers for the plan, not for a segment the RT names: no segment
  // reported at all is still that plan's track.
  generation = 0;
  ASSERT_TRUE(ring.FollowedTrack(Rt(0), generation));
  EXPECT_EQ(generation, kTrack);
  // Another plan, or no active plan: nothing published for it.
  generation = 123;
  EXPECT_FALSE(ring.FollowedTrack(Rt(1, 0, kPlan + 1), generation));
  PlannerRtState inactive = Rt(1);
  inactive.plan_active = false;
  EXPECT_FALSE(ring.FollowedTrack(inactive, generation));
  EXPECT_EQ(generation, 123U);
}

// ── A ring with nothing to remember ──────────────────────────────────────────

TEST(SegmentRingNoPayload, TheSamePolicyHoldsForAPayloadThatIsEmpty) {
  static_assert(std::is_empty_v<NoSegmentPayload>);
  SegmentRing<NoSegmentPayload> ring;
  for (std::uint32_t seq = 1; seq <= kFull; ++seq) {
    ring.Push(Seg(seq), NoSegmentPayload{});
  }
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(Seg(9), NoSegmentPayload{});
  EXPECT_NE(ring.Find(1), nullptr);
  EXPECT_NE(ring.Find(2), nullptr);
  EXPECT_EQ(ring.Find(3), nullptr);
  EXPECT_NE(ring.Find(9), nullptr);
  static_cast<void>(ring.PayloadOf(ring.Find(9)));
  EXPECT_TRUE(ring.IsOf(Rt(1)));
  EXPECT_EQ(ring.SourceSeq(Rt(1), 0), 1U);
  ring.Push(Seg(1, 0, kPlan + 1), NoSegmentPayload{});
  EXPECT_EQ(ring.Find(2), nullptr);
}

}  // namespace
