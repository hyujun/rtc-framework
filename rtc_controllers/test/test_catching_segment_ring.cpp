// The ring of published segments both segment planners keep (E1-F16 #742,
// segment_ring.hpp). Header-only pure logic: no model, no solver.
//
// What is tested: with no plan noted as followed the ring holds the segments
// of ONE plan (a segment of another plan empties it); with one noted it holds
// that plan's and the newest plan's — the followed plan and its replacement —
// and a report is read plan by plan; it holds eight, and a full ring drops the
// oldest segment the RT did not last report, so the followed and the pending
// segment survive a burst of pushes; a payload moves with its segment; and the
// three questions the cycle asks — which segments the RT reports (Reported),
// which one a solve at an instant starts from (SourceSeq), whose track the
// plan is (FollowedTrack) — are answered from what the RT reports, never
// inferred.
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

// ── Two plans: the followed one and its replacement ──────────────────────────
//
// The first segment of a replacement plan is published while the arm is still
// on the followed plan's segments: both plans are in the ring until the RT
// switches (or drops the replacement), and a report is read plan by plan.

constexpr std::uint32_t kNewPlan = kPlan + 1;
constexpr std::int64_t kNewCatch = kCatch + 40'000'000;
constexpr std::uint64_t kNewTrack = kTrack + 1;

[[nodiscard]] SegmentSnapshot NewSeg(std::uint32_t seq, std::int64_t t0_ns = 0) {
  return Seg(seq, t0_ns, kNewPlan, kNewCatch, kNewTrack);
}

// The RT following (kPlan, kCatch) while it holds the replacement (kNewPlan,
// kNewCatch): `pending` is then the replacement's first segment.
[[nodiscard]] PlannerRtState RtHolding(std::uint32_t following, std::uint32_t pending) {
  PlannerRtState rt = Rt(following, pending);
  rt.plan_pending = true;
  rt.plan_pending_id = kNewPlan;
  rt.plan_pending_t_c_ns = kNewCatch;
  return rt;
}

// The RT after the switch: it follows the replacement.
[[nodiscard]] PlannerRtState RtSwitched(std::uint32_t following, std::uint32_t pending = 0) {
  return Rt(following, pending, kNewPlan, kNewCatch);
}

TEST(SegmentRingTwoPlans, TheFollowedPlansSegmentsStayWhenTheReplacementsFirstIsPushed) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 3);
  ring.NoteReported(Rt(/*following=*/2, /*pending=*/3));
  ring.Push(NewSeg(4), PayloadFor(4));
  for (std::uint32_t seq = 1; seq <= 4; ++seq) {
    ASSERT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
  for (std::uint32_t seq = 1; seq <= 3; ++seq) {
    EXPECT_EQ(ring.Find(seq)->plan_id, kPlan) << seq;
    EXPECT_EQ(ring.Find(seq)->t_c_ns, kCatch) << seq;
  }
  EXPECT_EQ(ring.Find(4)->plan_id, kNewPlan);
  EXPECT_EQ(ring.Find(4)->t_c_ns, kNewCatch);
  // The ring is of both plans now.
  EXPECT_TRUE(ring.IsOf(Rt(2)));
  EXPECT_TRUE(ring.IsOf(RtSwitched(4)));
  EXPECT_FALSE(ring.IsOf(Rt(2, 0, kPlan + 2, kCatch)));
  // An unreported segment of the followed plan stays too: the plan is kept,
  // not only the two segments the RT named.
  EXPECT_NE(ring.Find(1), nullptr);
}

TEST(SegmentRingTwoPlans, AThirdPlansPushDropsOnlyWhatIsNeitherFollowedNorNewest) {
  constexpr std::uint32_t kThirdPlan = kPlan + 2;
  constexpr std::int64_t kThirdCatch = kCatch + 90'000'000;
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 2);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(NewSeg(3), PayloadFor(3));
  // The RT did not take that replacement and still follows the first plan; the
  // search replaces it again.
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(Seg(4, 0, kThirdPlan, kThirdCatch), PayloadFor(4));
  EXPECT_EQ(ring.Find(3), nullptr) << "the replacement nobody follows";
  for (const std::uint32_t seq : {1U, 2U, 4U}) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
  EXPECT_EQ(ring.Find(1)->plan_id, kPlan);
  EXPECT_EQ(ring.Find(4)->plan_id, kThirdPlan);

  // The same id with another catch instant is another plan.
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(Seg(5, 0, kThirdPlan, kThirdCatch + 1), PayloadFor(5));
  EXPECT_EQ(ring.Find(4), nullptr);
  for (const std::uint32_t seq : {1U, 2U, 5U}) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }

  // A later segment of the FOLLOWED plan (a replan after the RT dropped the
  // replacement) leaves that plan alone in the ring.
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(Seg(6), PayloadFor(6));
  EXPECT_EQ(ring.Find(5), nullptr);
  for (const std::uint32_t seq : {1U, 2U, 6U}) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
}

TEST(SegmentRingTwoPlans, AfterTheSwitchTheNextPushDropsThePlanTheRtLeft) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, 2);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(NewSeg(3), PayloadFor(3));
  // The RT switched: it follows the replacement, and a replan of it is stored.
  ring.NoteReported(RtSwitched(/*following=*/3));
  ring.Push(NewSeg(4), PayloadFor(4));
  EXPECT_EQ(ring.Find(1), nullptr);
  EXPECT_EQ(ring.Find(2), nullptr);
  EXPECT_TRUE(HoldsWithPayload(ring, 3));
  EXPECT_TRUE(HoldsWithPayload(ring, 4));
  EXPECT_FALSE(ring.IsOf(Rt(1)));
  EXPECT_TRUE(ring.IsOf(RtSwitched(3)));
}

TEST(SegmentRingTwoPlans, TheReportIsReadPlanByPlanInTheThreeStatesOfAReplacement) {
  SegmentRing<int> ring;
  ring.Push(Seg(1, /*t0_ns=*/1'000), 10);
  ring.Push(Seg(2, /*t0_ns=*/2'000), 20);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(NewSeg(3, /*t0_ns=*/3'000), 30);
  std::uint64_t generation = 0;
  ReportedSegments out;

  // (1) The RT follows the old plan and holds nothing: the replacement's
  // segment is in the ring and is not reported.
  {
    const PlannerRtState rt = Rt(/*following=*/1);
    const SegmentRing<int>::Report r = ring.ReportedIn(rt);
    EXPECT_EQ(r.following, ring.Find(1));
    EXPECT_EQ(r.pending, nullptr);
    EXPECT_EQ(ring.SourceSeq(rt, 2'999), 1U);
    EXPECT_EQ(ring.SourceSeq(rt, 3'000), 1U);
    EXPECT_EQ(ring.SourceSeq(rt, 9'000), 1U);
    generation = 0;
    ASSERT_TRUE(ring.FollowedTrack(rt, generation));
    EXPECT_EQ(generation, kTrack);
    // ... and a pending segment of the old plan is read as before.
    const PlannerRtState with_pending = Rt(/*following=*/1, /*pending=*/2);
    EXPECT_EQ(ring.ReportedIn(with_pending).pending, ring.Find(2));
    EXPECT_EQ(ring.SourceSeq(with_pending, 1'999), 1U);
    EXPECT_EQ(ring.SourceSeq(with_pending, 2'000), 2U);
  }
  // (2) The RT follows the old plan and holds the replacement, whose first
  // segment waits: the followed one is the old plan's, the pending one the
  // replacement's.
  {
    const PlannerRtState rt = RtHolding(/*following=*/1, /*pending=*/3);
    const SegmentRing<int>::Report r = ring.ReportedIn(rt);
    ASSERT_EQ(r.following, ring.Find(1));
    ASSERT_EQ(r.pending, ring.Find(3));
    EXPECT_EQ(r.following->plan_id, kPlan);
    EXPECT_EQ(r.pending->plan_id, kNewPlan);
    EXPECT_EQ(ring.PayloadOf(r.following), 10);
    EXPECT_EQ(ring.PayloadOf(r.pending), 30);
    EXPECT_EQ(ring.SourceSeq(rt, 2'999), 1U);
    EXPECT_EQ(ring.SourceSeq(rt, 3'000), 3U);
    EXPECT_EQ(ring.SourceSeq(rt, 3'001), 3U);
    // The plan the RT follows is still the old one, and so is its track.
    EXPECT_TRUE(ring.IsOf(rt));
    generation = 0;
    ASSERT_TRUE(ring.FollowedTrack(rt, generation));
    EXPECT_EQ(generation, kTrack);
    ring.Reported(rt, out);
    ASSERT_TRUE(out.has_following);
    ASSERT_TRUE(out.has_pending);
    EXPECT_EQ(out.following.segment_seq, 1U);
    EXPECT_EQ(out.following.t_c_ns, kCatch);
    EXPECT_EQ(out.pending.segment_seq, 3U);
    EXPECT_EQ(out.pending.t_c_ns, kNewCatch);
    EXPECT_EQ(out.pending.t0_ns, 3'000);
  }
  // (3) The RT switched: it follows the replacement on its first segment.
  {
    const PlannerRtState rt = RtSwitched(/*following=*/3);
    const SegmentRing<int>::Report r = ring.ReportedIn(rt);
    EXPECT_EQ(r.following, ring.Find(3));
    EXPECT_EQ(r.pending, nullptr);
    EXPECT_EQ(ring.SourceSeq(rt, 0), 3U);
    EXPECT_EQ(ring.SourceSeq(rt, 9'000), 3U);
    generation = 0;
    ASSERT_TRUE(ring.FollowedTrack(rt, generation));
    EXPECT_EQ(generation, kNewTrack);
    // The old plan's segments are still in the ring (nothing was pushed
    // since) and are not the new plan's: a seq of theirs is not reported.
    ASSERT_NE(ring.Find(1), nullptr);
    const SegmentRing<int>::Report stale = ring.ReportedIn(RtSwitched(/*following=*/1, 2));
    EXPECT_EQ(stale.following, nullptr);
    EXPECT_EQ(stale.pending, nullptr);
    EXPECT_EQ(ring.SourceSeq(RtSwitched(/*following=*/1, 2), 9'000), 0U);
    ring.Reported(RtSwitched(/*following=*/1, 2), out);
    EXPECT_FALSE(out.has_following);
    EXPECT_FALSE(out.has_pending);
  }
}

TEST(SegmentRingTwoPlans, WithoutAHeldReplacementASeqOfTheOtherPlanIsNotPending) {
  SegmentRing<int> ring;
  ring.Push(Seg(1, /*t0_ns=*/1'000), 10);
  ring.Push(Seg(2, /*t0_ns=*/2'000), 20);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(NewSeg(3, /*t0_ns=*/3'000), 30);
  ASSERT_NE(ring.Find(3), nullptr);

  // plan_pending false: seq 3 is in the ring under another plan than the one
  // the RT follows, so it is not what the RT reports pending.
  const PlannerRtState rt = Rt(/*following=*/1, /*pending=*/3);
  const SegmentRing<int>::Report r = ring.ReportedIn(rt);
  EXPECT_EQ(r.following, ring.Find(1));
  EXPECT_EQ(r.pending, nullptr);
  EXPECT_EQ(ring.SourceSeq(rt, 3'000), 1U) << "the followed one: nothing is pending";
  ReportedSegments out;
  out.has_pending = true;
  ring.Reported(rt, out);
  EXPECT_TRUE(out.has_following);
  EXPECT_FALSE(out.has_pending);

  // A held replacement that is not the one in the ring — another id, another
  // catch instant — does not find it either.
  PlannerRtState other = RtHolding(/*following=*/1, /*pending=*/3);
  other.plan_pending_id = kNewPlan + 5;
  EXPECT_EQ(ring.ReportedIn(other).pending, nullptr);
  other = RtHolding(/*following=*/1, /*pending=*/3);
  other.plan_pending_t_c_ns = kNewCatch + 1;
  EXPECT_EQ(ring.ReportedIn(other).pending, nullptr);
  // The held plan's fields alone do not make a replacement: the flag does.
  other = RtHolding(/*following=*/1, /*pending=*/3);
  other.plan_pending = false;
  EXPECT_EQ(ring.ReportedIn(other).pending, nullptr);

  // The followed segment is never looked up in the replacement.
  const SegmentRing<int>::Report wrong_side = ring.ReportedIn(RtHolding(/*following=*/3, 0));
  EXPECT_EQ(wrong_side.following, nullptr);
  EXPECT_EQ(ring.SourceSeq(RtHolding(/*following=*/3, 0), 9'000), 0U);
  // A pending segment of the followed plan is still found while a
  // replacement is held.
  EXPECT_EQ(ring.ReportedIn(RtHolding(/*following=*/1, /*pending=*/2)).pending, ring.Find(2));
  // With nothing of the followed plan in the ring a report finds nothing —
  // the replacement's segment included.
  PlannerRtState unknown = RtHolding(/*following=*/1, /*pending=*/3);
  unknown.plan_id = kPlan + 7;
  EXPECT_EQ(ring.ReportedIn(unknown).pending, nullptr);
  EXPECT_EQ(ring.ReportedIn(unknown).following, nullptr);
}

TEST(SegmentRingTwoPlans, ASeqBothPlansCarryIsReadUnderThePlanItIsReportedFor) {
  SegmentRing<int> ring;
  ring.Push(Seg(1, /*t0_ns=*/1'000), 10);
  ring.NoteReported(Rt(/*following=*/1));
  ring.Push(NewSeg(1, /*t0_ns=*/3'000), 111);
  const SegmentRing<int>::Report held = ring.ReportedIn(RtHolding(/*following=*/1, /*pending=*/1));
  ASSERT_NE(held.following, nullptr);
  ASSERT_NE(held.pending, nullptr);
  EXPECT_NE(held.following, held.pending);
  EXPECT_EQ(held.following->plan_id, kPlan);
  EXPECT_EQ(ring.PayloadOf(held.following), 10);
  EXPECT_EQ(held.pending->plan_id, kNewPlan);
  EXPECT_EQ(ring.PayloadOf(held.pending), 111);
  const SegmentRing<int>::Report switched = ring.ReportedIn(RtSwitched(/*following=*/1));
  ASSERT_NE(switched.following, nullptr);
  EXPECT_EQ(ring.PayloadOf(switched.following), 111);
}

TEST(SegmentRingTwoPlans, AFullRingTakesTheReplacementAndKeepsTheReportedSegments) {
  SegmentRing<int> ring;
  PushSeqs(ring, 1, kFull);
  ring.NoteReported(Rt(/*following=*/1, /*pending=*/2));
  ring.Push(NewSeg(9), PayloadFor(9));
  // The oldest segment the RT did not report made room; the plan stays.
  EXPECT_EQ(ring.Find(3), nullptr);
  for (const std::uint32_t seq : {1U, 2U, 4U, 5U, 6U, 7U, 8U, 9U}) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
  EXPECT_EQ(ring.ReportedIn(RtHolding(/*following=*/1, /*pending=*/9)).pending, ring.Find(9));
  // While the RT holds the replacement, its first segment is one of the two
  // reported — in the replacement's plan, where the eviction has to look for
  // it: a burst of pushes into the full ring drops neither it nor the
  // followed segment.
  ring.NoteReported(RtHolding(/*following=*/1, /*pending=*/9));
  for (std::uint32_t seq = 10; seq <= 30; ++seq) {
    ring.Push(NewSeg(seq), PayloadFor(seq));
  }
  EXPECT_TRUE(HoldsWithPayload(ring, 1));
  EXPECT_TRUE(HoldsWithPayload(ring, 9));
  for (std::uint32_t seq = 2; seq <= 8; ++seq) {
    EXPECT_EQ(ring.Find(seq), nullptr) << seq;
  }
  for (std::uint32_t seq = 25; seq <= 30; ++seq) {
    EXPECT_TRUE(HoldsWithPayload(ring, seq)) << seq;
  }
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
