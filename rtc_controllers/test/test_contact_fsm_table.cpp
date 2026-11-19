// ── Contact FSM alone: hand-written transition oracle (test-only) ─────────────
//
// Inputs are written by hand per tick (planned bits, contact-point speeds,
// the report the update would give) and the expected mode / reason / core
// action of every tick is written next to them — nothing here derives an
// expectation from the FSM. Dwell counts of FsmFixture(): enter 5, confirm 5,
// release 2, slip 3, NIS 3, recover 10 ticks.
//
//   rows        every row of the header's transition table, at its dwell
//               boundary (n − 1 ticks: no transition, n ticks: transition)
//   priority    release > slip (stance), slip > NIS (stance), release >
//               speed reject (candidate), release > recover (unknown)
//   samples     one sample of k ≥ n ticks alone never meets a multi-tick dwell
//   invalid     a non-finite speed: no update, streaks reset, mode kept,
//               release still progresses
//   reject      after a candidate rejection the enter streak starts from 0
//   anchorless  no stance / unknown: the speed is the spread to the other
//               planned slots (a lone planned slot is exempt)
//   same tick   PostUpdate changes no mode, and this tick's report reaches
//               the next tick only (random inputs, 1000 ticks × 8 seeds)
//   Init        every refusal, plus a passing control
#include "contact_fsm_testing.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace rtc::estimation::testing {
namespace {

using Mode = ContactMode;
using Why = ContactReason;
using Action = ContactFsm::SlotAction;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kSlow = 0.01;      // ≤ v_enter
constexpr double kMid = 0.1;        // between v_enter and v_slip
constexpr double kFast = 0.3;       // > v_slip
constexpr double kLow = 2.0;        // m² ≤ χ²_lo
constexpr double kNeutralM2 = 8.0;  // between
constexpr double kHighM2 = 13.0;    // > χ²_hi

/// The report the update gives a slot, read by the NEXT tick.
enum class Rep : std::uint8_t { kNone, kLowR, kNeutralR, kHighR, kGatedR };

ContactUpdateReport MakeReport(Rep r) {
  ContactUpdateReport out;
  switch (r) {
    case Rep::kNone:
      out.result = ContactUpdateResult::kNotRequested;
      break;
    case Rep::kLowR:
      out.result = ContactUpdateResult::kUsed;
      out.mahalanobis2 = kLow;
      break;
    case Rep::kNeutralR:
      out.result = ContactUpdateResult::kUsed;
      out.mahalanobis2 = kNeutralM2;
      break;
    case Rep::kHighR:
      out.result = ContactUpdateResult::kUsed;
      out.mahalanobis2 = kHighM2;
      break;
    case Rep::kGatedR:
      out.result = ContactUpdateResult::kGated;
      out.mahalanobis2 = 20.0;
      break;
  }
  return out;
}

/// Two-slot FSM. Slot 1 can be parked in stance as the confirmed anchor that
/// makes slot 0's speed absolute.
class Fsm2 {
 public:
  Fsm2() { f_.Init(FsmFixture(), 2); }

  /// One tick: decide, then store the given reports.
  void Tick(bool p0, double s0, Rep r0, bool p1 = true, double s1 = kSlow, Rep r1 = Rep::kLowR,
            int k = 1) {
    const std::array<Vec3, 2> u{Vec3(s0, 0.0, 0.0), Vec3(s1, 0.0, 0.0)};
    const std::uint32_t mask = (p0 ? 1U : 0U) | (p1 ? 2U : 0U);
    ASSERT_EQ(f_.PreUpdate(k, mask, u, a_), ContactFsmStatus::kOk);
    const std::array<ContactUpdateReport, 2> rep{MakeReport(r0), MakeReport(r1)};
    f_.PostUpdate(rep, true);
  }

  /// Both slots planned and still → both reach stance at tick 10 (insert at
  /// 5, first report from 6 read at 7, confirm at age 5).
  void ParkBothInStance() {
    for (int t = 1; t <= 10; ++t) {
      const Rep r = t >= 6 ? Rep::kLowR : Rep::kNone;
      Tick(true, kSlow, r, true, kSlow, r);
    }
    ASSERT_EQ(f_.mode(0), Mode::kStance);
    ASSERT_EQ(f_.mode(1), Mode::kStance);
  }

  /// Slot 0 alone back to swing (released) with slot 1 parked in stance.
  void ParkSlot0InSwing() {
    ParkBothInStance();
    Tick(false, kSlow, Rep::kNone);
    Tick(false, kSlow, Rep::kNone);
    ASSERT_EQ(f_.mode(0), Mode::kSwing);
  }

  /// Slot 0: swing → candidate (tick 5 of slow + planned) with slot 1 stance.
  void ToCandidate() {
    ParkSlot0InSwing();
    for (int t = 1; t <= 5; ++t) {
      Tick(true, kSlow, Rep::kNone);
    }
    ASSERT_EQ(f_.mode(0), Mode::kCandidate);
  }

  /// Slot 0 in unknown (stance, then 3 high reports).
  void ToUnknown() {
    ParkBothInStance();
    Tick(true, kSlow, Rep::kHighR);
    Tick(true, kSlow, Rep::kHighR);
    Tick(true, kSlow, Rep::kHighR);
    Tick(true, kSlow, Rep::kLowR);  // reads the 3rd high → unknown
    ASSERT_EQ(f_.mode(0), Mode::kUnknown);
  }

  void ToSlip() {
    ParkBothInStance();
    for (int t = 0; t < 3; ++t) {
      Tick(true, kFast, Rep::kNone);
    }
    ASSERT_EQ(f_.mode(0), Mode::kSlip);
  }

  const Action& a0() const { return a_[0]; }

  ContactFsm& f() { return f_; }

 private:
  ContactFsm f_;
  std::array<Action, 2> a_{};
};

void ExpectAction(const Action& a, bool insert, bool remove, bool scale, bool use, double s = 1.0) {
  EXPECT_EQ(a.insert, insert);
  EXPECT_EQ(a.remove, remove);
  EXPECT_EQ(a.set_scale, scale);
  if (scale) {
    EXPECT_EQ(a.noise_scale, s);
  }
  EXPECT_EQ(a.use, use);
}

// ═════════════════════════════════════════════════════════════════════════════
// Table rows
// ═════════════════════════════════════════════════════════════════════════════

TEST(ContactFsmTable, SwingToCandidateAtTheEnterDwell) {
  Fsm2 h;
  h.ParkSlot0InSwing();
  for (int t = 1; t <= 4; ++t) {
    h.Tick(true, kSlow, Rep::kNone);
    EXPECT_EQ(h.f().mode(0), Mode::kSwing) << t;
    ExpectAction(h.a0(), false, false, false, false);
  }
  h.Tick(true, kSlow, Rep::kNone);
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
  EXPECT_EQ(h.f().reason(0), Why::kTouchdown);
  ExpectAction(h.a0(), true, false, false, true);
}

TEST(ContactFsmTable, SwingNeedsThePlannedBit) {
  Fsm2 h;
  h.ParkSlot0InSwing();
  for (int t = 0; t < 20; ++t) {
    h.Tick(false, kSlow, Rep::kNone);
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  }
  // Planned again: the slow streak built while unplanned counts (it is a
  // speed streak), so the next planned tick inserts.
  h.Tick(true, kSlow, Rep::kNone);
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
}

TEST(ContactFsmTable, CandidateReleasedAtTheReleaseDwell) {
  Fsm2 h;
  h.ToCandidate();
  h.Tick(false, kSlow, Rep::kNone);
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
  h.Tick(false, kSlow, Rep::kNone);
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  EXPECT_EQ(h.f().reason(0), Why::kReleased);
  ExpectAction(h.a0(), false, true, false, false);
}

TEST(ContactFsmTable, CandidateRejectedBySpeed) {
  Fsm2 h;
  h.ToCandidate();
  h.Tick(true, kFast, Rep::kLowR);
  h.Tick(true, kFast, Rep::kLowR);
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
  h.Tick(true, kFast, Rep::kLowR);
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  EXPECT_EQ(h.f().reason(0), Why::kRejectedSpeed);
  ExpectAction(h.a0(), false, true, false, false);
}

TEST(ContactFsmTable, CandidateRejectedByOneHighReport) {
  for (const Rep r : {Rep::kHighR, Rep::kGatedR}) {
    Fsm2 h;
    h.ToCandidate();
    h.Tick(true, kSlow, r);  // the report of this tick …
    EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
    h.Tick(true, kSlow, Rep::kLowR);  // … is read here
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
    EXPECT_EQ(h.f().reason(0), Why::kRejectedNis);
    ExpectAction(h.a0(), false, true, false, false);
  }
}

TEST(ContactFsmTable, CandidateConfirmedAtAgeAndDwellWithALowReport) {
  Fsm2 h;
  h.ToCandidate();
  // Insertion tick reported kInsertedThisStep (none); ages 1 … 4 below.
  h.Tick(true, kSlow, Rep::kLowR);  // age 1
  for (int age = 2; age <= 4; ++age) {
    h.Tick(true, kSlow, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kCandidate) << age;
    ExpectAction(h.a0(), false, false, false, true);
  }
  h.Tick(true, kSlow, Rep::kLowR);  // age 5 = max(n_confirm, 2), slow 5, last low
  EXPECT_EQ(h.f().mode(0), Mode::kStance);
  EXPECT_EQ(h.f().reason(0), Why::kConfirmed);
  ExpectAction(h.a0(), false, false, false, true);
}

TEST(ContactFsmTable, CandidateConfirmNeedsTheSlowDwellNotJustTheAge) {
  Fsm2 h;
  h.ToCandidate();
  h.Tick(true, kSlow, Rep::kLowR);  // age 1, slow 1
  h.Tick(true, kMid, Rep::kLowR);   // age 2, slow streak broken
  for (int age = 3; age <= 6; ++age) {
    h.Tick(true, kSlow, Rep::kLowR);  // slow age − 2
    EXPECT_EQ(h.f().mode(0), Mode::kCandidate) << age;
  }
  h.Tick(true, kSlow, Rep::kLowR);  // age 7, slow 5
  EXPECT_EQ(h.f().mode(0), Mode::kStance);
  EXPECT_EQ(h.f().reason(0), Why::kConfirmed);
}

TEST(ContactFsmTable, CandidateWithoutALowReportIsNeverConfirmed) {
  for (const Rep r : {Rep::kNone, Rep::kNeutralR}) {
    Fsm2 h;
    h.ToCandidate();
    for (int t = 0; t < 30; ++t) {
      h.Tick(true, kSlow, r);
      EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
    }
    h.Tick(true, kSlow, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kCandidate);  // the low report is read next tick
    h.Tick(true, kSlow, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kStance);
  }
}

TEST(ContactFsmTable, StanceRows) {
  {  // release
    Fsm2 h;
    h.ParkBothInStance();
    h.Tick(false, kSlow, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kStance);
    h.Tick(false, kSlow, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
    EXPECT_EQ(h.f().reason(0), Why::kReleased);
    ExpectAction(h.a0(), false, true, false, false);
  }
  {  // slip at n_slip, mid speeds do not count
    Fsm2 h;
    h.ParkBothInStance();
    h.Tick(true, kFast, Rep::kLowR);
    h.Tick(true, kFast, Rep::kLowR);
    h.Tick(true, kMid, Rep::kLowR);
    h.Tick(true, kFast, Rep::kLowR);
    h.Tick(true, kFast, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kStance);
    h.Tick(true, kFast, Rep::kNone);
    EXPECT_EQ(h.f().mode(0), Mode::kSlip);
    EXPECT_EQ(h.f().reason(0), Why::kSlip);
    ExpectAction(h.a0(), false, true, false, false);
  }
  {  // NIS at n_nis; a neutral report breaks the streak, no report freezes it
    Fsm2 h;
    h.ParkBothInStance();
    h.Tick(true, kSlow, Rep::kHighR);
    h.Tick(true, kSlow, Rep::kHighR);
    h.Tick(true, kSlow, Rep::kNeutralR);  // reads high #2
    h.Tick(true, kSlow, Rep::kHighR);     // reads neutral: streak 0
    h.Tick(true, kSlow, Rep::kNone);      // reads high: 1
    h.Tick(true, kSlow, Rep::kHighR);     // reads none: still 1
    h.Tick(true, kSlow, Rep::kHighR);     // reads high: 2
    EXPECT_EQ(h.f().mode(0), Mode::kStance);
    h.Tick(true, kSlow, Rep::kHighR);  // reads high: 3
    EXPECT_EQ(h.f().mode(0), Mode::kUnknown);
    EXPECT_EQ(h.f().reason(0), Why::kNisHigh);
    ExpectAction(h.a0(), false, false, true, true, 100.0);
  }
}

TEST(ContactFsmTable, UnknownRows) {
  {  // release
    Fsm2 h;
    h.ToUnknown();
    h.Tick(false, kSlow, Rep::kLowR);
    h.Tick(false, kSlow, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
    EXPECT_EQ(h.f().reason(0), Why::kReleased);
    ExpectAction(h.a0(), false, true, false, false);
  }
  {  // slip
    Fsm2 h;
    h.ToUnknown();
    h.Tick(true, kFast, Rep::kLowR);
    h.Tick(true, kFast, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kUnknown);
    h.Tick(true, kFast, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kSlip);
    EXPECT_EQ(h.f().reason(0), Why::kSlip);
    ExpectAction(h.a0(), false, true, false, false);
  }
  {  // recover after 10 low reports AND 10 slow ticks; the scale goes back to 1
    Fsm2 h;
    h.ToUnknown();  // that tick stored a low report, read by tick 1 below
    for (int t = 1; t <= 9; ++t) {
      h.Tick(true, kSlow, Rep::kLowR);
      EXPECT_EQ(h.f().mode(0), Mode::kUnknown) << t;  // low t, slow t
    }
    h.Tick(true, kSlow, Rep::kLowR);  // low 10, slow 10
    EXPECT_EQ(h.f().mode(0), Mode::kStance);
    EXPECT_EQ(h.f().reason(0), Why::kRecovered);
    ExpectAction(h.a0(), false, false, true, true, 1.0);
  }
  {  // a mid speed breaks the slow streak: no recovery yet
    Fsm2 h;
    h.ToUnknown();
    for (int t = 1; t <= 8; ++t) {
      h.Tick(true, t == 5 ? kMid : kSlow, Rep::kLowR);
    }
    h.Tick(true, kSlow, Rep::kLowR);
    EXPECT_EQ(h.f().mode(0), Mode::kUnknown);
  }
}

TEST(ContactFsmTable, RecoveryNeedsTheNisDwellNotJustTheSpeedDwell) {
  Fsm2 h;
  h.ToUnknown();                        // stored a low report
  h.Tick(true, kSlow, Rep::kNeutralR);  // reads low: low 1, slow 1
  h.Tick(true, kSlow, Rep::kLowR);      // reads neutral: low 0, slow 2
  for (int t = 3; t <= 11; ++t) {
    h.Tick(true, kSlow, Rep::kLowR);  // low t − 2, slow t
    EXPECT_EQ(h.f().mode(0), Mode::kUnknown) << t;
  }
  h.Tick(true, kSlow, Rep::kLowR);  // low 10, slow 12
  EXPECT_EQ(h.f().mode(0), Mode::kStance);
  EXPECT_EQ(h.f().reason(0), Why::kRecovered);
}

TEST(ContactFsmTable, SlipRows) {
  {  // release, no core action (no anchor)
    Fsm2 h;
    h.ToSlip();
    h.Tick(false, kSlow, Rep::kNone);
    h.Tick(false, kSlow, Rep::kNone);
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
    EXPECT_EQ(h.f().reason(0), Why::kReleased);
    ExpectAction(h.a0(), false, false, false, false);
  }
  {  // re-acquire through candidate after the enter dwell
    Fsm2 h;
    h.ToSlip();
    for (int t = 1; t <= 4; ++t) {
      h.Tick(true, kSlow, Rep::kNone);
      EXPECT_EQ(h.f().mode(0), Mode::kSlip);
      ExpectAction(h.a0(), false, false, false, false);
    }
    h.Tick(true, kSlow, Rep::kNone);
    EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
    EXPECT_EQ(h.f().reason(0), Why::kTouchdown);
    ExpectAction(h.a0(), true, false, false, true);
  }
}

TEST(ContactFsmTable, UsedReportWithoutAFiniteNisCountsAsHigh) {
  Fsm2 h;
  h.ToCandidate();
  ContactUpdateReport bad;
  bad.result = ContactUpdateResult::kUsed;
  bad.mahalanobis2 = std::numeric_limits<double>::quiet_NaN();
  std::array<Action, 2> a{};
  const std::array<Vec3, 2> u{Vec3(kSlow, 0.0, 0.0), Vec3(kSlow, 0.0, 0.0)};
  ASSERT_EQ(h.f().PreUpdate(1, 3U, u, a), ContactFsmStatus::kOk);
  h.f().PostUpdate(std::array<ContactUpdateReport, 2>{bad, MakeReport(Rep::kLowR)}, true);
  h.Tick(true, kSlow, Rep::kLowR);  // reads the NaN report: high → rejected
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  EXPECT_EQ(h.f().reason(0), Why::kRejectedNis);
}

TEST(ContactFsmTable, RevertedInsertGoesBackToSwing) {
  Fsm2 h;
  h.ToCandidate();
  h.f().RevertInsert(0);
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  EXPECT_EQ(h.f().reason(0), Why::kInsertRefused);
  // Its streaks restart: four more slow ticks are not enough.
  for (int t = 1; t <= 4; ++t) {
    h.Tick(true, kSlow, Rep::kNone);
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  }
  h.Tick(true, kSlow, Rep::kNone);
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
  // A revert on a non-candidate slot (or out of range) is a no-op.
  h.f().RevertInsert(1);
  h.f().RevertInsert(7);
  EXPECT_EQ(h.f().mode(1), Mode::kStance);
}

// ═════════════════════════════════════════════════════════════════════════════
// Priority conflicts
// ═════════════════════════════════════════════════════════════════════════════

TEST(ContactFsmTable, StanceReleaseBeatsSlip) {
  Fsm2 h;
  h.ParkBothInStance();
  h.Tick(true, kFast, Rep::kLowR);
  h.Tick(false, kFast, Rep::kLowR);
  h.Tick(false, kFast, Rep::kLowR);  // release 2 and fast 3 on the same tick
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  EXPECT_EQ(h.f().reason(0), Why::kReleased);
}

TEST(ContactFsmTable, StanceSlipBeatsNis) {
  Fsm2 h;
  h.ParkBothInStance();
  h.Tick(true, kSlow, Rep::kHighR);  // reads the parked low report
  h.Tick(true, kFast, Rep::kHighR);  // high 1, fast 1
  h.Tick(true, kFast, Rep::kHighR);  // high 2, fast 2
  EXPECT_EQ(h.f().mode(0), Mode::kStance);
  h.Tick(true, kFast, Rep::kHighR);  // high 3 and fast 3 on the same tick
  EXPECT_EQ(h.f().mode(0), Mode::kSlip);
  EXPECT_EQ(h.f().reason(0), Why::kSlip);
}

TEST(ContactFsmTable, CandidateReleaseBeatsSpeedReject) {
  Fsm2 h;
  h.ToCandidate();
  h.Tick(true, kFast, Rep::kLowR);
  h.Tick(false, kFast, Rep::kLowR);
  h.Tick(false, kFast, Rep::kLowR);  // release 2 and fast 3
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  EXPECT_EQ(h.f().reason(0), Why::kReleased);
}

TEST(ContactFsmTable, UnknownReleaseBeatsRecovery) {
  Fsm2 h;
  h.ToUnknown();
  for (int t = 1; t <= 8; ++t) {
    h.Tick(true, kSlow, Rep::kLowR);
  }
  h.Tick(false, kSlow, Rep::kLowR);  // release 1, low 9, slow 9
  EXPECT_EQ(h.f().mode(0), Mode::kUnknown);
  h.Tick(false, kSlow, Rep::kLowR);  // release 2 with low 10, slow 10
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  EXPECT_EQ(h.f().reason(0), Why::kReleased);
}

// ═════════════════════════════════════════════════════════════════════════════
// Samples, invalid input, rejection, anchorless speed
// ═════════════════════════════════════════════════════════════════════════════

TEST(ContactFsmTable, OneLongSampleDoesNotMeetAMultiTickDwell) {
  Fsm2 h;
  h.ParkSlot0InSwing();
  h.Tick(true, kSlow, Rep::kNone, true, kSlow, Rep::kLowR, 6);  // 6 ticks, 1 sample
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);
  h.Tick(true, kSlow, Rep::kNone, true, kSlow, Rep::kLowR, 1);  // second sample
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);

  Fsm2 r;
  r.ParkBothInStance();
  r.Tick(false, kSlow, Rep::kLowR, true, kSlow, Rep::kLowR, 2);  // release 2 ticks, 1 sample
  EXPECT_EQ(r.f().mode(0), Mode::kStance);
  r.Tick(false, kSlow, Rep::kLowR, true, kSlow, Rep::kLowR, 1);
  EXPECT_EQ(r.f().mode(0), Mode::kSwing);
}

TEST(ContactFsmTable, StepCountAccumulatesTicks) {
  Fsm2 h;
  h.ParkSlot0InSwing();
  h.Tick(true, kSlow, Rep::kNone, true, kSlow, Rep::kLowR, 2);
  h.Tick(true, kSlow, Rep::kNone, true, kSlow, Rep::kLowR, 2);
  EXPECT_EQ(h.f().mode(0), Mode::kSwing);  // 4 ticks
  h.Tick(true, kSlow, Rep::kNone, true, kSlow, Rep::kLowR, 1);
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);  // 5 ticks, 3 samples
}

TEST(ContactFsmTable, InvalidInputKeepsTheModeAndResetsStreaks) {
  Fsm2 h;
  h.ParkBothInStance();
  h.Tick(true, kFast, Rep::kLowR);
  h.Tick(true, kFast, Rep::kLowR);
  h.Tick(true, kNan, Rep::kNone);  // invalid: fast streak reset, no update
  EXPECT_EQ(h.f().mode(0), Mode::kStance);
  ExpectAction(h.a0(), false, false, false, false);
  EXPECT_TRUE(std::isnan(h.f().tested_speed(0)));
  const double held = h.f().evidence(0);
  h.Tick(true, kNan, Rep::kNone);
  EXPECT_EQ(h.f().evidence(0), held);  // evidence held while invalid
  h.Tick(true, kFast, Rep::kLowR);
  h.Tick(true, kFast, Rep::kLowR);
  EXPECT_EQ(h.f().mode(0), Mode::kStance);  // only 2 since the reset
  h.Tick(true, kFast, Rep::kLowR);
  EXPECT_EQ(h.f().mode(0), Mode::kSlip);

  // Release still progresses while invalid.
  Fsm2 r;
  r.ParkBothInStance();
  r.Tick(false, kNan, Rep::kNone);
  r.Tick(false, kNan, Rep::kNone);
  EXPECT_EQ(r.f().mode(0), Mode::kSwing);
  EXPECT_EQ(r.f().reason(0), Why::kReleased);
  ExpectAction(r.a0(), false, true, false, false);

  // An invalid swing slot is never inserted.
  Fsm2 s;
  s.ParkSlot0InSwing();
  for (int t = 0; t < 10; ++t) {
    s.Tick(true, kNan, Rep::kNone);
    EXPECT_EQ(s.f().mode(0), Mode::kSwing);
  }
}

TEST(ContactFsmTable, RejectedCandidateStartsTheEnterStreakFromZero) {
  Fsm2 h;
  h.ToCandidate();
  h.Tick(true, kSlow, Rep::kGatedR);
  h.Tick(true, kSlow, Rep::kNone);  // rejected here
  ASSERT_EQ(h.f().mode(0), Mode::kSwing);
  for (int t = 1; t <= 4; ++t) {
    h.Tick(true, kSlow, Rep::kNone);
    EXPECT_EQ(h.f().mode(0), Mode::kSwing) << t;
  }
  h.Tick(true, kSlow, Rep::kNone);
  EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
}

TEST(ContactFsmTable, AnchorlessSpeedIsTheSpreadBetweenPlannedSlots) {
  {  // both swing, both moving together at 0.5 m/s: spread 0 → both inserted
    Fsm2 h;
    for (int t = 1; t <= 5; ++t) {
      h.Tick(true, 0.5, Rep::kNone, true, 0.5, Rep::kNone);
    }
    EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
    EXPECT_EQ(h.f().mode(1), Mode::kCandidate);
    EXPECT_EQ(h.f().tested_speed(0), 0.0);
  }
  {  // spread 0.3 → neither is slow
    Fsm2 h;
    for (int t = 1; t <= 10; ++t) {
      h.Tick(true, 0.5, Rep::kNone, true, 0.2, Rep::kNone);
    }
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
    EXPECT_NEAR(h.f().tested_speed(0), 0.3, 1e-15);
  }
  {  // a lone planned slot is exempt
    Fsm2 h;
    for (int t = 1; t <= 5; ++t) {
      h.Tick(true, 0.5, Rep::kNone, false, 0.0, Rep::kNone);
    }
    EXPECT_EQ(h.f().mode(0), Mode::kCandidate);
    EXPECT_EQ(h.f().mode(1), Mode::kSwing);
  }
  {  // a confirmed anchor makes the speed absolute
    Fsm2 h;
    h.ParkSlot0InSwing();
    for (int t = 1; t <= 10; ++t) {
      h.Tick(true, 0.5, Rep::kNone, true, kSlow, Rep::kLowR);
    }
    EXPECT_EQ(h.f().mode(0), Mode::kSwing);
    EXPECT_EQ(h.f().tested_speed(0), 0.5);
  }
}

TEST(ContactFsmTable, EvidenceFollowsTheScore) {
  Fsm2 h;
  h.ParkBothInStance();
  for (int t = 0; t < 200; ++t) {
    h.Tick(true, kSlow, Rep::kLowR);
  }
  EXPECT_GT(h.f().evidence(0), 0.99);
  EXPECT_LE(h.f().evidence(0), 1.0);
  // One tick of q = 0 from c: c(1 − α), α = 1 − exp(−T/τ_e).
  const double c = h.f().evidence(0);
  h.Tick(false, kSlow, Rep::kLowR);
  EXPECT_NEAR(h.f().evidence(0), c * std::exp(-kPeriod / 20e-3), 1e-15);
}

// ═════════════════════════════════════════════════════════════════════════════
// Same-tick rule
// ═════════════════════════════════════════════════════════════════════════════

TEST(ContactFsmTable, ThisTicksReportReachesOnlyTheNextTick) {
  int diverged = 0;
  for (int seed = 0; seed < 8; ++seed) {
    std::mt19937_64 rng(kSeed + static_cast<std::uint64_t>(seed));
    std::uniform_real_distribution<double> ud(0.0, 1.0);
    ContactFsm a;
    a.Init(FsmFixture(), 2);
    auto random_report = [&] {
      const double x = ud(rng);
      return MakeReport(x < 0.4   ? Rep::kLowR
                        : x < 0.6 ? Rep::kNeutralR
                        : x < 0.8 ? Rep::kHighR
                        : x < 0.9 ? Rep::kGatedR
                                  : Rep::kNone);
    };
    for (int t = 0; t < 1000; ++t) {
      ContactFsm b = a;
      const std::array<Vec3, 2> u{Vec3(0.2 * ud(rng), 0.0, 0.0), Vec3(0.2 * ud(rng), 0.0, 0.0)};
      const std::uint32_t mask = (ud(rng) < 0.9 ? 1U : 0U) | (ud(rng) < 0.9 ? 2U : 0U);
      std::array<Action, 2> aa{};
      std::array<Action, 2> ab{};
      ASSERT_EQ(a.PreUpdate(1, mask, u, aa), ContactFsmStatus::kOk);
      ASSERT_EQ(b.PreUpdate(1, mask, u, ab), ContactFsmStatus::kOk);
      const std::array<ContactUpdateReport, 2> ra{random_report(), random_report()};
      const std::array<ContactUpdateReport, 2> rb{random_report(), random_report()};
      a.PostUpdate(ra, true);
      b.PostUpdate(rb, true);
      for (int i = 0; i < 2; ++i) {
        const auto si = static_cast<std::size_t>(i);
        ASSERT_EQ(a.mode(i), b.mode(i)) << "seed " << seed << " tick " << t;
        ASSERT_EQ(a.reason(i), b.reason(i));
        ASSERT_EQ(a.evidence(i), b.evidence(i));
        ASSERT_EQ(aa[si].insert, ab[si].insert);
        ASSERT_EQ(aa[si].remove, ab[si].remove);
        ASSERT_EQ(aa[si].set_scale, ab[si].set_scale);
        ASSERT_EQ(aa[si].use, ab[si].use);
      }
      // Positive control: the next tick does see the difference somewhere.
      std::array<Action, 2> na{};
      std::array<Action, 2> nb{};
      ContactFsm a2 = a;
      ASSERT_EQ(a2.PreUpdate(1, mask, u, na), ContactFsmStatus::kOk);
      ASSERT_EQ(b.PreUpdate(1, mask, u, nb), ContactFsmStatus::kOk);
      diverged += ContactFsmTestAccess::SlotState(a2) != ContactFsmTestAccess::SlotState(b);
    }
  }
  EXPECT_GT(diverged, 1000);
}

// ═════════════════════════════════════════════════════════════════════════════
// Init and refusals
// ═════════════════════════════════════════════════════════════════════════════

TEST(ContactFsmTable, InitRefusesEveryBadField) {
  using Field = double ContactFsm::Config::*;
  const std::array<Field, 13> fields{&ContactFsm::Config::period,
                                     &ContactFsm::Config::enter_speed,
                                     &ContactFsm::Config::slip_speed,
                                     &ContactFsm::Config::nis_low,
                                     &ContactFsm::Config::nis_high,
                                     &ContactFsm::Config::enter_dwell,
                                     &ContactFsm::Config::confirm_dwell,
                                     &ContactFsm::Config::release_dwell,
                                     &ContactFsm::Config::slip_dwell,
                                     &ContactFsm::Config::nis_dwell,
                                     &ContactFsm::Config::recover_dwell,
                                     &ContactFsm::Config::evidence_time_constant,
                                     &ContactFsm::Config::unknown_noise_scale};
  const double inf = std::numeric_limits<double>::infinity();
  for (std::size_t f = 0; f < fields.size(); ++f) {
    for (const double bad : {kNan, inf, -inf, 0.0, -1.0}) {
      ContactFsm fsm;
      ContactFsm::Config c = FsmFixture();
      c.*fields[f] = bad;
      EXPECT_THROW(fsm.Init(c, 2), std::invalid_argument) << "field " << f << " = " << bad;
      EXPECT_FALSE(fsm.configured());
    }
  }
  auto refused = [](const ContactFsm::Config& c, int n = 2) {
    ContactFsm fsm;
    try {
      fsm.Init(c, n);
    } catch (const std::invalid_argument&) {
      return !fsm.configured();
    }
    return false;
  };
  ContactFsm::Config c = FsmFixture();
  c.enter_speed = c.slip_speed;
  EXPECT_TRUE(refused(c));
  c = FsmFixture();
  c.enter_speed = 0.2;
  EXPECT_TRUE(refused(c));
  c = FsmFixture();
  c.nis_low = c.nis_high;
  EXPECT_TRUE(refused(c));
  c = FsmFixture();
  c.nis_low = 20.0;
  EXPECT_TRUE(refused(c));
  c = FsmFixture();
  c.unknown_noise_scale = 0.999;
  EXPECT_TRUE(refused(c));
  c = FsmFixture();
  c.period = 1e-4;  // below the core's step range
  EXPECT_TRUE(refused(c));
  c = FsmFixture();
  c.period = 0.03;  // above
  EXPECT_TRUE(refused(c));
  c = FsmFixture();
  c.recover_dwell = 2e3 + 1.0;  // > 10⁶ periods
  EXPECT_TRUE(refused(c));
  EXPECT_TRUE(refused(FsmFixture(), 0));
  EXPECT_TRUE(refused(FsmFixture(), kInekfMaxContacts + 1));

  // Passing controls: the fixture, the range ends, s_u = 1, a sub-period dwell.
  for (int n = 1; n <= kInekfMaxContacts; ++n) {
    EXPECT_FALSE(refused(FsmFixture(), n)) << n;
  }
  c = FsmFixture();
  c.period = kMinPropagationStep;
  EXPECT_FALSE(refused(c));
  c.period = kMaxPropagationStep;
  EXPECT_FALSE(refused(c));
  c = FsmFixture();
  c.unknown_noise_scale = 1.0;
  c.release_dwell = 1e-4;  // ⌈0.05⌉ = 1 tick
  EXPECT_FALSE(refused(c));
}

TEST(ContactFsmTable, DwellsRoundUpToWholeTicks) {
  // 3 ms at 2 ms = 1.5 → 2 ticks of release; 4 ms → 2 (not 3).
  for (const double d : {3e-3, 4e-3}) {
    ContactFsm::Config c = FsmFixture();
    c.release_dwell = d;
    ContactFsm f;
    f.Init(c, 1);
    std::array<Action, 1> a{};
    const std::array<Vec3, 1> u{Vec3::Zero()};
    for (int t = 1; t <= 5; ++t) {  // lone planned slot: exempt → candidate at 5
      ASSERT_EQ(f.PreUpdate(1, 1U, u, a), ContactFsmStatus::kOk);
    }
    ASSERT_EQ(f.mode(0), Mode::kCandidate);
    ASSERT_EQ(f.PreUpdate(1, 0U, u, a), ContactFsmStatus::kOk);
    EXPECT_EQ(f.mode(0), Mode::kCandidate) << d;
    ASSERT_EQ(f.PreUpdate(1, 0U, u, a), ContactFsmStatus::kOk);
    EXPECT_EQ(f.mode(0), Mode::kSwing) << d;
  }
}

TEST(ContactFsmTable, RefusedCallsChangeNothing) {
  ContactFsm f;
  std::array<Action, 2> a{};
  std::array<Vec3, 2> u{Vec3::Zero(), Vec3::Zero()};
  EXPECT_EQ(f.PreUpdate(1, 3U, u, a), ContactFsmStatus::kNotConfigured);
  f.Init(FsmFixture(), 2);
  const std::vector<double> before = ContactFsmTestAccess::SlotState(f);
  EXPECT_EQ(f.PreUpdate(0, 3U, u, a), ContactFsmStatus::kBadPeriods);
  EXPECT_EQ(f.PreUpdate(-1, 3U, u, a), ContactFsmStatus::kBadPeriods);
  std::array<Vec3, 1> short_u{Vec3::Zero()};
  EXPECT_EQ(f.PreUpdate(1, 3U, short_u, a), ContactFsmStatus::kSizeMismatch);
  std::array<Action, 3> long_a{};
  EXPECT_EQ(f.PreUpdate(1, 3U, u, long_a), ContactFsmStatus::kSizeMismatch);
  EXPECT_TRUE(SameBits(before, ContactFsmTestAccess::SlotState(f)));

  // Reset returns to the post-Init state.
  for (int t = 0; t < 12; ++t) {
    ASSERT_EQ(f.PreUpdate(1, 3U, u, a), ContactFsmStatus::kOk);
  }
  ASSERT_NE(f.mode(0), Mode::kSwing);
  f.Reset();
  EXPECT_TRUE(SameBits(before, ContactFsmTestAccess::SlotState(f)));
}

}  // namespace
}  // namespace rtc::estimation::testing
