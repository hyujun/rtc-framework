// L7 supervisor "pure pieces" (dynamic_catching S1.8): decel target (L7.2,
// §4.3), transition table as data (L7.1, §4.1/§4.2), contact debounce (L7.3,
// §4.4). Gate IDs are those of L7_supervisor.md §9.
//
// no_malloc_scope.hpp MUST precede every Eigen header (it installs the Eigen
// allocation tripwire before <Eigen/Core> is ever seen) — see its own
// contract comment. alloc_gate.hpp DEFINES replacement global operators and
// must appear in exactly one TU per binary; this is that TU.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/contact_debounce.hpp"
#include "rtc_controllers/catching/decel_target.hpp"
#include "rtc_controllers/catching/joint_home.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::catching::CheckTransitionTableComplete;
using rtc::catching::CompletenessResult;
using rtc::catching::ContactDebounceConfig;
using rtc::catching::ContactDebouncer;
using rtc::catching::DecelEntryState;
using rtc::catching::DecelTarget;
using rtc::catching::EvaluateDecelTarget;
using rtc::catching::JointHomeStep;
using rtc::catching::JointSpaceHomeStep;
using rtc::catching::kAllModes;
using rtc::catching::kAllReasons;
using rtc::catching::kMaxFingertips;
using rtc::catching::kNumModes;
using rtc::catching::kNumReasons;
using rtc::catching::kTransitionTable;
using rtc::catching::Mode;
using rtc::catching::Reason;
using rtc::catching::TransitionRow;

// ═══════════════════════════════════════════════════════════════════════════
// G7-B — decel target: exact continuity at entry, continuity at the stop
// branch, acceleration consistent with finite differences of velocity.
// ═══════════════════════════════════════════════════════════════════════════

TEST(DecelTarget, AtEntryTheTargetEqualsTheEntryStateExactly) {
  DecelEntryState entry{Eigen::Vector3d(0.3, -0.1, 0.9), Eigen::Vector3d(1.2, -2.4, 0.5)};
  const DecelTarget t = EvaluateDecelTarget(entry, /*a_dec=*/6.0, /*tau=*/0.0);

  ASSERT_TRUE(t.valid);
  const double e = (entry.x_s - t.p_v).norm();
  const double e_dot = (entry.xdot_s - t.v_v).norm();
  EXPECT_EQ(e, 0.0) << "p_v(0) must equal x_s exactly, not just within tolerance";
  EXPECT_EQ(e_dot, 0.0) << "v_v(0) must equal ẋ_s exactly, not just within tolerance";
  EXPECT_LT(e, 1e-9);
  EXPECT_LT(e_dot, 1e-9);
  EXPECT_FALSE(t.stopped);
}

TEST(DecelTarget, ContinuousAtTheStopBranchBoundary) {
  const DecelEntryState entry{Eigen::Vector3d::Zero(), Eigen::Vector3d(2.0, 0.0, 0.0)};
  constexpr double kADec = 4.0;
  const double tau_s = entry.xdot_s.norm() / kADec;  // 0.5 s

  const DecelTarget before = EvaluateDecelTarget(entry, kADec, tau_s - 1e-6);
  const DecelTarget at = EvaluateDecelTarget(entry, kADec, tau_s);
  const DecelTarget after = EvaluateDecelTarget(entry, kADec, tau_s + 1e-6);
  ASSERT_TRUE(before.valid && at.valid && after.valid);

  EXPECT_FALSE(before.stopped);
  EXPECT_TRUE(at.stopped);
  EXPECT_TRUE(after.stopped);

  // p_v and v_v are continuous across the boundary (analytically exact; a
  // generous 1e-6 s straddle keeps this a numerics test, not a calculus one).
  EXPECT_NEAR((before.p_v - at.p_v).norm(), 0.0, 1e-5);
  EXPECT_NEAR((at.p_v - after.p_v).norm(), 0.0, 1e-5);
  EXPECT_NEAR((before.v_v - at.v_v).norm(), 0.0, 1e-4);
  EXPECT_NEAR((at.v_v - after.v_v).norm(), 0.0, 1e-9);
  EXPECT_TRUE(at.v_v.isZero());
  EXPECT_TRUE(after.v_v.isZero());

  // a_v is NOT continuous (§4.3: "기준 가속도는 불연속이다, jerk 무한") —
  // this is the documented discontinuity, not a bug.
  EXPECT_NEAR(before.a_v.norm(), kADec, 1e-9);
  EXPECT_TRUE(after.a_v.isZero());
}

TEST(DecelTarget, AccelerationMatchesFiniteDifferenceOfVelocityWithinTheRamp) {
  const DecelEntryState entry{Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 3.0, 4.0)};  // |v|=5
  constexpr double kADec = 2.0;  // tau_s=2.5
  constexpr double kTau = 0.8;
  constexpr double kH = 1e-6;

  const DecelTarget mid = EvaluateDecelTarget(entry, kADec, kTau);
  const DecelTarget plus = EvaluateDecelTarget(entry, kADec, kTau + kH);
  const DecelTarget minus = EvaluateDecelTarget(entry, kADec, kTau - kH);
  ASSERT_TRUE(mid.valid && plus.valid && minus.valid);
  ASSERT_FALSE(mid.stopped);

  const Eigen::Vector3d finite_diff = (plus.v_v - minus.v_v) / (2.0 * kH);
  EXPECT_NEAR((finite_diff - mid.a_v).norm(), 0.0, 1e-4);
  EXPECT_NEAR(mid.a_v.norm(), kADec, 1e-9);
}

TEST(DecelTarget, ZeroSpeedGuardHoldsEntryPosition) {
  const DecelEntryState entry{Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(1e-7, 0.0, 0.0)};
  const DecelTarget t = EvaluateDecelTarget(entry, 5.0, 0.2);
  ASSERT_TRUE(t.valid);
  EXPECT_TRUE(t.stopped);
  EXPECT_EQ((t.p_v - entry.x_s).norm(), 0.0);
  EXPECT_TRUE(t.v_v.isZero());
  EXPECT_TRUE(t.a_v.isZero());
}

TEST(DecelTarget, ExactZeroVelocityAtEntryIsAlsoTheZeroSpeedGuard) {
  const DecelEntryState entry{Eigen::Vector3d(0.4, 0.4, 0.4), Eigen::Vector3d::Zero()};
  const DecelTarget t = EvaluateDecelTarget(entry, 3.0, 0.0);
  ASSERT_TRUE(t.valid);
  EXPECT_TRUE(t.stopped);
  EXPECT_EQ((t.p_v - entry.x_s).norm(), 0.0);
}

TEST(DecelTarget, InvalidADecIsRejectedFailClosed) {
  const DecelEntryState entry{Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 0, 0)};
  for (double bad_a_dec : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
    const DecelTarget t = EvaluateDecelTarget(entry, bad_a_dec, 0.1);
    EXPECT_FALSE(t.valid);
    // Fail-closed: output must never be NaN even when the input was.
    EXPECT_TRUE(t.p_v.allFinite());
    EXPECT_TRUE(t.v_v.allFinite());
    EXPECT_TRUE(t.a_v.allFinite());
  }
}

TEST(DecelTarget, InvalidTauIsRejectedFailClosed) {
  const DecelEntryState entry{Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 0, 0)};
  for (double bad_tau :
       {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    const DecelTarget t = EvaluateDecelTarget(entry, 4.0, bad_tau);
    EXPECT_FALSE(t.valid);
    EXPECT_TRUE(t.p_v.allFinite());
    EXPECT_TRUE(t.v_v.allFinite());
    EXPECT_TRUE(t.a_v.allFinite());
  }
}

TEST(DecelTarget, NonFiniteEntryStateIsRejectedFailClosed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  {
    const DecelEntryState entry{Eigen::Vector3d(nan, 0, 0), Eigen::Vector3d(1, 0, 0)};
    const DecelTarget t = EvaluateDecelTarget(entry, 4.0, 0.1);
    EXPECT_FALSE(t.valid);
    EXPECT_TRUE(t.p_v.allFinite());
  }
  {
    const DecelEntryState entry{Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(nan, 0, 0)};
    const DecelTarget t = EvaluateDecelTarget(entry, 4.0, 0.1);
    EXPECT_FALSE(t.valid);
    EXPECT_TRUE(t.v_v.allFinite());
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// G7-A — transition table completeness (the scenario-replay half of G7-A is
// S7.2's RT supervisor body; this is the "전이표 완전성 검사 통과" half).
// ═══════════════════════════════════════════════════════════════════════════

TEST(TransitionTable, ShippedTableIsComplete) {
  const CompletenessResult result = CheckTransitionTableComplete(kTransitionTable);
  for (std::size_t i = 0; i < kNumModes; ++i) {
    EXPECT_FALSE(result.unreachable[i]) << "Mode index " << i << " unreachable";
    EXPECT_FALSE(result.no_exit[i]) << "Mode index " << i << " has no exit";
  }
  for (std::size_t i = 0; i < kNumReasons; ++i)
    EXPECT_FALSE(result.unused_reason[i]) << "Reason index " << i << " unused";
  EXPECT_FALSE(result.duplicate_cell);
  EXPECT_TRUE(result.Ok());
}

TEST(TransitionTable, EveryModeAndReasonEnumeratedExactlyOnceInTheIterationArrays) {
  // Non-vacuity for the completeness check itself: if kAllModes/kAllReasons
  // ever drift out of sync with the enum, the counts below catch it before
  // the completeness check silently iterates over the wrong universe.
  EXPECT_EQ(kAllModes.size(), kNumModes);
  EXPECT_EQ(kAllReasons.size(), kNumReasons);
}

TEST(TransitionTable, BrokenTableWithAnUnreachableStateIsDetected) {
  // Remove kHold's ONLY incoming edge (kDecel, kNone) -> kHold. kDecel keeps
  // other exits (its fatal-reason rows), and Reason::kNone stays used
  // elsewhere, so this isolates the unreachable-state finding.
  std::vector<TransitionRow> broken(kTransitionTable.begin(), kTransitionTable.end());
  const auto it = std::find_if(broken.begin(), broken.end(), [](const TransitionRow& r) {
    return r.from == Mode::kDecel && r.reason == Reason::kNone && r.to == Mode::kHold;
  });
  ASSERT_NE(it, broken.end());
  broken.erase(it);

  const CompletenessResult result = CheckTransitionTableComplete(broken);
  const std::size_t hold_index = static_cast<std::size_t>(Mode::kHold);
  EXPECT_TRUE(result.unreachable[hold_index]);
  EXPECT_FALSE(result.no_exit[hold_index]) << "kHold still has its own outgoing rows";
  for (std::size_t i = 0; i < kNumReasons; ++i)
    EXPECT_FALSE(result.unused_reason[i]) << "removing this row must not orphan a Reason too";
  EXPECT_FALSE(result.duplicate_cell);
  EXPECT_FALSE(result.Ok());
}

TEST(TransitionTable, BrokenTableWithAnUnusedReasonIsDetected) {
  // kNoCatchablePlan is used in exactly one row, a kTracking self-loop;
  // removing it changes no Mode's reachability or exit, isolating the
  // unused-reason finding. (kFaultReset is NOT usable here: it is kFault's
  // only exit, see FaultLeavesOnlyViaFaultReset.)
  std::vector<TransitionRow> broken(kTransitionTable.begin(), kTransitionTable.end());
  const auto it = std::find_if(broken.begin(), broken.end(), [](const TransitionRow& r) {
    return r.reason == Reason::kNoCatchablePlan;
  });
  ASSERT_NE(it, broken.end());
  broken.erase(it);

  const CompletenessResult result = CheckTransitionTableComplete(broken);
  for (std::size_t i = 0; i < kNumModes; ++i) {
    EXPECT_FALSE(result.unreachable[i]) << "index " << i;
    EXPECT_FALSE(result.no_exit[i]) << "index " << i;
  }
  const std::size_t reason_index = static_cast<std::size_t>(Reason::kNoCatchablePlan);
  EXPECT_TRUE(result.unused_reason[reason_index]);
  EXPECT_FALSE(result.duplicate_cell);
  EXPECT_FALSE(result.Ok());
}

TEST(TransitionTable, FaultLeavesOnlyViaFaultReset) {
  // §4.1 P-1 / S5.1(d): the fault is a controller latch separate from E-STOP.
  // ClearEstop (Reason::kEstop) must keep kFault; only kFaultReset leaves it.
  Mode to = Mode::kIdle;
  ASSERT_TRUE(LookupTransition(kTransitionTable, Mode::kFault, Reason::kEstop, to));
  EXPECT_EQ(to, Mode::kFault);
  ASSERT_TRUE(LookupTransition(kTransitionTable, Mode::kFault, Reason::kFaultReset, to));
  EXPECT_EQ(to, Mode::kIdle);
  for (const TransitionRow& row : kTransitionTable) {
    if (row.from == Mode::kFault && row.to != Mode::kFault) {
      EXPECT_EQ(row.reason, Reason::kFaultReset) << "unexpected kFault exit";
    }
  }
}

TEST(TransitionTable, BrokenTableWithAnUndefinedAmbiguousCellIsDetected) {
  // Add a second (kIdle, kNone) row with a DIFFERENT destination than the
  // shipped one — the table can no longer answer this query, which is what
  // "undefined cell" means here (see the header's completeness-check note).
  std::vector<TransitionRow> broken(kTransitionTable.begin(), kTransitionTable.end());
  broken.push_back({Mode::kIdle, Reason::kNone, Mode::kTracking});

  const CompletenessResult result = CheckTransitionTableComplete(broken);
  for (std::size_t i = 0; i < kNumModes; ++i) {
    EXPECT_FALSE(result.unreachable[i]) << "index " << i;
    EXPECT_FALSE(result.no_exit[i]) << "index " << i;
  }
  for (std::size_t i = 0; i < kNumReasons; ++i)
    EXPECT_FALSE(result.unused_reason[i]) << "index " << i;
  EXPECT_TRUE(result.duplicate_cell);
  EXPECT_EQ(result.duplicate_from, Mode::kIdle);
  EXPECT_EQ(result.duplicate_reason, Reason::kNone);
  EXPECT_FALSE(result.Ok());
}

TEST(TransitionTable, LookupFindsEveryShippedRow) {
  for (const TransitionRow& row : kTransitionTable) {
    Mode to{};
    ASSERT_TRUE(rtc::catching::LookupTransition(kTransitionTable, row.from, row.reason, to));
    EXPECT_EQ(to, row.to);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// G7-C — contact debounce: synthetic noise false-alarm behaviour, exact-N
// step confirmation, NaN handling.
// ═══════════════════════════════════════════════════════════════════════════

class ContactDebounceTest : public ::testing::Test {
 protected:
  static ContactDebounceConfig DefaultConfig() {
    ContactDebounceConfig cfg;
    cfg.f_min = 0.5;
    cfg.k_sigma = 3.0;
    cfg.n_debounce = 3;
    cfg.baseline_alpha = 0.05;
    return cfg;
  }
};

TEST_F(ContactDebounceTest, ConfigureRejectsInvalidConfig) {
  ContactDebouncer d;
  ContactDebounceConfig bad = DefaultConfig();
  bad.n_debounce = 0;
  EXPECT_FALSE(d.Configure(bad));
  EXPECT_FALSE(d.ConfigValid());

  bad = DefaultConfig();
  bad.k_sigma = -1.0;
  EXPECT_FALSE(d.Configure(bad));

  bad = DefaultConfig();
  bad.baseline_alpha = 0.0;
  EXPECT_FALSE(d.Configure(bad));

  EXPECT_TRUE(d.Configure(DefaultConfig()));
  EXPECT_TRUE(d.ConfigValid());
}

TEST_F(ContactDebounceTest, BaselineLearnsBiasFromConstantOffsetSamples) {
  ContactDebouncer d;
  ASSERT_TRUE(d.Configure(DefaultConfig()));
  const Eigen::Vector3d true_bias(0.1, -0.05, 0.02);
  for (int i = 0; i < 500; ++i)
    ASSERT_TRUE(d.UpdateBaseline(0, true_bias));

  EXPECT_NEAR((d.Baseline(0).bias - true_bias).norm(), 0.0, 1e-6);
  EXPECT_NEAR(d.Baseline(0).variance, 0.0, 1e-9);
}

TEST_F(ContactDebounceTest, NonFiniteBaselineSampleIsRejectedAndDoesNotCorruptTheEstimate) {
  ContactDebouncer d;
  ASSERT_TRUE(d.Configure(DefaultConfig()));
  const Eigen::Vector3d good(0.2, 0.0, 0.0);
  ASSERT_TRUE(d.UpdateBaseline(0, good));

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(d.UpdateBaseline(0, Eigen::Vector3d(nan, 0.0, 0.0)));
  EXPECT_FALSE(
      d.UpdateBaseline(0, Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), 0.0)));

  // The rejected samples must not have moved the estimate at all.
  EXPECT_EQ((d.Baseline(0).bias - good).norm(), 0.0);
  EXPECT_EQ(d.Baseline(0).variance, 0.0);
}

TEST_F(ContactDebounceTest, OutOfRangeFingertipIndexIsRejected) {
  ContactDebouncer d;
  ASSERT_TRUE(d.Configure(DefaultConfig()));
  EXPECT_FALSE(d.UpdateBaseline(kMaxFingertips, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(d.UpdateContact(kMaxFingertips, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(d.IsConfirmed(kMaxFingertips));
  EXPECT_EQ(d.ConsecutiveTrueCount(kMaxFingertips), 0u);
}

TEST_F(ContactDebounceTest, StepContactIsConfirmedAfterExactlyNDebounceSamples) {
  ContactDebouncer d;
  ContactDebounceConfig cfg = DefaultConfig();
  cfg.n_debounce = 4;
  ASSERT_TRUE(d.Configure(cfg));

  const Eigen::Vector3d rest(0.0, 0.0, 0.0);
  for (int i = 0; i < 200; ++i)
    ASSERT_TRUE(d.UpdateBaseline(0, rest));
  ASSERT_NEAR(d.Baseline(0).variance, 0.0, 1e-12);

  const Eigen::Vector3d contact_force(5.0, 0.0, 0.0);  // well above f_min=0.5
  for (std::uint32_t sample = 1; sample <= cfg.n_debounce; ++sample) {
    const bool confirmed = d.UpdateContact(0, contact_force);
    if (sample < cfg.n_debounce) {
      EXPECT_FALSE(confirmed) << "sample " << sample << " of " << cfg.n_debounce;
      EXPECT_FALSE(d.IsConfirmed(0));
    } else {
      EXPECT_TRUE(confirmed) << "sample " << sample << " of " << cfg.n_debounce;
      EXPECT_TRUE(d.IsConfirmed(0));
    }
  }
}

TEST_F(ContactDebounceTest, NonFiniteLiveSampleBreaksTheDebounceStreak) {
  ContactDebouncer d;
  ContactDebounceConfig cfg = DefaultConfig();
  cfg.n_debounce = 3;
  ASSERT_TRUE(d.Configure(cfg));
  for (int i = 0; i < 200; ++i)
    ASSERT_TRUE(d.UpdateBaseline(0, Eigen::Vector3d::Zero()));

  const Eigen::Vector3d contact_force(5.0, 0.0, 0.0);
  EXPECT_FALSE(d.UpdateContact(0, contact_force));  // 1/3
  EXPECT_FALSE(d.UpdateContact(0, contact_force));  // 2/3
  EXPECT_EQ(d.ConsecutiveTrueCount(0), 2u);

  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(d.UpdateContact(0, Eigen::Vector3d(inf, 0.0, 0.0)));
  EXPECT_EQ(d.ConsecutiveTrueCount(0), 0u) << "a non-finite sample must reset the streak";

  // Confirmation now needs a full fresh run of n_debounce samples.
  EXPECT_FALSE(d.UpdateContact(0, contact_force));  // 1/3
  EXPECT_FALSE(d.UpdateContact(0, contact_force));  // 2/3
  EXPECT_TRUE(d.UpdateContact(0, contact_force));   // 3/3
}

TEST_F(ContactDebounceTest, ResetForRearmClearsBaselineAndDebounceState) {
  ContactDebouncer d;
  ASSERT_TRUE(d.Configure(DefaultConfig()));
  for (int i = 0; i < 50; ++i)
    ASSERT_TRUE(d.UpdateBaseline(0, Eigen::Vector3d(0.3, 0.0, 0.0)));
  for (std::uint32_t i = 0; i < DefaultConfig().n_debounce; ++i)
    d.UpdateContact(0, Eigen::Vector3d(5.0, 0.0, 0.0));
  ASSERT_TRUE(d.IsConfirmed(0));

  d.ResetForRearm();

  EXPECT_FALSE(d.Baseline(0).initialized);
  EXPECT_EQ(d.Baseline(0).bias, Eigen::Vector3d::Zero());
  EXPECT_EQ(d.Baseline(0).variance, 0.0);
  EXPECT_EQ(d.ConsecutiveTrueCount(0), 0u);
  EXPECT_FALSE(d.IsConfirmed(0));
  EXPECT_TRUE(d.ConfigValid()) << "rearm must not require re-Configure()";
}

TEST_F(ContactDebounceTest, GaussianNoiseNeverFiresWithALargeKSigmaMargin) {
  ContactDebouncer d;
  ContactDebounceConfig cfg = DefaultConfig();
  cfg.f_min = 0.0;
  cfg.k_sigma = 10.0;  // generous margin — see G7-C: threshold is a user
                       // decision, this test only requires "never fires".
  cfg.n_debounce = 3;
  ASSERT_TRUE(d.Configure(cfg));

  std::mt19937 rng(12345u);
  std::normal_distribution<double> noise(0.0, 0.05);  // sigma = 0.05 N

  constexpr int kLearnSamples = 2000;
  for (int i = 0; i < kLearnSamples; ++i) {
    const Eigen::Vector3d f(noise(rng), noise(rng), noise(rng));
    ASSERT_TRUE(d.UpdateBaseline(0, f));
  }

  constexpr int kTestSamples = 20000;
  int false_alarms = 0;
  for (int i = 0; i < kTestSamples; ++i) {
    const Eigen::Vector3d f(noise(rng), noise(rng), noise(rng));
    if (d.UpdateContact(0, f))
      ++false_alarms;
  }

  const double false_alarm_rate =
      static_cast<double>(false_alarms) / static_cast<double>(kTestSamples);
  EXPECT_TRUE(std::isfinite(false_alarm_rate));
  EXPECT_EQ(false_alarms, 0) << "false-alarm rate " << false_alarm_rate
                             << " — k_sigma=10 must not fire on pure noise";
}

TEST_F(ContactDebounceTest, GaussianNoiseFalseAlarmRateAtTheDocDefaultKSigmaIsRecorded) {
  // Same setup at the doc's default k_sigma=3.0 (kOne-sided ≈0.13% per
  // sample under a Gaussian, §4.4) — recorded, not gated: "임계는 사용자
  // 결정" (L7 §9 G7-C), so this test only requires the rate to be finite.
  ContactDebouncer d;
  const ContactDebounceConfig cfg = DefaultConfig();  // k_sigma = 3.0
  ASSERT_TRUE(d.Configure(cfg));

  std::mt19937 rng(54321u);
  std::normal_distribution<double> noise(0.0, 0.05);

  constexpr int kLearnSamples = 2000;
  for (int i = 0; i < kLearnSamples; ++i) {
    const Eigen::Vector3d f(noise(rng), noise(rng), noise(rng));
    ASSERT_TRUE(d.UpdateBaseline(0, f));
  }

  constexpr int kTestSamples = 20000;
  int false_alarms = 0;
  for (int i = 0; i < kTestSamples; ++i) {
    const Eigen::Vector3d f(noise(rng), noise(rng), noise(rng));
    if (d.UpdateContact(0, f))
      ++false_alarms;
  }
  const double false_alarm_rate =
      static_cast<double>(false_alarms) / static_cast<double>(kTestSamples);
  EXPECT_TRUE(std::isfinite(false_alarm_rate));
  // Counts, not a formatted rate: std::to_string prints 6 decimals and would
  // show a small non-zero rate as 0.000000.
  RecordProperty("false_alarms_k_sigma_3",
                 std::to_string(false_alarms) + "/" + std::to_string(kTestSamples));
}

// ═══════════════════════════════════════════════════════════════════════════
// G7-D — zero RT allocation for the two per-tick pure pieces.
// ═══════════════════════════════════════════════════════════════════════════

// ── QP-independent joint-space stop (A-S5-10, S5.3) ─────────────────────────
//
// The path ABORT_SAFE takes when the abort was CAUSED by the joint command
// layer (QP_FAILED / JOINT_CONFLICT), where routing the stop back through that
// layer is the one thing that cannot be done.

namespace {

struct DecelArrays {
  std::array<double, 3> q{0.0, 0.0, 0.0};
  std::array<double, 3> qd{0.0, 0.0, 0.0};
  std::array<double, 3> qdd_max{10.0, 10.0, 10.0};
  std::array<double, 3> q_min{-3.0, -3.0, -3.0};
  std::array<double, 3> q_max{3.0, 3.0, 3.0};

  rtc::catching::JointDecelStep Step(double dt) {
    return rtc::catching::JointSpaceDecelStep(q, qd, qdd_max, q_min, q_max, 3, dt);
  }
};

}  // namespace

TEST(JointSpaceDecel, RampsEachJointDownAtItsOwnLimitAndIntegratesTheCommand) {
  // Per-joint limits, not a shared scalar: the wrist and the shoulder stop at
  // different rates, and a stop that used one number for both would either
  // over-brake one joint or under-brake the other.
  DecelArrays a;
  a.qd = {1.0, -0.5, 0.2};
  a.qdd_max = {10.0, 5.0, 100.0};
  const double dt = 0.002;

  const auto first = a.Step(dt);
  ASSERT_TRUE(first.valid);
  EXPECT_FALSE(first.stopped);
  EXPECT_DOUBLE_EQ(a.qd[0], 1.0 - 10.0 * dt);
  EXPECT_DOUBLE_EQ(a.qd[1], -(0.5 - 5.0 * dt));
  EXPECT_DOUBLE_EQ(a.qd[2], 0.0) << "0.2 rad/s at 100 rad/s^2 stops within one 2 ms tick";
  // The command is INTEGRATED from the reduced velocity, not jumped anywhere:
  // the whole point is that q_cmd stays continuous through the abort.
  EXPECT_DOUBLE_EQ(a.q[0], (1.0 - 10.0 * dt) * dt);
}

TEST(JointSpaceDecel, ReachesAFullStopInTheTimeTheLimitAllows) {
  // |q̇|/q̈max is the shortest stop the joint can perform; anything faster is
  // a number the drive will not honour.
  DecelArrays a;
  a.qd = {1.0, 0.0, 0.0};
  a.qdd_max = {10.0, 10.0, 10.0};
  const double dt = 0.002;

  int ticks = 0;
  for (; ticks < 1000; ++ticks) {
    if (a.Step(dt).stopped) {
      break;
    }
  }
  EXPECT_LT(ticks, 1000);
  EXPECT_DOUBLE_EQ(a.qd[0], 0.0);
  // 1.0 / 10.0 = 0.1 s = 50 ticks, give or take the final partial step.
  EXPECT_NEAR(static_cast<double>(ticks) * dt, 0.1, 2.0 * dt);
  // And the distance travelled is the ramp's area, not a jump.
  EXPECT_NEAR(a.q[0], 0.5 * 1.0 * 0.1, 0.01);
}

TEST(JointSpaceDecel, StopsAtThePositionBoxRatherThanPastIt) {
  // The box handed in is the one CLIK was given, already narrowed by
  // limit_margin. A stop that overshot it would put the backend's own clamp in
  // the loop, and then the command and the motion disagree.
  DecelArrays a;
  a.q = {2.999, 0.0, 0.0};
  a.qd = {1.0, 0.0, 0.0};
  const auto step = a.Step(0.002);
  ASSERT_TRUE(step.valid);
  EXPECT_TRUE(step.clamped);
  EXPECT_DOUBLE_EQ(a.q[0], 3.0);
  EXPECT_DOUBLE_EQ(a.qd[0], 0.0) << "a joint parked on its limit still reports motion";
}

TEST(JointSpaceDecel, FailsClosedOnGarbageInput) {
  // "Where should this joint go" has no honest answer when the inputs are not
  // finite, and the safe answer is "nowhere new" — NOT the measured pose,
  // which is the jump this path exists to avoid.
  DecelArrays a;
  a.q = {0.5, 0.5, 0.5};
  a.qd = {1.0, 1.0, 1.0};
  const auto bad_dt = a.Step(0.0);
  EXPECT_FALSE(bad_dt.valid);
  EXPECT_DOUBLE_EQ(a.q[0], 0.5) << "a bad dt moved the command";

  a.qdd_max[1] = std::numeric_limits<double>::quiet_NaN();
  const auto step = a.Step(0.002);
  EXPECT_TRUE(step.valid) << "one bad joint must not void the others' stop";
  EXPECT_TRUE(step.non_finite);
  EXPECT_LT(a.qd[0], 1.0) << "the usable joints still decelerated";
  // NOT stopped, and the bad joint is left ALONE. `stopped` is the caller's
  // cue to leave the abort, so claiming it here would end the stop with a
  // value that is not a number still latched in the command — and the
  // supervisor would carry it into the next trial. Zeroing the velocity would
  // also be the one-tick stop this function exists to avoid.
  EXPECT_FALSE(step.stopped);
  EXPECT_DOUBLE_EQ(a.qd[1], 1.0) << "the unusable joint was integrated anyway";
}

TEST(JointSpaceDecel, ANonFiniteCommandNeverReportsACompletedStop) {
  // The path that matters on hardware: a solver that returned NaN (ProxQP
  // reports SOLVED on a NaN problem) leaves it in the command, and the abort
  // is what is supposed to bring the arm to rest FROM there. It cannot, and
  // saying it did would hand the next trial a NaN.
  DecelArrays a;
  a.q = {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  a.qd = {0.0, 0.0, 0.0};  // every other joint is already stopped
  const auto step = a.Step(0.002);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.non_finite);
  EXPECT_FALSE(step.stopped) << "a NaN command was reported as a completed stop";
}

TEST(GateG7D, DecelTargetAndContactDebounceAllocateNothing) {
  ContactDebouncer d;
  ContactDebounceConfig cfg;
  cfg.f_min = 0.5;
  cfg.k_sigma = 3.0;
  cfg.n_debounce = 3;
  cfg.baseline_alpha = 0.05;
  ASSERT_TRUE(d.Configure(cfg));
  for (int i = 0; i < 100; ++i)
    ASSERT_TRUE(d.UpdateBaseline(0, Eigen::Vector3d::Zero()));

  const DecelEntryState entry{Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(1.0, 0.5, -0.2)};

  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;

    for (int i = 0; i < 50; ++i) {
      const DecelTarget t = EvaluateDecelTarget(entry, 5.0, 0.01 * static_cast<double>(i));
      const bool confirmed = d.UpdateContact(0, Eigen::Vector3d(5.0, 0.0, 0.0));
      // Touch the results so the compiler cannot elide the calls above.
      ASSERT_TRUE(t.valid);
      (void)confirmed;
    }

    EXPECT_EQ(heap_gate.count(), 0u);
    EXPECT_EQ(eigen_gate.violations(), 0u);
  }
}

// ── Joint-space homing / return law (S7.2, #537 S7 Q3) ──────────────────────

namespace home {

constexpr std::size_t kN = 6;
constexpr double kDt = 0.002;
constexpr double kEtaA = 0.5;
constexpr double kVMax = 0.5;
const std::array<double, kN> kQddMax{3.0, 3.0, 3.0, 5.0, 5.0, 5.0};
const std::array<double, kN> kLo{-3.0, -3.0, -3.0, -3.0, -3.0, -3.0};
const std::array<double, kN> kHi{3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

struct Run {
  int steps{-1};
  double max_speed{0.0};
  double max_dqd_over_limit{0.0};  // max over steps/joints of |Δq̇| − a·Δt
};

/// Step the law until it reports arrival (or a cap), checking the two bounds
/// the law exists to keep on every step.
Run HomeUntilArrived(std::array<double, kN>& q, std::array<double, kN>& qd,
                     const std::array<double, kN>& target, int cap = 20000) {
  Run r;
  for (int k = 0; k < cap; ++k) {
    const std::array<double, kN> qd_prev = qd;
    const JointHomeStep step =
        JointSpaceHomeStep(q, qd, target, kQddMax, kEtaA, kVMax, kLo, kHi, kN, kDt);
    EXPECT_TRUE(step.valid);
    for (std::size_t i = 0; i < kN; ++i) {
      r.max_speed = std::max(r.max_speed, std::abs(qd[i]));
      r.max_dqd_over_limit =
          std::max(r.max_dqd_over_limit, std::abs(qd[i] - qd_prev[i]) - kEtaA * kQddMax[i] * kDt);
      EXPECT_GE(q[i], kLo[i]);
      EXPECT_LE(q[i], kHi[i]);
    }
    if (step.arrived) {
      r.steps = k;
      return r;
    }
  }
  return r;
}

}  // namespace home

TEST(JointSpaceHome, FromRestArrivesExactlyWithinTheSpeedAndAccelerationBounds) {
  std::array<double, home::kN> q{0.0, -1.2, 1.4, -1.7, -1.5, 0.0};
  std::array<double, home::kN> qd{};
  const std::array<double, home::kN> target{0.212, -1.376, 1.107, -1.978, -3.296 + 1.0, 0.121};
  const home::Run r = home::HomeUntilArrived(q, qd, target);
  ASSERT_GE(r.steps, 0) << "never arrived";
  for (std::size_t i = 0; i < home::kN; ++i) {
    EXPECT_EQ(q[i], target[i]) << "arrival is exact, not approximate: joint " << i;
    EXPECT_EQ(qd[i], 0.0) << i;
  }
  EXPECT_LE(r.max_speed, home::kVMax + 1e-12);
  EXPECT_LE(r.max_dqd_over_limit, 1e-12);
  // Not faster than v_max allows: the farthest joint's distance at v_max is a
  // lower bound on the duration (a law that skipped the cap would beat it).
  const std::array<double, home::kN> start{0.0, -1.2, 1.4, -1.7, -1.5, 0.0};
  double farthest = 0.0;
  for (std::size_t i = 0; i < home::kN; ++i) {
    farthest = std::max(farthest, std::abs(target[i] - start[i]));
  }
  EXPECT_GE(r.steps, static_cast<int>(farthest / home::kVMax / home::kDt));
}

TEST(JointSpaceHome, ACarriedVelocityAwayFromTheTargetTurnsAroundAtTheLimit) {
  // A RETREAT that starts from a moving command, or a homing that starts
  // before the ramp finished: the velocity must not reverse in one tick.
  std::array<double, home::kN> q{};
  std::array<double, home::kN> qd{-0.8, -0.8, -0.8, 0.8, 0.8, 0.8};
  const std::array<double, home::kN> target{0.5, 0.5, 0.5, -0.5, -0.5, -0.5};
  const home::Run r = home::HomeUntilArrived(q, qd, target);
  ASSERT_GE(r.steps, 0);
  EXPECT_LE(r.max_dqd_over_limit, 1e-12);
  for (std::size_t i = 0; i < home::kN; ++i) {
    EXPECT_EQ(q[i], target[i]);
  }
}

TEST(JointSpaceHome, ApproachFromRestNeverOvershootsByMoreThanTheSnapBand) {
  std::array<double, home::kN> q{};
  std::array<double, home::kN> qd{};
  const std::array<double, home::kN> target{1.0, 0.3, 0.01, -1.0, -0.3, -0.01};
  for (int k = 0; k < 20000; ++k) {
    const JointHomeStep step =
        JointSpaceHomeStep(q, qd, target, home::kQddMax, home::kEtaA, home::kVMax, home::kLo,
                           home::kHi, home::kN, home::kDt);
    for (std::size_t i = 0; i < home::kN; ++i) {
      // From rest the discrete brake overshoots by at most a·Δt²/8; the snap
      // band a·Δt² is the bound the law promises, with that margin inside it.
      const double band = home::kEtaA * home::kQddMax[i] * home::kDt * home::kDt;
      const double past = target[i] > 0.0 ? q[i] - target[i] : target[i] - q[i];
      EXPECT_LE(past, band) << "joint " << i << " step " << k;
    }
    if (step.arrived) {
      return;
    }
  }
  ADD_FAILURE() << "never arrived";
}

TEST(JointSpaceHome, ATargetOutsideTheBoxEndsAtTheBox) {
  std::array<double, home::kN> q{};
  std::array<double, home::kN> qd{};
  std::array<double, home::kN> target{};
  target[0] = 4.0;  // box is ±3
  const home::Run r = home::HomeUntilArrived(q, qd, target);
  ASSERT_GE(r.steps, 0);
  EXPECT_EQ(q[0], home::kHi[0]);
}

TEST(JointSpaceHome, ANonFiniteJointIsLeftAloneAndTheStepIsNotArrived) {
  std::array<double, home::kN> q{};
  std::array<double, home::kN> qd{};
  q[2] = std::numeric_limits<double>::quiet_NaN();
  const std::array<double, home::kN> target{};  // every finite joint already there
  const JointHomeStep step =
      JointSpaceHomeStep(q, qd, target, home::kQddMax, home::kEtaA, home::kVMax, home::kLo,
                         home::kHi, home::kN, home::kDt);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.non_finite);
  EXPECT_FALSE(step.arrived);
  EXPECT_TRUE(std::isnan(q[2]));
}

TEST(JointSpaceHome, InvalidArgumentsChangeNothing) {
  std::array<double, home::kN> q{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  std::array<double, home::kN> qd{};
  const std::array<double, home::kN> target{};
  const std::array<double, home::kN> q0 = q;
  EXPECT_FALSE(JointSpaceHomeStep(q, qd, target, home::kQddMax, 0.0, home::kVMax, home::kLo,
                                  home::kHi, home::kN, home::kDt)
                   .valid);  // eta_a
  EXPECT_FALSE(JointSpaceHomeStep(q, qd, target, home::kQddMax, home::kEtaA, 0.0, home::kLo,
                                  home::kHi, home::kN, home::kDt)
                   .valid);  // v_max
  EXPECT_FALSE(JointSpaceHomeStep(q, qd, target, home::kQddMax, home::kEtaA, home::kVMax, home::kLo,
                                  home::kHi, home::kN, 0.0)
                   .valid);  // dt
  EXPECT_FALSE(JointSpaceHomeStep(q, qd, target, home::kQddMax, home::kEtaA, home::kVMax, home::kLo,
                                  home::kHi, home::kN + 1, home::kDt)
                   .valid);  // n wider than the spans
  EXPECT_EQ(q, q0);
}

TEST(JointSpaceHome, StepDoesNotAllocate) {
  std::array<double, home::kN> q{};
  std::array<double, home::kN> qd{};
  const std::array<double, home::kN> target{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  rtc::testing::ScopedAllocGate heap_gate;
  for (int k = 0; k < 500; ++k) {
    const JointHomeStep step =
        JointSpaceHomeStep(q, qd, target, home::kQddMax, home::kEtaA, home::kVMax, home::kLo,
                           home::kHi, home::kN, home::kDt);
    ASSERT_TRUE(step.valid);
  }
  EXPECT_EQ(heap_gate.count(), 0u);
}

}  // namespace
