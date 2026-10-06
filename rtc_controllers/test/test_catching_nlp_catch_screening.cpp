// NLP catch search — the lattice, the necessary conditions and the outer cost
// (E1-F14 #740; reference §8.4, §9.5, §11.1, §11.3).
//
// Every expectation here is the reference's formula written out again in the
// test, on scalars, and every inequality is driven from BOTH sides of its
// boundary: a condition tested on one side only passes with its sign flipped.
// The arm in the reach tests has a different limit on every joint and a
// non-zero start velocity, so a swapped index or a dropped v_0·T term shows.
#include "rtc_controllers/catching/nlp_catch_screening.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace {

using rtc::catching::CeilDiv;
using rtc::catching::FloorDiv;
using rtc::catching::NlpCandidateGrid;
using rtc::catching::NlpCandidateGridAt;
using rtc::catching::NlpCandidateInstant;
using rtc::catching::NlpCandidateRange;
using rtc::catching::NlpCandidatesInWindow;
using rtc::catching::NlpCellOf;
using rtc::catching::NlpClosingSpeedWindow;
using rtc::catching::NlpReachCheck;
using rtc::catching::NlpReachLimit;
using rtc::catching::NlpReachVerdict;
using rtc::catching::NlpSpeedWindow;
using rtc::catching::NlpSpeedWindowInput;
using rtc::catching::NlpSwitchCost;
using rtc::catching::NlpTimeCost;
using rtc::catching::ProjectStartIntoBox;

constexpr std::int64_t kMs = 1'000'000;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// A relative step off a boundary: far above rounding, far below any margin.
constexpr double kEdge = 1e-9;

// ── 1. Lattice ───────────────────────────────────────────────────────────────

TEST(NlpLattice, FloorAndCeilDivisionRoundTheRightWayOnBothSigns) {
  for (const std::int64_t b : {1LL, 3LL, 7LL, 1000LL}) {
    for (std::int64_t a = -50; a <= 50; ++a) {
      const double exact = static_cast<double>(a) / static_cast<double>(b);
      EXPECT_EQ(FloorDiv(a, b), static_cast<std::int64_t>(std::floor(exact))) << a << "/" << b;
      EXPECT_EQ(CeilDiv(a, b), static_cast<std::int64_t>(std::ceil(exact))) << a << "/" << b;
    }
  }
}

TEST(NlpLattice, TheWindowsCandidatesAreExactlyTheLatticeInstantsInsideIt) {
  // Brute force over a wide index range: membership by the definition
  // t_ref + i·h ∈ [lo, hi]. The anchor lies before, inside and after the
  // window, and the spacing divides neither the window nor the offsets.
  struct Case {
    std::int64_t t_ref, h, lo, hi;
  };

  for (const Case c :
       {Case{1000 * kMs, 20 * kMs, 1205 * kMs, 1950 * kMs},
        Case{1000 * kMs, 20 * kMs, 1200 * kMs, 1940 * kMs},     // both ends ON the lattice
        Case{1500 * kMs, 33 * kMs, 1205 * kMs, 1950 * kMs},     // anchor inside
        Case{3000 * kMs, 33 * kMs, 1205 * kMs, 1950 * kMs},     // anchor after: i < 0
        Case{1000 * kMs, 7 * kMs + 1, 1001 * kMs, 1002 * kMs},  // no instant inside
        Case{1000 * kMs, 20 * kMs, 1300 * kMs, 1300 * kMs}}) {  // one instant
    const NlpCandidateRange r = NlpCandidatesInWindow(c.t_ref, c.h, c.lo, c.hi);
    std::int64_t count = 0;
    for (std::int64_t i = -400; i <= 400; ++i) {
      const std::int64_t t = NlpCandidateInstant(c.t_ref, c.h, i);
      const bool inside = t >= c.lo && t <= c.hi;
      EXPECT_EQ(i >= r.first && i <= r.last, inside) << "index " << i << " h " << c.h;
      count += inside ? 1 : 0;
    }
    EXPECT_EQ(r.Count(), count);
  }
  // No spacing, or a window turned inside out: nothing.
  EXPECT_EQ(NlpCandidatesInWindow(0, 0, 0, 100).Count(), 0);
  EXPECT_EQ(NlpCandidatesInWindow(0, -5, 0, 100).Count(), 0);
  EXPECT_EQ(NlpCandidatesInWindow(0, 10, 100, 99).Count(), 0);
}

TEST(NlpLattice, ACandidatesGridUsesTheMostIntervalsThatFitAndWaitsLessThanOne) {
  constexpr std::int64_t kDt = 100 * kMs;
  constexpr std::int64_t kT0 = 5000 * kMs + 7;  // not a round instant
  // Every lead from 1 ns to past ten intervals, in steps coprime with Δ_a.
  for (std::int64_t lead = 1; lead <= 10 * kDt + 3; lead += 7'000'003) {
    const NlpCandidateGrid g = NlpCandidateGridAt(kT0 + lead, kT0, kDt);
    EXPECT_EQ(static_cast<std::int64_t>(g.n_pre) * kDt + g.wait_ns, lead) << lead;
    EXPECT_GE(g.wait_ns, 0) << lead;
    EXPECT_LT(g.wait_ns, kDt) << lead;
    EXPECT_EQ(g.t_s_ns, kT0 + g.wait_ns) << lead;
    EXPECT_EQ(g.t_s_ns, kT0 + lead - static_cast<std::int64_t>(g.n_pre) * kDt) << lead;
  }
  // Both sides of a whole number of intervals.
  for (int k = 1; k <= 10; ++k) {
    const NlpCandidateGrid on = NlpCandidateGridAt(kT0 + k * kDt, kT0, kDt);
    EXPECT_EQ(on.n_pre, k);
    EXPECT_EQ(on.wait_ns, 0);
    const NlpCandidateGrid below = NlpCandidateGridAt(kT0 + k * kDt - 1, kT0, kDt);
    EXPECT_EQ(below.n_pre, k - 1);
    EXPECT_EQ(below.wait_ns, kDt - 1);
    const NlpCandidateGrid above = NlpCandidateGridAt(kT0 + k * kDt + 1, kT0, kDt);
    EXPECT_EQ(above.n_pre, k);
    EXPECT_EQ(above.wait_ns, 1);
  }
  // Not ahead of t_0, or no spacing: no grid.
  EXPECT_EQ(NlpCandidateGridAt(kT0, kT0, kDt).n_pre, 0);
  EXPECT_EQ(NlpCandidateGridAt(kT0 - 1, kT0, kDt).n_pre, 0);
  EXPECT_EQ(NlpCandidateGridAt(kT0 + 5 * kDt, kT0, 0).n_pre, 0);
}

// ── 2. (S4) Reach ────────────────────────────────────────────────────────────

struct ReachArm {
  // Distinct on every joint, and v_0 of both signs.
  std::array<double, 4> q_0{0.30, -1.10, 0.75, 2.00};
  std::array<double, 4> v_0{0.40, -0.25, 0.00, 0.90};
  std::array<double, 4> v_max{2.0, 1.5, 3.0, 1.0};
  std::array<double, 4> a_max{8.0, 5.0, 12.0, 3.0};
};

// The reference's two conditions for one joint, written out again.
bool VelocityOk(double q_c, double q_0, double v_max, double T) {
  return std::fabs(q_c - q_0) <= v_max * T;
}

bool AccelerationOk(double q_c, double q_0, double v_0, double a_max, double T) {
  return std::fabs(q_c - q_0 - v_0 * T) <= 0.5 * a_max * T * T;
}

// A pose every joint reaches under both conditions: the drift point
// q_0 + v_0·T — the centre of the acceleration condition, and inside the
// velocity one because |v_0| < v_max on every joint.
std::array<double, 4> Reachable(const ReachArm& a, double T) {
  std::array<double, 4> q_c{};
  for (std::size_t j = 0; j < 4; ++j) {
    q_c[j] = a.q_0[j] + a.v_0[j] * T;
  }
  return q_c;
}

TEST(NlpReach, TheVelocityConditionHoldsUpToItsBoundaryOnEveryJointAndBothSides) {
  const ReachArm a;
  for (const double T : {0.2, 0.55}) {
    for (std::size_t j = 0; j < 4; ++j) {
      for (const double sign : {1.0, -1.0}) {
        SCOPED_TRACE("joint " + std::to_string(j) + " sign " + std::to_string(sign) + " T " +
                     std::to_string(T));
        std::array<double, 4> q_c = Reachable(a, T);
        const double edge = a.v_max[j] * T;
        // The acceleration condition is switched off (empty span): the boundary
        // under test is the first condition's alone.
        q_c[j] = a.q_0[j] + sign * edge * (1.0 - kEdge);
        ASSERT_TRUE(VelocityOk(q_c[j], a.q_0[j], a.v_max[j], T));
        EXPECT_TRUE(NlpReachCheck(q_c, a.q_0, a.v_0, a.v_max, {}, T).Reachable());
        q_c[j] = a.q_0[j] + sign * edge * (1.0 + kEdge);
        ASSERT_FALSE(VelocityOk(q_c[j], a.q_0[j], a.v_max[j], T));
        const NlpReachVerdict v = NlpReachCheck(q_c, a.q_0, a.v_0, a.v_max, {}, T);
        EXPECT_EQ(v.limit, NlpReachLimit::kVelocity);
        EXPECT_EQ(v.joint, static_cast<int>(j));
      }
    }
  }
}

TEST(NlpReach, TheAccelerationConditionIsCentredOnTheDriftPoint) {
  // |q^c − q_0 − v_0·T| ≤ ½ a_max T²: the reachable set under an acceleration
  // limit is centred where the start velocity carries the joint, not on q_0.
  // The velocity limits are made huge so that only this condition is in play.
  const ReachArm a;
  const std::array<double, 4> v_huge{1e6, 1e6, 1e6, 1e6};
  for (const double T : {0.2, 0.55}) {
    for (std::size_t j = 0; j < 4; ++j) {
      for (const double sign : {1.0, -1.0}) {
        SCOPED_TRACE("joint " + std::to_string(j) + " sign " + std::to_string(sign) + " T " +
                     std::to_string(T));
        std::array<double, 4> q_c = Reachable(a, T);
        const double centre = a.q_0[j] + a.v_0[j] * T;
        const double half = 0.5 * a.a_max[j] * T * T;
        q_c[j] = centre + sign * half * (1.0 - kEdge);
        ASSERT_TRUE(AccelerationOk(q_c[j], a.q_0[j], a.v_0[j], a.a_max[j], T));
        EXPECT_TRUE(NlpReachCheck(q_c, a.q_0, a.v_0, v_huge, a.a_max, T).Reachable());
        q_c[j] = centre + sign * half * (1.0 + kEdge);
        ASSERT_FALSE(AccelerationOk(q_c[j], a.q_0[j], a.v_0[j], a.a_max[j], T));
        const NlpReachVerdict v = NlpReachCheck(q_c, a.q_0, a.v_0, v_huge, a.a_max, T);
        EXPECT_EQ(v.limit, NlpReachLimit::kAcceleration);
        EXPECT_EQ(v.joint, static_cast<int>(j));
        // Without the limit the same pose is reachable.
        EXPECT_TRUE(NlpReachCheck(q_c, a.q_0, a.v_0, v_huge, {}, T).Reachable());
      }
    }
  }
}

TEST(NlpReach, AStartVelocityAwayFromTheTargetMakesANearTargetUnreachable) {
  // One joint, moving away at 0.9 rad/s, the target 0.05 rad behind it, 0.2 s:
  // the drift alone is 0.18 rad the wrong way, and ½·3·0.2² = 0.06 rad is all
  // the acceleration limit gives back. With v_0 = 0 the same target is trivial
  // — a check that dropped the v_0·T term, or took T for T², would accept it.
  const std::array<double, 1> q_0{2.0};
  const std::array<double, 1> q_c{1.95};
  const std::array<double, 1> v_max{1.0};
  const std::array<double, 1> a_max{3.0};
  const double T = 0.2;
  const std::array<double, 1> away{0.9};
  const std::array<double, 1> rest{0.0};
  ASSERT_TRUE(VelocityOk(q_c[0], q_0[0], v_max[0], T));
  ASSERT_FALSE(AccelerationOk(q_c[0], q_0[0], away[0], a_max[0], T));
  ASSERT_TRUE(AccelerationOk(q_c[0], q_0[0], rest[0], a_max[0], T));
  const NlpReachVerdict moving = NlpReachCheck(q_c, q_0, away, v_max, a_max, T);
  EXPECT_EQ(moving.limit, NlpReachLimit::kAcceleration);
  EXPECT_EQ(moving.joint, 0);
  EXPECT_TRUE(NlpReachCheck(q_c, q_0, rest, v_max, a_max, T).Reachable());
  // The two conditions are independent: a target 0.23 rad AHEAD is inside the
  // acceleration condition from this start velocity (|0.23 − 0.18| = 0.05 ≤
  // 0.06) and still out of reach — the velocity limit caps the travel at
  // v_max·T = 0.2 rad.
  const std::array<double, 1> ahead{2.0 + 0.23};
  ASSERT_TRUE(AccelerationOk(ahead[0], q_0[0], away[0], a_max[0], T));
  ASSERT_FALSE(VelocityOk(ahead[0], q_0[0], v_max[0], T));
  const NlpReachVerdict fast = NlpReachCheck(ahead, q_0, away, v_max, a_max, T);
  EXPECT_EQ(fast.limit, NlpReachLimit::kVelocity);
}

TEST(NlpReach, TheFirstFailingJointIsNamedVelocityBeforeAcceleration) {
  const ReachArm a;
  const double T = 0.3;
  std::array<double, 4> q_c = Reachable(a, T);
  // Joint 3 fails both, joint 1 the acceleration condition only.
  q_c[3] = a.q_0[3] + 10.0;
  q_c[1] = a.q_0[1] + a.v_0[1] * T + 0.5 * a.a_max[1] * T * T * 1.5;
  ASSERT_TRUE(VelocityOk(q_c[1], a.q_0[1], a.v_max[1], T));
  NlpReachVerdict v = NlpReachCheck(q_c, a.q_0, a.v_0, a.v_max, a.a_max, T);
  EXPECT_EQ(v.limit, NlpReachLimit::kAcceleration);
  EXPECT_EQ(v.joint, 1);
  q_c[1] = Reachable(a, T)[1];
  v = NlpReachCheck(q_c, a.q_0, a.v_0, a.v_max, a.a_max, T);
  EXPECT_EQ(v.limit, NlpReachLimit::kVelocity);
  EXPECT_EQ(v.joint, 3);
}

TEST(NlpReach, UnusableInputIsNotReachable) {
  const ReachArm a;
  const std::array<double, 4> q_c = Reachable(a, 0.3);
  EXPECT_TRUE(NlpReachCheck(q_c, a.q_0, a.v_0, a.v_max, a.a_max, 0.3).Reachable());
  // No time, negative time, NaN time.
  for (const double T : {0.0, -0.3, kNaN}) {
    EXPECT_EQ(NlpReachCheck(q_c, a.q_0, a.v_0, a.v_max, a.a_max, T).limit, NlpReachLimit::kInput);
  }
  // Spans that do not match.
  const std::array<double, 3> short3{0.0, 0.0, 0.0};
  EXPECT_EQ(NlpReachCheck(q_c, short3, a.v_0, a.v_max, a.a_max, 0.3).limit, NlpReachLimit::kInput);
  EXPECT_EQ(NlpReachCheck(q_c, a.q_0, short3, a.v_max, a.a_max, 0.3).limit, NlpReachLimit::kInput);
  EXPECT_EQ(NlpReachCheck(q_c, a.q_0, a.v_0, short3, a.a_max, 0.3).limit, NlpReachLimit::kInput);
  EXPECT_EQ(NlpReachCheck(q_c, a.q_0, a.v_0, a.v_max, short3, 0.3).limit, NlpReachLimit::kInput);
  // A NaN anywhere fails the joint it is on — never "inside".
  for (std::size_t j = 0; j < 4; ++j) {
    std::array<double, 4> bad = q_c;
    bad[j] = kNaN;
    const NlpReachVerdict v = NlpReachCheck(bad, a.q_0, a.v_0, a.v_max, a.a_max, 0.3);
    EXPECT_EQ(v.limit, NlpReachLimit::kVelocity);
    EXPECT_EQ(v.joint, static_cast<int>(j));
    std::array<double, 4> bad_v = a.v_0;
    bad_v[j] = kNaN;
    const NlpReachVerdict w = NlpReachCheck(q_c, a.q_0, bad_v, a.v_max, a.a_max, 0.3);
    EXPECT_EQ(w.limit, NlpReachLimit::kAcceleration);
    EXPECT_EQ(w.joint, static_cast<int>(j));
    std::array<double, 4> bad_limit = a.v_max;
    bad_limit[j] = kNaN;
    EXPECT_EQ(NlpReachCheck(q_c, a.q_0, a.v_0, bad_limit, a.a_max, 0.3).joint, static_cast<int>(j));
  }
}

// ── 3. (S3) Closing-speed window ─────────────────────────────────────────────

NlpSpeedWindowInput SetOnly() {
  NlpSpeedWindowInput in;
  in.c_min = 0.2;
  in.c_cap_max = 1.0;
  return in;
}

TEST(NlpSpeedWindowTest, WithNoRowInForceItIsTheVelocitySetsOwnInterval) {
  NlpSpeedWindowInput in = SetOnly();
  // Values that WOULD empty the window if they were read.
  in.sigma_s = 100.0;
  in.sigma_max = 1e-3;
  in.m_red = 1e6;
  in.e_max = 1e-9;
  in.p_max = 1e-9;
  const NlpSpeedWindow w = NlpClosingSpeedWindow(in);
  EXPECT_FALSE(w.empty);
  EXPECT_EQ(w.lo, 0.2);
  EXPECT_EQ(w.hi, 1.0);
  // The set itself may be empty.
  in.c_min = 1.0 + kEdge;
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
  in.c_min = 1.0;
  EXPECT_FALSE(NlpClosingSpeedWindow(in).empty);
}

TEST(NlpSpeedWindowTest, TheTimingRowRaisesTheLowerEdge) {
  // c_t,lo = σ_s / √(σ_max² − σ_τ²).
  NlpSpeedWindowInput in = SetOnly();
  in.timing = true;
  in.sigma_max = 0.012;
  in.sigma_tau = 0.004;
  const double k = std::sqrt(0.012 * 0.012 - 0.004 * 0.004);
  // Below c_min: the set's own edge stays.
  in.sigma_s = 0.1 * k;
  NlpSpeedWindow w = NlpClosingSpeedWindow(in);
  EXPECT_FALSE(w.empty);
  EXPECT_DOUBLE_EQ(w.c_t_lo, 0.1);
  EXPECT_EQ(w.lo, 0.2);
  // Between the edges.
  in.sigma_s = 0.6 * k;
  w = NlpClosingSpeedWindow(in);
  EXPECT_FALSE(w.empty);
  EXPECT_DOUBLE_EQ(w.lo, 0.6);
  EXPECT_EQ(w.hi, 1.0);
  // Both sides of the upper edge.
  in.sigma_s = 1.0 * k * (1.0 - kEdge);
  EXPECT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.sigma_s = 1.0 * k * (1.0 + kEdge);
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
  // A larger latency jitter narrows the window: the same σ_s that fitted does
  // not once σ_τ has eaten the budget (the term is σ_max² − σ_τ², not + ).
  in.sigma_s = 0.9 * k;
  ASSERT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.sigma_tau = 0.008;
  EXPECT_DOUBLE_EQ(NlpClosingSpeedWindow(in).c_t_lo,
                   0.9 * k / std::sqrt(0.012 * 0.012 - 0.008 * 0.008));
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
  // σ_τ at or above σ_max: the jitter alone breaks the requirement.
  in.sigma_s = 1e-6;
  in.sigma_tau = 0.012;
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
  in.sigma_tau = 0.012 * (1.0 - 1e-6);
  EXPECT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.sigma_tau = 0.02;
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
}

TEST(NlpSpeedWindowTest, TheImpactRowsLowerTheUpperEdgeWhicheverBinds) {
  // c_n,hi = min{ √(2 E_max / m_red),  P_max / ((1 + e) m_red) }.
  NlpSpeedWindowInput in = SetOnly();
  in.impact = true;
  in.m_red = 0.05;
  in.restitution = 0.4;
  const auto by_energy = [&](double c) { return 0.5 * in.m_red * c * c; };
  const auto by_impulse = [&](double c) { return (1.0 + in.restitution) * in.m_red * c; };
  // Energy binds at 0.7 m/s, the impulse row is off.
  in.e_max = by_energy(0.7);
  in.p_max = kInf;
  NlpSpeedWindow w = NlpClosingSpeedWindow(in);
  EXPECT_FALSE(w.empty);
  EXPECT_DOUBLE_EQ(w.hi, 0.7);
  EXPECT_EQ(w.lo, 0.2);
  // Impulse binds at 0.5 m/s, the energy row is off.
  in.e_max = kInf;
  in.p_max = by_impulse(0.5);
  w = NlpClosingSpeedWindow(in);
  EXPECT_FALSE(w.empty);
  EXPECT_DOUBLE_EQ(w.hi, 0.5);
  // Both on: the smaller.
  in.e_max = by_energy(0.4);
  EXPECT_DOUBLE_EQ(NlpClosingSpeedWindow(in).hi, 0.4);
  in.e_max = by_energy(0.9);
  EXPECT_DOUBLE_EQ(NlpClosingSpeedWindow(in).hi, 0.5);
  // Both off: the set's own edge.
  in.e_max = kInf;
  in.p_max = kInf;
  w = NlpClosingSpeedWindow(in);
  EXPECT_FALSE(w.empty);
  EXPECT_EQ(w.hi, 1.0);
  // Both sides of the lower edge, for each row.
  in.e_max = by_energy(0.2 * (1.0 + kEdge));
  EXPECT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.e_max = by_energy(0.2 * (1.0 - kEdge));
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
  in.e_max = kInf;
  in.p_max = by_impulse(0.2 * (1.0 + kEdge));
  EXPECT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.p_max = by_impulse(0.2 * (1.0 - kEdge));
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
  // A heavier contact needs a slower ball: the same limits, a larger m_red.
  in.p_max = by_impulse(0.5);
  ASSERT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.m_red *= 4.0;
  EXPECT_DOUBLE_EQ(NlpClosingSpeedWindow(in).hi, 0.125);
  EXPECT_TRUE(NlpClosingSpeedWindow(in).empty);
}

TEST(NlpSpeedWindowTest, BothRowsTogetherCanEmptyAWindowEitherLeavesOpen) {
  NlpSpeedWindowInput in = SetOnly();
  in.sigma_max = 0.010;
  in.sigma_tau = 0.0;
  in.sigma_s = 0.006;  // c_t,lo = 0.6
  in.m_red = 0.05;
  in.restitution = 0.0;
  in.e_max = kInf;
  in.p_max = 0.05 * 0.5;  // c_n,hi = 0.5
  in.timing = true;
  EXPECT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.timing = false;
  in.impact = true;
  EXPECT_FALSE(NlpClosingSpeedWindow(in).empty);
  in.timing = true;
  const NlpSpeedWindow w = NlpClosingSpeedWindow(in);
  EXPECT_TRUE(w.empty);
  EXPECT_DOUBLE_EQ(w.lo, 0.6);
  EXPECT_DOUBLE_EQ(w.hi, 0.5);
}

TEST(NlpSpeedWindowTest, ATermThatCannotBeEvaluatedEmptiesTheWindow) {
  NlpSpeedWindowInput ok = SetOnly();
  ok.timing = true;
  ok.sigma_max = 0.010;
  ok.sigma_s = 0.003;
  ok.impact = true;
  ok.m_red = 0.05;
  ok.e_max = kInf;
  ok.p_max = kInf;
  ASSERT_FALSE(NlpClosingSpeedWindow(ok).empty);
  const auto broken = [&](auto&& edit) {
    NlpSpeedWindowInput in = ok;
    edit(in);
    return NlpClosingSpeedWindow(in).empty;
  };
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.sigma_s = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.sigma_max = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.sigma_tau = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.m_red = 0.0; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.m_red = -0.05; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.m_red = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.e_max = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.p_max = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.restitution = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.c_min = kNaN; }));
  EXPECT_TRUE(broken([](NlpSpeedWindowInput& in) { in.c_cap_max = kNaN; }));
}

// ── 4. The outer cost (§9.5) ─────────────────────────────────────────────────

TEST(NlpOuterCost, TimeAndSwitchTermsAreTheReferencesFormulas) {
  // J_time = w_T · T / T_ref.
  EXPECT_DOUBLE_EQ(NlpTimeCost(0.3, 0.45, 0.9), 0.3 * 0.45 / 0.9);
  EXPECT_EQ(NlpTimeCost(0.0, 0.45, 0.9), 0.0);
  EXPECT_GT(NlpTimeCost(0.3, 0.60, 0.9), NlpTimeCost(0.3, 0.45, 0.9));
  // J_switch = w_sw · ((t_c − t_c,prev) / T_ref)²: even in the difference, zero
  // at the instant in force, and nothing without one.
  const std::int64_t prev = 7000 * kMs;
  const double w = 2.5;
  const double t_ref = 0.5;
  EXPECT_EQ(NlpSwitchCost(w, prev, true, prev, t_ref), 0.0);
  for (const std::int64_t d : {20 * kMs, 60 * kMs, 333 * kMs}) {
    const double expected = w * std::pow(static_cast<double>(d) * 1e-9 / t_ref, 2);
    EXPECT_DOUBLE_EQ(NlpSwitchCost(w, prev + d, true, prev, t_ref), expected);
    EXPECT_DOUBLE_EQ(NlpSwitchCost(w, prev - d, true, prev, t_ref), expected);
    EXPECT_EQ(NlpSwitchCost(w, prev + d, false, prev, t_ref), 0.0);
  }
  // One nanosecond is a difference: the anchor is compared as an instant.
  EXPECT_GT(NlpSwitchCost(w, prev + 1, true, prev, t_ref), 0.0);
}

// ── 5. The start state and the box ───────────────────────────────────────────

TEST(NlpStartBox, AStateInsideIsUntouchedAndOneOutsideIsMovedToTheNearestFace) {
  const std::array<double, 3> lo{-1.0, -2.0, 0.5};
  const std::array<double, 3> hi{1.0, 0.0, 0.9};
  const std::array<double, 3> v_hi{2.0, 0.5, 1.0};
  {
    // On every face: inside.
    std::array<double, 3> q{-1.0, 0.0, 0.7};
    std::array<double, 3> qd{2.0, -0.5, 0.0};
    const auto q_was = q;
    const auto qd_was = qd;
    EXPECT_FALSE(ProjectStartIntoBox(q, qd, lo, hi, v_hi));
    EXPECT_EQ(q, q_was);
    EXPECT_EQ(qd, qd_was);
  }
  {
    std::array<double, 3> q{-1.0 - 1e-9, 0.3, 0.95};
    std::array<double, 3> qd{0.1, 0.5 + 1e-9, -3.0};
    EXPECT_TRUE(ProjectStartIntoBox(q, qd, lo, hi, v_hi));
    EXPECT_EQ(q, (std::array<double, 3>{-1.0, 0.0, 0.9}));
    EXPECT_EQ(qd, (std::array<double, 3>{0.1, 0.5, -1.0}));
    // A second pass has nothing to do.
    EXPECT_FALSE(ProjectStartIntoBox(q, qd, lo, hi, v_hi));
  }
  {
    // One entry alone is enough to report it — position or velocity.
    std::array<double, 3> q{0.0, -1.0, 0.7};
    std::array<double, 3> qd{0.0, 0.0, 1.0 + 1e-12};
    EXPECT_TRUE(ProjectStartIntoBox(q, qd, lo, hi, v_hi));
    EXPECT_EQ(qd[2], 1.0);
    std::array<double, 3> q2{0.0, 1e-12, 0.7};
    std::array<double, 3> qd2{0.0, 0.0, 0.0};
    EXPECT_TRUE(ProjectStartIntoBox(q2, qd2, lo, hi, v_hi));
    EXPECT_EQ(q2[1], 0.0);
  }
  {
    // Sizes that do not match: nothing is touched.
    std::array<double, 3> q{9.0, 9.0, 9.0};
    std::array<double, 2> qd{9.0, 9.0};
    EXPECT_FALSE(ProjectStartIntoBox(q, qd, lo, hi, v_hi));
    EXPECT_EQ(q[0], 9.0);
  }
}

// Every instant is in exactly one cell, the cell's own lattice instant is in
// it, and the cells are h long — for an even and an odd spacing, on both
// sides of the anchor.
TEST(NlpCatchScreening, ALatticeCellIsHalfOpenAroundItsInstant) {
  for (const std::int64_t h : {std::int64_t{40'000'000}, std::int64_t{7}}) {
    const std::int64_t t_ref = 1'000'000'000'123LL;
    for (std::int64_t i = -3; i <= 3; ++i) {
      const std::int64_t t = NlpCandidateInstant(t_ref, h, i);
      EXPECT_EQ(NlpCellOf(t_ref, h, t), i);
      // The cell is [t − ⌊h/2⌋, t − ⌊h/2⌋ + h).
      EXPECT_EQ(NlpCellOf(t_ref, h, t - h / 2), i);
      EXPECT_EQ(NlpCellOf(t_ref, h, t - h / 2 - 1), i - 1);
      EXPECT_EQ(NlpCellOf(t_ref, h, t - h / 2 + h - 1), i);
      EXPECT_EQ(NlpCellOf(t_ref, h, t - h / 2 + h), i + 1);
    }
    // Consecutive instants never skip or repeat a boundary.
    std::int64_t last = NlpCellOf(t_ref, h, t_ref - 3 * h);
    for (std::int64_t t = t_ref - 3 * h + 1; t <= t_ref + 3 * h && h < 100; ++t) {
      const std::int64_t cell = NlpCellOf(t_ref, h, t);
      EXPECT_TRUE(cell == last || cell == last + 1) << t;
      last = cell;
    }
  }
}

}  // namespace
