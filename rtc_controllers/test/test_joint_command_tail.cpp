// §7.3 joint command tail — the ORDER is the contract, so every test here is
// built to go red when one of the three steps is removed or the first two are
// swapped. A test that only checks "the command ended up in range" passes under
// the reversed order too, which is the failure this unit exists to prevent.
#include "rtc_controllers/compliance/joint_command_tail.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <span>
#include <vector>

namespace {

using rtc::compliance::IntegrateAndBoundJointCommand;
using rtc::compliance::JointCommandBounds;

JointCommandBounds MakeBounds(const std::vector<double>& lo, const std::vector<double>& hi,
                              const std::vector<double>& v, double margin = 0.0) {
  JointCommandBounds b;
  b.lower = std::span<const double>(lo);
  b.upper = std::span<const double>(hi);
  b.max_velocity = std::span<const double>(v);
  b.margin = margin;
  return b;
}

// The case §7.3 is written about: the command sits OUTSIDE the band and the law
// asks for a tiny step. The position clamp yanks it to the boundary — a 0.1 rad
// jump from a 0.001 rad request — and only a rate rebound that runs AFTER the
// clamp bounds what actually reaches the wire.
//
// Reversing the two steps produces 0.0 here (the full jump) while every value in
// the row still looks sane, which is why this is the first test in the file.
TEST(JointCommandTail, TheClampRunsBeforeTheRateRebound) {
  const std::vector<double> lo{0.0}, hi{1.0}, vmax{0.1};
  std::array<double, 1> q{-0.1};
  std::array<double, 1> dq{0.01};  // 0.001 rad over a 0.1 s tick
  const double dt = 0.1;
  const double q_base = q[0];

  const auto report = IntegrateAndBoundJointCommand(q, dq, 1, dt, MakeBounds(lo, hi, vmax));

  // Both bounds fired: the clamp pulled it to 0.0, the rebound pulled it back.
  EXPECT_EQ(report.position_clamped, 1);
  EXPECT_EQ(report.rate_rebounded, 1);
  // The step the arm is asked to take is bounded by v_max·dt = 0.01, NOT by the
  // 0.1 the clamp alone would have produced.
  EXPECT_LE(std::abs(q[0] - q_base), 0.01 + 1e-12);
  EXPECT_NEAR(q[0], -0.09, 1e-12);
  // Reversed order would land exactly here. Pinned so the distinction is stated
  // rather than implied by a tolerance.
  EXPECT_NE(q[0], 0.0);
}

// Kills "drop the position clamp": without it the command walks straight past
// the joint limit, and no other step in the tail can stop it (the rebound only
// bounds the STEP, and a bounded step past a limit is still past the limit).
TEST(JointCommandTail, AStepPastAJointLimitStopsAtTheBand) {
  const std::vector<double> lo{-1.0}, hi{1.0}, vmax{10.0};
  std::array<double, 1> q{0.9};
  std::array<double, 1> dq{5.0};  // 0.5 rad over the tick — well inside v_max
  const double dt = 0.1;

  const auto report = IntegrateAndBoundJointCommand(q, dq, 1, dt, MakeBounds(lo, hi, vmax));

  EXPECT_EQ(report.position_clamped, 1);
  EXPECT_EQ(report.rate_rebounded, 0);  // v_max·dt = 1.0 ≥ the 0.1 that survived
  EXPECT_DOUBLE_EQ(q[0], 1.0);
}

// The margin shrinks the band on BOTH sides. Same input as above, so the only
// thing that can move the answer is δ.
TEST(JointCommandTail, TheMarginShrinksTheBandOnBothSides) {
  const std::vector<double> lo{-1.0}, hi{1.0}, vmax{10.0};
  std::array<double, 1> q{0.9};
  std::array<double, 1> dq{5.0};
  IntegrateAndBoundJointCommand(q, dq, 1, 0.1, MakeBounds(lo, hi, vmax, 0.08));
  EXPECT_DOUBLE_EQ(q[0], 0.92);

  std::array<double, 1> q_lo{-0.9};
  std::array<double, 1> dq_lo{-5.0};
  IntegrateAndBoundJointCommand(q_lo, dq_lo, 1, 0.1, MakeBounds(lo, hi, vmax, 0.08));
  EXPECT_DOUBLE_EQ(q_lo[0], -0.92);
}

// Kills "drop the round-trip": `target_velocities` is what the log and the
// publish lane report, and after two bounds the solve output is no longer the
// velocity the arm was asked for. Asserted as an exact identity, not a bound —
// an approximate check passes on the un-rewritten value whenever the bounds
// happen to be loose.
TEST(JointCommandTail, TheReportedVelocityIsTheOneActuallyCommanded) {
  const std::vector<double> lo{-1.0}, hi{1.0}, vmax{10.0};
  std::array<double, 1> q{0.9};
  std::array<double, 1> dq{5.0};
  const double dt = 0.1;
  const double q_base = q[0];

  IntegrateAndBoundJointCommand(q, dq, 1, dt, MakeBounds(lo, hi, vmax));

  EXPECT_DOUBLE_EQ(dq[0], (q[0] - q_base) / dt);
  EXPECT_DOUBLE_EQ(dq[0], 1.0);  // and NOT the 5.0 that went in
}

// A δ wide enough to invert a band skips the clamp for that joint. It must not
// collapse the command onto a point or hand back a reversed interval — this is
// the RT half of a defect whose configure-time half the bindings reject (#473).
TEST(JointCommandTail, AnInvertingMarginSkipsTheClampRatherThanCollapsingTheCommand) {
  const std::vector<double> lo{-0.1}, hi{0.1}, vmax{10.0};
  std::array<double, 1> q{0.0};
  std::array<double, 1> dq{1.0};

  const auto report = IntegrateAndBoundJointCommand(q, dq, 1, 0.1, MakeBounds(lo, hi, vmax, 0.5));

  EXPECT_EQ(report.position_clamped, 0);
  EXPECT_DOUBLE_EQ(q[0], 0.1);  // the un-clamped integration, rebound-bounded only
}

// dt ≤ 0 is a no-op on BOTH spans. The round-trip divides by dt, so the
// alternative is a NaN command from a harness that simply had no clock yet.
TEST(JointCommandTail, ANonPositiveDtMovesNothing) {
  const std::vector<double> lo{-1.0}, hi{1.0}, vmax{10.0};
  for (const double dt : {0.0, -0.001}) {
    std::array<double, 1> q{0.25};
    std::array<double, 1> dq{7.0};
    const auto report = IntegrateAndBoundJointCommand(q, dq, 1, dt, MakeBounds(lo, hi, vmax));
    EXPECT_EQ(report.position_clamped, 0);
    EXPECT_EQ(report.rate_rebounded, 0);
    EXPECT_DOUBLE_EQ(q[0], 0.25);
    EXPECT_DOUBLE_EQ(dq[0], 7.0);
  }
}

// A config that describes only some joints must behave the same way here as it
// does in rtc::utils::ClampRange: the described joints use their limits, the
// rest fall back. Joint 1 is deliberately left uncovered by all three vectors.
TEST(JointCommandTail, ShortLimitVectorsFallBackPerJoint) {
  const std::vector<double> lo{-0.2}, hi{0.2}, vmax{100.0};
  std::array<double, 2> q{0.0, 0.0};
  std::array<double, 2> dq{10.0, 10.0};

  IntegrateAndBoundJointCommand(q, dq, 2, 0.1, MakeBounds(lo, hi, vmax));

  EXPECT_DOUBLE_EQ(q[0], 0.2);  // covered: clamped at its own limit
  EXPECT_DOUBLE_EQ(q[1], 0.2);  // uncovered: ±2π band, bounded by the 2.0 rad/s default
}

// A non-finite command is passed through, not laundered into a plausible one.
// Every comparison against NaN is false, so both bounds decline to move it and
// it reaches the fault classification that owns it.
TEST(JointCommandTail, ANonFiniteCommandIsNotScrubbed) {
  const std::vector<double> lo{-1.0}, hi{1.0}, vmax{10.0};
  std::array<double, 1> q{0.0};
  std::array<double, 1> dq{std::nan("")};

  IntegrateAndBoundJointCommand(q, dq, 1, 0.1, MakeBounds(lo, hi, vmax));

  EXPECT_TRUE(std::isnan(q[0]));
}

// The span-shortest wins, so a caller whose n outruns its buffers cannot walk
// off either one (#172's hazard, in the shape this helper can meet it).
TEST(JointCommandTail, TheBoundIsTheShortestOfNAndBothSpans) {
  const std::vector<double> lo{-1.0}, hi{1.0}, vmax{10.0};
  std::array<double, 2> q{0.5, 0.5};
  std::array<double, 1> dq{10.0};

  IntegrateAndBoundJointCommand(q, dq, 8, 0.1, MakeBounds(lo, hi, vmax));

  EXPECT_DOUBLE_EQ(q[0], 1.0);  // bounded by the dq span, not by n
  EXPECT_DOUBLE_EQ(q[1], 0.5);  // untouched
}

}  // namespace
