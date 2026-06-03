// Unit tests for QuinticPolynomial (rtc::trajectory::trajectory_utils).
//
// The quintic blend/spline trajectories exercise the regular T>0 evaluation
// path, but the degenerate T<=0 short-circuit — where compute_coefficients()
// collapses to the boundary state and compute() returns that constant for any
// t — was previously uncovered. These tests pin both the boundary-condition
// math and that short-circuit directly on the standalone polynomial.

#include "rtc_controllers/trajectory/trajectory_utils.hpp"

#include <gtest/gtest.h>

using rtc::trajectory::QuinticPolynomial;

// Rest-to-rest quintic: exact boundary conditions at t=0 and t=T, plus the
// symmetry midpoint (pos = halfway at t=T/2).
TEST(QuinticPolynomial, RestToRestBoundaryConditions) {
  QuinticPolynomial poly(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);
  EXPECT_DOUBLE_EQ(poly.duration(), 2.0);

  const auto s0 = poly.compute(0.0);
  EXPECT_NEAR(s0.pos, 0.0, 1e-12);
  EXPECT_NEAR(s0.vel, 0.0, 1e-12);
  EXPECT_NEAR(s0.acc, 0.0, 1e-12);

  const auto sf = poly.compute(2.0);
  EXPECT_NEAR(sf.pos, 1.0, 1e-9);
  EXPECT_NEAR(sf.vel, 0.0, 1e-9);
  EXPECT_NEAR(sf.acc, 0.0, 1e-9);

  // Symmetric rest-to-rest profile crosses the halfway point at t = T/2.
  const auto smid = poly.compute(1.0);
  EXPECT_NEAR(smid.pos, 0.5, 1e-9);
}

// compute() clamps t into [0, T] before evaluating, so out-of-range samples
// return the boundary states.
TEST(QuinticPolynomial, ClampsTimeOutsideRange) {
  QuinticPolynomial poly(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);
  EXPECT_NEAR(poly.compute(-1.0).pos, 0.0, 1e-9);
  EXPECT_NEAR(poly.compute(5.0).pos, 1.0, 1e-9);
}

// T == 0 short-circuit: coefficients collapse to (pf, vf, af/2); compute()
// returns that constant boundary state for *any* t (incl. negative / large).
TEST(QuinticPolynomial, ZeroDurationReturnsFinalBoundaryState) {
  QuinticPolynomial poly(0.0, 0.0, 0.0, 3.0, 1.5, 4.0, 0.0);
  EXPECT_DOUBLE_EQ(poly.duration(), 0.0);

  for (double t : {-1.0, 0.0, 0.5, 10.0}) {
    const auto s = poly.compute(t);
    EXPECT_DOUBLE_EQ(s.pos, 3.0) << "t=" << t;  // c0_ = pf
    EXPECT_DOUBLE_EQ(s.vel, 1.5) << "t=" << t;  // c1_ = vf
    EXPECT_DOUBLE_EQ(s.acc, 4.0) << "t=" << t;  // 2*c2_ = af
  }
}

// Negative duration takes the same T<=0 branch as zero.
TEST(QuinticPolynomial, NegativeDurationTreatedAsZero) {
  QuinticPolynomial poly(0.0, 0.0, 0.0, 2.0, 0.0, 0.0, -1.0);
  EXPECT_DOUBLE_EQ(poly.duration(), 0.0);
  EXPECT_DOUBLE_EQ(poly.compute(0.3).pos, 2.0);
}

// Default-constructed polynomial: all coefficients + T_ are zero, so compute()
// yields the zero state regardless of t.
TEST(QuinticPolynomial, DefaultConstructedIsZeroState) {
  QuinticPolynomial poly;
  EXPECT_DOUBLE_EQ(poly.duration(), 0.0);
  const auto s = poly.compute(1.0);
  EXPECT_DOUBLE_EQ(s.pos, 0.0);
  EXPECT_DOUBLE_EQ(s.vel, 0.0);
  EXPECT_DOUBLE_EQ(s.acc, 0.0);
}

// compute_coefficients() re-arms a default-constructed instance in place.
TEST(QuinticPolynomial, ComputeCoefficientsRecomputes) {
  QuinticPolynomial poly;
  poly.compute_coefficients(1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(poly.duration(), 1.0);
  EXPECT_NEAR(poly.compute(0.0).pos, 1.0, 1e-12);
  EXPECT_NEAR(poly.compute(1.0).pos, 2.0, 1e-9);
}
