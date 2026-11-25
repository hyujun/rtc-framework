#include "rtc_controllers/trajectory/continuous_retarget.hpp"
#include "rtc_controllers/trajectory/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

namespace rtc::trajectory {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPeakFactor = 15.0 / 8.0;  // rest-to-rest quintic: peak / mean velocity
constexpr int kOracleSamples = 2000;

/// What a caller builds the base from: max distance / speed, each joint's
/// rest-to-rest peak held to its cap, and a one-tick floor.
double BaseDuration(const std::vector<double>& q0, const std::vector<double>& goal,
                    const std::vector<double>& cap, double speed, double dt) {
  double max_dist = 0.0;
  double duration = 0.0;
  for (std::size_t i = 0; i < q0.size(); ++i) {
    const double dist = std::abs(goal[i] - q0[i]);
    max_dist = std::max(max_dist, dist);
    duration = std::max(duration, kPeakFactor * dist / cap[i]);
  }
  return std::max({duration, max_dist / speed, dt});
}

struct Case {
  std::vector<double> q0, v0, goal, cap, lo, hi;
  double base{0.0};
};

/// The trajectory the plan describes, sampled densely and independently of the
/// planner's closed form: every bound the planner promises, checked.
struct OracleResult {
  double peak_ratio{0.0};  // max over joints of peak |v| / cap
  double limit_excess{0.0};
  double goal_pass{0.0};  // largest travel past the goal by a joint moving towards it
};

OracleResult Oracle(const Case& c, const std::vector<double>& v0_in,
                    const std::vector<double>& v0_out, double duration) {
  OracleResult r;
  for (std::size_t i = 0; i < c.q0.size(); ++i) {
    const QuinticPolynomial poly(c.q0[i], v0_out[i], 0.0, c.goal[i], 0.0, 0.0, duration);
    const double delta = c.goal[i] - c.q0[i];
    const bool towards = v0_in[i] * delta > 0.0;
    for (int k = 0; k <= kOracleSamples; ++k) {
      const auto s = poly.compute(duration * k / kOracleSamples);
      r.peak_ratio = std::max(r.peak_ratio, std::abs(s.vel) / c.cap[i]);
      r.limit_excess = std::max({r.limit_excess, s.pos - c.hi[i], c.lo[i] - s.pos});
      if (towards) {
        r.goal_pass = std::max(r.goal_pass, (s.pos - c.goal[i]) * (delta > 0.0 ? 1.0 : -1.0));
      }
    }
    const auto end = poly.compute(duration);
    EXPECT_NEAR(end.pos, c.goal[i], 1e-9);
    EXPECT_NEAR(end.vel, 0.0, 1e-9);
  }
  return r;
}

ContinuousRetargetPlan Plan(const Case& c, std::vector<double>& v0) {
  return PlanContinuousRetarget(c.q0, v0, c.goal, c.cap, c.lo, c.hi, c.base);
}

Case OneJoint(double q0, double v0, double goal, double cap, double base, double lo = -kInf,
              double hi = kInf) {
  return Case{{q0}, {v0}, {goal}, {cap}, {lo}, {hi}, base};
}

TEST(ContinuousRetarget, AllAtRestReturnsTheBaseAndZeroVelocities) {
  Case c{{0.1, -0.2}, {0.0, 0.0}, {0.5, 0.3}, {2.0, 2.0}, {-1.0, -1.0}, {1.0, 1.0}, 0.37};
  auto v0 = c.v0;
  const auto plan = Plan(c, v0);
  EXPECT_EQ(plan.duration, 0.37);  // bit for bit: the caller's rest-to-rest move
  EXPECT_EQ(v0, (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(plan.kept, 0);
  EXPECT_EQ(plan.dropped, 0);
}

// A switch at 50 % of a rest-to-rest move: the joint is at its peak velocity
// with half the distance left. Over a short enough duration it keeps its
// velocity; over the old move's duration it would already pass the goal by
// ≈ 4.6 % of the move (|v0|·T = 1.875 > 2.5·0.5), so it is slowed — never zeroed.
TEST(ContinuousRetarget, AJointMovingTowardsItsGoalNeverPassesIt) {
  const double v_peak = kPeakFactor * 1.0 / 1.0;  // Δ_old = 1 rad over 1 s
  {
    auto c = OneJoint(0.5, v_peak, 1.0, 10.0, 0.6);
    auto v0 = c.v0;
    const auto plan = Plan(c, v0);
    EXPECT_EQ(plan.duration, 0.6);
    EXPECT_EQ(v0[0], v_peak);  // |v0|·T = 1.125 <= 2.5·0.5: kept as is
    EXPECT_EQ(plan.kept, 1);
    EXPECT_EQ(plan.reduced, 0);
  }
  {
    auto c = OneJoint(0.5, v_peak, 1.0, 10.0, 1.0);
    auto v0 = c.v0;
    const auto plan = Plan(c, v0);
    EXPECT_EQ(plan.reduced, 1);
    EXPECT_DOUBLE_EQ(v0[0], 2.5 * 0.5 / 1.0);
    EXPECT_LE(Oracle(c, c.v0, v0, plan.duration).goal_pass, 1e-12);
    // Unslowed, the same move passes the goal: the bound is what prevents it.
    EXPECT_GT(Oracle(c, c.v0, c.v0, plan.duration).goal_pass, 0.04);
  }
}

TEST(ContinuousRetarget, AReversingJointKeepsItsVelocityAndTravelsOn) {
  auto c = OneJoint(0.5, 1.875, 0.0, 10.0, 1.0, -1.0, 1.0);
  auto v0 = c.v0;
  const auto plan = Plan(c, v0);
  EXPECT_EQ(v0[0], 1.875);
  EXPECT_EQ(plan.kept, 1);
  double furthest = 0.5;
  const QuinticPolynomial poly(0.5, v0[0], 0.0, 0.0, 0.0, 0.0, plan.duration);
  for (int k = 0; k <= 1000; ++k) {
    furthest = std::max(furthest, poly.compute(plan.duration * k / 1000).pos);
  }
  EXPECT_GT(furthest, 0.5 + 0.2);  // goes on before it turns: the wave, not a stop
}

TEST(ContinuousRetarget, AReversalThatWouldLeaveTheLimitsStartsAtRest) {
  auto c = OneJoint(0.95, 1.875, 0.0, 10.0, 1.0, -1.0, 1.0);
  auto v0 = c.v0;
  const auto plan = Plan(c, v0);
  EXPECT_EQ(v0[0], 0.0);
  EXPECT_EQ(plan.dropped, 1);
  EXPECT_EQ(plan.duration, 1.0);  // nobody left moving: the base
  EXPECT_LE(Oracle(c, c.v0, v0, plan.duration).limit_excess, 0.0);
}

TEST(ContinuousRetarget, AGoalOnTheLimitDoesNotDropAJointMovingTowardsIt) {
  auto c = OneJoint(0.5, 1.0, 1.0, 10.0, 0.5, -1.0, 1.0);
  auto v0 = c.v0;
  const auto plan = Plan(c, v0);
  EXPECT_EQ(plan.dropped, 0);
  EXPECT_EQ(v0[0], 1.0);
}

TEST(ContinuousRetarget, NoDistanceLeftMeansTheJointStops) {
  auto c = OneJoint(0.3, 2.0, 0.3, 10.0, 0.002);
  auto v0 = c.v0;
  const auto plan = Plan(c, v0);
  EXPECT_EQ(v0[0], 0.0);  // any speed would pass a goal it is already at
  EXPECT_EQ(plan.dropped, 1);
  EXPECT_EQ(plan.duration, 0.002);
}

TEST(ContinuousRetarget, UnusableVelocitiesAreSanitised) {
  Case c{{0.0, 0.0, 0.0, 0.0},
         {std::nan(""), kInf, 1e-12, -50.0},
         {-0.5, -0.5, 0.5, 0.5},
         {2.0, 2.0, 2.0, 2.0},
         {-kInf, -kInf, -kInf, -kInf},
         {kInf, kInf, kInf, kInf},
         1.0};
  auto v0 = c.v0;
  const auto plan = Plan(c, v0);
  EXPECT_EQ(v0[0], 0.0);
  EXPECT_EQ(v0[1], 0.0);
  EXPECT_EQ(v0[2], 0.0);
  EXPECT_EQ(v0[3], -2.0);  // lowered to the cap, sign kept (a reversal)
  EXPECT_LE(Oracle(c, c.v0, v0, plan.duration).peak_ratio, 1.0 + 1e-9);
}

TEST(ContinuousRetarget, MismatchedSizesZeroEveryVelocity) {
  std::vector<double> q0{0.0, 0.0}, v0{1.0, 1.0}, goal{1.0}, cap{1.0, 1.0}, lo{-1, -1}, hi{1, 1};
  const auto plan = PlanContinuousRetarget(q0, v0, goal, cap, lo, hi, 0.5);
  EXPECT_EQ(plan.duration, 0.5);
  EXPECT_EQ(v0, (std::vector<double>{0.0, 0.0}));
}

// The planner's closed form against a dense sampling of the trajectory it
// returns, over random moving states — including start speeds at the cap,
// reversals, tiny or zero distances and limits close by. The sweep must also
// exercise every branch (growth, slowing, dropping) for the check to mean
// anything.
TEST(ContinuousRetarget, EveryPlanHoldsTheCapTheLimitsAndTheGoal) {
  std::mt19937 rng(20261125);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  int grew = 0, slowed = 0, dropped = 0;
  OracleResult worst;
  for (int trial = 0; trial < 3000; ++trial) {
    const std::size_t n = 1 + static_cast<std::size_t>(unit(rng) * 4.0);
    Case c;
    for (std::size_t i = 0; i < n; ++i) {
      const double cap = 0.5 + 12.5 * unit(rng);
      const double lo = -1.0 - unit(rng);
      const double hi = 1.0 + unit(rng);
      const double q0 = lo + (hi - lo) * unit(rng);
      double goal = lo + (hi - lo) * unit(rng);
      if (unit(rng) < 0.15) {
        goal = q0 + 1e-3 * (unit(rng) - 0.5);  // tiny distance
      }
      if (unit(rng) < 0.05) {
        goal = q0;  // no distance
      }
      if (unit(rng) < 0.1) {
        goal = unit(rng) < 0.5 ? lo : hi;  // goal on a limit
      }
      double v0 = cap * (2.4 * unit(rng) - 1.2);  // up to 20 % beyond the cap
      if (unit(rng) < 0.1) {
        v0 = std::copysign(cap, v0);  // exactly at the cap
      }
      if (unit(rng) < 0.1) {
        v0 = 0.0;  // a joint at rest beside moving ones
      }
      c.q0.push_back(q0);
      c.v0.push_back(v0);
      c.goal.push_back(goal);
      c.cap.push_back(cap);
      c.lo.push_back(lo);
      c.hi.push_back(hi);
    }
    const double speed = 0.05 + 13.0 * unit(rng);
    c.base = BaseDuration(c.q0, c.goal, c.cap, speed, 0.002);
    auto v0 = c.v0;
    const auto plan = Plan(c, v0);
    ASSERT_GE(plan.duration, c.base);
    // The bound in the header: never more than 1.25³ ≈ 1.95 × the base.
    ASSERT_LE(plan.duration, c.base * 1.25 * 1.25 * 1.25 * (1.0 + 1e-12)) << "trial " << trial;
    grew += plan.duration > c.base ? 1 : 0;
    slowed += plan.reduced;
    dropped += plan.dropped;
    const auto r = Oracle(c, c.v0, v0, plan.duration);
    worst.peak_ratio = std::max(worst.peak_ratio, r.peak_ratio);
    worst.limit_excess = std::max(worst.limit_excess, r.limit_excess);
    worst.goal_pass = std::max(worst.goal_pass, r.goal_pass);
    // Joints at rest stay at rest, and a velocity is never reversed or raised.
    for (std::size_t i = 0; i < n; ++i) {
      const double in = std::isfinite(c.v0[i]) ? std::clamp(c.v0[i], -c.cap[i], c.cap[i]) : 0.0;
      ASSERT_LE(std::abs(v0[i]), std::abs(in));
      ASSERT_GE(v0[i] * in, 0.0);
    }
  }
  EXPECT_LE(worst.peak_ratio, 1.0 + 1e-9);
  EXPECT_LE(worst.limit_excess, 1e-9);
  EXPECT_LE(worst.goal_pass, 1e-9);
  EXPECT_GT(grew, 50);
  EXPECT_GT(slowed, 50);
  EXPECT_GT(dropped, 50);
}

}  // namespace
}  // namespace rtc::trajectory
