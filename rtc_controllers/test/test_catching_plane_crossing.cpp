// The instant the ball crosses a plane of the hand's catch frame
// (plane_crossing.hpp): against closed-form roots, and every refusal.
//
// no_malloc_scope.hpp MUST precede every Eigen header, and alloc_gate.hpp
// replaces the global operator new — so this suite is its own binary.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/plane_crossing.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace {

using rtc::catching::PlaneCrossing;
using rtc::catching::PlaneCrossingParams;
using rtc::catching::PlaneGap;
using rtc::catching::SolvePlaneCrossing;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kStart = 5'000 * kMs;  // the starting guess, a large instant

PlaneCrossingParams Params() {
  PlaneCrossingParams p;
  p.window_ns = 100 * kMs;
  p.tol_ns = kMs / 2;
  return p;
}

double Seconds(std::int64_t ns) {
  return static_cast<double>(ns) * 1e-9;
}

/// A hand and a ball that both move at constant velocity, the hand's
/// orientation fixed: g is linear in t and its root is known in closed form.
struct Linear {
  Eigen::Matrix3d R{
      Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()).toRotationMatrix()};
  Eigen::Vector3d p_h0{0.4, -0.1, 0.6};
  Eigen::Vector3d v_h{0.2, 0.1, -0.3};
  Eigen::Vector3d p_b0{0.0, 0.0, 0.0};
  Eigen::Vector3d v_b{0.0, 0.0, 0.0};
  double s_plane{0.0};

  /// Put the ball so that it crosses the plane `dt_s` after kStart, closing on
  /// it at `speed` along −z of the catch frame.
  void CrossAt(double dt_s, double speed) {
    const Eigen::Vector3d n = R.col(2);
    v_b = v_h - speed * n + 0.4 * R.col(0);  // sideways motion does not move the crossing
    p_b0 = p_h0 + s_plane * n + 0.05 * R.col(1) - (v_b - v_h) * dt_s;
  }

  [[nodiscard]] bool Gap(std::int64_t t_ns, double& g) const {
    const double t = Seconds(t_ns - kStart);
    g = PlaneGap(R, p_h0 + v_h * t, p_b0 + v_b * t, s_plane);
    return true;
  }
};

TEST(PlaneCrossing, TheGapIsTheBallsCoordinateAlongTheFrameAxisLessTheOffset) {
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.1, 0.9, -0.2).normalized()).toRotationMatrix();
  const Eigen::Vector3d p_h(0.3, 0.2, 0.5);
  // A ball 0.12 m out along the frame's z, 0.3 m off to its side.
  const Eigen::Vector3d p_b = p_h + 0.12 * R.col(2) + 0.3 * R.col(0);
  EXPECT_NEAR(PlaneGap(R, p_h, p_b, 0.0), 0.12, 1e-15);
  EXPECT_NEAR(PlaneGap(R, p_h, p_b, 0.05), 0.07, 1e-15);
  EXPECT_NEAR(PlaneGap(R, p_h, p_h, 0.05), -0.05, 1e-15);
}

TEST(PlaneCrossing, ALinearGapIsSolvedToItsClosedFormRootForBothPlanes) {
  for (const double s_plane : {0.0, 0.035}) {
    for (const double dt_s : {-0.060, -0.004, 0.0, 0.0002, 0.013, 0.085}) {
      Linear m;
      m.s_plane = s_plane;
      m.CrossAt(dt_s, 1.4);
      const PlaneCrossing x = SolvePlaneCrossing(
          [&m](std::int64_t t, double& g) { return m.Gap(t, g); }, kStart, Params());
      ASSERT_TRUE(x.valid) << "s_plane " << s_plane << ", dt " << dt_s;
      EXPECT_NEAR(Seconds(x.t_ns - kStart), dt_s, 1e-6) << "s_plane " << s_plane;
      EXPECT_LE(x.iterations, 3) << "a linear gap needs one secant step and its confirmation";
    }
  }
}

TEST(PlaneCrossing, ACurvedGapConvergesToARootOfIt) {
  // The ball falls (a parabola) while the hand turns about its own x axis and
  // decelerates: neither term is linear. The root is checked on g itself.
  const Eigen::Vector3d p_h0(0.5, 0.0, 0.4);
  const Eigen::Vector3d v_h(0.3, 0.0, 0.2);
  const Eigen::Vector3d a_h(-1.5, 0.0, -1.0);
  const Eigen::Vector3d p_b0(0.5, 0.02, 0.52);
  const Eigen::Vector3d v_b(0.2, 0.0, -2.6);
  const Eigen::Vector3d grav(0.0, 0.0, -9.81);
  const auto gap = [&](std::int64_t t_ns, double& g) {
    const double t = Seconds(t_ns - kStart);
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(0.2 + 1.5 * t, Eigen::Vector3d::UnitX()).toRotationMatrix();
    g = PlaneGap(R, p_h0 + v_h * t + 0.5 * a_h * t * t, p_b0 + v_b * t + 0.5 * grav * t * t, 0.03);
    return true;
  };
  PlaneCrossingParams p = Params();
  p.tol_ns = 10'000;  // 10 µs: the residual below is then a statement about the root
  const PlaneCrossing x = SolvePlaneCrossing(gap, kStart, p);
  ASSERT_TRUE(x.valid);
  double g = 1.0;
  ASSERT_TRUE(gap(x.t_ns, g));
  EXPECT_LT(std::fabs(g), 1e-4) << "|g| at the returned instant, closing at ~3 m/s";
  EXPECT_GT(x.t_ns, kStart);
  EXPECT_LT(x.t_ns, kStart + 100 * kMs);
  EXPECT_LE(x.iterations, p.max_iterations);
}

TEST(PlaneCrossing, ARootOutsideTheWindowIsRefused) {
  for (const double dt_s : {-0.140, 0.101, 0.4}) {
    Linear m;
    m.CrossAt(dt_s, 1.4);
    const PlaneCrossing x = SolvePlaneCrossing(
        [&m](std::int64_t t, double& g) { return m.Gap(t, g); }, kStart, Params());
    EXPECT_FALSE(x.valid) << "dt " << dt_s;
  }
}

TEST(PlaneCrossing, AnInstantThatCannotBeEvaluatedIsRefused) {
  Linear m;
  m.CrossAt(0.040, 1.4);
  // Not at the starting guess …
  EXPECT_FALSE(
      SolvePlaneCrossing([](std::int64_t, double&) { return false; }, kStart, Params()).valid);
  // … and not at an iterate: the prediction ends 20 ms after the guess.
  const auto gap = [&m](std::int64_t t, double& g) {
    return t <= kStart + 20 * kMs && m.Gap(t, g);
  };
  EXPECT_FALSE(SolvePlaneCrossing(gap, kStart, Params()).valid);
  // A non-finite gap is not a gap.
  const auto nan_gap = [](std::int64_t, double& g) {
    g = std::numeric_limits<double>::quiet_NaN();
    return true;
  };
  EXPECT_FALSE(SolvePlaneCrossing(nan_gap, kStart, Params()).valid);
}

TEST(PlaneCrossing, ABallThatDoesNotCloseOnThePlaneIsRefused) {
  // The ball keeps its distance to the plane: no slope, no root.
  Linear m;
  m.CrossAt(0.040, 0.0);
  m.p_b0 += 0.1 * m.R.col(2);
  EXPECT_FALSE(
      SolvePlaneCrossing([&m](std::int64_t t, double& g) { return m.Gap(t, g); }, kStart, Params())
          .valid);
}

TEST(PlaneCrossing, TheIterationCountIsBoundedAndParametersAreChecked) {
  // A gap whose secant steps never settle inside the tolerance: sign of the
  // step alternates with a fixed size.
  int evaluations = 0;
  const auto restless = [&evaluations](std::int64_t t, double& g) {
    ++evaluations;
    const double x = static_cast<double>(t - kStart) * 1e-9;
    g = std::cbrt(x - 0.02);  // the secant method diverges on a cube root
    return true;
  };
  PlaneCrossingParams p = Params();
  p.max_iterations = 5;
  const PlaneCrossing x = SolvePlaneCrossing(restless, kStart, p);
  EXPECT_FALSE(x.valid);
  EXPECT_LE(evaluations, p.max_iterations + 1);

  Linear m;
  m.CrossAt(0.010, 1.4);
  const auto gap = [&m](std::int64_t t, double& g) { return m.Gap(t, g); };
  PlaneCrossingParams bad = Params();
  bad.tol_ns = 0;
  EXPECT_FALSE(SolvePlaneCrossing(gap, kStart, bad).valid);
  bad = Params();
  bad.window_ns = 0;
  EXPECT_FALSE(SolvePlaneCrossing(gap, kStart, bad).valid);
  bad = Params();
  bad.max_iterations = 0;
  EXPECT_FALSE(SolvePlaneCrossing(gap, kStart, bad).valid);
}

TEST(PlaneCrossing, TheSolveAllocatesNothing) {
  Linear m;
  m.s_plane = 0.035;
  m.CrossAt(0.013, 1.4);
  PlaneCrossing x{};
  {
    const rtc::testing::ScopedAllocGate heap_gate;
    const rtc::testing::ScopedNoMalloc eigen_gate;
    for (int k = 0; k < 200; ++k) {
      x = SolvePlaneCrossing([&m](std::int64_t t, double& g) { return m.Gap(t, g); }, kStart,
                             Params());
    }
    EXPECT_EQ(heap_gate.count(), 0U);
    EXPECT_EQ(eigen_gate.violations(), 0U);
  }
  EXPECT_TRUE(x.valid);
  static_assert(noexcept(SolvePlaneCrossing([](std::int64_t, double&) noexcept { return true; },
                                            std::int64_t{0}, PlaneCrossingParams{})));
}

}  // namespace
