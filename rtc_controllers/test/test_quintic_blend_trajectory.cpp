#include "rtc_controllers/trajectory/quintic_blend_trajectory.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include <gtest/gtest.h>
#include <cmath>

// NOLINTNEXTLINE - cppcheck cannot parse GTest TEST() macros
// cppcheck-suppress syntaxError
namespace rtc::trajectory
{

using Blend3 = QuinticBlendTrajectory<3>;
using Blend6 = QuinticBlendTrajectory<6>;

// --- Helper ---
std::array<Blend3::Waypoint, kMaxWaypoints> make_waypoints_3dof(
  const std::vector<std::pair<std::array<double, 3>, double>> & pts)
{
  std::array<Blend3::Waypoint, kMaxWaypoints> wps{};
  for (std::size_t i = 0; i < pts.size(); ++i) {
    wps[i].positions = pts[i].first;
    wps[i].time = pts[i].second;
  }
  return wps;
}

// --- Two waypoints should match JointSpaceTrajectory (rest-to-rest) ---
TEST(QuinticBlendTrajectory, TwoWaypointsMatchJointSpace)
{
  constexpr std::size_t N = 3;
  Blend3 blend;
  JointSpaceTrajectory<N> jst;

  std::array<double, N> start = {0.0, 1.0, -1.0};
  std::array<double, N> goal = {1.0, 0.0, 2.0};
  constexpr double T = 2.0;

  auto wps = make_waypoints_3dof({
    {start, 0.0},
    {goal, T},
  });
  blend.initialize(wps, 2);

  JointSpaceTrajectory<N>::State s0{}, sg{};
  s0.positions = start;
  sg.positions = goal;
  jst.initialize(s0, sg, T);

  EXPECT_DOUBLE_EQ(blend.duration(), T);
  EXPECT_EQ(blend.num_segments(), 1u);

  for (double t = 0.0; t <= T; t += 0.1) {
    auto bs = blend.compute(t);
    auto js = jst.compute(t);
    for (std::size_t j = 0; j < N; ++j) {
      EXPECT_NEAR(bs.positions[j], js.positions[j], 1e-12) << "t=" << t << " j=" << j;
      EXPECT_NEAR(bs.velocities[j], js.velocities[j], 1e-12) << "t=" << t << " j=" << j;
      EXPECT_NEAR(bs.accelerations[j], js.accelerations[j], 1e-12) << "t=" << t << " j=" << j;
    }
  }
}

// --- Three waypoints: C2 continuity at via-point ---
TEST(QuinticBlendTrajectory, ThreeWaypointsContinuity)
{
  Blend3 blend;
  auto wps = make_waypoints_3dof({
    {{0.0, 0.0, 0.0}, 0.0},
    {{1.0, 2.0, -1.0}, 1.0},
    {{3.0, 1.0, 0.5}, 2.5},
  });
  blend.initialize(wps, 3);

  EXPECT_EQ(blend.num_segments(), 2u);
  EXPECT_DOUBLE_EQ(blend.duration(), 2.5);

  // Evaluate just before and after the via-point at t=1.0
  constexpr double eps = 1e-8;
  auto s_left = blend.compute(1.0 - eps);
  auto s_right = blend.compute(1.0 + eps);
  auto s_mid = blend.compute(1.0);

  for (std::size_t j = 0; j < 3; ++j) {
    // Position continuity
    EXPECT_NEAR(s_left.positions[j], s_mid.positions[j], 1e-6) << "pos j=" << j;
    EXPECT_NEAR(s_right.positions[j], s_mid.positions[j], 1e-6) << "pos j=" << j;
    // Passes through waypoint
    EXPECT_NEAR(s_mid.positions[j], wps[1].positions[j], 1e-10) << "via j=" << j;
    // Velocity continuity
    EXPECT_NEAR(s_left.velocities[j], s_right.velocities[j], 1e-4) << "vel j=" << j;
    // Acceleration continuity
    EXPECT_NEAR(s_left.accelerations[j], s_right.accelerations[j], 1e-3) << "acc j=" << j;
  }
}

// --- Start/end velocity is zero ---
TEST(QuinticBlendTrajectory, RestToRest)
{
  Blend3 blend;
  auto wps = make_waypoints_3dof({
    {{0.0, 0.0, 0.0}, 0.0},
    {{1.0, 2.0, -1.0}, 1.0},
    {{3.0, 1.0, 0.5}, 2.0},
    {{2.0, 0.0, 1.0}, 3.0},
  });
  blend.initialize(wps, 4);

  auto s_start = blend.compute(0.0);
  auto s_end = blend.compute(3.0);

  for (std::size_t j = 0; j < 3; ++j) {
    EXPECT_NEAR(s_start.velocities[j], 0.0, 1e-12) << "start vel j=" << j;
    EXPECT_NEAR(s_end.velocities[j], 0.0, 1e-12) << "end vel j=" << j;
  }
}

// --- Clamp behavior ---
TEST(QuinticBlendTrajectory, ClampTime)
{
  Blend3 blend;
  auto wps = make_waypoints_3dof({
    {{0.0, 1.0, 2.0}, 0.0},
    {{3.0, 4.0, 5.0}, 1.0},
  });
  blend.initialize(wps, 2);

  auto s_neg = blend.compute(-1.0);
  auto s_zero = blend.compute(0.0);
  auto s_over = blend.compute(10.0);
  auto s_end = blend.compute(1.0);

  for (std::size_t j = 0; j < 3; ++j) {
    EXPECT_DOUBLE_EQ(s_neg.positions[j], s_zero.positions[j]);
    EXPECT_DOUBLE_EQ(s_over.positions[j], s_end.positions[j]);
  }
}

// --- Single waypoint: hold position ---
TEST(QuinticBlendTrajectory, SingleWaypointHold)
{
  Blend3 blend;
  auto wps = make_waypoints_3dof({{{1.0, 2.0, 3.0}, 0.0}});
  blend.initialize(wps, 1);

  EXPECT_DOUBLE_EQ(blend.duration(), 0.0);
  EXPECT_EQ(blend.num_segments(), 0u);

  auto s = blend.compute(0.0);
  EXPECT_DOUBLE_EQ(s.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(s.positions[1], 2.0);
  EXPECT_DOUBLE_EQ(s.positions[2], 3.0);
  EXPECT_DOUBLE_EQ(s.velocities[0], 0.0);
}

// --- Zero waypoints: safe ---
TEST(QuinticBlendTrajectory, ZeroWaypoints)
{
  Blend3 blend;
  std::array<Blend3::Waypoint, kMaxWaypoints> wps{};
  blend.initialize(wps, 0);

  EXPECT_DOUBLE_EQ(blend.duration(), 0.0);
  EXPECT_EQ(blend.num_segments(), 0u);

  auto s = blend.compute(0.0);
  EXPECT_DOUBLE_EQ(s.positions[0], 0.0);
}

// --- Five waypoints: all internal via-points have continuity ---
TEST(QuinticBlendTrajectory, FiveWaypointsContinuity)
{
  Blend3 blend;
  auto wps = make_waypoints_3dof({
    {{0.0, 0.0, 0.0}, 0.0},
    {{1.0, 0.5, -0.5}, 0.5},
    {{2.0, 1.0, 0.0}, 1.2},
    {{1.5, 0.0, 1.0}, 2.0},
    {{3.0, 2.0, -1.0}, 3.0},
  });
  blend.initialize(wps, 5);

  EXPECT_EQ(blend.num_segments(), 4u);

  // Check continuity at each internal via-point
  const double via_times[] = {0.5, 1.2, 2.0};
  constexpr double eps = 1e-8;
  for (double vt : via_times) {
    auto sl = blend.compute(vt - eps);
    auto sr = blend.compute(vt + eps);
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_NEAR(sl.positions[j], sr.positions[j], 1e-5) << "pos at t=" << vt;
      EXPECT_NEAR(sl.velocities[j], sr.velocities[j], 1e-3) << "vel at t=" << vt;
      EXPECT_NEAR(sl.accelerations[j], sr.accelerations[j], 1e-1) << "acc at t=" << vt;
    }
  }
}

}  // namespace rtc::trajectory
