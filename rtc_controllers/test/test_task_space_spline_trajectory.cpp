#include "rtc_controllers/trajectory/task_space_spline_trajectory.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <cmath>

// NOLINTNEXTLINE - cppcheck cannot parse GTest TEST() macros
// cppcheck-suppress syntaxError
namespace rtc::trajectory
{

pinocchio::SE3 make_se3(double x, double y, double z,
                         double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd ya(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = (ya * p * r).toRotationMatrix();
  return pinocchio::SE3(R, Eigen::Vector3d(x, y, z));
}

// --- Two waypoints: natural spline matches rest-to-rest TaskSpaceTrajectory ---
TEST(TaskSpaceSplineTrajectory, TwoWaypointsMatch)
{
  TaskSpaceSplineTrajectory spline;
  TaskSpaceTrajectory single;

  auto pose0 = pinocchio::SE3::Identity();
  auto pose1 = make_se3(1.0, 0.5, -0.3, 0.1, 0.2, 0.0);

  std::array<TaskSpaceSplineTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {pose0, 0.0};
  wps[1] = {pose1, 2.0};

  spline.initialize(wps, 2);
  single.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(), 2.0);

  for (double t = 0.0; t <= 2.0; t += 0.2) {
    auto ss = spline.compute(t);
    auto ts = single.compute(t);

    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(ss.pose.translation()[i], ts.pose.translation()[i], 1e-10)
        << "t=" << t << " i=" << i;
    }
    double rot_diff = (ss.pose.rotation() - ts.pose.rotation()).norm();
    EXPECT_NEAR(rot_diff, 0.0, 1e-10) << "t=" << t;
  }
}

// --- Three waypoints: continuity at via-point ---
TEST(TaskSpaceSplineTrajectory, ThreeWaypointsContinuity)
{
  TaskSpaceSplineTrajectory spline;
  std::array<TaskSpaceSplineTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0};
  wps[1] = {make_se3(0.5, 0.3, 0.1, 0.05, 0.1, 0.0), 1.0};
  wps[2] = {make_se3(1.0, 0.0, 0.5, 0.1, 0.0, 0.15), 2.0};

  spline.initialize(wps, 3);

  constexpr double eps = 1e-7;
  auto sl = spline.compute(1.0 - eps);
  auto sm = spline.compute(1.0);
  auto sr = spline.compute(1.0 + eps);

  // Translation continuity
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(sl.pose.translation()[i], sm.pose.translation()[i], 1e-5);
    EXPECT_NEAR(sr.pose.translation()[i], sm.pose.translation()[i], 1e-5);
  }

  // Passes through waypoint
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(sm.pose.translation()[i], wps[1].pose.translation()[i], 1e-8);
  }

  // Velocity continuity
  double vel_diff = (sl.velocity.toVector() - sr.velocity.toVector()).norm();
  EXPECT_LT(vel_diff, 1e-3);
}

// --- Natural boundary: rest-to-rest ---
TEST(TaskSpaceSplineTrajectory, NaturalBoundary)
{
  TaskSpaceSplineTrajectory spline;
  std::array<TaskSpaceSplineTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {pinocchio::SE3::Identity(), 0.0};
  wps[1] = {make_se3(0.5, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0};
  wps[2] = {make_se3(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), 2.0};

  spline.initialize(wps, 3);

  auto s0 = spline.compute(0.0);
  auto sf = spline.compute(2.0);

  EXPECT_NEAR(s0.velocity.toVector().norm(), 0.0, 1e-10);
  EXPECT_NEAR(sf.velocity.toVector().norm(), 0.0, 1e-10);
}

// --- Four waypoints: higher order continuity ---
TEST(TaskSpaceSplineTrajectory, FourWaypointsSmoothness)
{
  TaskSpaceSplineTrajectory spline;
  std::array<TaskSpaceSplineTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0};
  wps[1] = {make_se3(0.3, 0.2, 0.1, 0.02, 0.05, 0.0), 1.0};
  wps[2] = {make_se3(0.7, 0.1, 0.3, 0.05, 0.02, 0.08), 2.0};
  wps[3] = {make_se3(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), 3.0};

  spline.initialize(wps, 4);

  EXPECT_EQ(spline.num_segments(), 3u);
  EXPECT_DOUBLE_EQ(spline.duration(), 3.0);

  // Check continuity at each internal knot
  const double knot_times[] = {1.0, 2.0};
  constexpr double eps = 1e-7;
  for (double kt : knot_times) {
    auto sl = spline.compute(kt - eps);
    auto sr = spline.compute(kt + eps);

    // Translation continuity
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(sl.pose.translation()[i], sr.pose.translation()[i], 1e-4)
        << "trans at t=" << kt << " i=" << i;
    }

    // Velocity continuity
    double vel_diff = (sl.velocity.toVector() - sr.velocity.toVector()).norm();
    EXPECT_LT(vel_diff, 1e-3) << "vel at t=" << kt;
  }
}

// --- Single waypoint hold ---
TEST(TaskSpaceSplineTrajectory, SingleWaypointHold)
{
  TaskSpaceSplineTrajectory spline;
  std::array<TaskSpaceSplineTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_se3(1.0, 2.0, 3.0, 0.0, 0.0, 0.0), 0.0};

  spline.initialize(wps, 1);
  EXPECT_DOUBLE_EQ(spline.duration(), 0.0);

  auto s = spline.compute(0.0);
  EXPECT_NEAR(s.pose.translation()[0], 1.0, 1e-10);
  EXPECT_NEAR(s.velocity.toVector().norm(), 0.0, 1e-10);
}

// --- Clamp ---
TEST(TaskSpaceSplineTrajectory, ClampTime)
{
  TaskSpaceSplineTrajectory spline;
  std::array<TaskSpaceSplineTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {pinocchio::SE3::Identity(), 0.0};
  wps[1] = {make_se3(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0};

  spline.initialize(wps, 2);

  auto s_neg = spline.compute(-1.0);
  auto s_zero = spline.compute(0.0);
  auto s_over = spline.compute(5.0);
  auto s_end = spline.compute(1.0);

  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(s_neg.pose.translation()[i], s_zero.pose.translation()[i]);
    EXPECT_DOUBLE_EQ(s_over.pose.translation()[i], s_end.pose.translation()[i]);
  }
}

// --- Pure translation: no rotation ---
TEST(TaskSpaceSplineTrajectory, PureTranslation)
{
  TaskSpaceSplineTrajectory spline;
  std::array<TaskSpaceSplineTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0};
  wps[1] = {make_se3(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0};
  wps[2] = {make_se3(2.0, 0.0, 0.0, 0.0, 0.0, 0.0), 2.0};

  spline.initialize(wps, 3);

  auto s_mid = spline.compute(1.0);
  EXPECT_NEAR(s_mid.pose.translation()[0], 1.0, 1e-10);

  // Rotation should remain identity
  double rot_diff = (s_mid.pose.rotation() - Eigen::Matrix3d::Identity()).norm();
  EXPECT_NEAR(rot_diff, 0.0, 1e-10);
}

}  // namespace rtc::trajectory
