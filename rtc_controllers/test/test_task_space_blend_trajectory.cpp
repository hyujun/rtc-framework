#include "rtc_controllers/trajectory/task_space_blend_trajectory.hpp"
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

namespace rtc::trajectory
{

// Helper to create an SE3 from translation + RPY
pinocchio::SE3 make_pose(double x, double y, double z,
                          double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd ya(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = (ya * p * r).toRotationMatrix();
  return pinocchio::SE3(R, Eigen::Vector3d(x, y, z));
}

// --- Two waypoints match TaskSpaceTrajectory (rest-to-rest) ---
TEST(TaskSpaceBlendTrajectory, TwoWaypointsMatch)
{
  TaskSpaceBlendTrajectory blend;
  TaskSpaceTrajectory single;

  auto pose0 = make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  auto pose1 = make_pose(1.0, 0.5, -0.3, 0.1, 0.2, 0.3);

  std::array<TaskSpaceBlendTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0].pose = pose0;
  wps[0].time = 0.0;
  wps[1].pose = pose1;
  wps[1].time = 2.0;

  blend.initialize(wps, 2);
  single.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(), 2.0);

  EXPECT_DOUBLE_EQ(blend.duration(), 2.0);

  for (double t = 0.0; t <= 2.0; t += 0.2) {
    auto bs = blend.compute(t);
    auto ss = single.compute(t);

    // Compare translations
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(bs.pose.translation()[i], ss.pose.translation()[i], 1e-10)
        << "t=" << t << " axis=" << i;
    }

    // Compare rotations (Frobenius norm of difference)
    double rot_diff = (bs.pose.rotation() - ss.pose.rotation()).norm();
    EXPECT_NEAR(rot_diff, 0.0, 1e-10) << "t=" << t;
  }
}

// --- Three waypoints: continuity at via-point ---
TEST(TaskSpaceBlendTrajectory, ThreeWaypointsContinuity)
{
  TaskSpaceBlendTrajectory blend;
  std::array<TaskSpaceBlendTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0};
  wps[1] = {make_pose(0.5, 0.3, 0.1, 0.05, 0.1, 0.0), 1.0};
  wps[2] = {make_pose(1.0, 0.0, 0.5, 0.1, 0.0, 0.2), 2.0};

  blend.initialize(wps, 3);

  EXPECT_EQ(blend.num_segments(), 2u);

  // Check continuity at t=1.0
  constexpr double eps = 1e-7;
  auto sl = blend.compute(1.0 - eps);
  auto sm = blend.compute(1.0);
  auto sr = blend.compute(1.0 + eps);

  // Position (translation) continuity
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(sl.pose.translation()[i], sm.pose.translation()[i], 1e-5);
    EXPECT_NEAR(sr.pose.translation()[i], sm.pose.translation()[i], 1e-5);
  }

  // Passes through via-point pose (at least translation)
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(sm.pose.translation()[i], wps[1].pose.translation()[i], 1e-10);
  }
}

// --- Rest-to-rest: zero velocity at start and end ---
TEST(TaskSpaceBlendTrajectory, RestToRest)
{
  TaskSpaceBlendTrajectory blend;
  std::array<TaskSpaceBlendTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0};
  wps[1] = {make_pose(1.0, 0.5, 0.0, 0.0, 0.1, 0.0), 1.0};
  wps[2] = {make_pose(2.0, 0.0, 0.5, 0.0, 0.0, 0.2), 2.0};

  blend.initialize(wps, 3);

  auto s0 = blend.compute(0.0);
  auto sf = blend.compute(2.0);

  // Velocity should be zero at start and end
  EXPECT_NEAR(s0.velocity.toVector().norm(), 0.0, 1e-10);
  EXPECT_NEAR(sf.velocity.toVector().norm(), 0.0, 1e-10);
}

// --- Single waypoint hold ---
TEST(TaskSpaceBlendTrajectory, SingleWaypointHold)
{
  TaskSpaceBlendTrajectory blend;
  std::array<TaskSpaceBlendTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0), 0.0};

  blend.initialize(wps, 1);
  EXPECT_DOUBLE_EQ(blend.duration(), 0.0);

  auto s = blend.compute(0.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(s.pose.translation()[i], wps[0].pose.translation()[i]);
  }
}

// --- Clamp ---
TEST(TaskSpaceBlendTrajectory, ClampTime)
{
  TaskSpaceBlendTrajectory blend;
  std::array<TaskSpaceBlendTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {pinocchio::SE3::Identity(), 0.0};
  wps[1] = {make_pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0};

  blend.initialize(wps, 2);

  auto s_neg = blend.compute(-1.0);
  auto s_zero = blend.compute(0.0);
  auto s_over = blend.compute(5.0);
  auto s_end = blend.compute(1.0);

  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(s_neg.pose.translation()[i], s_zero.pose.translation()[i]);
    EXPECT_DOUBLE_EQ(s_over.pose.translation()[i], s_end.pose.translation()[i]);
  }
}

// --- Identity poses: straight line interpolation ---
TEST(TaskSpaceBlendTrajectory, PureTranslation)
{
  TaskSpaceBlendTrajectory blend;
  std::array<TaskSpaceBlendTrajectory::Waypoint, kMaxWaypoints> wps{};
  wps[0] = {make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0};
  wps[1] = {make_pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0};
  wps[2] = {make_pose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0), 2.0};

  blend.initialize(wps, 3);

  // At midpoint, should be near the middle waypoint
  auto s_mid = blend.compute(1.0);
  EXPECT_NEAR(s_mid.pose.translation()[0], 1.0, 1e-10);
  EXPECT_NEAR(s_mid.pose.translation()[1], 0.0, 1e-10);
  EXPECT_NEAR(s_mid.pose.translation()[2], 0.0, 1e-10);
}

}  // namespace rtc::trajectory
