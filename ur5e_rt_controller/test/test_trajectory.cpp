#include <gtest/gtest.h>
#include "ur5e_rt_controller/trajectory/trajectory_utils.hpp"
#include "ur5e_rt_controller/trajectory/joint_space_trajectory.hpp"
#include "ur5e_rt_controller/trajectory/task_space_trajectory.hpp"

using namespace ur5e_rt_controller;

TEST(TrajectoryTest, QuinticPolynomial_RestToRest) {
  trajectory::QuinticPolynomial poly(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);

  auto start = poly.compute(0.0);
  EXPECT_DOUBLE_EQ(start.pos, 0.0);
  EXPECT_DOUBLE_EQ(start.vel, 0.0);
  EXPECT_DOUBLE_EQ(start.acc, 0.0);

  auto mid = poly.compute(1.0);
  EXPECT_GT(mid.pos, 0.0);
  EXPECT_LT(mid.pos, 1.0);
  EXPECT_GT(mid.vel, 0.0);

  auto end = poly.compute(2.0);
  EXPECT_DOUBLE_EQ(end.pos, 1.0);
  EXPECT_DOUBLE_EQ(end.vel, 0.0);
  EXPECT_DOUBLE_EQ(end.acc, 0.0);
}

TEST(TrajectoryTest, QuinticPolynomial_NonZeroVelocity) {
  trajectory::QuinticPolynomial poly(0.0, 1.0, 0.0, 10.0, 2.0, 0.0, 5.0);

  auto start = poly.compute(0.0);
  EXPECT_DOUBLE_EQ(start.pos, 0.0);
  EXPECT_DOUBLE_EQ(start.vel, 1.0);
  EXPECT_DOUBLE_EQ(start.acc, 0.0);

  auto end = poly.compute(5.0);
  EXPECT_DOUBLE_EQ(end.pos, 10.0);
  EXPECT_DOUBLE_EQ(end.vel, 2.0);
  EXPECT_DOUBLE_EQ(end.acc, 0.0);
}

TEST(TrajectoryTest, TaskSpaceTrajectory_SE3) {
  trajectory::TaskSpaceTrajectory tst;

  pinocchio::SE3 start_pose = pinocchio::SE3::Identity();
  pinocchio::SE3 goal_pose = pinocchio::SE3::Identity();
  goal_pose.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  goal_pose.rotation() = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  tst.initialize(start_pose, pinocchio::Motion::Zero(),
                 goal_pose, pinocchio::Motion::Zero(),
                 2.0);

  auto res_start = tst.compute(0.0);
  EXPECT_NEAR(res_start.pose.translation().norm(), 0.0, 1e-6);
  EXPECT_NEAR(res_start.velocity.toVector().norm(), 0.0, 1e-6);
  EXPECT_NEAR(res_start.acceleration.toVector().norm(), 0.0, 1e-6);

  auto res_mid = tst.compute(1.0);
  EXPECT_GT(res_mid.pose.translation().norm(), 0.0);
  EXPECT_GT(res_mid.velocity.toVector().norm(), 0.0);

  auto res_end = tst.compute(2.0);
  EXPECT_NEAR((res_end.pose.translation() - goal_pose.translation()).norm(), 0.0, 1e-6);
  EXPECT_NEAR((res_end.pose.rotation() - goal_pose.rotation()).norm(), 0.0, 1e-6);
  EXPECT_NEAR(res_end.velocity.toVector().norm(), 0.0, 1e-6);
  EXPECT_NEAR(res_end.acceleration.toVector().norm(), 0.0, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
