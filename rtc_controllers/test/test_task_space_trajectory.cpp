#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <cmath>

namespace rtc::trajectory {

// Helper to create an SE3 from translation + RPY
static pinocchio::SE3 make_pose(double x, double y, double z, double roll, double pitch,
                                double yaw) {
  Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd ya(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = (ya * p * r).toRotationMatrix();
  return pinocchio::SE3(R, Eigen::Vector3d(x, y, z));
}

// --- dJexp/dt correction is zero for rest-to-rest (geodesic property) ---
// For rest-to-rest quintic, v(t) ∝ delta_X at all times.
// Since Jexp(s*ν)*ν = ν (constant body velocity along geodesic),
// dJexp/dt * v = 0 exactly. Verify this property holds numerically.
TEST(TaskSpaceTrajectory, RestToRestCorrectionIsZero) {
  TaskSpaceTrajectory traj_corrected;
  TaskSpaceTrajectory traj_approx;

  auto pose0 = pinocchio::SE3::Identity();
  auto pose1 = make_pose(0.5, 0.3, -0.2, 0.3, 0.5, 1.2);

  const double duration = 2.0;
  const double dt = 0.002;

  traj_corrected.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(),
                            duration);
  traj_approx.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(),
                         duration);

  double max_diff = 0.0;
  for (double t = 0.0; t <= duration; t += dt) {
    auto s_corrected = traj_corrected.compute(t, dt);
    auto s_approx = traj_approx.compute(t, 0.0);

    double acc_diff =
        (s_corrected.acceleration.toVector() - s_approx.acceleration.toVector()).norm();
    max_diff = std::max(max_diff, acc_diff);
  }

  // The correction must be zero (to numerical precision) for rest-to-rest
  EXPECT_LT(max_diff, 1e-10) << "dJexp/dt * v should be zero for rest-to-rest (geodesic property)";
}

// --- dJexp/dt correction is nonzero when boundary velocities are off-axis ---
// When start/goal velocities are NOT aligned with delta_X, the velocity
// vector has components orthogonal to the geodesic direction, breaking
// the Jexp(s*ν)*ν = ν identity.
TEST(TaskSpaceTrajectory, NonRestToRestCorrectionIsNonzero) {
  TaskSpaceTrajectory traj_corrected;
  TaskSpaceTrajectory traj_approx;

  auto pose0 = pinocchio::SE3::Identity();
  auto pose1 = make_pose(0.5, 0.3, -0.2, 0.3, 0.5, 1.2);

  // Off-axis boundary velocities: not aligned with log6(pose0^-1 * pose1)
  pinocchio::Motion v_start(Eigen::Vector3d(0.1, -0.05, 0.0), Eigen::Vector3d(0.0, 0.2, -0.1));
  pinocchio::Motion v_goal(Eigen::Vector3d(-0.05, 0.0, 0.1), Eigen::Vector3d(0.1, 0.0, 0.15));

  const double duration = 2.0;
  const double dt = 0.002;

  traj_corrected.initialize(pose0, v_start, pose1, v_goal, duration);
  traj_approx.initialize(pose0, v_start, pose1, v_goal, duration);

  double max_diff = 0.0;
  bool found_nonzero_diff = false;

  for (double t = 0.0; t <= duration; t += dt) {
    auto s_corrected = traj_corrected.compute(t, dt);
    auto s_approx = traj_approx.compute(t, 0.0);

    // Pose and velocity must match (correction only affects acceleration)
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(s_corrected.pose.translation()[i], s_approx.pose.translation()[i], 1e-12);
    }
    EXPECT_NEAR(s_corrected.velocity.toVector().norm(), s_approx.velocity.toVector().norm(), 1e-12);

    double acc_diff =
        (s_corrected.acceleration.toVector() - s_approx.acceleration.toVector()).norm();
    max_diff = std::max(max_diff, acc_diff);
    if (acc_diff > 1e-10) {
      found_nonzero_diff = true;
    }
  }

  EXPECT_TRUE(found_nonzero_diff)
      << "dJexp/dt correction should be nonzero for off-axis boundary velocities";
  EXPECT_GT(max_diff, 1e-4)
      << "Max acceleration difference should be significant for off-axis velocities";
}

// --- At rest (identity → identity), correction should be zero ---
TEST(TaskSpaceTrajectory, AccelerationCorrectionZeroAtRest) {
  TaskSpaceTrajectory traj;

  auto pose = pinocchio::SE3::Identity();
  const double duration = 1.0;
  const double dt = 0.002;

  traj.initialize(pose, pinocchio::Motion::Zero(), pose, pinocchio::Motion::Zero(), duration);

  for (double t = 0.0; t <= duration; t += dt) {
    auto s = traj.compute(t, dt);
    EXPECT_NEAR(s.acceleration.toVector().norm(), 0.0, 1e-12)
        << "Acceleration should be zero for zero-displacement trajectory at t=" << t;
  }
}

// --- Pure translation: Jexp = I (constant), so dJexp/dt = 0 ---
TEST(TaskSpaceTrajectory, PureTranslationNoCorrectionNeeded) {
  TaskSpaceTrajectory traj_corrected;
  TaskSpaceTrajectory traj_approx;

  auto pose0 = pinocchio::SE3::Identity();
  auto pose1 = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0));

  const double duration = 2.0;
  const double dt = 0.002;

  traj_corrected.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(),
                            duration);
  traj_approx.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(),
                         duration);

  for (double t = 0.0; t <= duration; t += dt) {
    auto s_corrected = traj_corrected.compute(t, dt);
    auto s_approx = traj_approx.compute(t, 0.0);

    double acc_diff =
        (s_corrected.acceleration.toVector() - s_approx.acceleration.toVector()).norm();
    EXPECT_NEAR(acc_diff, 0.0, 1e-10) << "Pure translation: dJexp/dt should be zero at t=" << t;
  }
}

// --- Reinitialize resets dJexp/dt state ---
TEST(TaskSpaceTrajectory, ReinitializeResetsState) {
  TaskSpaceTrajectory traj;

  auto pose0 = pinocchio::SE3::Identity();
  auto pose1 = make_pose(0.5, 0.0, 0.0, 0.3, 0.5, 1.2);

  const double dt = 0.002;

  // First trajectory
  traj.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(), 2.0);
  traj.compute(0.0, dt);
  traj.compute(dt, dt);

  // Reinitialize — should reset Jexp state
  traj.initialize(pose0, pinocchio::Motion::Zero(), pose1, pinocchio::Motion::Zero(), 2.0);

  // First compute after reinit: should use Jexp*a only (no prev Jexp)
  auto s1 = traj.compute(0.0, dt);
  EXPECT_FALSE(std::isnan(s1.acceleration.toVector().norm()));

  // Second compute should now have prev Jexp and use correction
  auto s2 = traj.compute(dt, dt);
  EXPECT_FALSE(std::isnan(s2.acceleration.toVector().norm()));
}

// --- Default State is identity/zero, never uninitialized (#150) ---
// pinocchio::SE3 / Motion default-construct with garbage. Controllers read a
// default State in log-fill paths before any trajectory starts; a garbage
// rotation trips pinocchio's assert(R.isUnitary()) in matrixToRpy on
// non-NDEBUG builds and silently logs garbage on Release. Pin the contract.
TEST(TaskSpaceTrajectory, DefaultStateIsIdentityPoseZeroMotion) {
  TaskSpaceTrajectory::State state;
  EXPECT_TRUE(state.pose.rotation().isUnitary());
  EXPECT_TRUE(state.pose.isIdentity());
  EXPECT_EQ(state.velocity.toVector().norm(), 0.0);
  EXPECT_EQ(state.acceleration.toVector().norm(), 0.0);
}

// --- Aligned velocity / classical acceleration (#779) ---
// AlignedVelocity() and AlignedClassicalAcceleration() against central
// differences of the pose: origin velocity, angular velocity in the pose's
// reference axes, and their time derivatives. The segment starts moving and
// turns about two axes, so ω×v and the dJexp/dt correction are both non-zero.
// The acceleration comes from its own trajectory instance stepped forward in
// time (compute() differences Jexp against the previous call).
TEST(TaskSpaceTrajectory, AlignedFeedForwardMatchesFiniteDifferencesOfThePose) {
  const pinocchio::SE3 start = make_pose(0.3, -0.2, 0.5, 0.0, 0.0, 0.4);
  const pinocchio::SE3 goal = make_pose(0.6, 0.1, 0.2, -0.6, 0.2, 1.1);
  pinocchio::Motion v0;
  v0.linear() << 0.2, -0.1, 0.05;
  v0.angular() << 0.3, 0.1, -0.2;
  auto make = [&] {
    TaskSpaceTrajectory traj;
    traj.initialize(start, v0, goal, pinocchio::Motion::Zero(), 1.0);
    return traj;
  };

  constexpr double kH = 1e-5;
  // ω at the midpoint of [a, b], in the reference axes.
  auto omega = [](const pinocchio::SE3& a, const pinocchio::SE3& b, double dt) {
    return Eigen::Vector3d(a.rotation() * pinocchio::log3(a.rotation().transpose() * b.rotation()) /
                           dt);
  };
  for (const double t : {0.3, 0.5, 0.7}) {
    TaskSpaceTrajectory poses = make();
    const auto smm = poses.compute(t - 2.0 * kH);
    const auto sm = poses.compute(t - kH);
    const auto s0 = poses.compute(t);
    const auto sp = poses.compute(t + kH);
    const auto spp = poses.compute(t + 2.0 * kH);
    TaskSpaceTrajectory stepped = make();
    (void)stepped.compute(t - kH, kH);
    const auto s = stepped.compute(t, kH);  // acceleration with the dJexp/dt term

    const Eigen::Vector3d p_dot = (sp.pose.translation() - sm.pose.translation()) / (2.0 * kH);
    const Eigen::Vector3d p_ddot =
        (sp.pose.translation() - 2.0 * s0.pose.translation() + sm.pose.translation()) / (kH * kH);
    const Eigen::Vector3d w = omega(sm.pose, sp.pose, 2.0 * kH);
    const Eigen::Vector3d w_dot =
        (omega(s0.pose, spp.pose, 2.0 * kH) - omega(smm.pose, s0.pose, 2.0 * kH)) / (2.0 * kH);

    ASSERT_GT(s.velocity.angular().cross(s.velocity.linear()).norm(), 1e-3) << t;
    const Eigen::Matrix<double, 6, 1> v = s.AlignedVelocity();
    const Eigen::Matrix<double, 6, 1> a = s.AlignedClassicalAcceleration();
    EXPECT_LT((v.head<3>() - p_dot).norm(), 1e-6) << t;
    EXPECT_LT((v.tail<3>() - w).norm(), 1e-6) << t;
    EXPECT_LT((a.head<3>() - p_ddot).norm(), 1e-3) << t;
    EXPECT_LT((a.tail<3>() - w_dot).norm(), 1e-3) << t;
  }
}

}  // namespace rtc::trajectory
