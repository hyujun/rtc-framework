// ── "The reach bound removes nothing the IK accepts", as one check ───────────
// (test-only; shared by the robot-neutral suite and the shipped-model suite)
//
// Targets are drawn just past the bound — from the IK's own position tolerance
// to 0.3 m beyond it, weighted to the near side, where a bound that is too
// tight would show first — in every direction, and the catch-pose IK is run on
// each for a ball flying at the arm (the pose a stretched arm takes best), away
// from it, and across. Every target must be one the filter refuses AND one the
// IK refuses.
#pragma once

#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/reach_bound.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <random>

namespace rtc::testing {

/// @param seed called once per solve with the generator; returns the IK seed
///        (model order, `handle.nv()` entries)
template <typename SeedFn>
void ExpectTheIkRefusesWhatTheFilterRefuses(rtc_urdf_bridge::RtModelHandle& handle,
                                            pinocchio::FrameIndex frame,
                                            const catching::ReachBound& bound,
                                            const catching::CatchPoseIkOptions& options,
                                            int targets, double ball_speed, unsigned rng_seed,
                                            SeedFn&& seed) {
  ASSERT_TRUE(bound.Bounded());
  catching::CatchPoseIk ik;
  ik.Resize(handle.nv());
  std::mt19937 rng(rng_seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const auto direction = [&] {
    const Eigen::Vector3d d(gauss(rng), gauss(rng), gauss(rng));
    return Eigen::Vector3d(d.normalized());
  };
  for (int i = 0; i < targets; ++i) {
    const double beyond = options.eps_pos + 1e-4 + 0.3 * unit(rng) * unit(rng);
    const Eigen::Vector3d out = direction();
    const Eigen::Vector3d target = bound.centre + (bound.radius + beyond) * out;
    ASSERT_FALSE(catching::WithinReach(bound, target, options.eps_pos));
    for (const Eigen::Vector3d& v :
         {Eigen::Vector3d(-ball_speed * out), Eigen::Vector3d(ball_speed * out),
          Eigen::Vector3d(ball_speed * direction())}) {
      const Eigen::VectorXd q0 = seed(rng);
      const catching::CatchPoseIkResult r = ik.Solve(handle, frame, target, v, q0, options);
      EXPECT_FALSE(r.accepted) << "target " << target.transpose() << " is " << beyond
                               << " m past the bound and the IK accepted it";
    }
  }
}

}  // namespace rtc::testing
