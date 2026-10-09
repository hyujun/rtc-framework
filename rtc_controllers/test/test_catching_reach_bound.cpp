// ── The catch frame's reach bound (reach_bound.hpp, L3 §4.1) ─────────────────
//
//   1. It IS a bound: no posture inside the joint limits puts the frame farther
//      from the centre than the radius — on a 6R wrist arm, a 7R arm and a
//      chain with prismatic joints (at their stroke ends too).
//   2. Stopping the iteration early never breaks it: with no iteration at all
//      (the joint origins) it is still a bound, and iterating only tightens.
//   3. It is tight where the geometry allows: on the two revolute arms some
//      posture comes within a few percent of the radius.
//   4. A chain it cannot bound has an infinite radius and passes everything.
//   5. WithinReach is fail-closed on a non-finite point.
//   6. The filter removes nothing the IK accepts: a target it refuses (beyond
//      the radius plus the IK's position tolerance) is one CatchPoseIk refuses
//      too, from any seed and for any ball direction.
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/reach_bound.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"
#include "rtc_controllers/testing/reach_bound_fixture.hpp"

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace {

using rtc::catching::ComputeReachBound;
using rtc::catching::ReachBound;
using rtc::catching::WithinReach;

// FK rounding only: the bound itself carries no margin.
constexpr double kSlack = 1e-9;

/// The largest distance of `frame` from `centre` over `n` postures drawn inside
/// the position limits, plus every posture with all joints at one end of them.
double FarthestSampled(const pinocchio::Model& model, pinocchio::FrameIndex frame,
                       const Eigen::Vector3d& centre, int n, unsigned seed) {
  pinocchio::Data data(model);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const Eigen::VectorXd lo = model.lowerPositionLimit;
  const Eigen::VectorXd hi = model.upperPositionLimit;
  double farthest = 0.0;
  const auto measure = [&](const Eigen::VectorXd& q) {
    pinocchio::framesForwardKinematics(model, data, q);
    farthest = std::max(farthest, (data.oMf[frame].translation() - centre).norm());
  };
  Eigen::VectorXd q(model.nq);
  for (int i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j < model.nq; ++j) {
      q[j] = lo[j] + (hi[j] - lo[j]) * unit(rng);
    }
    measure(q);
  }
  // The corners of the box a few joints at a time: where a stroke is spent.
  for (unsigned mask = 0; mask < (1U << std::min<Eigen::Index>(model.nq, 10)); ++mask) {
    for (Eigen::Index j = 0; j < model.nq; ++j) {
      q[j] = ((mask >> (j % 10)) & 1U) != 0U ? hi[j] : lo[j];
    }
    measure(q);
  }
  return farthest;
}

struct Case {
  const char* urdf;
  const char* frame;
};

class ReachBoundOnAnArm : public ::testing::TestWithParam<Case> {};

// ── 1, 2. A bound, at any number of iterations ───────────────────────────────

TEST_P(ReachBoundOnAnArm, NoPostureInsideTheLimitsLeavesTheSphere) {
  rtc::testing::Arm arm = rtc::testing::MakeArm(GetParam().urdf, GetParam().frame);
  ASSERT_NE(arm.frame, 0U);
  double previous = std::numeric_limits<double>::infinity();
  for (const int iterations : {0, 1, 3, 200}) {
    const ReachBound bound = ComputeReachBound(*arm.model, arm.frame, iterations);
    ASSERT_TRUE(bound.Bounded()) << bound.unbounded_by;
    EXPECT_EQ(bound.joints, arm.nv);
    EXPECT_TRUE(bound.unbounded_by.empty());
    const double farthest = FarthestSampled(*arm.model, arm.frame, bound.centre, 20000, 793U);
    EXPECT_LE(farthest, bound.radius + kSlack) << "iterations " << iterations;
    // Iterating only tightens.
    EXPECT_LE(bound.radius, previous + kSlack) << "iterations " << iterations;
    previous = bound.radius;
  }
}

INSTANTIATE_TEST_SUITE_P(Arms, ReachBoundOnAnArm,
                         ::testing::Values(Case{"serial_6r_wrist.urdf", "catch_frame"},
                                           Case{"serial_7dof.urdf", "tool_link"},
                                           Case{"mixed_prismatic_revolute.urdf", "tool_link"}));

// ── 3. Tight on the revolute arms ────────────────────────────────────────────

TEST(ReachBound, SomePostureComesCloseToTheRadiusOnARevoluteArm) {
  for (const Case& c :
       {Case{"serial_6r_wrist.urdf", "catch_frame"}, Case{"serial_7dof.urdf", "tool_link"}}) {
    rtc::testing::Arm arm = rtc::testing::MakeArm(c.urdf, c.frame);
    const ReachBound bound = ComputeReachBound(*arm.model, arm.frame);
    ASSERT_TRUE(bound.Bounded());
    const double farthest = FarthestSampled(*arm.model, arm.frame, bound.centre, 200000, 7931U);
    RecordProperty(std::string("radius_um_") + c.urdf, static_cast<int>(bound.radius * 1e6));
    RecordProperty(std::string("farthest_um_") + c.urdf, static_cast<int>(farthest * 1e6));
    EXPECT_GT(farthest, 0.9 * bound.radius) << c.urdf;
  }
}

// ── 4. Chains it cannot bound ────────────────────────────────────────────────

TEST(ReachBound, AJointItCannotReadLeavesNoBound) {
  pinocchio::Model model;
  const auto shoulder = model.addJoint(
      0, pinocchio::JointModelRZ(),
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.3)), "shoulder");
  const auto ball = model.addJoint(
      shoulder, pinocchio::JointModelSpherical(),
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.4, 0.0, 0.0)), "ball");
  const auto frame = model.addFrame(pinocchio::Frame(
      "tip", ball, 0, pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.2, 0.0, 0.0)),
      pinocchio::OP_FRAME));
  const ReachBound bound = ComputeReachBound(model, frame);
  EXPECT_FALSE(bound.Bounded());
  EXPECT_EQ(bound.unbounded_by, "ball");
  EXPECT_TRUE(WithinReach(bound, Eigen::Vector3d(100.0, 0.0, 0.0), 0.0));
}

TEST(ReachBound, APrismaticJointWithoutAStrokeLeavesNoBound) {
  pinocchio::Model model;
  // addJoint without limits: the position limits are infinite.
  const auto slide =
      model.addJoint(0, pinocchio::JointModelPX(), pinocchio::SE3::Identity(), "slide");
  const auto frame = model.addFrame(
      pinocchio::Frame("tip", slide, 0, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  const ReachBound bound = ComputeReachBound(model, frame);
  EXPECT_FALSE(bound.Bounded());
  EXPECT_EQ(bound.unbounded_by, "slide");
}

TEST(ReachBound, AFrameOnTheUniverseDoesNotMove) {
  pinocchio::Model model;
  const Eigen::Vector3d at(0.1, -0.2, 0.3);
  const auto frame = model.addFrame(pinocchio::Frame(
      "fixed", 0, 0, pinocchio::SE3(Eigen::Matrix3d::Identity(), at), pinocchio::OP_FRAME));
  const ReachBound bound = ComputeReachBound(model, frame);
  ASSERT_TRUE(bound.Bounded());
  EXPECT_EQ(bound.radius, 0.0);
  EXPECT_EQ(bound.centre, at);
  EXPECT_EQ(bound.joints, 0);
}

TEST(ReachBound, AnUnknownFrameIsRefused) {
  rtc::testing::Arm arm = rtc::testing::Arm6R();
  EXPECT_THROW(static_cast<void>(ComputeReachBound(*arm.model, arm.model->frames.size())),
               std::invalid_argument);
}

// ── 5. WithinReach ───────────────────────────────────────────────────────────

TEST(ReachBound, WithinReachIsTheSphereWithItsToleranceAndFailsClosed) {
  ReachBound bound;
  bound.centre = Eigen::Vector3d(0.0, 0.0, 1.0);
  bound.radius = 0.5;
  EXPECT_TRUE(WithinReach(bound, Eigen::Vector3d(0.5, 0.0, 1.0), 0.0));
  EXPECT_FALSE(WithinReach(bound, Eigen::Vector3d(0.501, 0.0, 1.0), 0.0));
  EXPECT_TRUE(WithinReach(bound, Eigen::Vector3d(0.501, 0.0, 1.0), 0.002));
  EXPECT_FALSE(WithinReach(bound, Eigen::Vector3d(0.503, 0.0, 1.0), 0.002));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(WithinReach(bound, Eigen::Vector3d(nan, 0.0, 1.0), 0.002));
  EXPECT_FALSE(WithinReach(bound, Eigen::Vector3d(0.0, 0.0, 1.0), nan));
  // No bound passes everything — a non-finite point included: it is the IK's
  // input check that refuses that one.
  EXPECT_TRUE(WithinReach(ReachBound{}, Eigen::Vector3d(nan, 0.0, 0.0), 0.0));
}

// ── 6. Nothing the IK accepts is removed ─────────────────────────────────────

TEST(ReachBound, ATargetTheFilterRefusesIsOneTheIkRefuses) {
  for (const Case& c :
       {Case{"serial_6r_wrist.urdf", "catch_frame"}, Case{"serial_7dof.urdf", "tool_link"}}) {
    SCOPED_TRACE(c.urdf);
    rtc::testing::Arm arm = rtc::testing::MakeArm(c.urdf, c.frame);
    const ReachBound bound = ComputeReachBound(*arm.model, arm.frame);
    rtc::catching::CatchPoseIkOptions options;
    options.max_iter = 60;
    options.manipulability_min = 0.0;
    // From any posture inside the limits.
    const Eigen::VectorXd lo = arm.model->lowerPositionLimit;
    const Eigen::VectorXd hi = arm.model->upperPositionLimit;
    rtc::testing::ExpectTheIkRefusesWhatTheFilterRefuses(
        *arm.handle, arm.frame, bound, options, /*targets=*/300, /*ball_speed=*/3.0, 7936U,
        [&](std::mt19937& rng) {
          std::uniform_real_distribution<double> unit(0.0, 1.0);
          Eigen::VectorXd seed(arm.nv);
          for (int j = 0; j < arm.nv; ++j) {
            seed[j] = lo[j] + (hi[j] - lo[j]) * unit(rng);
          }
          return seed;
        });
  }
}

}  // namespace
