// ── TaskTargetFrames: the frames a task goal may name (#779) ─────────────────
//
// The model is synthetic: one revolute joint, a root_link frame and a
// scene_world frame on the universe, both away from the origin and turned
// about two axes, so a conversion that uses R where it needs Rᵀ (or forgets
// the offset) lands somewhere else. The oracle is the SE(3) composition
// root_T_goal = root_T_world · world_T_goal, computed independently here.

#include "integrated_bringup/support/task_target_frames.hpp"
#include <rtc_math/se3/so3.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/joint/joints.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>

namespace {

using integrated_bringup::TaskTargetFrames;
using Layout = TaskTargetFrames::Layout;

constexpr const char* kRoot = "arm_root";

pinocchio::SE3 Turned(const Eigen::Vector3d& t, double rz, double rx) {
  return {(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
              .toRotationMatrix(),
          t};
}

// root_link in the universe, and the world in root_link.
const pinocchio::SE3 kUniverseRoot = Turned({0.3, -0.2, 0.5}, 0.4, -0.2);
const pinocchio::SE3 kRootWorld = Turned({0.12, -0.08, -0.7}, 0.7, 0.3);

/// `with_world`: add scene_world; `world_on_joint`: hang it off the moving
/// joint instead of the universe (not rigid to root_link).
pinocchio::Model MakeModel(bool with_world, bool world_on_joint = false) {
  pinocchio::Model model;
  const auto j1 = model.addJoint(0, pinocchio::JointModelRZ(), kUniverseRoot, "j1");
  model.addFrame(pinocchio::Frame(kRoot, 0, 0, kUniverseRoot, pinocchio::OP_FRAME));
  model.addFrame(pinocchio::Frame("tip", j1, 0, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  if (with_world) {
    const auto parent = world_on_joint ? j1 : pinocchio::JointIndex{0};
    const pinocchio::SE3 placement =
        world_on_joint ? kRootWorld : pinocchio::SE3(kUniverseRoot * kRootWorld);
    model.addFrame(pinocchio::Frame(std::string(TaskTargetFrames::kSceneWorld), parent, 0,
                                    placement, pinocchio::OP_FRAME));
  }
  return model;
}

std::uint64_t Bits(double x) {
  std::uint64_t b = 0;
  std::memcpy(&b, &x, sizeof(b));
  return b;
}

const std::array<double, 6> kGoal{0.25, -0.1, 0.4, 0.3, -0.5, 1.2};

TEST(TaskTargetFrames, TheRootLinkPassesThroughBitForBit) {
  TaskTargetFrames frames;
  ASSERT_EQ(frames.Configure(MakeModel(true), kRoot), "");
  std::array<double, 6> out{};
  ASSERT_EQ(frames.ToRoot(kRoot, kGoal, Layout::kPose6d, out), nullptr);
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_EQ(Bits(out[i]), Bits(kGoal[i])) << i;
  }
}

TEST(TaskTargetFrames, AWorldGoalIsTheWorldPoseSeenFromTheRoot) {
  TaskTargetFrames frames;
  ASSERT_EQ(frames.Configure(MakeModel(true), kRoot), "");
  ASSERT_TRUE(frames.WorldDefined());
  EXPECT_TRUE(frames.RootFromWorld().isApprox(kRootWorld, 1e-12));

  std::array<double, 6> out{};
  ASSERT_EQ(frames.ToRoot(TaskTargetFrames::kSceneWorld, kGoal, Layout::kPose6d, out), nullptr);

  const pinocchio::SE3 world_goal(
      rtc::math::se3::RpyToRotationZyx(Eigen::Vector3d(kGoal[3], kGoal[4], kGoal[5])),
      Eigen::Vector3d(kGoal[0], kGoal[1], kGoal[2]));
  const pinocchio::SE3 want = kRootWorld * world_goal;
  const Eigen::Vector3d got_p(out[0], out[1], out[2]);
  const Eigen::Matrix3d got_R =
      rtc::math::se3::RpyToRotationZyx(Eigen::Vector3d(out[3], out[4], out[5]));
  EXPECT_LT((got_p - want.translation()).norm(), 1e-12);
  EXPECT_LT((got_R - want.rotation()).norm(), 1e-12);
  // Not the identity, not the inverse placement: the fixture tells them apart.
  const pinocchio::SE3 wrong = kRootWorld.inverse() * world_goal;
  EXPECT_GT((got_p - wrong.translation()).norm(), 0.1);
}

TEST(TaskTargetFrames, PositionOnlyConvertsXyzAndLeavesTheRestUntouched) {
  TaskTargetFrames frames;
  ASSERT_EQ(frames.Configure(MakeModel(true), kRoot), "");
  std::array<double, 6> out{};
  ASSERT_EQ(frames.ToRoot(TaskTargetFrames::kSceneWorld, kGoal, Layout::kPositionOnly, out),
            nullptr);
  const Eigen::Vector3d want = kRootWorld.act(Eigen::Vector3d(kGoal[0], kGoal[1], kGoal[2]));
  EXPECT_LT((Eigen::Vector3d(out[0], out[1], out[2]) - want).norm(), 1e-12);
  for (std::size_t i = 3; i < 6; ++i) {
    EXPECT_EQ(Bits(out[i]), Bits(kGoal[i])) << i;  // null-space posture, not RPY
  }
}

TEST(TaskTargetFrames, AnEmptyFrameIsRefused) {
  TaskTargetFrames frames;
  ASSERT_EQ(frames.Configure(MakeModel(true), kRoot), "");
  std::array<double, 6> out{};
  EXPECT_NE(frames.ToRoot("", kGoal, Layout::kPose6d, out), nullptr);
}

TEST(TaskTargetFrames, AFrameThatIsNeitherIsRefused) {
  TaskTargetFrames frames;
  ASSERT_EQ(frames.Configure(MakeModel(true), kRoot), "");
  std::array<double, 6> out{};
  EXPECT_NE(frames.ToRoot("world", kGoal, Layout::kPose6d, out), nullptr);
  EXPECT_NE(frames.ToRoot("tip", kGoal, Layout::kPose6d, out), nullptr);  // a model frame
}

TEST(TaskTargetFrames, WithoutAWorldConfigureSucceedsAndWorldGoalsAreRefused) {
  TaskTargetFrames frames;
  ASSERT_EQ(frames.Configure(MakeModel(false), kRoot), "");
  EXPECT_FALSE(frames.WorldDefined());
  std::array<double, 6> out{};
  EXPECT_NE(frames.ToRoot(TaskTargetFrames::kSceneWorld, kGoal, Layout::kPose6d, out), nullptr);
  EXPECT_EQ(frames.ToRoot(kRoot, kGoal, Layout::kPose6d, out), nullptr);
}

TEST(TaskTargetFrames, AWorldThatMovesWithTheArmIsMalformed) {
  TaskTargetFrames frames;
  EXPECT_NE(frames.Configure(MakeModel(true, /*world_on_joint=*/true), kRoot), "");
  EXPECT_FALSE(frames.WorldDefined());
}

TEST(TaskTargetFrames, ARootTheModelLacksIsReported) {
  TaskTargetFrames frames;
  EXPECT_NE(frames.Configure(MakeModel(true), "no_such_link"), "");
  EXPECT_FALSE(frames.Configured());
  std::array<double, 6> out{};
  EXPECT_NE(frames.ToRoot("no_such_link", kGoal, Layout::kPose6d, out), nullptr);
}

TEST(TaskTargetFrames, AWrongSizedGoalIsRefused) {
  TaskTargetFrames frames;
  frames.ConfigureWithoutWorld(kRoot);
  std::array<double, 6> out{};
  const std::array<double, 5> short_goal{};
  EXPECT_NE(frames.ToRoot(kRoot, short_goal, Layout::kPose6d, out), nullptr);
}

}  // namespace
