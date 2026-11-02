// ── A kinematic tree built in code, for joint counts no fixture URDF has ──────
// The catching suites load one arm (6 or 7 joints). What a wider model does to
// the segment payload and to the planners' capacity checks needs a model that
// is wider than an arm and is not a robot's: a trunk of revolute joints with
// arms branching off its last link. The joint count is the caller's, so a test
// can sit exactly at a capacity, or one past it.
//
// Test-only, NEVER installed (mpc_segment_core_fixture.hpp records why).
#pragma once

#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/joint/joints.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <memory>
#include <string>

namespace rtc::testing {

/// @brief A trunk of `n_trunk` revolute joints and `n_branches` arms of
///        `n_branch` joints each, all off the trunk's last link (off the base
///        when there is no trunk). nq == nv == n_trunk + n_branches·n_branch.
///
/// The catch frame is at the tip of the FIRST arm (of the trunk when there is
/// no arm), so the frame's chain is the trunk and one arm and the other arms
/// are joints the frame does not move — the shape a segment model has when it
/// is wider than the chain a catch is searched on. Axes cycle z, y, x along a
/// chain; every joint has the same symmetric position box, velocity and
/// effort limits, wide enough to hold the tree against gravity anywhere.
[[nodiscard]] inline mpc_segment_core::ArmModel SyntheticTree(int n_trunk, int n_branch,
                                                              int n_branches) {
  auto model = std::make_shared<pinocchio::Model>();
  model->name = "synthetic_tree";
  const Eigen::VectorXd effort = Eigen::VectorXd::Constant(1, 300.0);
  const Eigen::VectorXd velocity = Eigen::VectorXd::Constant(1, 3.0);
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(1, 2.6);
  const Eigen::VectorXd lower = -upper;
  const pinocchio::Inertia link(0.4, Eigen::Vector3d(0.05, 0.0, 0.0),
                                Eigen::Matrix3d(1e-3 * Eigen::Matrix3d::Identity()));
  const auto add = [&](pinocchio::JointIndex parent, int depth, const Eigen::Vector3d& offset,
                       const std::string& name) {
    const pinocchio::SE3 placement(Eigen::Matrix3d::Identity(), offset);
    pinocchio::JointModel joint;
    if (depth % 3 == 0) {
      joint = pinocchio::JointModelRZ();
    } else if (depth % 3 == 1) {
      joint = pinocchio::JointModelRY();
    } else {
      joint = pinocchio::JointModelRX();
    }
    const pinocchio::JointIndex id =
        model->addJoint(parent, joint, placement, name, effort, velocity, lower, upper);
    model->appendBodyToJoint(id, link, pinocchio::SE3::Identity());
    return id;
  };
  pinocchio::JointIndex trunk_tip = 0;
  for (int j = 0; j < n_trunk; ++j) {
    trunk_tip = add(trunk_tip, j, Eigen::Vector3d(0.0, 0.0, 0.12), "trunk_" + std::to_string(j));
  }
  pinocchio::JointIndex frame_joint = trunk_tip;
  for (int b = 0; b < n_branches; ++b) {
    // Arms leave the trunk on alternating sides.
    const double side = b % 2 == 0 ? 0.2 : -0.2;
    pinocchio::JointIndex tip = trunk_tip;
    for (int j = 0; j < n_branch; ++j) {
      const Eigen::Vector3d offset =
          j == 0 ? Eigen::Vector3d(0.0, side, 0.1) : Eigen::Vector3d(0.1, 0.0, 0.0);
      tip = add(tip, j, offset, "arm_" + std::to_string(b) + "_" + std::to_string(j));
    }
    if (b == 0) {
      frame_joint = tip;
    }
  }
  mpc_segment_core::ArmModel arm;
  arm.frame = model->addFrame(
      pinocchio::Frame("catch_frame", frame_joint, 0,
                       pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.1, 0.0, 0.0)),
                       pinocchio::OP_FRAME));
  // A bent posture, away from the straight (singular) one and from the box.
  arm.q_nominal.resize(model->nv);
  for (int m = 0; m < model->nv; ++m) {
    arm.q_nominal[m] = (m % 2 == 0 ? 0.35 : -0.5) + 0.01 * m;
  }
  arm.name = "synthetic_tree_" + std::to_string(model->nv);
  arm.model = std::move(model);
  return arm;
}

/// @brief SyntheticTree with a BODY frame "<joint>_link" on every joint: a
///        capsule spec names links, and the synthetic tree declares none.
[[nodiscard]] inline mpc_segment_core::ArmModel SyntheticTreeWithLinkFrames(int n_trunk,
                                                                            int n_branch,
                                                                            int n_branches) {
  mpc_segment_core::ArmModel arm = SyntheticTree(n_trunk, n_branch, n_branches);
  auto model = std::make_shared<pinocchio::Model>(*arm.model);
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model->njoints); ++j) {
    model->addJointFrame(j);
    model->addBodyFrame(model->names[j] + "_link", j);
  }
  arm.model = model;
  return arm;
}

/// @brief A capsule spec (rtc_urdf_bridge CapsuleModel YAML) for
///        SyntheticTreeWithLinkFrames(n_trunk, n_branch ≥ 7, 2): a capsule on
///        the trunk's top link and on every link of both arms (0.1 m along +x
///        of the joint frame), the cross pairs a0_{1,3,5} × a1_{1,3,5}, the
///        trunk pairs with a{0,1}_{3,5} when `trunk_pairs`, and the OTHER
///        arm's links a1_{2,4,6} as the ball capsules when `with_ball` (the
///        catch arm may not carry one: it would avoid the ball).
[[nodiscard]] inline std::string SyntheticTreeCapsuleYaml(int n_trunk, int n_branch,
                                                          bool trunk_pairs, bool with_ball) {
  std::string y = "collision_capsules:\n  capsules:\n";
  y += "    - {name: trunk, link: trunk_" + std::to_string(n_trunk - 1) +
       "_link, p0: [0, 0, 0], p1: [0, 0, 0.1], radius: 0.05}\n";
  for (int b = 0; b < 2; ++b) {
    for (int j = 0; j < n_branch; ++j) {
      y += "    - {name: a" + std::to_string(b) + "_" + std::to_string(j) + ", link: arm_" +
           std::to_string(b) + "_" + std::to_string(j) +
           "_link, p0: [0, 0, 0], p1: [0.1, 0, 0], radius: 0.03}\n";
    }
  }
  y += "  pairs:\n";
  for (const int i : {1, 3, 5}) {
    for (const int j : {1, 3, 5}) {
      y += "    - [a0_" + std::to_string(i) + ", a1_" + std::to_string(j) + "]\n";
    }
  }
  if (trunk_pairs) {
    for (const int j : {3, 5}) {
      y += "    - [trunk, a1_" + std::to_string(j) + "]\n    - [trunk, a0_" + std::to_string(j) +
           "]\n";
    }
  }
  if (with_ball) {
    y += "  ball_capsules: [a1_2, a1_4, a1_6]\n";
  }
  return y;
}

}  // namespace rtc::testing
