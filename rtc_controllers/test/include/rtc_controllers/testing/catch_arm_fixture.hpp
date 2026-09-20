// ── Arm fixtures and oracles shared by the S1.9 catch-pose suites (test-only) ──
// test_catch_pose_ik.cpp (the correctness suite) and bench_catch_pose_ik_qp.cpp
// (the DLS-vs-QP measurement) must drive the SAME arms, build targets the same
// way, and read manipulability off the same reference Jacobian — otherwise the
// bench's comparison table and the suite's verdicts describe different problems
// and neither can be used to interpret the other. That is why these live here
// once instead of as a copy in each file (design-principles P5, and the same
// reasoning serial7dof_fixture.hpp records).
//
// The three arms and what each is for:
//   • serial_6r_wrist — nv = 6. The 5-row catch task leaves EXACTLY one free
//     degree of freedom (roll about the palm normal), so a dense roll sweep
//     enumerates the whole solution family. Its catch_frame carries a
//     translation offset AND a nonzero rpy, so its local +z is not the flange z
//     and not any base axis: a frame or row mix-up changes the answer instead
//     of cancelling.
//   • serial_7dof — nv = 7, redundant beyond the roll. Same code, 2-dimensional
//     null space, non-square Jacobian.
//   • serial_6dof — all six axes parallel to Z. The LOCAL angular rows about x
//     and y vanish identically, so J₅ can never reach rank 5. This is the
//     rank-deficient NEGATIVE fixture, never a positive one.
//
// Test-only, NEVER installed: ament's symlink install ignores
// install(PATTERN EXCLUDE), so a header placed under include/ would ship.
#pragma once

#include "rtc_math/se3/so3.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"
#include "test_urdf_path.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <random>
#include <span>
#include <string>

namespace rtc::testing {

/// A URDF fixture plus the handle, frame id and nv every caller needs together.
struct Arm {
  std::shared_ptr<const pinocchio::Model> model;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  pinocchio::FrameIndex frame{0};
  int nv{0};
};

[[nodiscard]] inline Arm MakeArm(const std::string& urdf, const std::string& frame_name) {
  Arm a;
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = rtc::test::TestUrdfPath(urdf);
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  a.model = builder.GetFullModel();
  a.handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(a.model);
  a.frame = a.handle->GetFrameId(frame_name);
  a.nv = a.handle->nv();
  return a;
}

[[nodiscard]] inline Arm Arm6R() {
  return MakeArm("serial_6r_wrist.urdf", "catch_frame");
}

[[nodiscard]] inline Arm Arm7R() {
  return MakeArm("serial_7dof.urdf", "tool_link");
}

[[nodiscard]] inline std::span<const double> AsSpan(const Eigen::VectorXd& q) {
  return {q.data(), static_cast<std::size_t>(q.size())};
}

struct Pose {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
};

[[nodiscard]] inline Pose Fk(Arm& a, const Eigen::VectorXd& q) {
  a.handle->ComputeForwardKinematics(AsSpan(q));
  return {a.handle->GetFramePosition(a.frame), a.handle->GetFrameRotation(a.frame)};
}

/// A catch target that the configuration `q` satisfies EXACTLY, so any
/// acceptance failure belongs to the solver and not to an unreachable goal.
struct Target {
  Eigen::Vector3d p_c{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ball{Eigen::Vector3d::Zero()};
};

[[nodiscard]] inline Target TargetAt(Arm& a, const Eigen::VectorXd& q, double speed = 7.0) {
  const Pose pose = Fk(a, q);
  Target t;
  t.p_c = pose.p;
  // a_d = −v̂, and a_d must be the frame's local +z ⇒ v_ball = −speed·(R·ê_z).
  t.v_ball = -speed * (pose.R * Eigen::Vector3d::UnitZ());
  return t;
}

/// [J^LWA_p ; J^L_ω] at `q` — the 6×nv stack the 5-row task selects from.
///
/// Pins ARITHMETIC, not the row selection: it would agree with an
/// implementation that picked the wrong rows, because it makes the same two
/// calls. FdJacobian() is the oracle that fixes the convention.
[[nodiscard]] inline Eigen::MatrixXd StackJacobianRef(Arm& a, const Eigen::VectorXd& q) {
  Eigen::MatrixXd j_world = Eigen::MatrixXd::Zero(6, a.nv);
  Eigen::MatrixXd j_local = Eigen::MatrixXd::Zero(6, a.nv);
  a.handle->ComputeJacobians(AsSpan(q));
  a.handle->GetFrameJacobian(a.frame, pinocchio::LOCAL_WORLD_ALIGNED, j_world);
  a.handle->GetFrameJacobian(a.frame, pinocchio::LOCAL, j_local);
  Eigen::MatrixXd j(6, a.nv);
  j.topRows(3) = j_world.topRows(3);
  j.bottomRows(3) = j_local.bottomRows(3);
  return j;
}

/// The same stacked J from forward kinematics alone — no Jacobian routine.
///
/// Linear rows: dp/dq_i in world axes, which IS the LOCAL_WORLD_ALIGNED linear
/// block. Angular rows: R(q+h) = R(q−h)·exp([δ]×) with δ = log3(R(q−h)ᵀR(q+h)),
/// so δ/(2h) is the angular velocity in the frame's OWN axes — the LOCAL block.
[[nodiscard]] inline Eigen::MatrixXd FdJacobian(Arm& a, const Eigen::VectorXd& q, double h = 1e-6) {
  Eigen::MatrixXd j(6, a.nv);
  Eigen::VectorXd probe = q;
  for (int i = 0; i < a.nv; ++i) {
    const double qi = q(i);
    probe(i) = qi + h;
    const Pose up = Fk(a, probe);
    probe(i) = qi - h;
    const Pose dn = Fk(a, probe);
    probe(i) = qi;
    j.block<3, 1>(0, i) = (up.p - dn.p) / (2.0 * h);
    j.block<3, 1>(3, i) = rtc::math::se3::log3(dn.R.transpose() * up.R) / (2.0 * h);
  }
  return j;
}

/// √det(M Mᵀ) over the top `rows` of `j`, by a plain dense determinant — the
/// naive form the production code deliberately does NOT use, which is exactly
/// what makes it a usable cross-check.
[[nodiscard]] inline double ManipFromRows(const Eigen::MatrixXd& j, int rows) {
  const Eigen::MatrixXd block = j.topRows(rows);
  const Eigen::MatrixXd gram = block * block.transpose();
  return std::sqrt(std::max(0.0, gram.determinant()));
}

/// A configuration strictly inside the model's limits, deterministic in `rng`.
/// `inset` keeps it off the bounds so a sampled pose is a usable seed.
[[nodiscard]] inline Eigen::VectorXd SampleQ(const Arm& a, std::mt19937& rng, double inset = 0.25) {
  Eigen::VectorXd q(a.nv);
  for (int i = 0; i < a.nv; ++i) {
    const double lo = a.model->lowerPositionLimit(i) + inset;
    const double hi = a.model->upperPositionLimit(i) - inset;
    std::uniform_real_distribution<double> dist(lo, hi);
    q(i) = dist(rng);
  }
  return q;
}

}  // namespace rtc::testing
