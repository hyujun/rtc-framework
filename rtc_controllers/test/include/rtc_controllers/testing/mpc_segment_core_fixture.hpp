// ── Arm fixtures shared by the decel MPC core suites (test-only) ──────────────
// test_catching_mpc_segment_core.cpp (E1-F01: the stop segment) and
// test_catching_mpc_segment_core_approach.cpp (E1-F07: the pre-catch grid and the
// catch terms) must load the SAME arms with the SAME limits and nominal
// postures: the second suite's golden regression and timing table are read
// against the first suite's, and two copies of these loaders would let them
// drift apart (design-principles P5; catch_arm_fixture.hpp records the same
// reasoning for the catch-pose suites).
//
// The including TU's target must define RTC_TEST_ROBOT_DESCRIPTIONS_DIR (the
// sibling robot_descriptions tree, by source path — CMakeLists note).
//
// Test-only, NEVER installed: ament's symlink install ignores
// install(PATTERN EXCLUDE), so a header placed under include/ would ship.
#pragma once

#include "rtc_controllers/catching/mpc_segment_core.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"
#include "test_urdf_path.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rtc::testing::mpc_segment_core {

struct ArmModel {
  std::shared_ptr<const pinocchio::Model> model;
  pinocchio::FrameIndex frame{0};
  Eigen::VectorXd q_nominal;
  std::string name;
};

inline ArmModel LoadArm(const std::string& path, const std::string& frame,
                        Eigen::VectorXd q_nominal, const std::string& name) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = path;
  config.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  ArmModel arm;
  arm.model = builder.GetFullModel();
  arm.frame = arm.model->getFrameId(frame);
  arm.q_nominal = std::move(q_nominal);
  arm.name = name;
  return arm;
}

inline std::string DescriptionPath(const std::string& relative) {
  return std::string(RTC_TEST_ROBOT_DESCRIPTIONS_DIR) + "/" + relative;
}

// Synthetic 6R with a wrist offset (rtc_urdf_bridge fixture).
inline ArmModel Synthetic6R() {
  Eigen::VectorXd q(6);
  q << 0.2, -0.5, 0.9, -0.4, 0.6, 0.1;
  return LoadArm(rtc::test::TestUrdfPath("serial_6r_wrist.urdf"), "catch_frame", q, "synthetic_6r");
}

// Real arm dimensions (#627 asks for the two target robots' n = 6 and n = 7).
inline ArmModel RealArm6() {
  Eigen::VectorXd q(6);
  q << 0.0, -1.2, 1.3, -1.6, -1.57, 0.0;
  return LoadArm(DescriptionPath("ur5e/urdf/ur5e.urdf"), "tool0", q, "real_6dof");
}

inline ArmModel RealArm7() {
  Eigen::VectorXd q(7);
  q << 0.0, 0.6, 0.0, -1.2, 0.0, 0.9, 0.0;
  return LoadArm(DescriptionPath("iiwa7/urdf/iiwa7.urdf"), "ee_link", q, "real_7dof");
}

inline rtc::catching::MpcSegmentCoreLimits LimitsFromModel(const pinocchio::Model& m,
                                                           double armature = 0.0) {
  rtc::catching::MpcSegmentCoreLimits lim;
  lim.q_min = m.lowerPositionLimit;
  lim.q_max = m.upperPositionLimit;
  lim.qd_max = m.velocityLimit;
  lim.tau_max = m.effortLimit;
  lim.armature = Eigen::VectorXd::Constant(m.nv, armature);
  return lim;
}

inline rtc::catching::MpcSegmentCoreInput RestInput(const Eigen::VectorXd& q0) {
  rtc::catching::MpcSegmentCoreInput in;
  in.q0 = q0;
  in.qd0 = Eigen::VectorXd::Zero(q0.size());
  in.qdd0 = Eigen::VectorXd::Zero(q0.size());
  return in;
}

// The next cycle's reference: the solution just returned.
inline void UseAsReference(const rtc::catching::MpcSegmentCoreResult& r,
                           rtc::catching::MpcSegmentCoreInput& in) {
  in.q_ref = r.q;
  in.qd_ref = r.qd;
  in.qdd_ref = r.qdd;
  in.reference_valid = true;
}

inline int Rank(const Eigen::MatrixXd& a) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(a);
  const Eigen::VectorXd& s = svd.singularValues();
  const double tol = 1e-10 * std::max(1.0, s.size() > 0 ? s[0] : 0.0);
  return static_cast<int>((s.array() > tol).count());
}

inline double Percentile(std::vector<double> v, double p) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const auto idx = static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size())) - 1.0);
  return v[std::min(idx, v.size() - 1)];
}

// Integer microseconds: RecordProperty(string, double) would go through
// to_string and squash small values; ints stay exact.
inline void RecordMicros(const std::string& key, double us) {
  ::testing::Test::RecordProperty(key, static_cast<int>(std::lround(us)));
}

}  // namespace rtc::testing::mpc_segment_core
