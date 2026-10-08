// The synthetic 17-DoF tree the multi-frame CLIK suites run on
// (test/urdf/tree_17dof.urdf, found through RTC_TSID_TEST_URDF_DIR — set per
// test target in CMakeLists.txt): a 3-joint trunk carrying two 7-joint arms.
// It stands in for "a waist with two arms" without being any robot — the
// suites need a tree, a frame that is the base of another, and enough joints
// that the Eigen product kernels differ from the 6- and 9-DoF fixtures.

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/types/wbc_types.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace rtc::tsid::test {

inline constexpr int kTreeNv = 17;

/// A fresh tree model, parsed from the URDF on every call.
inline std::shared_ptr<pinocchio::Model> LoadTreeModel() {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(std::string(RTC_TSID_TEST_URDF_DIR) + "/tree_17dof.urdf", *model);
  return model;
}

/// Velocity indices of the joints whose name starts with `prefix`, in model
/// order ("trunk_", "arm_a_", "arm_b_").
inline std::vector<int> VelocityIndices(const pinocchio::Model& model, std::string_view prefix) {
  std::vector<int> idx;
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model.njoints); ++j) {
    if (std::string_view(model.names[j]).substr(0, prefix.size()) == prefix) {
      idx.push_back(model.joints[j].idx_v());
    }
  }
  return idx;
}

/// A bent, collision-free-looking posture well inside every joint's range:
/// both elbows at −1.2 rad, the shoulders opened, the trunk pitched a little.
inline Eigen::VectorXd TreeHomePosture(const pinocchio::Model& model) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  const auto set = [&](const char* joint, double value) {
    q(model.joints[model.getJointId(joint)].idx_q()) = value;
  };
  set("trunk_pitch", 0.1);
  set("arm_a_1", -0.4);
  set("arm_a_2", 0.3);
  set("arm_a_4", -1.2);
  set("arm_a_6", 0.4);
  set("arm_b_1", -0.4);
  set("arm_b_2", -0.3);
  set("arm_b_4", -1.2);
  set("arm_b_6", -0.4);
  return q;
}

}  // namespace rtc::tsid::test
