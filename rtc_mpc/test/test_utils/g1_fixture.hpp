#pragma once

/// @file g1_fixture.hpp
/// @brief The public G1 on a free-flyer (23 actuated joints, two 6-D sole
///        contacts): the floating-base model of the dynamics suites. Its URDF
///        path is set per test target by add_mpc_gtest. The URDF-missing
///        GTEST_SKIP stays in each SetUp — a skip in a helper only leaves the
///        helper. Not installed — header is test-local by design.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

namespace rtc::mpc::test_utils {

inline constexpr const char* kG1Urdf = RTC_G1_URDF_PATH;

/// Contact 0 = left sole, contact 1 = right sole, both 6-D wrenches.
inline constexpr const char* kG1SolesYaml = R"(
end_effector_frame: pelvis
base_frame: universe
contact_frames:
  - name: left_sole
    dim: 6
  - name: right_sole
    dim: 6
)";

/// The G1 URDF on a free-flyer root.
inline pinocchio::Model BuildG1Floating() {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(kG1Urdf, pinocchio::JointModelFreeFlyer(), model);
  return model;
}

}  // namespace rtc::mpc::test_utils
