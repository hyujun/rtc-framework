#pragma once

/// @file g1_fixture.hpp
/// @brief The public G1 on a free-flyer (23 actuated joints, two 6-D sole
///        contacts): the floating-base model of the dynamics suites. Its URDF
///        path is set per test target by add_mpc_gtest. The URDF-missing
///        GTEST_SKIP stays in each SetUp — a skip in a helper only leaves the
///        helper. Not installed — header is test-local by design.

#include "test_utils/kinodynamic_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Geometry>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

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

/// The pre-registered gate set on the G1: 200 seeded draws (mode = i mod 4),
/// then 20 knees-at-lower-limit, 10 high-speed, 10 near-π attitude (5 ±q
/// pairs), then 10 knees at exactly zero (added after registration, at the
/// end so the registered draws are unchanged) — 250 samples.
inline std::vector<KinoSample> G1GateSamples(const KinoOracle& oracle, std::uint32_t seed) {
  const pinocchio::Model& model = oracle.model();
  std::mt19937 rng(seed);
  std::vector<KinoSample> out;
  for (int i = 0; i < 200; ++i) {
    out.push_back(oracle.Draw(rng, static_cast<std::uint32_t>(i % 4)));
  }
  const std::array<int, 2> knees{JointQIndex(model, "left_knee_joint"),
                                 JointQIndex(model, "right_knee_joint")};
  for (int i = 0; i < 20; ++i) {
    KinoSample s = oracle.Draw(rng, static_cast<std::uint32_t>(i % 4));
    for (const int k : knees) {
      s.x[k] = model.lowerPositionLimit[k];
    }
    out.push_back(s);
  }
  KinoRanges fast;
  fast.v_lin = 3.0 / std::sqrt(3.0);
  fast.v_ang = 5.0 / std::sqrt(3.0);
  fast.v_joint = 10.0;
  for (int i = 0; i < 10; ++i) {
    out.push_back(oracle.Draw(rng, static_cast<std::uint32_t>(i % 4), fast));
  }
  std::normal_distribution<double> gauss(0.0, 1.0);
  for (int i = 0; i < 5; ++i) {
    KinoSample s = oracle.Draw(rng, static_cast<std::uint32_t>((i + 1) % 4));
    Eigen::Vector3d axis(gauss(rng), gauss(rng), gauss(rng));
    axis.normalize();
    const Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI - 1e-6 * (i + 1), axis));
    s.x.segment<4>(3) = quat.coeffs();  // (x, y, z, w)
    out.push_back(s);
    s.x.segment<4>(3) = -quat.coeffs();
    out.push_back(s);
  }
  for (int i = 0; i < 10; ++i) {
    KinoSample s = oracle.Draw(rng, static_cast<std::uint32_t>(i % 4));
    for (const int k : knees) {
      s.x[k] = 0.0;
    }
    out.push_back(s);
  }
  return out;
}

/// The G1's two sole frames, contact order.
inline std::vector<pinocchio::FrameIndex> G1SoleFrames(const pinocchio::Model& model) {
  return {model.getFrameId("left_sole"), model.getFrameId("right_sole")};
}

}  // namespace rtc::mpc::test_utils
