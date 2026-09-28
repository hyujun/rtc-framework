#pragma once

/// @file panda_fixture.hpp
/// @brief The Panda (9-DoF, fixed base, 2×3D fingertip contacts) the rtc_mpc
///        suites use as their generic model: its URDF path (set per test
///        target by add_mpc_gtest), its RobotModelHandler config, and the
///        neutral-pose target and state they start from. The URDF-missing
///        GTEST_SKIP stays in each SetUp — a skip in a helper only leaves the
///        helper. Not installed — header is test-local by design.

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <Eigen/Core>

#include <cstddef>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

namespace rtc::mpc::test_utils {

inline constexpr const char* kPandaUrdf = RTC_PANDA_URDF_PATH;

/// RobotModelHandler::Init config: panda_hand over panda_link0, both fingers
/// as 3-D point contacts.
inline constexpr const char* kPandaTwoFingerYaml = R"(
end_effector_frame: panda_hand
base_frame: panda_link0
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)";

/// The end-effector pose at the neutral configuration — the suites' default
/// ee_target.
inline pinocchio::SE3 NeutralEeTarget(const pinocchio::Model& model,
                                      const RobotModelHandler& handler) {
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, pinocchio::neutral(model));
  return data.oMf[static_cast<std::size_t>(handler.end_effector_frame_id())];
}

/// The arm at rest in the neutral configuration.
inline MPCStateSnapshot NeutralStateSnapshot(const pinocchio::Model& model,
                                             const RobotModelHandler& handler) {
  MPCStateSnapshot s{};
  const Eigen::VectorXd q = pinocchio::neutral(model);
  s.nq = handler.nq();
  s.nv = handler.nv();
  for (int i = 0; i < s.nq; ++i) {
    s.q[static_cast<std::size_t>(i)] = q[i];
  }
  return s;  // v and timestamp_ns stay value-initialised (0)
}

}  // namespace rtc::mpc::test_utils
