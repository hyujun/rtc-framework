// file: include/ur5e_bt_coordinator/robot_profile.hpp
#pragma once

#include "ur5e_bt_coordinator/topic_namer.hpp"

namespace rtc_bt {

// Compile-time DEFAULT joint widths (ur5e 6-DoF arm + 10-DoF hand — both
// assm_v1 3/3/3/1 and proto_1b 4/3/2/1 total 10). The *runtime* value lives in
// RobotProfile and is overridable via the arm_dof / hand_dof ROS 2 parameters
// so a different manipulator (e.g. iiwa7_leap) sets its own width without a
// recompile. Unifies the former duplicate constants bt_types.hpp::kHandDof and
// hand_pose_config.hpp::kHandDofCount into one default source.
inline constexpr int kDefaultArmDof = 6;
inline constexpr int kDefaultHandDof = 10;

/// Robot-agnostic profile (Seam A/B): the device-group topic tokens plus the
/// arm/hand joint widths that used to be hardcoded across the coordinator. The
/// default reproduces the legacy ur5e/hand 6/10-DoF behavior byte-for-byte.
struct RobotProfile {
  TopicNamer topics{};
  int arm_dof{kDefaultArmDof};
  int hand_dof{kDefaultHandDof};
};

}  // namespace rtc_bt
