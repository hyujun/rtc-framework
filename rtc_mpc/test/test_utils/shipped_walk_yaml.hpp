#pragma once

/// @file shipped_walk_yaml.hpp
/// @brief The g1_loco profile's gait, reference, swing and terrain sections
///        and its online iteration cap as shipped — copies, since this
///        package reads no profile file. Text
///        only, with no dependency, so a suite that does read the installed
///        profile (integrated_bringup's test_locomotion_mpc_config) can hold
///        these copies to it; shipped_walk_fixture.hpp builds the walk from
///        them. Not installed — reached through rtc_mpc's test include dir.

namespace rtc::mpc::test_utils {

/// The g1_loco profile's walk gait and reference, as shipped (copies — this
/// package reads no profile file): walk single support 0.45 s, double support
/// 0.1 s; q_default knees 0.6, hip and ankle pitch −0.3, shoulder rolls ±0.15
/// (left +, right −), arm swing on.
inline constexpr const char* kShippedGaitYaml = R"(
sequence: [stance, slow_walk, walk]
min_dwell: 0.2
switch_lead: 1.0
gaits:
  stance:
    modes: [[left_foot, right_foot]]
    switching_times: [0.0, 0.5]
    faster_above: [0.1, 0.1]
    slower_below: [0.0, 0.0]
    measured_tolerance: [10.0, 10.0]
  slow_walk:
    modes: [[left_foot], [left_foot, right_foot], [right_foot], [left_foot, right_foot]]
    switching_times: [0.0, 0.45, 0.65, 1.1, 1.3]
    faster_above: [0.3, 0.2]
    slower_below: [0.05, 0.05]
    measured_tolerance: [0.05, 0.05]
  walk:
    modes: [[left_foot], [left_foot, right_foot], [right_foot], [left_foot, right_foot]]
    switching_times: [0.0, 0.45, 0.55, 1.0, 1.1]
    faster_above: [0.5, 0.35]
    slower_below: [0.25, 0.15]
    measured_tolerance: [0.05, 0.05]
)";

inline constexpr const char* kShippedReferenceYaml = R"(
intermediate_fraction: 0.7
q_default:
  left_hip_pitch_joint: -0.3
  left_hip_roll_joint: 0.0
  left_hip_yaw_joint: 0.0
  left_knee_joint: 0.6
  left_ankle_pitch_joint: -0.3
  left_ankle_roll_joint: 0.0
  right_hip_pitch_joint: -0.3
  right_hip_roll_joint: 0.0
  right_hip_yaw_joint: 0.0
  right_knee_joint: 0.6
  right_ankle_pitch_joint: -0.3
  right_ankle_roll_joint: 0.0
  waist_yaw_joint: 0.0
  waist_roll_joint: 0.0
  waist_pitch_joint: 0.0
  left_shoulder_pitch_joint: 0.0
  left_shoulder_roll_joint: 0.15
  left_shoulder_yaw_joint: 0.0
  left_elbow_joint: 0.0
  right_shoulder_pitch_joint: 0.0
  right_shoulder_roll_joint: -0.15
  right_shoulder_yaw_joint: 0.0
  right_elbow_joint: 0.0
arm_swing:
  enabled: true
  gain: 0.15
  phase_offset: 0.15
  joints:
    - {name: left_shoulder_pitch_joint, sign: -1}
    - {name: left_elbow_joint, sign: -1}
    - {name: right_shoulder_pitch_joint, sign: 1}
    - {name: right_elbow_joint, sign: 1}
)";

/// The shipped swing and terrain sections (the values of `ShippedSwing()`).
inline constexpr const char* kShippedSwingYaml = R"(
height: 0.08
lift_off_velocity: 0.05
touch_down_velocity: 0.0
touch_down_offset: -0.001
time_scale: 0.4
impact_proximity:
  mid: 0.005
  lift_off_velocity: -0.15
  touch_down_velocity: 0.3
)";

inline constexpr const char* kShippedTerrainYaml = R"(
height: 0.0
)";

/// The g1_loco profile's solve budget and period, as shipped — copies, held
/// to the installed profile like the sections above: `mpc.solver.max_iters`,
/// `mpc.solver.max_al_iters`, `mpc.period_nodes` and
/// `mpc.solver.publish_prim_tol`.
inline constexpr int kShippedOnlineMaxIters = 6;
inline constexpr int kShippedOnlineAlIters = 1;
inline constexpr int kShippedPeriodNodes = 2;
inline constexpr double kShippedPublishPrimTol = 0.05;

}  // namespace rtc::mpc::test_utils
