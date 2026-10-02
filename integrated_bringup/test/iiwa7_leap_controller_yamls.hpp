// ── Minimal controller YAMLs for the iiwa7_leap fixture, one per controller ──
//
// The four controllers that publish fingertip poses (joint, task, compliance,
// wbc), each with the keys its LoadConfig requires and nothing else, on the
// device groups of iiwa7_leap_test_fixture.hpp. A test that has to put the
// SAME question to all four — a configure-time rule every one of them owes —
// brings them up through ControllerFixture<Ctrl> below instead of carrying four
// YAMLs of its own.
//
// The wbc YAML has no `tsid:` block on purpose: with TSID initialised that
// controller already refuses a configure for reasons of its own, and a test of
// a refusal it shares with the other three would pass on that one whether or
// not the shared rule held.
#pragma once

#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

#include <memory>
#include <string>

namespace integrated_bringup::testfx {

inline const char* const kIiwa7LeapJointYaml = R"(
arm_dof: 7
robot_trajectory_speed: 2.0
hand_trajectory_speed: 3.0
robot_max_traj_velocity: 3.14
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

inline const char* const kIiwa7LeapTaskYaml = R"(
arm_dof: 7
kp_translation: [5.0, 5.0, 5.0]
kp_rotation: [2.0, 2.0, 2.0]
singularity_threshold: 0.02
max_damping: 0.05
null_kp: 0.5
enable_null_space: true
control_6dof: false
trajectory_speed: 0.5
trajectory_angular_speed: 2.0
hand_trajectory_speed: 3.0
max_traj_velocity: 1.0
max_traj_angular_velocity: 4.0
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  pi_rotation_margin: 0.15
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

// demo_compliance reads the same blocks as demo_task under its own gain names,
// plus the wrench source it owns.
inline const char* const kIiwa7LeapComplianceYaml = R"(
arm_dof: 7
ik_kp_pos: [5.0, 5.0, 5.0]
ik_kp_rot: [2.0, 2.0, 2.0]
singularity_threshold: 0.02
max_damping: 0.05
nullspace_kp: 0.5
enable_null_space: true
control_6dof: false
trajectory_speed: 0.5
trajectory_angular_speed: 2.0
hand_trajectory_speed: 3.0
max_traj_velocity: 1.0
max_traj_angular_velocity: 4.0
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  pi_rotation_margin: 0.15
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
external_wrench:
  source: "pull_estimator"
)";

inline const char* const kIiwa7LeapWbcYaml = R"(
arm_dof: 7
command_type: "position"
arm_trajectory_speed: 0.5
hand_trajectory_speed: 1.0
arm_max_traj_velocity: 1.71
hand_max_traj_velocity: 4.0
tcp_trajectory_speed: 0.1
tcp_trajectory_angular_speed: 0.5
tcp_max_traj_velocity: 0.5
tcp_max_traj_angular_velocity: 1.0
pi_rotation_margin: 0.15
integration:
  position_margin: 0.02
  velocity_scale: 0.95
  force_rate_alpha: 0.1
clik:
  damping_sq: 1.0e-4
  v_limit: 1.5
  kx_pos: 5.0
  kx_rot: 2.0
  ka: 1.0
  kh: 1.0
  anchor_drift_max: 0.5
fsm:
  epsilon_pregrasp: 0.005
  force_contact_threshold: 0.2
  min_contacts_for_hold: 2
  slip_rate_threshold: 5.0
  deformation_threshold: 0.015
  max_qp_fail_before_fallback: 5
  approach_speed: 0.5
  release_ramp_sec: 0.03
estop:
  arm_safe_position: [0.0, 0.7, 0.0, -1.4, 0.0, 0.7, 0.0]
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

/// What differs between the four controllers when a test brings one up: its
/// YAML, a short name for messages and node names, and how it is constructed
/// (task and compliance take a Gains struct).
template <class Ctrl>
struct ControllerFixture;

template <>
struct ControllerFixture<DemoJointController> {
  static constexpr const char* kName = "joint";

  static std::string Yaml() { return kIiwa7LeapJointYaml; }

  static std::unique_ptr<DemoJointController> Make() {
    return std::make_unique<DemoJointController>("");
  }
};

template <>
struct ControllerFixture<DemoTaskController> {
  static constexpr const char* kName = "task";

  static std::string Yaml() { return kIiwa7LeapTaskYaml; }

  static std::unique_ptr<DemoTaskController> Make() {
    return std::make_unique<DemoTaskController>("", DemoTaskController::Gains{});
  }
};

template <>
struct ControllerFixture<DemoComplianceController> {
  static constexpr const char* kName = "compliance";

  static std::string Yaml() { return kIiwa7LeapComplianceYaml; }

  static std::unique_ptr<DemoComplianceController> Make() {
    return std::make_unique<DemoComplianceController>("", DemoComplianceController::Gains{});
  }
};

template <>
struct ControllerFixture<DemoWbcController> {
  static constexpr const char* kName = "wbc";

  static std::string Yaml() { return kIiwa7LeapWbcYaml; }

  static std::unique_ptr<DemoWbcController> Make() {
    return std::make_unique<DemoWbcController>("");
  }
};

}  // namespace integrated_bringup::testfx
