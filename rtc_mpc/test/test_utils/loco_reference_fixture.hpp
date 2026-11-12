#pragma once

/// @file loco_reference_fixture.hpp
/// @brief The W2-F02 PR B rig the stage-reference suites share: the Panda on
///        a free-flyer with two contact frames (an operational frame hung off
///        the root — closed-form oracle — and panda_leftfinger — FK oracle),
///        a two-contact gait table (stance · walk-like · hop with flight), the
///        shipped swing config, the registered snapshot and command. Not
///        installed — test-local by design.

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
#include "test_utils/panda_fixture.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace rtc::mpc::test_utils {

inline constexpr double kFootOffsetX = 0.10;
inline constexpr double kFootOffsetY = -0.05;
inline constexpr double kFootOffsetZ = -0.70;
inline constexpr double kFootYaw = 0.4;

/// The free-flyer Panda plus `foot_a`: an operational frame on the root joint
/// at (0.10, −0.05, −0.70), yawed 0.4 rad.
inline pinocchio::Model BuildLocoPanda() {
  pinocchio::Model model = BuildPandaFreeFlyer();
  const pinocchio::SE3 placement(
      Eigen::AngleAxisd(kFootYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d(kFootOffsetX, kFootOffsetY, kFootOffsetZ));
  model.addFrame(pinocchio::Frame("foot_a", 1, model.getFrameId("root_joint"), placement,
                                  pinocchio::OP_FRAME));
  return model;
}

inline constexpr const char* kLocoHandlerYaml = R"(
end_effector_frame: panda_hand
base_frame: universe
contact_frames:
  - name: foot_a
    dim: 6
  - name: panda_leftfinger
    dim: 3
)";

/// Contacts [contact_a, contact_b] = bits 0, 1. walk: b swings [0, 0.13), a
/// swings [0.37, 0.52) of a 0.90 s stride; hop: single supports with flight.
inline constexpr const char* kLocoGaitYaml = R"(
sequence: [stance, walk, hop]
min_dwell: 0.2
switch_lead: 0.63
gaits:
  stance:
    modes: [[contact_a, contact_b]]
    switching_times: [0.0, 0.5]
    faster_above: [0.1, 0.1]
    slower_below: [0.0, 0.0]
    measured_tolerance: [10.0, 10.0]
  walk:
    modes: [[contact_a], [contact_a, contact_b], [contact_b], [contact_a, contact_b]]
    switching_times: [0.0, 0.13, 0.37, 0.52, 0.90]
    faster_above: [0.5, 0.5]
    slower_below: [0.05, 0.05]
    measured_tolerance: [0.05, 0.05]
  hop:
    modes: [[contact_a], [], [contact_b], []]
    switching_times: [0.0, 0.15, 0.25, 0.40, 0.5]
    faster_above: [0.9, 0.9]
    slower_below: [0.45, 0.45]
    measured_tolerance: [0.05, 0.05]
)";

inline const std::vector<std::string>& LocoContactNames() {
  static const std::vector<std::string> names = {"contact_a", "contact_b"};
  return names;
}

inline GaitTable LocoGaitTable() {
  GaitTable t;
  const GaitTableError e = LoadGaitTable(YAML::Load(kLocoGaitYaml), LocoContactNames(), t);
  if (e != GaitTableError::kNoError) {
    throw std::runtime_error("loco gait table did not load");
  }
  return t;
}

inline SwingConfig ShippedSwing() {
  SwingConfig c;
  c.height = 0.08;
  c.lift_off_velocity = 0.05;
  c.touch_down_velocity = 0.0;
  c.touch_down_offset = -0.001;
  c.time_scale = 0.4;
  c.impact_proximity.mid = 0.005;
  c.impact_proximity.lift_off_velocity = -0.15;
  c.impact_proximity.touch_down_velocity = 0.3;
  return c;
}

/// The model's joints after the root, in model order.
inline std::vector<std::string> ActuatedJointNames(const pinocchio::Model& model) {
  std::vector<std::string> names;
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(model.njoints); ++j) {
    names.push_back(model.names[j]);
  }
  return names;
}

/// One value per joint, at least `margin` inside its limits (seeded).
inline std::vector<double> RandomInsideLimits(const pinocchio::Model& model, unsigned seed,
                                              double margin) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<double> q;
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(model.njoints); ++j) {
    const Eigen::Index idx = model.joints[j].idx_q();
    const double lo = model.lowerPositionLimit[idx];
    const double hi = model.upperPositionLimit[idx];
    const double m = (hi - lo) > 2.0 * margin ? margin : 0.25 * (hi - lo);
    q.push_back(lo + m + u(rng) * (hi - lo - 2.0 * m));
  }
  return q;
}

/// `reference:` YAML for `model`: q_default from `q_joints` (model order), arm
/// swing on `arm_joints` (name, sign).
inline YAML::Node ReferenceYaml(const pinocchio::Model& model, const std::vector<double>& q_joints,
                                bool arm_enabled,
                                const std::vector<std::pair<std::string, double>>& arm_joints) {
  YAML::Node n;
  n["intermediate_fraction"] = 0.7;
  const std::vector<std::string> names = ActuatedJointNames(model);
  for (std::size_t i = 0; i < names.size(); ++i) {
    n["q_default"][names[i]] = q_joints[i];
  }
  n["arm_swing"]["enabled"] = arm_enabled;
  n["arm_swing"]["gain"] = 0.15;
  n["arm_swing"]["phase_offset"] = 0.15;
  n["arm_swing"]["joints"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& [name, sign] : arm_joints) {
    YAML::Node j;
    j["name"] = name;
    j["sign"] = sign;
    n["arm_swing"]["joints"].push_back(j);
  }
  return n;
}

/// ZYX (yaw, pitch, roll) as a rotation matrix.
inline Eigen::Matrix3d Rzyx(double yaw, double pitch, double roll) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

/// The registered snapshot: base (0.3, −0.2, 0.71), ZYX (1.3, −0.05, 0.1),
/// LOCAL twist (0.2, −0.1, 0.03 | 0.02, −0.01, 0.1), joints `q_joints` (the
/// fingers 0.011 / 0.034 unless `q_joints` says otherwise).
inline MPCStateSnapshot RegisteredSnapshot(const pinocchio::Model& model,
                                           const std::vector<double>& q_joints, double yaw = 1.3,
                                           double pitch = -0.05, double roll = 0.1) {
  MPCStateSnapshot s{};
  s.nq = model.nq;
  s.nv = model.nv;
  s.q[0] = 0.3;
  s.q[1] = -0.2;
  s.q[2] = 0.71;
  const Eigen::Quaterniond q(Rzyx(yaw, pitch, roll));
  s.q[3] = q.x();
  s.q[4] = q.y();
  s.q[5] = q.z();
  s.q[6] = q.w();
  for (std::size_t i = 0; i < q_joints.size(); ++i) {
    s.q[7 + i] = q_joints[i];
  }
  s.v[0] = 0.2;
  s.v[1] = -0.1;
  s.v[2] = 0.03;
  s.v[3] = 0.02;
  s.v[4] = -0.01;
  s.v[5] = 0.1;
  s.sequence = 7;
  s.model_epoch = 3;
  s.valid = true;
  return s;
}

/// Puts `s` (Init done) into the walk-like gait: stance anchored at `t0 − 2`,
/// walk requested at once (a single-mode gait switches at the 0.63 s lead, so
/// the walk runs since `t0 − 1.27`), the window rebuilt at `t0`. The policy
/// stays off unless `run_policy`.
inline void WalkFrom(GaitScheduler& s, double t0, bool run_policy = false) {
  if (!run_policy) {
    s.SetPolicyEnabled(false);
  }
  FilteredCommand cmd;
  cmd.v_x = 0.5;
  cmd.valid = true;
  if (s.Advance(t0 - 2.0, cmd, PlanarVelocity{}) != GaitSchedulerError::kNoError) {
    throw std::runtime_error("WalkFrom: first Advance failed");
  }
  s.RequestGait(1);
  if (s.Advance(t0 - 1.9, cmd, PlanarVelocity{}) != GaitSchedulerError::kNoError ||
      s.Advance(t0, cmd, PlanarVelocity{}) != GaitSchedulerError::kNoError ||
      s.current_gait() != 1U || s.switch_pending()) {
    throw std::runtime_error("WalkFrom: the walk did not take over");
  }
}

inline FilteredCommand RegisteredCommand(bool valid = true) {
  FilteredCommand c;
  c.v_x = 0.5;
  c.v_y = 0.1;
  c.omega_z = 0.3;
  c.base_height = 0.75;
  c.valid = valid;
  c.sequence = 11;
  return c;
}

}  // namespace rtc::mpc::test_utils
