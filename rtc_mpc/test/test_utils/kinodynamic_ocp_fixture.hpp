#pragma once

/// @file kinodynamic_ocp_fixture.hpp
/// @brief The kinodynamic OCP rig on the public G1: the reference whole-body
///        task's weights in this repo's `mpc.kinodynamic:` schema, the MJCF
///        `stand` keyframe, and hand-made horizons (full stance; one foot
///        swinging over a node range). Not installed — test-local by design.

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace rtc::mpc::test_utils {

inline constexpr int kKinoNodes = 32;     ///< stages
inline constexpr double kKinoDt = 0.035;  ///< [s]

/// The reference task's state, terminal and input weights (its R scaling
/// 1e-3 folded in; the base orientation in tangent order roll, pitch, yaw),
/// the foot weights it applies, and its contact constraint gains, friction,
/// CoP rectangle, joint-limit barrier and the 16 foot / knee sphere pairs
/// (collision off, as shipped). The foot spheres sit in the ankle-roll frame
/// at the sole centre and ±0.6·x_max along x from it.
inline constexpr const char* kG1KinodynamicWeights = R"(
state_weights:
  base_position: [0.0, 0.0, 10.0]
  base_orientation: [20.0, 20.0, 0.0]
  joints:
    left_hip_pitch_joint: 0.05
    left_hip_roll_joint: 0.2
    left_hip_yaw_joint: 0.2
    left_knee_joint: 0.1
    left_ankle_pitch_joint: 0.1
    left_ankle_roll_joint: 0.1
    right_hip_pitch_joint: 0.05
    right_hip_roll_joint: 0.2
    right_hip_yaw_joint: 0.2
    right_knee_joint: 0.1
    right_ankle_pitch_joint: 0.1
    right_ankle_roll_joint: 0.1
    waist_yaw_joint: 2.0
    waist_roll_joint: 2.0
    waist_pitch_joint: 2.0
    left_shoulder_pitch_joint: 0.4
    left_shoulder_roll_joint: 2.0
    left_shoulder_yaw_joint: 0.4
    left_elbow_joint: 0.4
    right_shoulder_pitch_joint: 0.4
    right_shoulder_roll_joint: 2.0
    right_shoulder_yaw_joint: 0.4
    right_elbow_joint: 0.4
  base_linear_velocity: [3.0, 3.0, 3.0]
  base_angular_velocity: [3.0, 3.0, 3.0]
  joint_velocities:
    left_hip_pitch_joint: 0.001
    left_hip_roll_joint: 0.001
    left_hip_yaw_joint: 0.001
    left_knee_joint: 0.001
    left_ankle_pitch_joint: 0.001
    left_ankle_roll_joint: 0.001
    right_hip_pitch_joint: 0.001
    right_hip_roll_joint: 0.001
    right_hip_yaw_joint: 0.001
    right_knee_joint: 0.001
    right_ankle_pitch_joint: 0.001
    right_ankle_roll_joint: 0.001
    waist_yaw_joint: 0.2
    waist_roll_joint: 0.2
    waist_pitch_joint: 0.2
    left_shoulder_pitch_joint: 0.02
    left_shoulder_roll_joint: 0.02
    left_shoulder_yaw_joint: 0.02
    left_elbow_joint: 0.02
    right_shoulder_pitch_joint: 0.02
    right_shoulder_roll_joint: 0.02
    right_shoulder_yaw_joint: 0.02
    right_elbow_joint: 0.02
terminal_weights:
  base_position: [0.0, 0.0, 10.0]
  base_orientation: [20.0, 20.0, 0.0]
  joints:
    left_hip_pitch_joint: 0.05
    left_hip_roll_joint: 0.1
    left_hip_yaw_joint: 0.1
    left_knee_joint: 0.1
    left_ankle_pitch_joint: 0.1
    left_ankle_roll_joint: 0.1
    right_hip_pitch_joint: 0.05
    right_hip_roll_joint: 0.1
    right_hip_yaw_joint: 0.1
    right_knee_joint: 0.1
    right_ankle_pitch_joint: 0.1
    right_ankle_roll_joint: 0.1
    waist_yaw_joint: 2.0
    waist_roll_joint: 2.0
    waist_pitch_joint: 2.0
    left_shoulder_pitch_joint: 0.4
    left_shoulder_roll_joint: 2.0
    left_shoulder_yaw_joint: 0.4
    left_elbow_joint: 0.4
    right_shoulder_pitch_joint: 0.4
    right_shoulder_roll_joint: 2.0
    right_shoulder_yaw_joint: 0.4
    right_elbow_joint: 0.4
  base_linear_velocity: [3.0, 3.0, 3.0]
  base_angular_velocity: [3.0, 3.0, 3.0]
  joint_velocities:
    left_hip_pitch_joint: 0.001
    left_hip_roll_joint: 0.001
    left_hip_yaw_joint: 0.001
    left_knee_joint: 0.001
    left_ankle_pitch_joint: 0.001
    left_ankle_roll_joint: 0.001
    right_hip_pitch_joint: 0.001
    right_hip_roll_joint: 0.001
    right_hip_yaw_joint: 0.001
    right_knee_joint: 0.001
    right_ankle_pitch_joint: 0.001
    right_ankle_roll_joint: 0.001
    waist_yaw_joint: 0.2
    waist_roll_joint: 0.2
    waist_pitch_joint: 0.2
    left_shoulder_pitch_joint: 0.02
    left_shoulder_roll_joint: 0.02
    left_shoulder_yaw_joint: 0.02
    left_elbow_joint: 0.02
    right_shoulder_pitch_joint: 0.02
    right_shoulder_roll_joint: 0.02
    right_shoulder_yaw_joint: 0.02
    right_elbow_joint: 0.02
input_weights:
  contact_force: [3.0e-6, 3.0e-6, 1.0e-6]
  contact_moment: [1.0e-5, 1.0e-5, 1.0e-4]
  joint_accelerations:
    left_hip_pitch_joint: 5.0e-6
    left_hip_roll_joint: 5.0e-6
    left_hip_yaw_joint: 5.0e-6
    left_knee_joint: 5.0e-6
    left_ankle_pitch_joint: 5.0e-6
    left_ankle_roll_joint: 5.0e-6
    right_hip_pitch_joint: 5.0e-6
    right_hip_roll_joint: 5.0e-6
    right_hip_yaw_joint: 5.0e-6
    right_knee_joint: 5.0e-6
    right_ankle_pitch_joint: 5.0e-6
    right_ankle_roll_joint: 5.0e-6
    waist_yaw_joint: 5.0e-6
    waist_roll_joint: 5.0e-6
    waist_pitch_joint: 5.0e-6
    left_shoulder_pitch_joint: 5.0e-6
    left_shoulder_roll_joint: 5.0e-6
    left_shoulder_yaw_joint: 5.0e-6
    left_elbow_joint: 5.0e-6
    right_shoulder_pitch_joint: 5.0e-6
    right_shoulder_roll_joint: 5.0e-6
    right_shoulder_yaw_joint: 5.0e-6
    right_elbow_joint: 5.0e-6
contact_tracking:
  position: [0.0, 0.0, 0.0]
  orientation: [10000.0, 10000.0, 0.0]
  linear_velocity: [5.0, 5.0, 0.0]
  angular_velocity: [2.0, 2.0, 2.0]
  linear_acceleration: [0.01, 0.01, 0.01]
  angular_acceleration: [0.01, 0.01, 0.01]
constraints:
  foot_constraint:
    position_z: 100.0
    orientation: 80.0
    linear_velocity_xy: 20.0
    linear_velocity_z: 10.0
    angular_velocity: 20.0
    linear_acceleration_xy: 1.0
    linear_acceleration_z: 1.0
    angular_acceleration: 1.0
  friction: {mu: 0.4, grip_force: 0.0, regularization: 25.0, barrier_mu: 0.2, barrier_delta: 5.0}
  contact_rectangle: {x_min: -0.09, x_max: 0.09, y_min: -0.03, y_max: 0.03, barrier_mu: 0.6,
                      barrier_delta: 0.03}
  joint_limits: {mu: 1200.0, delta: 0.1}
  collision:
    enabled: false
    mu: 1500.0
    delta: 0.04
    pairs:
    - {frame_a: left_ankle_roll_joint, offset_a: [0.089, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [0.089, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [0.089, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [-0.019, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [-0.019, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [0.089, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [-0.019, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [-0.019, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [0.035, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [0.089, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [0.035, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [-0.019, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: right_ankle_roll_joint, offset_a: [0.035, 0.0, -0.035], radius_a: 0.05,
       frame_b: left_ankle_roll_joint, offset_b: [0.089, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: right_ankle_roll_joint, offset_a: [0.035, 0.0, -0.035], radius_a: 0.05,
       frame_b: left_ankle_roll_joint, offset_b: [-0.019, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [0.035, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [0.035, 0.0, -0.035], radius_b: 0.05}
    - {frame_a: left_knee_joint, offset_a: [0.0, 0.0, 0.0], radius_a: 0.075,
       frame_b: right_knee_joint, offset_b: [0.0, 0.0, 0.0], radius_b: 0.075}
    - {frame_a: left_ankle_roll_joint, offset_a: [0.035, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [0.0, 0.0, 0.0], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [0.089, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [0.0, 0.0, 0.0], radius_b: 0.05}
    - {frame_a: left_ankle_roll_joint, offset_a: [-0.019, 0.0, -0.035], radius_a: 0.05,
       frame_b: right_ankle_roll_joint, offset_b: [0.0, 0.0, 0.0], radius_b: 0.05}
    - {frame_a: right_ankle_roll_joint, offset_a: [0.035, 0.0, -0.035], radius_a: 0.05,
       frame_b: left_ankle_roll_joint, offset_b: [0.0, 0.0, 0.0], radius_b: 0.05}
    - {frame_a: right_ankle_roll_joint, offset_a: [0.089, 0.0, -0.035], radius_a: 0.05,
       frame_b: left_ankle_roll_joint, offset_b: [0.0, 0.0, 0.0], radius_b: 0.05}
    - {frame_a: right_ankle_roll_joint, offset_a: [-0.019, 0.0, -0.035], radius_a: 0.05,
       frame_b: left_ankle_roll_joint, offset_b: [0.0, 0.0, 0.0], radius_b: 0.05}
)";

/// `mpc.kinodynamic:` for the G1 (32 nodes, dt 0.035, terminal scaling 4).
inline YAML::Node G1KinodynamicNode(const char* integrator = "semi_implicit_euler") {
  YAML::Node n = YAML::Load(kG1KinodynamicWeights);
  n["horizon_nodes"] = kKinoNodes;
  n["dt"] = kKinoDt;
  n["integrator"] = integrator;
  n["terminal_scaling"] = 4.0;
  return n;
}

/// The MJCF `stand` keyframe: base at 0.764431 m, level; hips −0.3, knees
/// 0.6, ankles −0.3 (pitch); everything else 0.
inline Eigen::VectorXd G1Stand(const pinocchio::Model& model) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  q[2] = 0.764431;
  for (const char* side : {"left", "right"}) {
    const std::string s(side);
    q[model.joints[model.getJointId(s + "_hip_pitch_joint")].idx_q()] = -0.3;
    q[model.joints[model.getJointId(s + "_knee_joint")].idx_q()] = 0.6;
    q[model.joints[model.getJointId(s + "_ankle_pitch_joint")].idx_q()] = -0.3;
  }
  return q;
}

/// A horizon of `kKinoNodes + 1` nodes at `q` (at rest): every node in full
/// stance, the contacts where forward kinematics puts them, each stance
/// contact carrying `m·|g| / n_stance`, impact proximity 1, flat ground at 0.
inline std::unique_ptr<HorizonReference> StanceHorizon(const RobotModelHandler& handler,
                                                       const Eigen::VectorXd& q) {
  const pinocchio::Model& model = handler.model();
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, q);
  auto h = std::make_unique<HorizonReference>();
  const int nc = handler.n_contacts();
  h->n_stages = kKinoNodes + 1;
  h->n_contacts = static_cast<std::uint32_t>(nc);
  h->nq = static_cast<std::uint32_t>(model.nq);
  h->nv = static_cast<std::uint32_t>(model.nv);
  h->n_contact_vars = static_cast<std::uint32_t>(handler.layout().n_contact_vars);
  h->dt = kKinoDt;
  h->terrain = SurfaceFrame{};
  const std::uint32_t all = (1U << static_cast<unsigned>(nc)) - 1U;
  for (int k = 0; k <= kKinoNodes; ++k) {
    StageReference& s = h->stages[static_cast<std::size_t>(k)];
    s.time = k * kKinoDt;
    for (int i = 0; i < model.nq; ++i) {
      s.q[static_cast<std::size_t>(i)] = q[i];
    }
    for (int c = 0; c < nc; ++c) {
      const auto i = static_cast<std::size_t>(c);
      const pinocchio::SE3& m =
          data.oMf[static_cast<std::size_t>(handler.contact_frames()[i].frame_id)];
      const Eigen::Quaterniond quat(m.rotation());
      for (std::size_t j = 0; j < 3; ++j) {
        s.contact_pose[7 * i + j] = m.translation()[static_cast<Eigen::Index>(j)];
      }
      s.contact_pose[7 * i + 3] = quat.x();
      s.contact_pose[7 * i + 4] = quat.y();
      s.contact_pose[7 * i + 5] = quat.z();
      s.contact_pose[7 * i + 6] = quat.w();
      s.impact_proximity[i] = 1.0;
    }
    s.mode = all;
    s.n_stance = static_cast<std::uint32_t>(nc);
  }
  const double mg = pinocchio::computeTotalMass(model) * model.gravity.linear().norm();
  for (int k = 0; k <= kKinoNodes; ++k) {
    StageReference& s = h->stages[static_cast<std::size_t>(k)];
    for (int c = 0; c < nc; ++c) {
      s.wrench[static_cast<std::size_t>(
          handler.layout().contact_u_offset[static_cast<std::size_t>(c)] + 2)] = mg / nc;
    }
  }
  return h;
}

/// Contact `swing` lifted off over nodes `[first, last]`: its bit cleared,
/// its wrench 0 and the others' `m·|g| / n_stance`, its height reference a
/// half sine of `height` (with ż, z̈), its impact proximity `impact`.
inline void SwingContact(const RobotModelHandler& handler, HorizonReference& h, int swing,
                         int first, int last, double height, double impact) {
  const pinocchio::Model& model = handler.model();
  const double mg = pinocchio::computeTotalMass(model) * model.gravity.linear().norm();
  const int nc = handler.n_contacts();
  const double T = (last - first + 2) * h.dt;
  const double w = std::acos(-1.0) / T;
  const auto i = static_cast<std::size_t>(swing);
  for (int k = first; k <= last; ++k) {
    StageReference& s = h.stages[static_cast<std::size_t>(k)];
    const double t = (k - first + 1) * h.dt;
    const double z0 = h.stages[0].contact_pose[7 * i + 2];
    s.mode &= ~(1U << static_cast<unsigned>(swing));
    s.n_stance = 0;
    for (int c = 0; c < nc; ++c) {
      s.n_stance += (s.mode >> static_cast<unsigned>(c)) & 1U;
    }
    for (int c = 0; c < nc; ++c) {
      const auto off =
          static_cast<std::size_t>(handler.layout().contact_u_offset[static_cast<std::size_t>(c)]);
      const bool stance = ((s.mode >> static_cast<unsigned>(c)) & 1U) != 0U;
      s.wrench[off + 2] = stance ? mg / s.n_stance : 0.0;
    }
    s.contact_pose[7 * i + 2] = z0 + height * std::sin(w * t);
    s.contact_twist[6 * i + 2] = height * w * std::cos(w * t);
    s.contact_acceleration[6 * i + 2] = -height * w * w * std::sin(w * t);
    s.impact_proximity[i] = impact;
  }
}

/// `q` at rest as a valid snapshot.
inline MPCStateSnapshot SnapshotAt(const pinocchio::Model& model, const Eigen::VectorXd& q) {
  MPCStateSnapshot s{};
  s.nq = model.nq;
  s.nv = model.nv;
  for (int i = 0; i < model.nq; ++i) {
    s.q[static_cast<std::size_t>(i)] = q[i];
  }
  s.valid = true;
  return s;
}

}  // namespace rtc::mpc::test_utils
