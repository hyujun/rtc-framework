// The W6-F01 PR B rig: the public G1 free-flyer with its sole frames standing
// on both feet through LocomotionFormulationBuilder, and the deterministic
// stance q_stand (designed with pinocchio: hip pitch −0.05, knee 0.1, ankle
// pitch −0.05 on both legs, base z so the soles sit flat at z = 0). The
// whole_body_qp map is the shipped g1 values with the gates' solver caps.
// Test-local, not installed.
#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "g1_fixture.hpp"
#include "rtc_tsid/locomotion/locomotion_formulation.hpp"

#include <yaml-cpp/yaml.h>

#include <initializer_list>
#include <memory>
#include <string>

namespace rtc::tsid::test {

inline constexpr const char* kLocoWbq = R"(
formulation_type: wqp
solver: {eps_abs: 1.0e-6, max_iter: 200, max_iter_in: 500, dense_backend: PrimalDualLDLT}
contact:
  friction_coeff: 0.4
  friction_faces: 4
  patch_half_length_x: 0.09
  patch_half_length_y: 0.03
  torsional_friction_coeff: 0.0
  t_ramp: 0.0
tasks:
  posture: {kp: 50.0, kd: 10.0, weight: 1.0}
  feet: {kp: [400, 400, 400, 400, 400, 400], kd: [40, 40, 40, 40, 40, 40], weight: 100.0}
  torso: {kp: [0, 0, 0, 80, 80, 80], kd: [0, 0, 0, 20, 20, 20], mask: [0, 0, 0, 1, 1, 1], weight: 10.0}
  force: {weight: 1.0e-3}
constraints:
  eom: {}
  contact: {baumgarte: {kp: [0, 0, 100, 80, 80, 80], kd: [20, 20, 10, 20, 20, 20]}}
  friction_cone: {}
  joint_limit: {position_margin: 0.05, velocity_margin: 0.1}
  torque_limit: {tau_scale: 1.0}
)";

inline constexpr double kLocoDt = 0.002;  // [s] the control period

inline LocomotionModelSpec LocoSpec() {
  return {{{"foot_a", "left_sole", 6}, {"foot_b", "right_sole", 6}}, "torso_link"};
}

inline void SetJoint(const pinocchio::Model& m, Eigen::VectorXd& q, const std::string& name,
                     double value) {
  q(m.joints[m.getJointId(name)].idx_q()) = value;
}

inline Eigen::VectorXd QStand(const pinocchio::Model& m) {
  Eigen::VectorXd q = pinocchio::neutral(m);
  for (const char* s : {"left", "right"}) {
    SetJoint(m, q, std::string(s) + "_hip_pitch_joint", -0.05);
    SetJoint(m, q, std::string(s) + "_knee_joint", 0.1);
    SetJoint(m, q, std::string(s) + "_ankle_pitch_joint", -0.05);
  }
  q(2) = 0.7910682772570018;
  return q;
}

struct LocoRig {
  std::shared_ptr<const pinocchio::Model> model = SharedG1Model();
  RobotModelInfo info;
  PinocchioCache cache;
  LocomotionFormulationBuilder b;
  ContactState contacts;
  ControlState state;
  ControlReference ref;

  explicit LocoRig(const YAML::Node& wbq = YAML::Load(kLocoWbq),
                   const LocomotionModelSpec& spec = LocoSpec()) {
    info.Build(*model, YAML::Node{});
    cache.Init(model, LocomotionFormulationBuilder::ContactFrameIds(*model, spec));
    b.Init(*model, info, cache, spec, wbq, kLocoDt);
    contacts = b.MakeContactState();
    ref.Init(info.nq, info.nv, info.n_actuated, b.Contacts().max_contact_vars);
  }

  // State and reference at rest at (q, v): q_des = q, v_des = v, a_des = 0.
  void Set(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    state.q = q;
    state.v = v;
    cache.Update(q, v);
    ref.q_des = q;
    ref.v_des = v;
    ref.a_des.setZero();
  }

  // The weight m g shared evenly by the given (surface) contact slots as λ_des.
  void WeightOn(std::initializer_list<int> slots) {
    const double share =
        pinocchio::computeTotalMass(*model) * 9.81 / static_cast<double>(slots.size());
    ref.lambda_des.setZero();
    for (const int c : slots) {
      ref.lambda_des(6 * c + 2) = share;
    }
  }

  void Release(std::size_t c) {
    contacts.contacts[c].active = false;
    contacts.contacts[c].activation = 0.0;
    contacts.contacts[c].activation_target = 0.0;
    contacts.RecomputeActive(b.Contacts());
  }

  const CommandOutput& Solve() { return b.Compute(state, ref, cache, contacts); }
};

// A cache Init'd for `spec`'s contacts on `model`, not yet locked.
inline void InitUnlockedCache(PinocchioCache& cache,
                              const std::shared_ptr<const pinocchio::Model>& model,
                              const LocomotionModelSpec& spec = LocoSpec()) {
  cache.Init(model, LocomotionFormulationBuilder::ContactFrameIds(*model, spec));
}

}  // namespace rtc::tsid::test
