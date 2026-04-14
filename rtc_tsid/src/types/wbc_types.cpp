#include "rtc_tsid/types/wbc_types.hpp"
#include "rtc_tsid/types/qp_types.hpp"

#include <stdexcept>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

namespace rtc::tsid {

// ════════════════════════════════════════════════
// RobotModelInfo
// ════════════════════════════════════════════════
void RobotModelInfo::build(const pinocchio::Model& model,
                           const YAML::Node& config) {
  nq = model.nq;
  nv = model.nv;

  // Floating base 판별: YAML 우선, 없으면 모델로부터 추론
  if (config && config["floating_base"]) {
    floating_base = config["floating_base"].as<bool>();
  } else {
    // Pinocchio: floating-base 모델의 첫 joint가 freeflyer (nq=7, nv=6)
    floating_base = (model.joints.size() > 1 &&
                     model.joints[1].nq() == 7 &&
                     model.joints[1].nv() == 6);
  }

  if (floating_base) {
    n_actuated = nv - 6;
  } else {
    n_actuated = nv;
  }

  if (config && config["n_actuated"]) {
    n_actuated = config["n_actuated"].as<int>();
  }

  // Selection matrix S: [n_actuated × nv]
  S.setZero(n_actuated, nv);
  if (floating_base) {
    S.rightCols(n_actuated).setIdentity();
  } else {
    S.setIdentity();
  }

  // Torque limits
  tau_max.resize(n_actuated);
  tau_min.resize(n_actuated);
  if (floating_base) {
    tau_max = model.effortLimit.tail(n_actuated);
    tau_min = -model.effortLimit.tail(n_actuated);
  } else {
    tau_max = model.effortLimit.head(n_actuated);
    tau_min = -model.effortLimit.head(n_actuated);
  }

  if (config && config["tau_max"]) {
    const auto& vec = config["tau_max"];
    for (int i = 0; i < n_actuated && i < static_cast<int>(vec.size()); ++i) {
      tau_max(i) = vec[static_cast<size_t>(i)].as<double>();
    }
  }
  if (config && config["tau_min"]) {
    const auto& vec = config["tau_min"];
    for (int i = 0; i < n_actuated && i < static_cast<int>(vec.size()); ++i) {
      tau_min(i) = vec[static_cast<size_t>(i)].as<double>();
    }
  }

  q_upper = model.upperPositionLimit;
  q_lower = model.lowerPositionLimit;
  v_max = model.velocityLimit;
}

// ════════════════════════════════════════════════
// ContactManagerConfig
// ════════════════════════════════════════════════
void ContactManagerConfig::load(const YAML::Node& config,
                                const pinocchio::Model& model) {
  contacts.clear();
  max_contact_vars = 0;

  if (!config || !config["contacts"]) {
    max_contacts = 0;
    return;
  }

  for (const auto& c : config["contacts"]) {
    ContactConfig cc;
    cc.name = c["name"].as<std::string>();
    cc.frame_name = c["frame"].as<std::string>();

    const auto type_str = c["type"].as<std::string>("point");
    cc.contact_dim = (type_str == "surface") ? 6 : 3;

    cc.friction_coeff = c["friction_coeff"].as<double>(0.7);
    cc.friction_faces = c["friction_faces"].as<int>(4);

    if (!model.existFrame(cc.frame_name)) {
      throw std::runtime_error(
          "Contact frame '" + cc.frame_name + "' not found in URDF model");
    }
    cc.frame_id = static_cast<int>(model.getFrameId(cc.frame_name));

    max_contact_vars += cc.contact_dim;
    contacts.push_back(std::move(cc));
  }

  max_contacts = static_cast<int>(contacts.size());
}

// ════════════════════════════════════════════════
// ContactState
// ════════════════════════════════════════════════
void ContactState::init(int max_contacts) {
  contacts.resize(static_cast<size_t>(max_contacts));
  for (int i = 0; i < max_contacts; ++i) {
    auto& e = contacts[static_cast<size_t>(i)];
    e.config_index = i;
    e.active = false;
    e.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  }
  active_count = 0;
  active_contact_vars = 0;
}

void ContactState::recompute_active(const ContactManagerConfig& manager) {
  active_count = 0;
  active_contact_vars = 0;
  for (size_t i = 0; i < contacts.size(); ++i) {
    if (contacts[i].active) {
      ++active_count;
      active_contact_vars += manager.contacts[i].contact_dim;
    }
  }
}

// ════════════════════════════════════════════════
// PinocchioCache
// ════════════════════════════════════════════════
void PinocchioCache::init(std::shared_ptr<const pinocchio::Model> model,
                          const ContactManagerConfig& contact_cfg) {
  model_ptr = std::move(model);
  const auto& mdl = *model_ptr;
  data = pinocchio::Data(mdl);

  const int nv = mdl.nv;
  const int nq = mdl.nq;

  M.setZero(nv, nv);
  h.setZero(nv);
  g.setZero(nv);
  q.setZero(nq);
  v.setZero(nv);

  contact_frames.resize(static_cast<size_t>(contact_cfg.max_contacts));
  for (int i = 0; i < contact_cfg.max_contacts; ++i) {
    auto& fc = contact_frames[static_cast<size_t>(i)];
    fc.frame_id = static_cast<pinocchio::FrameIndex>(
        contact_cfg.contacts[static_cast<size_t>(i)].frame_id);
    fc.J.setZero(6, nv);
    fc.dJv.setZero();
  }

  com_position.setZero();
  Jcom.setZero(3, nv);
  com_drift.setZero();
  h_centroidal.setZero();
  Ag.setZero(6, nv);
  hg_drift.setZero();

  registration_locked = false;
}

int PinocchioCache::register_frame(const std::string& name,
                                   pinocchio::FrameIndex frame_id) {
  if (registration_locked) {
    return -1;
  }

  // 중복 검사
  for (size_t i = 0; i < registered_frames.size(); ++i) {
    if (registered_frames[i].frame_id == frame_id) {
      return static_cast<int>(i);
    }
  }

  const int nv = model_ptr->nv;
  RegisteredFrame rf;
  rf.name = name;
  rf.frame_id = frame_id;
  rf.J.setZero(6, nv);
  rf.dJv.setZero();
  registered_frames.push_back(std::move(rf));

  return static_cast<int>(registered_frames.size()) - 1;
}

void PinocchioCache::update(const Eigen::VectorXd& q_in,
                            const Eigen::VectorXd& v_in,
                            const ContactState& contacts_state) noexcept {
  registration_locked = true;

  const auto& mdl = *model_ptr;
  q = q_in;
  v = v_in;

  // FK + CRBA(M) + NLE(h) + Jacobians + frame placements
  pinocchio::computeAllTerms(mdl, data, q, v);

  // Mass matrix (symmetrize)
  M = data.M;
  M.triangularView<Eigen::StrictlyLower>() =
      M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Nonlinear effects
  h = data.nle;

  // Gravity only
  pinocchio::computeGeneralizedGravity(mdl, data, q);
  g = data.g;

  // Contact frame Jacobian + dJv
  for (size_t i = 0; i < contacts_state.contacts.size(); ++i) {
    if (!contacts_state.contacts[i].active) continue;
    if (i >= contact_frames.size()) continue;

    auto& fc = contact_frames[i];
    fc.J.setZero();
    pinocchio::getFrameJacobian(mdl, data, fc.frame_id,
                                pinocchio::LOCAL_WORLD_ALIGNED, fc.J);
    fc.oMf = data.oMf[fc.frame_id];

    fc.dJv = pinocchio::getFrameClassicalAcceleration(
                 mdl, data, fc.frame_id, pinocchio::LOCAL_WORLD_ALIGNED)
                 .toVector();
  }

  // Registered frame Jacobian + dJv
  for (auto& rf : registered_frames) {
    rf.J.setZero();
    pinocchio::getFrameJacobian(mdl, data, rf.frame_id,
                                pinocchio::LOCAL_WORLD_ALIGNED, rf.J);
    rf.oMf = data.oMf[rf.frame_id];
    rf.dJv = pinocchio::getFrameClassicalAcceleration(
                 mdl, data, rf.frame_id, pinocchio::LOCAL_WORLD_ALIGNED)
                 .toVector();
  }

  // CoM (optional)
  if (compute_com) {
    // CoM 위치, 속도, Jacobian 계산
    pinocchio::centerOfMass(mdl, data, q, v, false);
    com_position = data.com[0];
    pinocchio::jacobianCenterOfMass(mdl, data, q, false);
    Jcom = data.Jcom;

    // CoM drift: zero acceleration에서의 CoM 가속도 = dJ_com·v
    // centerOfMass(model, data, q, v, a=0) → data.acom[0]
    const auto zero_a = Eigen::VectorXd::Zero(mdl.nv);
    pinocchio::centerOfMass(mdl, data, q, v, zero_a);
    com_drift = data.acom[0];
  }

  // Centroidal momentum (optional)
  if (compute_centroidal) {
    pinocchio::computeCentroidalMomentum(mdl, data, q, v);
    h_centroidal = data.hg.toVector();
    pinocchio::computeCentroidalMap(mdl, data, q);
    Ag = data.Ag;

    // Centroidal momentum drift: dAg·v (momentum rate at zero acceleration)
    pinocchio::computeCentroidalMomentumTimeVariation(mdl, data, q, v,
        Eigen::VectorXd::Zero(mdl.nv));
    hg_drift = data.dhg.toVector();
  }
}

// ════════════════════════════════════════════════
// ControlReference
// ════════════════════════════════════════════════
void ControlReference::init(int nq, int nv, int n_actuated,
                            int max_contact_vars) {
  q_des.setZero(nq);
  v_des.setZero(nv);
  a_des.setZero(nv);
  tau_ff.setZero(n_actuated);
  lambda_des.setZero(max_contact_vars);
}

// ════════════════════════════════════════════════
// CommandOutput
// ════════════════════════════════════════════════
void CommandOutput::init(int nv, int n_actuated, int max_contact_vars) {
  tau.setZero(n_actuated);
  a_opt.setZero(nv);
  lambda_opt.setZero(max_contact_vars);
}

// ════════════════════════════════════════════════
// PhasePreset loader
// ════════════════════════════════════════════════
std::unordered_map<std::string, PhasePreset> load_phase_presets(
    const YAML::Node& config) {
  std::unordered_map<std::string, PhasePreset> presets;

  if (!config || !config["phase_presets"]) return presets;

  for (auto it = config["phase_presets"].begin();
       it != config["phase_presets"].end(); ++it) {
    PhasePreset preset;
    preset.phase_name = it->first.as<std::string>();
    const auto& node = it->second;

    if (node["tasks"]) {
      for (auto t = node["tasks"].begin(); t != node["tasks"].end(); ++t) {
        TaskPreset tp;
        tp.task_name = t->first.as<std::string>();
        const auto& tn = t->second;
        tp.active = tn["active"].as<bool>(true);
        tp.weight = tn["weight"].as<double>(1.0);
        tp.priority = tn["priority"].as<int>(0);
        preset.task_presets.push_back(std::move(tp));
      }
    }

    if (node["constraints"]) {
      for (auto c = node["constraints"].begin();
           c != node["constraints"].end(); ++c) {
        ConstraintPreset cp;
        cp.constraint_name = c->first.as<std::string>();
        cp.active = c->second["active"].as<bool>(true);
        preset.constraint_presets.push_back(std::move(cp));
      }
    }

    presets[preset.phase_name] = std::move(preset);
  }

  return presets;
}

}  // namespace rtc::tsid
