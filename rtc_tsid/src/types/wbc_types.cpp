#include "rtc_tsid/types/wbc_types.hpp"

#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_urdf_bridge/free_flyer_root.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

// #unified-kindyn Phase 1: PinocchioCache::Update 이관으로 algorithm 헤더는 불필요.
// 남은 RobotModelInfo::Build / ContactManagerConfig::Load 는 pinocchio::Model 멤버만 쓴다.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <string>

namespace rtc::tsid {

// ════════════════════════════════════════════════
// RobotModelInfo
// ════════════════════════════════════════════════
void RobotModelInfo::Build(const pinocchio::Model& model, const YAML::Node& config) {
  nq = model.nq;
  nv = model.nv;

  // Floating base ⇔ the root joint is a free-flyer — the joint type, not its
  // nq / nv (a composite 7/6 root is not a free-flyer); the one rule in
  // rtc_urdf_bridge (#1016). Everything after the root is actuated. YAML
  // floating_base / n_actuated are not overrides: when given they must agree
  // with the model (non-RT, throws).
  floating_base = rtc_urdf_bridge::IsFreeFlyerRoot(model);
  n_base_q = floating_base ? rtc_urdf_bridge::kFreeFlyerNq : 0;
  n_base_v = floating_base ? rtc_urdf_bridge::kFreeFlyerNv : 0;
  n_actuated = nv - n_base_v;

  if (config && config["floating_base"] && config["floating_base"].as<bool>() != floating_base) {
    throw std::invalid_argument(
        std::string("RobotModelInfo: YAML floating_base disagrees with the model, whose root "
                    "joint is ") +
        (model.joints.size() > 1 ? model.joints[1].shortname() : std::string("absent")));
  }
  if (config && config["n_actuated"] && config["n_actuated"].as<int>() != n_actuated) {
    throw std::invalid_argument("RobotModelInfo: YAML n_actuated " +
                                std::to_string(config["n_actuated"].as<int>()) +
                                " disagrees with the model's " + std::to_string(n_actuated));
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
    tau_max = model.upperEffortLimit.tail(n_actuated);
    tau_min = -model.upperEffortLimit.tail(n_actuated);
  } else {
    tau_max = model.upperEffortLimit.head(n_actuated);
    tau_min = -model.upperEffortLimit.head(n_actuated);
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
  v_max = model.upperVelocityLimit;
}

// ════════════════════════════════════════════════
// ContactManagerConfig
// ════════════════════════════════════════════════
void ContactManagerConfig::Load(const YAML::Node& config, const pinocchio::Model& model) {
  contacts.clear();
  max_contact_vars = 0;
  friction_key_conflict = false;

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

    // Stage C-0.1: accept `mu` / `n_faces` as aliases for `friction_coeff` /
    // `friction_faces`. The shipped YAMLs use the `mu` / `n_faces` spelling;
    // before this aliasing they silently fell back to the defaults (0.7 / 4)
    // instead of the intended values. The canonical key wins; a disagreeing
    // alias sets friction_key_conflict so the controller can WARN.
    {
      const bool has_fc = static_cast<bool>(c["friction_coeff"]);
      const bool has_mu = static_cast<bool>(c["mu"]);
      if (has_fc) {
        cc.friction_coeff = c["friction_coeff"].as<double>();
        if (has_mu && std::fabs(c["mu"].as<double>() - cc.friction_coeff) > 1e-12) {
          friction_key_conflict = true;
        }
      } else {
        cc.friction_coeff = has_mu ? c["mu"].as<double>() : 0.7;
      }
    }
    {
      const bool has_ff = static_cast<bool>(c["friction_faces"]);
      const bool has_nf = static_cast<bool>(c["n_faces"]);
      if (has_ff) {
        cc.friction_faces = c["friction_faces"].as<int>();
        if (has_nf && c["n_faces"].as<int>() != cc.friction_faces) {
          friction_key_conflict = true;
        }
      } else {
        cc.friction_faces = has_nf ? c["n_faces"].as<int>() : 4;
      }
    }

    // Surface contact 전용 필드 (point 에서는 무시). Default 0 으로 두면
    // surface 라도 CoP/yaw 부등식이 trivial 행으로 남아 ineq_dim 일정.
    cc.patch_half_length_x = c["patch_half_length_x"].as<double>(0.0);
    cc.patch_half_length_y = c["patch_half_length_y"].as<double>(0.0);
    cc.torsional_friction_coeff = c["torsional_friction_coeff"].as<double>(0.0);

    // Normal-force bounds (both contact types). 0 ≤ min ≤ max, min finite.
    cc.normal_force_min = c["normal_force_min"].as<double>(0.0);
    cc.normal_force_max = c["normal_force_max"].as<double>(std::numeric_limits<double>::infinity());
    if (!(std::isfinite(cc.normal_force_min) && cc.normal_force_min >= 0.0 &&
          cc.normal_force_max >= cc.normal_force_min)) {
      throw std::invalid_argument("Contact '" + cc.name +
                                  "': need 0 <= normal_force_min <= normal_force_max");
    }

    // Stage A-4: optional `normal: [x, y, z]` world-frame seed. Default
    // (0, 0, 1) preserves pre-A-4 fixed-normal cone behaviour.
    if (c["normal"] && c["normal"].IsSequence() && c["normal"].size() == 3) {
      Eigen::Vector3d n(c["normal"][0].as<double>(), c["normal"][1].as<double>(),
                        c["normal"][2].as<double>());
      const double n_norm = n.norm();
      if (n_norm > 1e-9) {
        cc.default_normal = n / n_norm;
      }
    }

    if (!model.existFrame(cc.frame_name)) {
      throw std::runtime_error("Contact frame '" + cc.frame_name + "' not found in URDF model");
    }
    cc.frame_id = static_cast<int>(model.getFrameId(cc.frame_name));

    max_contact_vars += cc.contact_dim;
    contacts.push_back(std::move(cc));
  }

  // Stage A-4: optional sibling key for the runtime normal LPF gain.
  if (config["contacts_normal_filter_alpha"]) {
    const double a = config["contacts_normal_filter_alpha"].as<double>();
    if (a > 0.0 && a <= 1.0) {
      normal_filter_alpha = a;
    }
  }

  max_contacts = static_cast<int>(contacts.size());
}

// ════════════════════════════════════════════════
// ContactState
// ════════════════════════════════════════════════
void ContactState::Init(int max_contacts) {
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

void ContactState::RecomputeActive(const ContactManagerConfig& manager) {
  active_count = 0;
  active_contact_vars = 0;
  for (size_t i = 0; i < contacts.size(); ++i) {
    if (contacts[i].active) {
      ++active_count;
      active_contact_vars += manager.contacts[i].contact_dim;
    }
  }
}

void ContactState::SeedNormals(const ContactManagerConfig& manager) noexcept {
  const size_t n = std::min(contacts.size(), manager.contacts.size());
  for (size_t i = 0; i < n; ++i) {
    const auto& dn = manager.contacts[i].default_normal;
    const double dn_norm = dn.norm();
    contacts[i].normal = (dn_norm > 1e-9) ? (dn / dn_norm) : Eigen::Vector3d(0.0, 0.0, 1.0);
  }
}

void ContactState::UpdateNormal(int idx, const Eigen::Vector3d& n_raw, double alpha) noexcept {
  if (idx < 0 || static_cast<size_t>(idx) >= contacts.size()) {
    return;
  }
  const double raw_norm = n_raw.norm();
  if (!(raw_norm > 1e-9) || !std::isfinite(raw_norm)) {
    return;  // ignore degenerate input; keep prior normal
  }
  const double a = std::clamp(alpha, 0.0, 1.0);
  auto& n_prev = contacts[static_cast<size_t>(idx)].normal;
  Eigen::Vector3d blended = a * (n_raw / raw_norm) + (1.0 - a) * n_prev;
  const double bn = blended.norm();
  if (bn > 1e-9) {
    n_prev = blended / bn;
  }
}

void ContactState::SetActivationTarget(int idx, double target, double t_ramp_sec) noexcept {
  if (idx < 0 || static_cast<size_t>(idx) >= contacts.size()) {
    return;
  }
  auto& e = contacts[static_cast<size_t>(idx)];
  e.activation_target = std::clamp(target, 0.0, 1.0);
  // t_ramp_sec=0 is allowed and means "instant" — UpdateActivation snaps
  // to target in a single tick when 1/t_ramp_sec exceeds dt's reciprocal.
  e.t_ramp_sec = (t_ramp_sec > 0.0) ? t_ramp_sec : 0.0;
}

void ContactState::UpdateActivation(double dt) noexcept {
  if (!(dt > 0.0)) {
    return;
  }
  for (auto& e : contacts) {
    const double err = e.activation_target - e.activation;
    // Linear ramp with rate 1/t_ramp_sec. Instant (t_ramp_sec=0) snaps.
    const double step =
        (e.t_ramp_sec > 0.0) ? std::clamp(err, -dt / e.t_ramp_sec, dt / e.t_ramp_sec) : err;
    e.activation = std::clamp(e.activation + step, 0.0, 1.0);
    // Keep legacy `active : bool` aligned with the deadband. Downstream
    // skip paths (active_contact_vars, task `if (!active) continue` early
    // outs) follow this flip.
    e.active = (e.activation >= kActivationDeadband);
  }
}

// ════════════════════════════════════════════════
// ContactFrameIds — ContactManagerConfig → PinocchioCache::Init 리스트
// ════════════════════════════════════════════════
// #unified-kindyn Phase 1: PinocchioCache 는 rtc_urdf_bridge 로 이관됐고 ContactState/
// ContactManagerConfig 결합이 끊겼다 (Init 은 frame id 리스트만 받는다). TSID 소비자는 이 헬퍼로
// manager 의 contact frame id 를 동일 순서(cache.contact_frames[i] ↔ manager.contacts[i])로 넘긴다.
std::vector<pinocchio::FrameIndex> ContactFrameIds(const ContactManagerConfig& manager) {
  std::vector<pinocchio::FrameIndex> ids;
  ids.reserve(manager.contacts.size());
  for (const auto& c : manager.contacts) {
    ids.push_back(static_cast<pinocchio::FrameIndex>(c.frame_id));
  }
  return ids;
}

// ════════════════════════════════════════════════
// ControlReference
// ════════════════════════════════════════════════
void ControlReference::Init(int nq, int nv, int n_actuated, int max_contact_vars) {
  q_des.setZero(nq);
  v_des.setZero(nv);
  a_des.setZero(nv);
  tau_ff.setZero(n_actuated);
  lambda_des.setZero(max_contact_vars);
}

// ════════════════════════════════════════════════
// CommandOutput
// ════════════════════════════════════════════════
void CommandOutput::Init(int nv, int n_actuated, int max_contact_vars) {
  tau.setZero(n_actuated);
  a_opt.setZero(nv);
  lambda_opt.setZero(max_contact_vars);
  ResetDiagnostics();
}

void CommandOutput::ResetDiagnostics() noexcept {
  qp_converged = false;
  solve_time_us = 0.0;
  solve_levels = 0;
  qp_status = -1;
  qp_iterations = 0;
  qp_non_finite = false;
  tau_valid = false;
  n_eq_used = 0;
  n_ineq_used = 0;
  rows_dropped = 0;
  outer_iterations = 0;
}

// ════════════════════════════════════════════════
// PhasePreset loader
// ════════════════════════════════════════════════
std::unordered_map<std::string, PhasePreset> LoadPhasePresets(const YAML::Node& config) {
  std::unordered_map<std::string, PhasePreset> presets;

  if (!config || !config["phase_presets"])
    return presets;

  // yaml-cpp iterator::operator->() returns a proxy holding a prvalue
  // value_type, so binding the result by reference (`const auto& node =
  // it->second`) leaves a dangling handle once the proxy temporary dies at the
  // end of the statement. Always copy yaml-cpp Nodes by value — they are
  // lightweight handles sharing the underlying Memory.
  for (auto it = config["phase_presets"].begin(); it != config["phase_presets"].end(); ++it) {
    PhasePreset preset;
    preset.phase_name = it->first.as<std::string>();
    YAML::Node node = it->second;

    if (node["tasks"]) {
      for (auto t = node["tasks"].begin(); t != node["tasks"].end(); ++t) {
        TaskPreset tp;
        tp.task_name = t->first.as<std::string>();
        YAML::Node tn = t->second;
        tp.active = tn["active"].as<bool>(true);
        tp.weight = tn["weight"].as<double>(1.0);
        tp.priority = tn["priority"].as<int>(0);
        preset.task_presets.push_back(std::move(tp));
      }
    }

    if (node["constraints"]) {
      for (auto c = node["constraints"].begin(); c != node["constraints"].end(); ++c) {
        ConstraintPreset cp;
        cp.constraint_name = c->first.as<std::string>();
        YAML::Node cn = c->second;
        cp.active = cn["active"].as<bool>(true);
        preset.constraint_presets.push_back(std::move(cp));
      }
    }

    presets[preset.phase_name] = std::move(preset);
  }

  return presets;
}

}  // namespace rtc::tsid
