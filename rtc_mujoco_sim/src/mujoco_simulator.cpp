// ── mujoco_simulator.cpp
// ─────────────────────────────────────────────────────── Lifecycle (ctor/dtor,
// Initialize, Start, Stop) and command/state I/O. Multi-group support:
// robot_response (MuJoCo physics) + fake_response (LPF).
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include "rtc_mujoco_sim/ros2_resource_provider.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <set>

namespace rtc {

// ── Pure parse helpers
// ─────────────────────────────────────────────────────────

int MuJoCoSimulator::SolverNameToEnum(std::string_view name) noexcept {
  if (name == "PGS")
    return mjSOL_PGS;
  if (name == "CG")
    return mjSOL_CG;
  if (name == "Newton")
    return mjSOL_NEWTON;
  return mjSOL_NEWTON;
}

int MuJoCoSimulator::ConeNameToEnum(std::string_view name) noexcept {
  if (name == "elliptic")
    return mjCONE_ELLIPTIC;
  if (name == "pyramidal")
    return mjCONE_PYRAMIDAL;
  return mjCONE_PYRAMIDAL;
}

int MuJoCoSimulator::JacobianNameToEnum(std::string_view name) noexcept {
  if (name == "dense")
    return mjJAC_DENSE;
  if (name == "sparse")
    return mjJAC_SPARSE;
  if (name == "auto")
    return mjJAC_AUTO;
  return mjJAC_AUTO;
}

int MuJoCoSimulator::IntegratorNameToEnum(std::string_view name) noexcept {
  if (name == "Euler")
    return mjINT_EULER;
  if (name == "RK4")
    return mjINT_RK4;
  if (name == "implicit")
    return mjINT_IMPLICIT;
  if (name == "implicitfast")
    return mjINT_IMPLICITFAST;
  return mjINT_EULER;
}

void MuJoCoSimulator::ApplyFakeLpfStep(std::vector<double>& state,
                                       const std::vector<double>& target, double alpha) noexcept {
  const auto n = std::min(state.size(), target.size());
  for (std::size_t i = 0; i < n; ++i) {
    const double t = target[i];
    if (std::isnan(t))
      continue;
    state[i] += alpha * (t - state[i]);
  }
}

// ── Constructor / Destructor
// ───────────────────────────────────────────────────

MuJoCoSimulator::MuJoCoSimulator(Config cfg) noexcept : cfg_(std::move(cfg)) {
  current_max_rtf_.store(cfg_.max_rtf, std::memory_order_relaxed);
  mjv_defaultPerturb(&shared_pert_);
}

MuJoCoSimulator::~MuJoCoSimulator() {
  Stop();
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

// ── XML joint discovery
// ────────────────────────────────────────────────────────

bool MuJoCoSimulator::DiscoverAllXmlJoints() noexcept {
  all_xml_joint_names_.clear();

  for (int j = 0; j < model_->njnt; ++j) {
    if (model_->jnt_type[j] != mjJNT_HINGE) {
      continue;
    }

    bool has_actuator = false;
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == j) {
        has_actuator = true;
        break;
      }
    }
    if (!has_actuator) {
      continue;
    }

    const char* name = mj_id2name(model_, mjOBJ_JOINT, j);
    all_xml_joint_names_.emplace_back(name ? name : "(unnamed)");
  }

  if (all_xml_joint_names_.empty()) {
    fprintf(stderr,
            "[MuJoCoSimulator] ERROR: No hinge joints with actuators "
            "found in XML\n");
    for (int j = 0; j < model_->njnt; ++j) {
      const char* name = mj_id2name(model_, mjOBJ_JOINT, j);
      fprintf(stderr, "  [%d] type=%d  %s\n", j, model_->jnt_type[j], name ? name : "(unnamed)");
    }
    return false;
  }

  std::string names_str;
  for (std::size_t i = 0; i < all_xml_joint_names_.size(); ++i) {
    if (i > 0)
      names_str += ", ";
    names_str += all_xml_joint_names_[i];
  }
  fprintf(stdout, "[MuJoCoSimulator] XML joints (%zu): [%s]\n", all_xml_joint_names_.size(),
          names_str.c_str());

  return true;
}

// ── Name-based index mapping for a single group
// ────────────────────────────────

bool MuJoCoSimulator::MapGroupIndices(JointGroup& group) noexcept {
  group.qpos_indices.clear();
  group.qvel_indices.clear();
  group.actuator_indices.clear();
  group.body_indices.clear();

  std::set<int> body_set;  // dedupe; 같은 body 에 여러 joint 가 매달릴 수 있음

  for (std::size_t i = 0; i < group.command_joint_names.size(); ++i) {
    const auto& jname = group.command_joint_names[i];
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, jname.c_str());
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' — command joint '%s' not "
              "found in XML\n",
              group.name.c_str(), jname.c_str());
      return false;
    }

    // Command/feedforward path assumes 1-dof joints: a single scalar value and
    // a single qfrc_applied[dofadr] slot per joint. Ball(3)/free(6) joints span
    // multiple dofs and would be silently under-driven — reject at mapping.
    const int jtype = model_->jnt_type[jnt_id];
    if (jtype != mjJNT_HINGE && jtype != mjJNT_SLIDE) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' — command joint '%s' must be "
              "hinge or slide (1-dof); ball/free joints are unsupported\n",
              group.name.c_str(), jname.c_str());
      return false;
    }

    group.qpos_indices.push_back(model_->jnt_qposadr[jnt_id]);
    group.qvel_indices.push_back(model_->jnt_dofadr[jnt_id]);
    body_set.insert(model_->jnt_bodyid[jnt_id]);

    bool found_actuator = false;
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == jnt_id) {
        group.actuator_indices.push_back(a);
        found_actuator = true;

        const char* act_name = mj_id2name(model_, mjOBJ_ACTUATOR, a);
        fprintf(stdout,
                "[MuJoCoSimulator] [%s] cmd '%s' → qpos[%d]  qvel[%d]  "
                "actuator[%d] '%s'\n",
                group.name.c_str(), jname.c_str(), group.qpos_indices.back(),
                group.qvel_indices.back(), a, act_name ? act_name : "(unnamed)");
        break;
      }
    }
    if (!found_actuator) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' — command joint '%s' has no "
              "actuator\n",
              group.name.c_str(), jname.c_str());
      return false;
    }
  }

  // Joint-bearing body들만으로는 그 children인 fixed link (joint 없는 강체)가
  // 누락되어 gravcomp이 일부 mass에만 적용된다. arm-leaf 아래에 joint-free
  // mount/wrist/tool 체인이 매달려 있으면 그 체인 mass의 중력이 leaf joint에
  // 외부 토크로 작용해 PD 정상상태 처짐을 만든다. seed body에서 child 방향으로
  // 탐색해, body_jntnum==0 (= fixed link) 인 descendant를 모두 그룹에 포함.
  // joint를 가진 body에서 stop — 다른 group joint면 그 group이 소유하고,
  // 같은 group joint면 이미 seed에 들어있다.
  std::set<int> extended_set = body_set;
  bool changed = true;
  while (changed) {
    changed = false;
    for (int b = 1; b < model_->nbody; ++b) {
      if (extended_set.count(b))
        continue;
      if (model_->body_jntnum[b] != 0)
        continue;
      const int parent = model_->body_parentid[b];
      if (extended_set.count(parent)) {
        extended_set.insert(b);
        changed = true;
      }
    }
  }
  group.body_indices.assign(extended_set.begin(), extended_set.end());
  fprintf(stdout,
          "[MuJoCoSimulator] [%s] gravcomp body chain: %zu body(ies) "
          "(seed %zu + fixed-link descendants %zu)\n",
          group.name.c_str(), group.body_indices.size(), body_set.size(),
          extended_set.size() - body_set.size());
  return true;
}

// ── State joint index mapping
// ─────────────────────────────────────────────────

bool MuJoCoSimulator::MapStateIndices(JointGroup& group) noexcept {
  group.state_qpos_indices.clear();
  group.state_qvel_indices.clear();

  for (std::size_t i = 0; i < group.state_joint_names.size(); ++i) {
    const auto& jname = group.state_joint_names[i];
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, jname.c_str());
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' — state joint '%s' not "
              "found in XML\n",
              group.name.c_str(), jname.c_str());
      return false;
    }

    group.state_qpos_indices.push_back(model_->jnt_qposadr[jnt_id]);
    group.state_qvel_indices.push_back(model_->jnt_dofadr[jnt_id]);

    fprintf(stdout, "[MuJoCoSimulator] [%s] state '%s' → qpos[%d]  qvel[%d]\n", group.name.c_str(),
            jname.c_str(), group.state_qpos_indices.back(), group.state_qvel_indices.back());
  }
  return true;
}

// ── Validate robot groups against XML (양방향 완전 일치)
// ────────────────────────

bool MuJoCoSimulator::ValidateAndMapRobotGroups() noexcept {
  // Collect all command joint names from robot groups
  std::set<std::string> cmd_joints;
  for (const auto& g : groups_) {
    if (!g->is_robot)
      continue;
    for (const auto& jn : g->command_joint_names) {
      if (!cmd_joints.insert(jn).second) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: duplicate command joint '%s' across "
                "robot groups\n",
                jn.c_str());
        return false;
      }
    }
  }

  // Build XML joint set
  std::set<std::string> xml_joints(all_xml_joint_names_.begin(), all_xml_joint_names_.end());

  // Check YAML → XML (command에 있는데 XML에 없음)
  bool ok = true;
  for (const auto& jn : cmd_joints) {
    if (xml_joints.find(jn) == xml_joints.end()) {
      fprintf(stderr, "[MuJoCoSimulator] ERROR: command joint '%s' not found in XML\n", jn.c_str());
      ok = false;
    }
  }

  // Check XML → YAML (XML에 있는데 command에 없음)
  for (const auto& jn : xml_joints) {
    if (cmd_joints.find(jn) == cmd_joints.end()) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: XML joint '%s' not covered by any "
              "robot_response command_joint_names\n",
              jn.c_str());
      ok = false;
    }
  }

  if (!ok) {
    fprintf(stderr, "[MuJoCoSimulator] command joints (%zu):", cmd_joints.size());
    for (const auto& jn : cmd_joints)
      fprintf(stderr, " %s", jn.c_str());
    fprintf(stderr, "\n[MuJoCoSimulator] XML joints (%zu):", xml_joints.size());
    for (const auto& jn : xml_joints)
      fprintf(stderr, " %s", jn.c_str());
    fprintf(stderr, "\n");
    return false;
  }

  fprintf(stdout,
          "[MuJoCoSimulator] Command joints validated: %zu command == %zu XML "
          "joints\n",
          cmd_joints.size(), xml_joints.size());

  // Map command indices for each robot group
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (!MapGroupIndices(*g))
      return false;
  }

  return true;
}

// ── Validate and map state joints ────────────────────────────────────────────

bool MuJoCoSimulator::ValidateAndMapStateJoints() noexcept {
  std::set<std::string> xml_joints(all_xml_joint_names_.begin(), all_xml_joint_names_.end());

  // Build the union of state_joint_names across all robot groups so we can
  // warn about XML joints that no group publishes.  Per-group warnings would
  // misfire when joints are partitioned between groups (e.g. arm vs hand).
  std::set<std::string> all_state_set;
  for (const auto& g : groups_) {
    if (!g->is_robot)
      continue;
    all_state_set.insert(g->state_joint_names.begin(), g->state_joint_names.end());
  }

  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;

    // Check all state_joint_names exist in XML
    bool ok = true;
    for (const auto& jn : g->state_joint_names) {
      if (xml_joints.find(jn) == xml_joints.end()) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: group '%s' — state joint '%s' not "
                "found in XML\n",
                g->name.c_str(), jn.c_str());
        ok = false;
      }
    }
    if (!ok)
      return false;

    // Map state indices
    if (!MapStateIndices(*g))
      return false;

    fprintf(stdout, "[MuJoCoSimulator] [%s] state joints: %d (command: %d)\n", g->name.c_str(),
            g->num_state_joints, g->num_command_joints);
  }

  // Warn about XML joints that no group publishes (global coverage check).
  for (const auto& jn : all_xml_joint_names_) {
    if (all_state_set.find(jn) == all_state_set.end()) {
      fprintf(stdout,
              "[MuJoCoSimulator] WARNING: XML joint '%s' not published by any "
              "group's state_joint_names\n",
              jn.c_str());
    }
  }

  return true;
}

// ── XML sensor discovery (always logged)
// ──────────────────────────────────────

void MuJoCoSimulator::LogAllXmlSensors() const noexcept {
  if (!model_ || model_->nsensor <= 0) {
    fprintf(stdout, "[MuJoCoSimulator] XML sensors: none\n");
    return;
  }

  fprintf(stdout, "[MuJoCoSimulator] XML sensors found (%d):\n",
          static_cast<int>(model_->nsensor));
  for (int i = 0; i < model_->nsensor; ++i) {
    const char* name = mj_id2name(model_, mjOBJ_SENSOR, i);
    const int type = model_->sensor_type[i];
    const int dim = model_->sensor_dim[i];
    const int adr = model_->sensor_adr[i];
    const int objtype = model_->sensor_objtype[i];
    const int objid = model_->sensor_objid[i];

    // Resolve attached object name (site, body, joint, etc.)
    const char* obj_name = nullptr;
    if (objid >= 0 && objtype > 0) {
      obj_name = mj_id2name(model_, objtype, objid);
    }

    fprintf(stdout, "  [%d] \"%s\"  type=%d  dim=%d  adr=%d  obj=%s\n", i,
            name ? name : "(unnamed)", type, dim, adr, obj_name ? obj_name : "(none)");
  }
}

std::vector<std::string> MuJoCoSimulator::CollectAllXmlSensorNames() const noexcept {
  std::vector<std::string> names;
  if (!model_)
    return names;
  names.reserve(static_cast<std::size_t>(model_->nsensor));
  for (int i = 0; i < model_->nsensor; ++i) {
    const char* name = mj_id2name(model_, mjOBJ_SENSOR, i);
    if (name) {
      names.emplace_back(name);
    }
  }
  return names;
}

// ── Sensor name → MuJoCo index mapping
// ────────────────────────────────────────

void MuJoCoSimulator::MapSensorInfos(JointGroup& group,
                                     const std::vector<std::string>& sensor_names) noexcept {
  group.sensor_infos.clear();
  int total_dim = 0;

  for (const auto& sname : sensor_names) {
    const int sid = mj_name2id(model_, mjOBJ_SENSOR, sname.c_str());
    if (sid < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] WARNING: group '%s' — sensor '%s' not found "
              "in XML, skipped\n",
              group.name.c_str(), sname.c_str());
      continue;
    }

    JointGroup::SensorInfo info;
    info.name = sname;
    info.type = model_->sensor_type[sid];
    info.adr = model_->sensor_adr[sid];
    info.dim = model_->sensor_dim[sid];
    total_dim += info.dim;

    fprintf(stdout, "[MuJoCoSimulator] [%s] sensor '%s' → type=%d  adr=%d  dim=%d\n",
            group.name.c_str(), sname.c_str(), info.type, info.adr, info.dim);

    group.sensor_infos.push_back(std::move(info));
  }

  group.sensor_buffer.resize(static_cast<std::size_t>(total_dim), 0.0);
}

// ── Contact wrench auto-discovery ────────────────────────────────────────
// Scans group.sensor_infos for mjSENS_CONTACT entries (dim==17 expected for
// data="found force torque dist pos normal tangent" with num=1). For each
// match, strips the configured sensor-name suffix to derive a target token,
// concatenates each configured site-name suffix to find the FT reference site,
// and resolves the site's owning body as the frame_id.
//
// allow_partial_discovery=false (default): if any contact sensor in the group
// fails to resolve, the function returns false (caller decides whether to
// abort initialization).
//
// Suffix lists are tried in declaration order — keep longest-first.
namespace {

[[nodiscard]] std::optional<std::string> StripFirstMatchingSuffix(
    std::string_view name, const std::vector<std::string>& suffixes) noexcept {
  for (const auto& suf : suffixes) {
    if (name.size() > suf.size() &&
        name.compare(name.size() - suf.size(), suf.size(), suf) == 0) {
      return std::string(name.substr(0, name.size() - suf.size()));
    }
  }
  return std::nullopt;
}

[[nodiscard]] int ResolveSiteByTarget(const mjModel* model, const std::string& target,
                                      const std::vector<std::string>& suffixes,
                                      std::string& matched_name) noexcept {
  for (const auto& suf : suffixes) {
    matched_name = target + suf;
    const int sid = mj_name2id(model, mjOBJ_SITE, matched_name.c_str());
    if (sid >= 0)
      return sid;
  }
  matched_name.clear();
  return -1;
}

}  // namespace

bool MuJoCoSimulator::DiscoverContactWrenches(JointGroup& group) noexcept {
  group.contact_wrench_infos.clear();
  group.contact_wrench_buffer.clear();

  if (!model_)
    return false;

  const JointGroupConfig::ContactWrenchConfig* gcfg = nullptr;
  for (const auto& group_cfg : cfg_.groups) {
    if (group_cfg.name == group.name) {
      gcfg = &group_cfg.contact_wrench;
      break;
    }
  }
  if (!gcfg || !gcfg->enabled)
    return true;  // disabled = success (nothing to discover)

  constexpr int kExpectedDim = 17;
  bool any_unresolved = false;

  for (const auto& sinfo : group.sensor_infos) {
    if (sinfo.type != mjSENS_CONTACT)
      continue;
    if (sinfo.dim != kExpectedDim) {
      fprintf(stderr,
              "[MuJoCoSimulator] [%s] contact sensor '%s' has dim=%d "
              "(expected %d) — skipped\n",
              group.name.c_str(), sinfo.name.c_str(), sinfo.dim, kExpectedDim);
      any_unresolved = true;
      continue;
    }

    // Two-stage naming:
    //   target_name (= ROS topic segment) is the sensor name verbatim, so the
    //   topic path mirrors the MJCF declaration and round-trips losslessly.
    //   site_stem (internal) strips the configured sensor-name suffix and is
    //   only used to locate the matching reference site by concatenating each
    //   reference_site_suffixes entry. Empty suffix list = site lookup uses
    //   the sensor name verbatim (rare; useful when the MJCF re-uses the same
    //   token for both objects).
    const auto site_stem = StripFirstMatchingSuffix(sinfo.name, gcfg->sensor_name_suffixes);
    if (!site_stem) {
      fprintf(stderr,
              "[MuJoCoSimulator] [%s] contact sensor '%s' did not match any "
              "configured sensor_name_suffixes — skipped\n",
              group.name.c_str(), sinfo.name.c_str());
      any_unresolved = true;
      continue;
    }

    std::string site_name;
    const int site_id =
        ResolveSiteByTarget(model_, *site_stem, gcfg->reference_site_suffixes, site_name);
    if (site_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] [%s] contact sensor '%s' → site_stem '%s' but no "
              "matching reference site found — skipped\n",
              group.name.c_str(), sinfo.name.c_str(), site_stem->c_str());
      any_unresolved = true;
      continue;
    }

    const int body_id = model_->site_bodyid[site_id];
    const char* body_name = mj_id2name(model_, mjOBJ_BODY, body_id);

    JointGroup::ContactWrenchInfo info;
    info.target_name = sinfo.name;  // ROS topic segment = MJCF sensor name verbatim.
    info.frame_id = body_name ? body_name : ("body_" + std::to_string(body_id));
    info.sensor_id = mj_name2id(model_, mjOBJ_SENSOR, sinfo.name.c_str());
    info.sensor_adr = sinfo.adr;
    info.ft_site_id = site_id;
    info.body_id = body_id;

    fprintf(stdout,
            "[MuJoCoSimulator] [%s] contact_wrench target='%s' site=%s frame=%s\n",
            group.name.c_str(), info.target_name.c_str(), site_name.c_str(),
            info.frame_id.c_str());

    group.contact_wrench_infos.push_back(std::move(info));
  }

  group.contact_wrench_buffer.resize(group.contact_wrench_infos.size());

  if (any_unresolved && !gcfg->allow_partial_discovery) {
    fprintf(stderr,
            "[MuJoCoSimulator] [%s] contact wrench discovery failed "
            "(allow_partial_discovery=false)\n",
            group.name.c_str());
    return false;
  }
  return true;
}

// ── Initialization
// ─────────────────────────────────────────────────────────────

bool MuJoCoSimulator::Initialize() noexcept {
  char error[1000] = {};
  model_ = mj_loadXML(cfg_.model_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    fprintf(stderr, "[MuJoCoSimulator] Failed to load '%s': %s\n", cfg_.model_path.c_str(), error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    fprintf(stderr, "[MuJoCoSimulator] mj_makeData failed\n");
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  original_gravity_z_ = static_cast<double>(model_->opt.gravity[2]);

  // Store XML solver defaults into atomics (may be overwritten by
  // ApplySolverConfig)
  solver_integrator_.store(model_->opt.integrator, std::memory_order_relaxed);
  solver_type_.store(model_->opt.solver, std::memory_order_relaxed);
  solver_iterations_.store(model_->opt.iterations, std::memory_order_relaxed);
  solver_tolerance_.store(static_cast<double>(model_->opt.tolerance), std::memory_order_relaxed);
  solver_cone_.store(model_->opt.cone, std::memory_order_relaxed);
  solver_jacobian_.store(model_->opt.jacobian, std::memory_order_relaxed);
  solver_ls_iterations_.store(model_->opt.ls_iterations, std::memory_order_relaxed);
  solver_ls_tolerance_.store(static_cast<double>(model_->opt.ls_tolerance),
                             std::memory_order_relaxed);
  solver_noslip_iterations_.store(model_->opt.noslip_iterations, std::memory_order_relaxed);
  solver_noslip_tolerance_.store(static_cast<double>(model_->opt.noslip_tolerance),
                                 std::memory_order_relaxed);
  solver_impratio_.store(static_cast<double>(model_->opt.impratio), std::memory_order_relaxed);

  // Apply YAML solver config with XML priority
  ApplySolverConfig();

  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);
  ext_xfrc_.assign(static_cast<std::size_t>(model_->nbody) * 6, 0.0);
  ext_point_.assign(static_cast<std::size_t>(model_->nbody) * 3, 0.0);

  // Save original actuator params
  orig_actuator_params_.resize(static_cast<std::size_t>(model_->nu));
  for (int i = 0; i < model_->nu; ++i) {
    auto& p = orig_actuator_params_[static_cast<std::size_t>(i)];
    p.gainprm0 = static_cast<double>(model_->actuator_gainprm[i * mjNGAIN + 0]);
    p.biasprm0 = static_cast<double>(model_->actuator_biasprm[i * mjNBIAS + 0]);
    p.biasprm1 = static_cast<double>(model_->actuator_biasprm[i * mjNBIAS + 1]);
    p.biasprm2 = static_cast<double>(model_->actuator_biasprm[i * mjNBIAS + 2]);
  }

  // ── Discover all XML joints ─────────────────────────────────────────────
  if (!DiscoverAllXmlJoints()) {
    return false;
  }

  // ── Create JointGroup objects from config ────────────────────────────────
  if (cfg_.groups.empty()) {
    fprintf(stderr, "[MuJoCoSimulator] ERROR: no groups configured\n");
    return false;
  }

  // Validate no duplicate group names between robot and fake
  {
    std::set<std::string> robot_names, fake_names;
    for (const auto& gc : cfg_.groups) {
      if (gc.is_robot) {
        robot_names.insert(gc.name);
      } else {
        fake_names.insert(gc.name);
      }
    }
    for (const auto& rn : robot_names) {
      if (fake_names.count(rn)) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: group '%s' exists in both "
                "robot_response and fake_response\n",
                rn.c_str());
        return false;
      }
    }
  }

  bool first_robot = true;
  for (const auto& gc : cfg_.groups) {
    // Resolve command_joint_names: prefer explicit, fallback to joint_names
    auto cmd_names = gc.command_joint_names.empty() ? gc.joint_names : gc.command_joint_names;
    if (cmd_names.empty()) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' has no command_joint_names "
              "(nor joint_names)\n",
              gc.name.c_str());
      return false;
    }
    if (gc.command_topic.empty() || gc.state_topic.empty()) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' has empty command_topic or "
              "state_topic\n",
              gc.name.c_str());
      return false;
    }

    auto g = std::make_unique<JointGroup>();
    g->name = gc.name;
    g->command_joint_names = cmd_names;
    g->num_command_joints = static_cast<int>(cmd_names.size());
    g->is_robot = gc.is_robot;
    g->command_topic = gc.command_topic;
    g->state_topic = gc.state_topic;
    g->sensor_topic = gc.sensor_topic;
    g->filter_alpha = gc.filter_alpha;

    // Resolve state_joint_names: prefer explicit, fallback to XML all joints
    if (!gc.state_joint_names.empty()) {
      g->state_joint_names = gc.state_joint_names;
    } else if (gc.is_robot) {
      g->state_joint_names = all_xml_joint_names_;
    } else {
      // fake group: state same as command
      g->state_joint_names = cmd_names;
    }
    g->num_state_joints = static_cast<int>(g->state_joint_names.size());

    if (gc.is_robot && first_robot) {
      g->is_primary = true;
      first_robot = false;
    }

    const auto ncj = static_cast<std::size_t>(g->num_command_joints);
    const auto nsj = static_cast<std::size_t>(g->num_state_joints);

    if (gc.is_robot) {
      // Command buffers
      g->pending_cmd.resize(ncj, 0.0);
      g->initial_qpos.resize(ncj, 0.0);
      g->gainprm_yaml.resize(ncj, 0.0);
      g->biasprm2_yaml.resize(ncj, 0.0);
      // State buffers
      g->positions.resize(nsj, 0.0);
      g->velocities.resize(nsj, 0.0);
      g->efforts.resize(nsj, 0.0);
    } else {
      // Fake group: LPF buffers (command size for target, state size for
      // output)
      g->fake_target.resize(ncj, 0.0);
      g->fake_state.resize(nsj, 0.0);
    }

    groups_.push_back(std::move(g));
  }

  // ── Validate command joints against XML (양방향 완전 일치) ────────────────
  if (!ValidateAndMapRobotGroups()) {
    return false;
  }

  // ── Validate state joints against XML ──────────────────────────────────
  if (!ValidateAndMapStateJoints()) {
    return false;
  }

  // ── Log all XML sensors (always, regardless of YAML config) ─────────
  LogAllXmlSensors();

  // ── Map sensor infos per group ─────────────────────────────────────────
  for (std::size_t gi = 0; gi < groups_.size(); ++gi) {
    const auto& gc = cfg_.groups[gi];
    if (gc.sensor_names.empty())
      continue;

    // "auto" keyword: map ALL XML sensors to this group
    if (gc.sensor_names.size() == 1 && gc.sensor_names[0] == "auto") {
      auto all_names = CollectAllXmlSensorNames();
      if (all_names.empty()) {
        fprintf(stdout,
                "[MuJoCoSimulator] [%s] sensor_names='auto' but no sensors in "
                "XML\n",
                gc.name.c_str());
      } else {
        fprintf(stdout,
                "[MuJoCoSimulator] [%s] sensor_names='auto' → mapping all %zu "
                "XML sensors\n",
                gc.name.c_str(), all_names.size());
        MapSensorInfos(*groups_[gi], all_names);
      }
    } else {
      MapSensorInfos(*groups_[gi], gc.sensor_names);
    }
  }

  // ── Contact wrench discovery (mjSENS_CONTACT auto-mapping) ─────────────
  for (const auto& grp : groups_) {
    if (!DiscoverContactWrenches(*grp)) {
      fprintf(stderr,
              "[MuJoCoSimulator] Initialize aborted: contact wrench discovery "
              "failed for group '%s'\n",
              grp->name.c_str());
      return false;
    }
  }

  // ── Physics timestep ────────────────────────────────────────────────────
  xml_timestep_ = static_cast<double>(model_->opt.timestep);

  if (cfg_.physics_timestep > 0.0) {
    constexpr double kEps = 1e-9;
    if (std::abs(cfg_.physics_timestep - xml_timestep_) > kEps) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: physics_timestep mismatch — "
              "yaml=%.6f s (%.1f Hz)  XML=%.6f s (%.1f Hz) → XML value used\n",
              cfg_.physics_timestep, 1.0 / cfg_.physics_timestep, xml_timestep_,
              1.0 / xml_timestep_);
    } else {
      fprintf(stdout, "[MuJoCoSimulator] physics_timestep OK: %.6f s (%.1f Hz)\n", xml_timestep_,
              1.0 / xml_timestep_);
    }
  }

  // ── Substep validation & timestep override ──────────────────────────────
  if (cfg_.n_substeps < 1) {
    fprintf(stderr,
            "[MuJoCoSimulator] ERROR: n_substeps=%d must be >= 1, "
            "falling back to 1\n",
            cfg_.n_substeps);
    cfg_.n_substeps = 1;
  }

  if (cfg_.n_substeps > 1) {
    const double substep_dt = xml_timestep_ / static_cast<double>(cfg_.n_substeps);
    model_->opt.timestep = static_cast<mjtNum>(substep_dt);
    fprintf(stdout,
            "[MuJoCoSimulator] Substepping: n_substeps=%d  "
            "control_period=%.4f s (%.1f Hz)  substep_dt=%.6f s (%.1f Hz)\n",
            cfg_.n_substeps, xml_timestep_, 1.0 / xml_timestep_, substep_dt, 1.0 / substep_dt);
  } else {
    fprintf(stdout,
            "[MuJoCoSimulator] Substepping: n_substeps=1  "
            "physics_dt=%.4f s (%.1f Hz)\n",
            xml_timestep_, 1.0 / xml_timestep_);
  }

  // ── Viewer refresh rate ─────────────────────────────────────────────────
  if (cfg_.viewer_refresh_rate <= 0.0) {
    fprintf(stderr,
            "[MuJoCoSimulator] ERROR: viewer_refresh_rate=%.1f must be > 0, "
            "falling back to 60.0\n",
            cfg_.viewer_refresh_rate);
    cfg_.viewer_refresh_rate = 60.0;
  }
  {
    const double interval = 1.0 / (xml_timestep_ * cfg_.viewer_refresh_rate);
    viz_update_interval_ =
        std::max(static_cast<uint64_t>(1), static_cast<uint64_t>(std::round(interval)));
    viewer_sleep_ms_ = std::max(1, static_cast<int>(std::round(1000.0 / cfg_.viewer_refresh_rate)));
    fprintf(stdout,
            "[MuJoCoSimulator] Viewer: target=%.0f Hz  "
            "viz_update_interval=%lu steps  render_sleep=%d ms\n",
            cfg_.viewer_refresh_rate, static_cast<unsigned long>(viz_update_interval_),
            viewer_sleep_ms_);
  }

  // ── Per-group servo gains & initial positions ───────────────────────────
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;

    const auto ncj = static_cast<std::size_t>(g->num_command_joints);

    // Servo gains: use per-group if provided, otherwise inherit global
    const auto& kp = (!cfg_.groups.empty()) ? [&]() -> const std::vector<double>& {
      for (const auto& gc : cfg_.groups) {
        if (gc.name == g->name && !gc.servo_kp.empty())
          return gc.servo_kp;
      }
      return cfg_.servo_kp;
    }()
        : cfg_.servo_kp;

    const auto& kd = (!cfg_.groups.empty()) ? [&]() -> const std::vector<double>& {
      for (const auto& gc : cfg_.groups) {
        if (gc.name == g->name && !gc.servo_kd.empty())
          return gc.servo_kd;
      }
      return cfg_.servo_kd;
    }()
        : cfg_.servo_kd;

    if (cfg_.use_yaml_servo_gains) {
      if (kp.size() != ncj || kd.size() != ncj) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: group '%s' servo_kp/kd size "
                "(%zu/%zu) != command joints (%zu)\n",
                g->name.c_str(), kp.size(), kd.size(), ncj);
        return false;
      }
    }

    for (std::size_t i = 0; i < ncj; ++i) {
      if (i < kp.size())
        g->gainprm_yaml[i] = kp[i];
      if (i < kd.size())
        g->biasprm2_yaml[i] = -kd[i];
    }

    // Initial positions from keyframe
    if (model_->nkey > 0) {
      for (std::size_t i = 0; i < ncj; ++i) {
        g->initial_qpos[i] = static_cast<double>(model_->key_qpos[g->qpos_indices[i]]);
      }
    }

    // Apply initial positions
    for (std::size_t i = 0; i < ncj; ++i) {
      data_->qpos[g->qpos_indices[i]] = g->initial_qpos[i];
      data_->ctrl[g->actuator_indices[i]] = g->initial_qpos[i];
    }
  }

  mj_forward(model_, data_);
  ReadState();

  if (model_->nkey > 0) {
    const char* key_name = mj_id2name(model_, mjOBJ_KEY, 0);
    fprintf(stdout, "[MuJoCoSimulator] Initial positions from keyframe '%s'\n",
            key_name ? key_name : "(unnamed)");
  } else {
    fprintf(stdout, "[MuJoCoSimulator] No keyframe — using zero initial positions\n");
  }

  // ── Initial gravity state ───────────────────────────────────────────────
  // World gravity stays ON; robot groups default to position-servo, which
  // enables per-body gravity compensation on their body chain. Free objects
  // (objects to lift) are unaffected and still fall under -9.81.
  world_gravity_enabled_.store(true, std::memory_order_relaxed);
  for (const auto& g : groups_) {
    if (!g->is_robot)
      continue;
    const double gravcomp =
        (g->control_mode.load(std::memory_order_relaxed) == JointControlMode::kPosition) ? 1.0 : 0.0;
    for (int body_id : g->body_indices) {
      if (body_id > 0 && body_id < model_->nbody)
        model_->body_gravcomp[body_id] = gravcomp;
    }
  }
  // mj_passive() early-outs when ngravcomp==0 (MuJoCo engine_passive.c).
  // The MJCF doesn't declare per-body gravcomp, so the compiled value is 0
  // — writing body_gravcomp[] alone would silently do nothing. Recount.
  RefreshNgravcomp();

  // ── Summary ─────────────────────────────────────────────────────────────
  int total_cmd_joints = 0;
  for (const auto& g : groups_) {
    if (g->is_robot)
      total_cmd_joints += g->num_command_joints;
    fprintf(stdout,
            "[MuJoCoSimulator] Group '%s': %s  cmd_joints=%d  state_joints=%d  "
            "sensors=%zu  cmd=%s  state=%s  sensor=%s%s\n",
            g->name.c_str(), g->is_robot ? "ROBOT" : "FAKE", g->num_command_joints,
            g->num_state_joints, g->sensor_infos.size(), g->command_topic.c_str(),
            g->state_topic.c_str(), g->sensor_topic.empty() ? "(none)" : g->sensor_topic.c_str(),
            g->is_primary ? "  [PRIMARY]" : "");
  }

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d nv=%d nu=%d  groups=%zu"
          "  command_joints=%d  dt=%.4f s\n",
          cfg_.model_path.c_str(), static_cast<int>(model_->nq), static_cast<int>(model_->nv),
          static_cast<int>(model_->nu), groups_.size(), total_cmd_joints, xml_timestep_);

  return true;
}

// ── ApplySolverConfig
// ───────────────────────────────────────────────────────── Parse the MJCF XML
// to detect which <option> attributes are explicitly set. For attributes NOT
// present in XML, apply YAML (SolverConfig) values. For attributes present in
// XML, keep the XML values (already in model_->opt).

void MuJoCoSimulator::ApplySolverConfig() noexcept {
  const auto& sc = cfg_.solver_config;

  // ── Parse XML to find explicit <option> attributes ────────────────────────
  // tinyxml2 doesn't go through MuJoCo's resource provider, so resolve any
  // "package://" URI to an absolute filesystem path first.
  const std::string resolved_path = ResolveModelPath(cfg_.model_path);
  std::set<std::string> xml_attrs;
  {
    tinyxml2::XMLDocument doc;
    if (!resolved_path.empty() && doc.LoadFile(resolved_path.c_str()) == tinyxml2::XML_SUCCESS) {
      const auto* root = doc.RootElement();
      if (root) {
        const auto* option = root->FirstChildElement("option");
        if (option) {
          for (const auto* attr = option->FirstAttribute(); attr != nullptr; attr = attr->Next()) {
            xml_attrs.insert(attr->Name());
          }
        }
      }
    } else {
      fprintf(stderr,
              "[MuJoCoSimulator] WARNING: Could not re-parse XML for <option> "
              "detection (path='%s'), "
              "using YAML solver config as fallback for all parameters\n",
              resolved_path.empty() ? cfg_.model_path.c_str() : resolved_path.c_str());
    }
  }

  // ── Apply YAML values where XML did not set them ──────────────────────────
  // For each parameter: if the attribute name is NOT in xml_attrs, apply YAML.

  if (!xml_attrs.count("solver")) {
    const int v = SolverNameToEnum(sc.solver);
    model_->opt.solver = v;
    solver_type_.store(v, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("cone")) {
    const int v = ConeNameToEnum(sc.cone);
    model_->opt.cone = v;
    solver_cone_.store(v, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("jacobian")) {
    const int v = JacobianNameToEnum(sc.jacobian);
    model_->opt.jacobian = v;
    solver_jacobian_.store(v, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("integrator")) {
    const int v = IntegratorNameToEnum(sc.integrator);
    model_->opt.integrator = static_cast<mjtIntegrator>(v);
    solver_integrator_.store(v, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("iterations")) {
    model_->opt.iterations = sc.iterations;
    solver_iterations_.store(sc.iterations, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("tolerance")) {
    model_->opt.tolerance = static_cast<mjtNum>(sc.tolerance);
    solver_tolerance_.store(sc.tolerance, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("ls_iterations")) {
    model_->opt.ls_iterations = sc.ls_iterations;
    solver_ls_iterations_.store(sc.ls_iterations, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("ls_tolerance")) {
    model_->opt.ls_tolerance = static_cast<mjtNum>(sc.ls_tolerance);
    solver_ls_tolerance_.store(sc.ls_tolerance, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("noslip_iterations")) {
    model_->opt.noslip_iterations = sc.noslip_iterations;
    solver_noslip_iterations_.store(sc.noslip_iterations, std::memory_order_relaxed);
  }
  if (!xml_attrs.count("noslip_tolerance")) {
    model_->opt.noslip_tolerance = static_cast<mjtNum>(sc.noslip_tolerance);
    solver_noslip_tolerance_.store(sc.noslip_tolerance, std::memory_order_relaxed);
  }
  // MuJoCo 3.2 부터 ccd_* (convex collision) 가 mpr_* 를 대체.
  // mjVERSION_HEADER 자릿수 체계가 3.5+ 부터 변경됨:
  //   3.1.x = 31x, 3.2.4 = 324, 3.5+ = MAJOR*1000000 + MINOR*1000 + PATCH (예: 3.7.0 = 3007000)
  // 두 자릿수 스케일 모두 cover 하도록 helper macro 로 추출.
#if defined(mjVERSION_HEADER) && \
    ((mjVERSION_HEADER >= 320 && mjVERSION_HEADER < 1000) || mjVERSION_HEADER >= 3002000)
  #define RTC_MUJOCO_HAS_CCD 1
#else
  #define RTC_MUJOCO_HAS_CCD 0
#endif

#if RTC_MUJOCO_HAS_CCD
  if (!xml_attrs.count("ccd_iterations")) {
    model_->opt.ccd_iterations = sc.ccd_iterations;
  }
  if (!xml_attrs.count("ccd_tolerance")) {
    model_->opt.ccd_tolerance = static_cast<mjtNum>(sc.ccd_tolerance);
  }
#else
  if (!xml_attrs.count("mpr_iterations")) {
    model_->opt.mpr_iterations = sc.ccd_iterations;
  }
  if (!xml_attrs.count("mpr_tolerance")) {
    model_->opt.mpr_tolerance = static_cast<mjtNum>(sc.ccd_tolerance);
  }
#endif
  if (!xml_attrs.count("sdf_iterations")) {
    model_->opt.sdf_iterations = sc.sdf_iterations;
  }
  if (!xml_attrs.count("sdf_initpoints")) {
    model_->opt.sdf_initpoints = sc.sdf_initpoints;
  }
  if (!xml_attrs.count("impratio")) {
    model_->opt.impratio = static_cast<mjtNum>(sc.impratio);
    solver_impratio_.store(sc.impratio, std::memory_order_relaxed);
  }

  // ── Flags (via enableflags/disableflags bitfields) ────────────────────────
  // These flags are set in <flag> element, not <option>, so parse separately.
  // Reuse resolved_path from <option> block above.
  {
    tinyxml2::XMLDocument doc;
    std::set<std::string> flag_attrs;
    if (!resolved_path.empty() && doc.LoadFile(resolved_path.c_str()) == tinyxml2::XML_SUCCESS) {
      const auto* root = doc.RootElement();
      if (root) {
        const auto* flag_elem = root->FirstChildElement("flag");
        if (flag_elem) {
          for (const auto* attr = flag_elem->FirstAttribute(); attr != nullptr;
               attr = attr->Next()) {
            flag_attrs.insert(attr->Name());
          }
        }
      }
    }

    // warmstart, refsafe, eulerdamp, filterparent are all mjDSBL_ bits
    // (disabled = flag set, enabled = flag cleared)
    if (!flag_attrs.count("warmstart")) {
      if (sc.warmstart) {
        model_->opt.disableflags &= ~mjDSBL_WARMSTART;
      } else {
        model_->opt.disableflags |= mjDSBL_WARMSTART;
      }
    }
    if (!flag_attrs.count("refsafe")) {
      if (sc.refsafe) {
        model_->opt.disableflags &= ~mjDSBL_REFSAFE;
      } else {
        model_->opt.disableflags |= mjDSBL_REFSAFE;
      }
    }
    if (!flag_attrs.count("island")) {
      // MuJoCo 3.3.6 부터 island discovery 가 default-on 으로 승격되면서
      // mjENBL_ISLAND 가 제거되고 mjDSBL_ISLAND disable flag 로 의미가 반전됨.
      // YAML 의 island=true 는 "활성화" 의미를 유지 — polarity 만 반전 처리.
      if (sc.island) {
        model_->opt.disableflags &= ~mjDSBL_ISLAND;
      } else {
        model_->opt.disableflags |= mjDSBL_ISLAND;
      }
    }
    if (!flag_attrs.count("eulerdamp")) {
      if (sc.eulerdamp) {
        model_->opt.disableflags &= ~mjDSBL_EULERDAMP;
      } else {
        model_->opt.disableflags |= mjDSBL_EULERDAMP;
      }
    }
    if (!flag_attrs.count("filterparent")) {
      if (sc.filterparent) {
        model_->opt.disableflags &= ~mjDSBL_FILTERPARENT;
      } else {
        model_->opt.disableflags |= mjDSBL_FILTERPARENT;
      }
    }
    if (!flag_attrs.count("override")) {
      if (sc.contact_override.enable) {
        model_->opt.enableflags |= mjENBL_OVERRIDE;
      } else {
        model_->opt.enableflags &= ~mjENBL_OVERRIDE;
      }
    }
    // multiccd: enableflags bit. Multi-point convex collision — gives several
    // contact points per convex pair, which materially improves grasp/pinch
    // stability over MuJoCo's default single-point contact.
    if (!flag_attrs.count("multiccd")) {
      if (sc.multiccd) {
        model_->opt.enableflags |= mjENBL_MULTICCD;
      } else {
        model_->opt.enableflags &= ~mjENBL_MULTICCD;
      }
    }
    // autoreset: disableflags bit (set = disabled). When disabled, NaN/blow-up
    // surfaces as a hard simulator error instead of silent mj_resetData — useful
    // during grasp-controller development.
    if (!flag_attrs.count("autoreset")) {
      if (sc.autoreset) {
        model_->opt.disableflags &= ~mjDSBL_AUTORESET;
      } else {
        model_->opt.disableflags |= mjDSBL_AUTORESET;
      }
    }
    // nativeccd: disableflags bit (set = disabled). Available in MuJoCo 3.3+;
    // earlier versions ignore the YAML key silently. Same version-scale handling
    // as RTC_MUJOCO_HAS_CCD above.
#if defined(mjVERSION_HEADER) && \
    ((mjVERSION_HEADER >= 330 && mjVERSION_HEADER < 1000) || mjVERSION_HEADER >= 3003000)
    if (!flag_attrs.count("nativeccd")) {
      if (sc.nativeccd) {
        model_->opt.disableflags &= ~mjDSBL_NATIVECCD;
      } else {
        model_->opt.disableflags |= mjDSBL_NATIVECCD;
      }
    }
#endif
  }

  // ── Contact override parameters ───────────────────────────────────────────
  if (sc.contact_override.enable || (model_->opt.enableflags & mjENBL_OVERRIDE)) {
    model_->opt.o_margin = static_cast<mjtNum>(sc.contact_override.o_margin);
    for (int i = 0; i < 2; ++i) {
      model_->opt.o_solref[i] =
          static_cast<mjtNum>(sc.contact_override.o_solref[static_cast<std::size_t>(i)]);
    }
    for (int i = 0; i < 5; ++i) {
      model_->opt.o_solimp[i] =
          static_cast<mjtNum>(sc.contact_override.o_solimp[static_cast<std::size_t>(i)]);
      model_->opt.o_friction[i] =
          static_cast<mjtNum>(sc.contact_override.o_friction[static_cast<std::size_t>(i)]);
    }
  }

  // ── Log summary ───────────────────────────────────────────────────────────
  static constexpr const char* kSolverNames[] = {"PGS", "CG", "Newton"};
  static constexpr const char* kConeNames[] = {"Pyramidal", "Elliptic"};
  static constexpr const char* kIntNames[] = {"Euler", "RK4", "Implicit", "ImplicitFast"};

  const int sol = model_->opt.solver;
  const int cone = model_->opt.cone;
  const int integ = model_->opt.integrator;

  fprintf(stdout,
          "[MuJoCoSimulator] Solver config: solver=%s  cone=%s  integrator=%s  "
          "iterations=%d  tolerance=%.1e  impratio=%.1f  noslip_iter=%d\n",
          (sol >= 0 && sol <= 2) ? kSolverNames[sol] : "?",
          (cone >= 0 && cone <= 1) ? kConeNames[cone] : "?",
          (integ >= 0 && integ <= 3) ? kIntNames[integ] : "?", model_->opt.iterations,
          static_cast<double>(model_->opt.tolerance), static_cast<double>(model_->opt.impratio),
          model_->opt.noslip_iterations);

  if (!xml_attrs.empty()) {
    fprintf(stdout, "[MuJoCoSimulator] XML <option> explicit attrs:");
    for (const auto& a : xml_attrs) {
      fprintf(stdout, " %s", a.c_str());
    }
    fprintf(stdout, "\n");
  } else {
    fprintf(stdout,
            "[MuJoCoSimulator] XML <option> not found — all solver "
            "params from YAML\n");
  }

  // Shrink contact-point viz disks to 1/10 of MuJoCo defaults
  // (0.3 / 0.1). Purely visual — does not affect physics.
  constexpr float kContactVizWidth = 0.03F;
  constexpr float kContactVizHeight = 0.01F;
  model_->vis.scale.contactwidth = kContactVizWidth;
  model_->vis.scale.contactheight = kContactVizHeight;
}

// ── Thread lifecycle
// ───────────────────────────────────────────────────────────

void MuJoCoSimulator::Start() noexcept {
  if (running_.exchange(true)) {
    return;
  }

  sim_thread_ = std::jthread([this](std::stop_token st) { SimLoop(st); });
  if (cfg_.enable_viewer) {
    viewer_thread_ = std::jthread([this](std::stop_token st) { ViewerLoop(st); });
  }
}

void MuJoCoSimulator::Stop() noexcept {
  running_.store(false);
  sync_cv_.notify_all();
  if (sim_thread_.joinable()) {
    sim_thread_.request_stop();
    sim_thread_.join();
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.request_stop();
    viewer_thread_.join();
  }
}

// ── Group-indexed Command / State I/O
// ───────────────────────────────────────────

void MuJoCoSimulator::SetCommand(std::size_t group_idx, const std::vector<double>& cmd) noexcept {
  if (group_idx >= groups_.size())
    return;
  auto& g = *groups_[group_idx];
  if (!g.is_robot)
    return;
  {
    std::lock_guard lock(g.cmd_mutex);
    g.pending_cmd = cmd;
  }
  g.cmd_pending.store(true, std::memory_order_release);
  if (g.is_primary) {
    sync_cv_.notify_one();
  }
}

void MuJoCoSimulator::StageCommand(std::size_t group_idx, JointControlMode mode,
                                   const std::vector<double>& cmd,
                                   const std::vector<double>& feedforward,
                                   const std::vector<double>& kp,
                                   const std::vector<double>& kd) noexcept {
  if (group_idx >= groups_.size())
    return;
  auto& g = *groups_[group_idx];
  if (!g.is_robot)
    return;

  const auto ncj = static_cast<std::size_t>(g.num_command_joints);
  const bool has_gains = (kp.size() == ncj && kd.size() == ncj);
  const bool mode_changed = (g.control_mode.load(std::memory_order_relaxed) != mode);

  {
    std::lock_guard lock(g.cmd_mutex);
    g.pending_cmd = cmd;
    // Feedforward is consumed only in kPdFeedforward; size to ncj (zero-fill) so
    // ApplyCommand can index without bounds surprises even on an empty/short msg.
    g.pending_feedforward.assign(ncj, 0.0);
    if (mode == JointControlMode::kPdFeedforward) {
      const auto n = std::min(feedforward.size(), ncj);
      std::copy_n(feedforward.begin(), n, g.pending_feedforward.begin());
    }
    if (has_gains) {
      g.staged_kp = kp;
      g.staged_kd = kd;
      g.staged_has_gains = true;
    }
    g.staged_mode = mode;
  }

  g.control_mode.store(mode, std::memory_order_relaxed);
  // Re-apply actuator params (mode flip or gain change) on the next SimLoop tick.
  if (mode_changed || has_gains) {
    g.control_mode_pending.store(true, std::memory_order_release);
  }
  // cmd_pending is the publish gate — release-store LAST so the SimLoop's
  // acquire-load sees the mode store and the cmd_mutex-protected staging above.
  g.cmd_pending.store(true, std::memory_order_release);
  if (g.is_primary) {
    sync_cv_.notify_one();
  }
}

void MuJoCoSimulator::SetStateCallback(std::size_t group_idx, StateCallback cb) noexcept {
  if (group_idx >= groups_.size())
    return;
  groups_[group_idx]->state_cb = std::move(cb);
}

void MuJoCoSimulator::SetSensorCallback(std::size_t group_idx,
                                        JointGroup::SensorCallback cb) noexcept {
  if (group_idx >= groups_.size())
    return;
  groups_[group_idx]->sensor_cb = std::move(cb);
}

const std::vector<JointGroup::SensorInfo>& MuJoCoSimulator::GetSensorInfos(
    std::size_t group_idx) const noexcept {
  static const std::vector<JointGroup::SensorInfo> empty;
  if (group_idx >= groups_.size())
    return empty;
  return groups_[group_idx]->sensor_infos;
}

bool MuJoCoSimulator::HasSensors(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return false;
  return !groups_[group_idx]->sensor_infos.empty();
}

void MuJoCoSimulator::SetContactWrenchCallback(
    std::size_t group_idx, JointGroup::ContactWrenchCallback callback) noexcept {
  if (group_idx >= groups_.size())
    return;
  groups_[group_idx]->contact_wrench_cb = std::move(callback);
}

const std::vector<JointGroup::ContactWrenchInfo>& MuJoCoSimulator::GetContactWrenchInfos(
    std::size_t group_idx) const noexcept {
  static const std::vector<JointGroup::ContactWrenchInfo> empty;
  if (group_idx >= groups_.size())
    return empty;
  return groups_[group_idx]->contact_wrench_infos;
}

bool MuJoCoSimulator::HasContactWrenches(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return false;
  return !groups_[group_idx]->contact_wrench_infos.empty();
}

std::vector<double> MuJoCoSimulator::GetPositions(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.state_mutex);
  return g.positions;
}

std::vector<double> MuJoCoSimulator::GetVelocities(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.state_mutex);
  return g.velocities;
}

std::vector<double> MuJoCoSimulator::GetEfforts(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.state_mutex);
  return g.efforts;
}

const std::vector<std::string>& MuJoCoSimulator::GetJointNames(
    std::size_t group_idx) const noexcept {
  static const std::vector<std::string> empty;
  if (group_idx >= groups_.size())
    return empty;
  return groups_[group_idx]->command_joint_names;
}

const std::vector<std::string>& MuJoCoSimulator::GetStateJointNames(
    std::size_t group_idx) const noexcept {
  static const std::vector<std::string> empty;
  if (group_idx >= groups_.size())
    return empty;
  return groups_[group_idx]->state_joint_names;
}

int MuJoCoSimulator::NumGroupJoints(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return 0;
  return groups_[group_idx]->num_command_joints;
}

int MuJoCoSimulator::NumStateJoints(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return 0;
  return groups_[group_idx]->num_state_joints;
}

bool MuJoCoSimulator::IsGroupRobot(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return false;
  return groups_[group_idx]->is_robot;
}

// ── Gravcomp counter refresh ──────────────────────────────────────────────────
//
// MuJoCo's mj_passive() guards the gravcomp loop with `if (!m->ngravcomp) return`
// (engine_passive.c). The MJCF doesn't declare per-body gravcomp, so the
// compiled value is 0 — and a runtime body_gravcomp[] write would otherwise be
// silently ignored. Recount whenever we mutate body_gravcomp[].
void MuJoCoSimulator::RefreshNgravcomp() noexcept {
  if (!model_)
    return;
  int count = 0;
  for (int b = 0; b < model_->nbody; ++b) {
    if (model_->body_gravcomp[b] != 0.0)
      ++count;
  }
  model_->ngravcomp = count;
}

// ── Per-group control mode
// ──────────────────────────────────────────────────────

void MuJoCoSimulator::SetControlMode(std::size_t group_idx, JointControlMode mode) noexcept {
  if (group_idx >= groups_.size())
    return;
  auto& g = *groups_[group_idx];
  if (!g.is_robot)
    return;

  g.control_mode.store(mode, std::memory_order_relaxed);
  // Both actuator-param and per-body gravcomp updates are picked up by the
  // SimLoop in PreparePhysicsStep() the next tick — keeps all mjModel mutation
  // on a single thread.
  g.control_mode_pending.store(true, std::memory_order_release);
}

JointControlMode MuJoCoSimulator::GetControlMode(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return JointControlMode::kPosition;
  return groups_[group_idx]->control_mode.load(std::memory_order_relaxed);
}

bool MuJoCoSimulator::IsGroupGravcompEnabled(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return false;
  const auto& group = *groups_[group_idx];
  if (!group.is_robot)
    return false;
  // Reflect the *intended* control mode rather than the live mjModel slot:
  // SetControlMode() defers the body_gravcomp write to the next SimLoop tick
  // (single-thread ownership), so a caller that polls right after the setter
  // would otherwise see stale state. Only kPosition enables gravcomp; kTorque
  // and kPdFeedforward both leave it to the controller.
  return group.control_mode.load(std::memory_order_relaxed) == JointControlMode::kPosition;
}

// ── Test-only synchronous step + observers ──────────────────────────────────

void MuJoCoSimulator::StepForTest() noexcept {
  if (!model_ || !data_)
    return;
  ApplyCommand();
  PreparePhysicsStep();
  mj_step(model_, data_);
  ReadState();
}

double MuJoCoSimulator::GetActuatorForceForTest(std::size_t group_idx,
                                                std::size_t joint_idx) const noexcept {
  if (!data_ || group_idx >= groups_.size())
    return 0.0;
  const auto& g = *groups_[group_idx];
  if (joint_idx >= g.qvel_indices.size())
    return 0.0;
  return data_->qfrc_actuator[g.qvel_indices[joint_idx]];
}

double MuJoCoSimulator::GetAppliedForceForTest(std::size_t group_idx,
                                               std::size_t joint_idx) const noexcept {
  if (!data_ || group_idx >= groups_.size())
    return 0.0;
  const auto& g = *groups_[group_idx];
  if (joint_idx >= g.qvel_indices.size())
    return 0.0;
  return data_->qfrc_applied[g.qvel_indices[joint_idx]];
}

double MuJoCoSimulator::GetActuatorGainForTest(std::size_t group_idx,
                                               std::size_t joint_idx) const noexcept {
  if (!model_ || group_idx >= groups_.size())
    return 0.0;
  const auto& g = *groups_[group_idx];
  if (joint_idx >= g.actuator_indices.size())
    return 0.0;
  return model_->actuator_gainprm[g.actuator_indices[joint_idx] * mjNGAIN + 0];
}

// ── Fake response API
// ──────────────────────────────────────────────────────────

void MuJoCoSimulator::SetFakeTarget(std::size_t group_idx,
                                    const std::vector<double>& target) noexcept {
  if (group_idx >= groups_.size())
    return;
  auto& g = *groups_[group_idx];
  if (g.is_robot)
    return;
  std::lock_guard lock(g.fake_mutex);
  const auto n = std::min(target.size(), g.fake_target.size());
  std::copy_n(target.begin(), n, g.fake_target.begin());
}

void MuJoCoSimulator::AdvanceFakeLPF(std::size_t group_idx) noexcept {
  if (group_idx >= groups_.size())
    return;
  auto& g = *groups_[group_idx];
  if (g.is_robot)
    return;
  std::lock_guard lock(g.fake_mutex);
  ApplyFakeLpfStep(g.fake_state, g.fake_target, g.filter_alpha);
}

std::vector<double> MuJoCoSimulator::GetFakeState(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size())
    return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.fake_mutex);
  return g.fake_state;
}

// ── Solver statistics
// ──────────────────────────────────────────────────────────

MuJoCoSimulator::SolverStats MuJoCoSimulator::GetSolverStats() const noexcept {
  std::lock_guard lock(solver_stats_mutex_);
  return latest_solver_stats_;
}

// ── External forces / perturbation
// ────────────────────────────────────────────

int MuJoCoSimulator::FindBodyId(const char* body_name) const noexcept {
  if (!model_ || !body_name || body_name[0] == '\0') {
    return -1;
  }
  const int id = mj_name2id(model_, mjOBJ_BODY, body_name);
  // Fold the world body into the "no such target" answer deliberately: a
  // wrench on body 0 is swallowed by the fixed base, so reporting it as
  // accepted would be the silent no-op this API exists to rule out.
  return id > 0 ? id : -1;
}

bool MuJoCoSimulator::SetExternalWrenchAtPoint(
    int body_id, const std::array<double, 3>& point_body,
    const std::array<double, 6>& wrench_world) noexcept {
  if (!model_ || body_id <= 0 || body_id >= model_->nbody) {
    return false;
  }
  std::lock_guard lock(pert_mutex_);
  const std::size_t w_off = static_cast<std::size_t>(body_id) * 6;
  const std::size_t p_off = static_cast<std::size_t>(body_id) * 3;
  for (std::size_t i = 0; i < 6; ++i) {
    ext_xfrc_[w_off + i] = wrench_world[i];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    ext_point_[p_off + i] = point_body[i];
  }
  ext_xfrc_dirty_ = true;
  return true;
}

bool MuJoCoSimulator::SetExternalForce(int body_id,
                                       const std::array<double, 6>& wrench_world) noexcept {
  return SetExternalWrenchAtPoint(body_id, {0.0, 0.0, 0.0}, wrench_world);
}

void MuJoCoSimulator::ClearExternalForce() noexcept {
  std::lock_guard lock(pert_mutex_);
  std::fill(ext_xfrc_.begin(), ext_xfrc_.end(), 0.0);
  std::fill(ext_point_.begin(), ext_point_.end(), 0.0);
  ext_xfrc_dirty_ = false;
}

void MuJoCoSimulator::UpdatePerturb(const mjvPerturb& pert) noexcept {
  std::lock_guard lock(pert_mutex_);
  shared_pert_ = pert;
  pert_active_ = (pert.active != 0);
}

void MuJoCoSimulator::ClearPerturb() noexcept {
  std::lock_guard lock(pert_mutex_);
  mjv_defaultPerturb(&shared_pert_);
  pert_active_ = false;
}

}  // namespace rtc
