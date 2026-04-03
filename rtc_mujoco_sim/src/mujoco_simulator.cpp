// ── mujoco_simulator.cpp ───────────────────────────────────────────────────────
// Lifecycle (ctor/dtor, Initialize, Start, Stop) and command/state I/O.
// Multi-group support: robot_response (MuJoCo physics) + fake_response (LPF).
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <set>

namespace rtc {

// ── Constructor / Destructor ───────────────────────────────────────────────────

MuJoCoSimulator::MuJoCoSimulator(Config cfg) noexcept
    : cfg_(std::move(cfg)) {
  current_max_rtf_.store(cfg_.max_rtf, std::memory_order_relaxed);
  mjv_defaultPerturb(&shared_pert_);
}

MuJoCoSimulator::~MuJoCoSimulator() {
  Stop();
  if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
  if (model_) { mj_deleteModel(model_); model_ = nullptr; }
}

// ── XML joint discovery ────────────────────────────────────────────────────────

bool MuJoCoSimulator::DiscoverAllXmlJoints() noexcept {
  all_xml_joint_names_.clear();

  for (int j = 0; j < model_->njnt; ++j) {
    if (model_->jnt_type[j] != mjJNT_HINGE) { continue; }

    bool has_actuator = false;
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT &&
          model_->actuator_trnid[2 * a] == j) {
        has_actuator = true;
        break;
      }
    }
    if (!has_actuator) { continue; }

    const char* name = mj_id2name(model_, mjOBJ_JOINT, j);
    all_xml_joint_names_.emplace_back(name ? name : "(unnamed)");
  }

  if (all_xml_joint_names_.empty()) {
    fprintf(stderr,
            "[MuJoCoSimulator] ERROR: No hinge joints with actuators found in XML\n");
    for (int j = 0; j < model_->njnt; ++j) {
      const char* name = mj_id2name(model_, mjOBJ_JOINT, j);
      fprintf(stderr, "  [%d] type=%d  %s\n", j, model_->jnt_type[j],
              name ? name : "(unnamed)");
    }
    return false;
  }

  std::string names_str;
  for (std::size_t i = 0; i < all_xml_joint_names_.size(); ++i) {
    if (i > 0) names_str += ", ";
    names_str += all_xml_joint_names_[i];
  }
  fprintf(stdout, "[MuJoCoSimulator] XML joints (%zu): [%s]\n",
          all_xml_joint_names_.size(), names_str.c_str());

  return true;
}

// ── Name-based index mapping for a single group ────────────────────────────────

bool MuJoCoSimulator::MapGroupIndices(JointGroup& group) noexcept {
  group.qpos_indices.clear();
  group.qvel_indices.clear();
  group.actuator_indices.clear();

  for (std::size_t i = 0; i < group.command_joint_names.size(); ++i) {
    const auto& jname = group.command_joint_names[i];
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, jname.c_str());
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' — command joint '%s' not found in XML\n",
              group.name.c_str(), jname.c_str());
      return false;
    }

    group.qpos_indices.push_back(model_->jnt_qposadr[jnt_id]);
    group.qvel_indices.push_back(model_->jnt_dofadr[jnt_id]);

    bool found_actuator = false;
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT &&
          model_->actuator_trnid[2 * a] == jnt_id) {
        group.actuator_indices.push_back(a);
        found_actuator = true;

        const char* act_name = mj_id2name(model_, mjOBJ_ACTUATOR, a);
        fprintf(stdout, "[MuJoCoSimulator] [%s] cmd '%s' → qpos[%d]  qvel[%d]  actuator[%d] '%s'\n",
                group.name.c_str(), jname.c_str(),
                group.qpos_indices.back(), group.qvel_indices.back(),
                a, act_name ? act_name : "(unnamed)");
        break;
      }
    }
    if (!found_actuator) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' — command joint '%s' has no actuator\n",
              group.name.c_str(), jname.c_str());
      return false;
    }
  }
  return true;
}

// ── State joint index mapping ─────────────────────────────────────────────────

bool MuJoCoSimulator::MapStateIndices(JointGroup& group) noexcept {
  group.state_qpos_indices.clear();
  group.state_qvel_indices.clear();

  for (std::size_t i = 0; i < group.state_joint_names.size(); ++i) {
    const auto& jname = group.state_joint_names[i];
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, jname.c_str());
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' — state joint '%s' not found in XML\n",
              group.name.c_str(), jname.c_str());
      return false;
    }

    group.state_qpos_indices.push_back(model_->jnt_qposadr[jnt_id]);
    group.state_qvel_indices.push_back(model_->jnt_dofadr[jnt_id]);

    fprintf(stdout, "[MuJoCoSimulator] [%s] state '%s' → qpos[%d]  qvel[%d]\n",
            group.name.c_str(), jname.c_str(),
            group.state_qpos_indices.back(), group.state_qvel_indices.back());
  }
  return true;
}

// ── Validate robot groups against XML (양방향 완전 일치) ────────────────────────

bool MuJoCoSimulator::ValidateAndMapRobotGroups() noexcept {
  // Collect all command joint names from robot groups
  std::set<std::string> cmd_joints;
  for (const auto& g : groups_) {
    if (!g->is_robot) continue;
    for (const auto& jn : g->command_joint_names) {
      if (!cmd_joints.insert(jn).second) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: duplicate command joint '%s' across robot groups\n",
                jn.c_str());
        return false;
      }
    }
  }

  // Build XML joint set
  std::set<std::string> xml_joints(all_xml_joint_names_.begin(),
                                    all_xml_joint_names_.end());

  // Check YAML → XML (command에 있는데 XML에 없음)
  bool ok = true;
  for (const auto& jn : cmd_joints) {
    if (xml_joints.find(jn) == xml_joints.end()) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: command joint '%s' not found in XML\n",
              jn.c_str());
      ok = false;
    }
  }

  // Check XML → YAML (XML에 있는데 command에 없음)
  for (const auto& jn : xml_joints) {
    if (cmd_joints.find(jn) == cmd_joints.end()) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: XML joint '%s' not covered by any robot_response command_joint_names\n",
              jn.c_str());
      ok = false;
    }
  }

  if (!ok) {
    fprintf(stderr, "[MuJoCoSimulator] command joints (%zu):", cmd_joints.size());
    for (const auto& jn : cmd_joints) fprintf(stderr, " %s", jn.c_str());
    fprintf(stderr, "\n[MuJoCoSimulator] XML joints (%zu):", xml_joints.size());
    for (const auto& jn : xml_joints) fprintf(stderr, " %s", jn.c_str());
    fprintf(stderr, "\n");
    return false;
  }

  fprintf(stdout,
          "[MuJoCoSimulator] Command joints validated: %zu command == %zu XML joints\n",
          cmd_joints.size(), xml_joints.size());

  // Map command indices for each robot group
  for (auto& g : groups_) {
    if (!g->is_robot) continue;
    if (!MapGroupIndices(*g)) return false;
  }

  return true;
}

// ── Validate and map state joints ────────────────────────────────────────────

bool MuJoCoSimulator::ValidateAndMapStateJoints() noexcept {
  std::set<std::string> xml_joints(all_xml_joint_names_.begin(),
                                    all_xml_joint_names_.end());

  for (auto& g : groups_) {
    if (!g->is_robot) continue;

    // Check all state_joint_names exist in XML
    bool ok = true;
    for (const auto& jn : g->state_joint_names) {
      if (xml_joints.find(jn) == xml_joints.end()) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: group '%s' — state joint '%s' not found in XML\n",
                g->name.c_str(), jn.c_str());
        ok = false;
      }
    }
    if (!ok) return false;

    // Warn about XML joints not in state_joint_names
    std::set<std::string> state_set(g->state_joint_names.begin(),
                                     g->state_joint_names.end());
    for (const auto& jn : all_xml_joint_names_) {
      if (state_set.find(jn) == state_set.end()) {
        fprintf(stdout,
                "[MuJoCoSimulator] WARNING: group '%s' — XML joint '%s' not in state_joint_names\n",
                g->name.c_str(), jn.c_str());
      }
    }

    // Map state indices
    if (!MapStateIndices(*g)) return false;

    fprintf(stdout,
            "[MuJoCoSimulator] [%s] state joints: %d (command: %d)\n",
            g->name.c_str(), g->num_state_joints, g->num_command_joints);
  }

  return true;
}

// ── Initialization ─────────────────────────────────────────────────────────────

bool MuJoCoSimulator::Initialize() noexcept {
  char error[1000] = {};
  model_ = mj_loadXML(cfg_.model_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    fprintf(stderr, "[MuJoCoSimulator] Failed to load '%s': %s\n",
            cfg_.model_path.c_str(), error);
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

  solver_integrator_.store(model_->opt.integrator, std::memory_order_relaxed);
  solver_type_.store(model_->opt.solver, std::memory_order_relaxed);
  solver_iterations_.store(model_->opt.iterations, std::memory_order_relaxed);
  solver_tolerance_.store(static_cast<double>(model_->opt.tolerance),
                          std::memory_order_relaxed);

  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);
  ext_xfrc_.assign(static_cast<std::size_t>(model_->nbody) * 6, 0.0);

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
                "[MuJoCoSimulator] ERROR: group '%s' exists in both robot_response and fake_response\n",
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
              "[MuJoCoSimulator] ERROR: group '%s' has no command_joint_names (nor joint_names)\n",
              gc.name.c_str());
      return false;
    }
    if (gc.command_topic.empty() || gc.state_topic.empty()) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: group '%s' has empty command_topic or state_topic\n",
              gc.name.c_str());
      return false;
    }

    auto g = std::make_unique<JointGroup>();
    g->name                = gc.name;
    g->command_joint_names = cmd_names;
    g->num_command_joints  = static_cast<int>(cmd_names.size());
    g->is_robot            = gc.is_robot;
    g->command_topic       = gc.command_topic;
    g->state_topic         = gc.state_topic;
    g->filter_alpha        = gc.filter_alpha;

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
      // Fake group: LPF buffers (command size for target, state size for output)
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

  // ── Physics timestep ────────────────────────────────────────────────────
  xml_timestep_ = static_cast<double>(model_->opt.timestep);

  if (cfg_.physics_timestep > 0.0) {
    constexpr double kEps = 1e-9;
    if (std::abs(cfg_.physics_timestep - xml_timestep_) > kEps) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: physics_timestep mismatch — "
              "yaml=%.6f s (%.1f Hz)  XML=%.6f s (%.1f Hz) → XML value used\n",
              cfg_.physics_timestep, 1.0 / cfg_.physics_timestep,
              xml_timestep_,         1.0 / xml_timestep_);
    } else {
      fprintf(stdout,
              "[MuJoCoSimulator] physics_timestep OK: %.6f s (%.1f Hz)\n",
              xml_timestep_, 1.0 / xml_timestep_);
    }
  }

  // ── Substep validation & timestep override ──────────────────────────────
  if (cfg_.n_substeps < 1) {
    fprintf(stderr,
            "[MuJoCoSimulator] ERROR: n_substeps=%d must be >= 1, "
            "falling back to 1\n", cfg_.n_substeps);
    cfg_.n_substeps = 1;
  }

  if (cfg_.n_substeps > 1) {
    const double substep_dt =
        xml_timestep_ / static_cast<double>(cfg_.n_substeps);
    model_->opt.timestep = static_cast<mjtNum>(substep_dt);
    fprintf(stdout,
            "[MuJoCoSimulator] Substepping: n_substeps=%d  "
            "control_period=%.4f s (%.1f Hz)  substep_dt=%.6f s (%.1f Hz)\n",
            cfg_.n_substeps,
            xml_timestep_, 1.0 / xml_timestep_,
            substep_dt, 1.0 / substep_dt);
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
            "falling back to 60.0\n", cfg_.viewer_refresh_rate);
    cfg_.viewer_refresh_rate = 60.0;
  }
  {
    const double interval =
        1.0 / (xml_timestep_ * cfg_.viewer_refresh_rate);
    viz_update_interval_ = std::max(static_cast<uint64_t>(1),
        static_cast<uint64_t>(std::round(interval)));
    viewer_sleep_ms_ = std::max(1,
        static_cast<int>(std::round(1000.0 / cfg_.viewer_refresh_rate)));
    fprintf(stdout,
            "[MuJoCoSimulator] Viewer: target=%.0f Hz  "
            "viz_update_interval=%lu steps  render_sleep=%d ms\n",
            cfg_.viewer_refresh_rate,
            static_cast<unsigned long>(viz_update_interval_),
            viewer_sleep_ms_);
  }

  // ── Per-group servo gains & initial positions ───────────────────────────
  for (auto& g : groups_) {
    if (!g->is_robot) continue;

    const auto ncj = static_cast<std::size_t>(g->num_command_joints);

    // Servo gains: use per-group if provided, otherwise inherit global
    const auto& kp = (!cfg_.groups.empty()) ? [&]() -> const std::vector<double>& {
      for (const auto& gc : cfg_.groups) {
        if (gc.name == g->name && !gc.servo_kp.empty()) return gc.servo_kp;
      }
      return cfg_.servo_kp;
    }() : cfg_.servo_kp;

    const auto& kd = (!cfg_.groups.empty()) ? [&]() -> const std::vector<double>& {
      for (const auto& gc : cfg_.groups) {
        if (gc.name == g->name && !gc.servo_kd.empty()) return gc.servo_kd;
      }
      return cfg_.servo_kd;
    }() : cfg_.servo_kd;

    if (cfg_.use_yaml_servo_gains) {
      if (kp.size() != ncj || kd.size() != ncj) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: group '%s' servo_kp/kd size (%zu/%zu) != command joints (%zu)\n",
                g->name.c_str(), kp.size(), kd.size(), ncj);
        return false;
      }
    }

    for (std::size_t i = 0; i < ncj; ++i) {
      if (i < kp.size()) g->gainprm_yaml[i] = kp[i] / xml_timestep_;
      if (i < kd.size()) g->biasprm2_yaml[i] = -kd[i];
    }

    // Initial positions from keyframe
    if (model_->nkey > 0) {
      for (std::size_t i = 0; i < ncj; ++i) {
        g->initial_qpos[i] = static_cast<double>(
            model_->key_qpos[g->qpos_indices[i]]);
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
  gravity_locked_by_servo_.store(true,  std::memory_order_relaxed);
  gravity_enabled_.store(false, std::memory_order_relaxed);

  // ── Summary ─────────────────────────────────────────────────────────────
  int total_cmd_joints = 0;
  for (const auto& g : groups_) {
    if (g->is_robot) total_cmd_joints += g->num_command_joints;
    fprintf(stdout, "[MuJoCoSimulator] Group '%s': %s  cmd_joints=%d  state_joints=%d  cmd=%s  state=%s%s\n",
            g->name.c_str(),
            g->is_robot ? "ROBOT" : "FAKE",
            g->num_command_joints, g->num_state_joints,
            g->command_topic.c_str(),
            g->state_topic.c_str(),
            g->is_primary ? "  [PRIMARY]" : "");
  }

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d nv=%d nu=%d  groups=%zu"
          "  command_joints=%d  dt=%.4f s\n",
          cfg_.model_path.c_str(),
          model_->nq, model_->nv, model_->nu,
          groups_.size(), total_cmd_joints,
          xml_timestep_);

  return true;
}

// ── Thread lifecycle ───────────────────────────────────────────────────────────

void MuJoCoSimulator::Start() noexcept {
  if (running_.exchange(true)) { return; }

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

// ── Group-indexed Command / State I/O ───────────────────────────────────────────

void MuJoCoSimulator::SetCommand(std::size_t group_idx,
                                  const std::vector<double>& cmd) noexcept {
  if (group_idx >= groups_.size()) return;
  auto& g = *groups_[group_idx];
  if (!g.is_robot) return;
  { std::lock_guard lock(g.cmd_mutex); g.pending_cmd = cmd; }
  g.cmd_pending.store(true, std::memory_order_release);
  if (g.is_primary) {
    sync_cv_.notify_one();
  }
}

void MuJoCoSimulator::SetStateCallback(std::size_t group_idx,
                                        StateCallback cb) noexcept {
  if (group_idx >= groups_.size()) return;
  groups_[group_idx]->state_cb = std::move(cb);
}

std::vector<double> MuJoCoSimulator::GetPositions(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.state_mutex);
  return g.positions;
}

std::vector<double> MuJoCoSimulator::GetVelocities(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.state_mutex);
  return g.velocities;
}

std::vector<double> MuJoCoSimulator::GetEfforts(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.state_mutex);
  return g.efforts;
}

const std::vector<std::string>& MuJoCoSimulator::GetJointNames(
    std::size_t group_idx) const noexcept {
  static const std::vector<std::string> empty;
  if (group_idx >= groups_.size()) return empty;
  return groups_[group_idx]->command_joint_names;
}

const std::vector<std::string>& MuJoCoSimulator::GetStateJointNames(
    std::size_t group_idx) const noexcept {
  static const std::vector<std::string> empty;
  if (group_idx >= groups_.size()) return empty;
  return groups_[group_idx]->state_joint_names;
}

int MuJoCoSimulator::NumGroupJoints(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return 0;
  return groups_[group_idx]->num_command_joints;
}

int MuJoCoSimulator::NumStateJoints(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return 0;
  return groups_[group_idx]->num_state_joints;
}

bool MuJoCoSimulator::IsGroupRobot(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return false;
  return groups_[group_idx]->is_robot;
}

// ── Per-group control mode ──────────────────────────────────────────────────────

void MuJoCoSimulator::SetControlMode(std::size_t group_idx,
                                      bool torque_mode) noexcept {
  if (group_idx >= groups_.size()) return;
  auto& g = *groups_[group_idx];
  if (!g.is_robot) return;

  g.torque_mode.store(torque_mode, std::memory_order_relaxed);
  g.control_mode_pending.store(true, std::memory_order_release);

  if (!torque_mode) {
    gravity_enabled_.store(false, std::memory_order_relaxed);
    gravity_locked_by_servo_.store(true, std::memory_order_relaxed);
  } else {
    // Check if any other robot group is still in position servo
    bool any_servo = false;
    for (const auto& og : groups_) {
      if (!og->is_robot) continue;
      if (og.get() == &g) continue;
      if (!og->torque_mode.load(std::memory_order_relaxed)) {
        any_servo = true;
        break;
      }
    }
    if (!any_servo) {
      gravity_locked_by_servo_.store(false, std::memory_order_relaxed);
      gravity_enabled_.store(true, std::memory_order_relaxed);
    }
  }
}

bool MuJoCoSimulator::IsInTorqueMode(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return false;
  return groups_[group_idx]->torque_mode.load(std::memory_order_relaxed);
}

// ── Fake response API ──────────────────────────────────────────────────────────

void MuJoCoSimulator::SetFakeTarget(std::size_t group_idx,
                                     const std::vector<double>& target) noexcept {
  if (group_idx >= groups_.size()) return;
  auto& g = *groups_[group_idx];
  if (g.is_robot) return;
  std::lock_guard lock(g.fake_mutex);
  const auto n = std::min(target.size(), g.fake_target.size());
  std::copy_n(target.begin(), n, g.fake_target.begin());
}

void MuJoCoSimulator::AdvanceFakeLPF(std::size_t group_idx) noexcept {
  if (group_idx >= groups_.size()) return;
  auto& g = *groups_[group_idx];
  if (g.is_robot) return;
  std::lock_guard lock(g.fake_mutex);
  const auto n = std::min(g.fake_state.size(), g.fake_target.size());
  for (std::size_t i = 0; i < n; ++i) {
    g.fake_state[i] += g.filter_alpha * (g.fake_target[i] - g.fake_state[i]);
  }
}

std::vector<double> MuJoCoSimulator::GetFakeState(std::size_t group_idx) const noexcept {
  if (group_idx >= groups_.size()) return {};
  auto& g = *groups_[group_idx];
  std::lock_guard lock(g.fake_mutex);
  return g.fake_state;
}

// ── Solver statistics ──────────────────────────────────────────────────────────

MuJoCoSimulator::SolverStats MuJoCoSimulator::GetSolverStats() const noexcept {
  std::lock_guard lock(solver_stats_mutex_);
  return latest_solver_stats_;
}

// ── External forces / perturbation ────────────────────────────────────────────

void MuJoCoSimulator::SetExternalForce(
    int body_id, const std::array<double, 6>& wrench_world) noexcept {
  if (body_id <= 0 || body_id >= model_->nbody) { return; }
  std::lock_guard lock(pert_mutex_);
  const std::size_t offset = static_cast<std::size_t>(body_id) * 6;
  for (std::size_t i = 0; i < 6; ++i) {
    ext_xfrc_[offset + i] = wrench_world[i];
  }
  ext_xfrc_dirty_ = true;
}

void MuJoCoSimulator::ClearExternalForce() noexcept {
  std::lock_guard lock(pert_mutex_);
  std::fill(ext_xfrc_.begin(), ext_xfrc_.end(), 0.0);
  ext_xfrc_dirty_ = false;
}

void MuJoCoSimulator::UpdatePerturb(const mjvPerturb& pert) noexcept {
  std::lock_guard lock(pert_mutex_);
  shared_pert_  = pert;
  pert_active_  = (pert.active != 0);
}

void MuJoCoSimulator::ClearPerturb() noexcept {
  std::lock_guard lock(pert_mutex_);
  mjv_defaultPerturb(&shared_pert_);
  pert_active_ = false;
}

}  // namespace rtc
