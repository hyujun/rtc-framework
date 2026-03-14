// ── mujoco_simulator.cpp ───────────────────────────────────────────────────────
// Lifecycle (ctor/dtor, Initialize, Start, Stop) and command/state I/O.
// ──────────────────────────────────────────────────────────────────────────────
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"

#include <algorithm>
#include <cstdio>

namespace ur5e_rt_controller {

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

// ── Initialization ─────────────────────────────────────────────────────────────

// XML에서 hinge joint + actuator 연결된 조인트를 자동 발견
bool MuJoCoSimulator::DiscoverRobotJoints() noexcept {
  joint_names_.clear();
  joint_qpos_indices_.clear();
  joint_qvel_indices_.clear();

  for (int j = 0; j < model_->njnt; ++j) {
    // hinge joint만 필터
    if (model_->jnt_type[j] != mjJNT_HINGE) { continue; }

    // 이 joint에 연결된 actuator가 있는지 확인
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
    joint_names_.emplace_back(name ? name : "(unnamed)");
    joint_qpos_indices_.push_back(model_->jnt_qposadr[j]);
    joint_qvel_indices_.push_back(model_->jnt_dofadr[j]);

    // actuator 정보 출력
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT &&
          model_->actuator_trnid[2 * a] == j) {
        const char* act_name = mj_id2name(model_, mjOBJ_ACTUATOR, a);
        fprintf(stdout, "[MuJoCoSimulator] '%s' → qpos[%d]  qvel[%d]  actuator[%d] '%s'\n",
                joint_names_.back().c_str(),
                joint_qpos_indices_.back(),
                joint_qvel_indices_.back(),
                a, act_name ? act_name : "(unnamed)");
        break;
      }
    }
  }

  num_robot_joints_ = static_cast<int>(joint_names_.size());

  if (num_robot_joints_ == 0) {
    fprintf(stderr,
            "[MuJoCoSimulator] ERROR: No hinge joints with actuators found in XML model\n");
    fprintf(stderr, "[MuJoCoSimulator] Available XML joints:\n");
    for (int j = 0; j < model_->njnt; ++j) {
      const char* name = mj_id2name(model_, mjOBJ_JOINT, j);
      fprintf(stderr, "  [%d] type=%d  %s\n", j, model_->jnt_type[j],
              name ? name : "(unnamed)");
    }
    return false;
  }

  // 조인트 이름 목록 출력
  std::string names_str;
  for (int i = 0; i < num_robot_joints_; ++i) {
    if (i > 0) names_str += ", ";
    names_str += joint_names_[static_cast<std::size_t>(i)];
  }
  fprintf(stdout, "[MuJoCoSimulator] Found %d robot joints from XML: [%s]\n",
          num_robot_joints_, names_str.c_str());

  return true;
}

bool MuJoCoSimulator::ResolveJointIndices(
    const std::vector<std::string>& names) noexcept {
  if (names.empty()) {
    fprintf(stderr,
            "[MuJoCoSimulator] WARN: ResolveJointIndices called with empty names\n");
    return false;
  }

  if (static_cast<int>(names.size()) != num_robot_joints_) {
    fprintf(stderr,
            "[MuJoCoSimulator] ERROR: joint names has %zu entries "
            "(expected %d from XML)\n", names.size(), num_robot_joints_);
    return false;
  }

  // msg 이름이 XML에서 발견된 joint_names_에 존재하는지 비교 검증
  bool all_ok = true;
  std::vector<int> new_qpos(static_cast<std::size_t>(num_robot_joints_));
  std::vector<int> new_qvel(static_cast<std::size_t>(num_robot_joints_));

  for (std::size_t i = 0; i < names.size(); ++i) {
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, names[i].c_str());
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: joint '%s' not found in XML model\n",
              names[i].c_str());
      all_ok = false;
      new_qpos[i] = static_cast<int>(i);
      new_qvel[i] = static_cast<int>(i);
    } else {
      // msg 이름이 XML 발견 목록에 있는지 확인
      auto it = std::find(joint_names_.begin(), joint_names_.end(), names[i]);
      if (it == joint_names_.end()) {
        fprintf(stderr,
                "[MuJoCoSimulator] ERROR: joint '%s' exists in XML but is not "
                "a robot joint (hinge+actuator)\n", names[i].c_str());
        all_ok = false;
      }
      new_qpos[i] = model_->jnt_qposadr[jnt_id];
      new_qvel[i] = model_->jnt_dofadr[jnt_id];
      fprintf(stdout, "[MuJoCoSimulator] ResolveJointIndices: '%s' → qpos[%d]  qvel[%d]\n",
              names[i].c_str(), new_qpos[i], new_qvel[i]);
    }
  }

  if (all_ok) {
    joint_qpos_indices_ = std::move(new_qpos);
    joint_qvel_indices_ = std::move(new_qvel);
    // 이름 순서도 msg 순서로 갱신
    joint_names_.assign(names.begin(), names.end());
  } else {
    fprintf(stderr, "[MuJoCoSimulator] Available robot joints from XML:\n");
    for (const auto& n : joint_names_) {
      fprintf(stderr, "  %s\n", n.c_str());
    }
  }

  return all_ok;
}

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

  // Store original gravity for toggle
  original_gravity_z_ = static_cast<double>(model_->opt.gravity[2]);

  // Sync atomics with values loaded from XML (or MuJoCo internal defaults)
  solver_integrator_.store(model_->opt.integrator, std::memory_order_relaxed);
  solver_type_.store(model_->opt.solver, std::memory_order_relaxed);
  solver_iterations_.store(model_->opt.iterations, std::memory_order_relaxed);
  solver_tolerance_.store(static_cast<double>(model_->opt.tolerance),
                          std::memory_order_relaxed);

  // Pre-size external force buffer
  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);
  ext_xfrc_.assign(static_cast<std::size_t>(model_->nbody) * 6, 0.0);

  // Save original actuator gain/bias params so SetControlMode() can restore them.
  orig_actuator_params_.resize(static_cast<std::size_t>(model_->nu));
  for (int i = 0; i < model_->nu; ++i) {
    auto & p = orig_actuator_params_[static_cast<std::size_t>(i)];
    p.gainprm0 = static_cast<double>(model_->actuator_gainprm[i * mjNGAIN + 0]);
    p.biasprm0 = static_cast<double>(model_->actuator_biasprm[i * mjNBIAS + 0]);
    p.biasprm1 = static_cast<double>(model_->actuator_biasprm[i * mjNBIAS + 1]);
    p.biasprm2 = static_cast<double>(model_->actuator_biasprm[i * mjNBIAS + 2]);
  }

  // ── XML 기반 로봇 조인트 자동 발견 ────────────────────────────────────────
  if (!DiscoverRobotJoints()) {
    return false;
  }

  // ── 동적 버퍼 resize ──────────────────────────────────────────────────────
  const auto nj = static_cast<std::size_t>(num_robot_joints_);
  pending_cmd_.resize(nj, 0.0);
  latest_positions_.resize(nj, 0.0);
  latest_velocities_.resize(nj, 0.0);
  latest_efforts_.resize(nj, 0.0);
  initial_qpos_.resize(nj, 0.0);
  gainprm_yaml_.resize(nj, 0.0);
  biasprm2_yaml_.resize(nj, 0.0);

  // ── Initial positions: XML keyframe → 0 fallback ──────────────────────
  if (model_->nkey > 0) {
    // 첫 번째 keyframe 사용
    for (std::size_t i = 0; i < nj; ++i) {
      initial_qpos_[i] = static_cast<double>(
          model_->key_qpos[joint_qpos_indices_[i]]);
    }
    const char* key_name = mj_id2name(model_, mjOBJ_KEY, 0);
    fprintf(stdout, "[MuJoCoSimulator] Initial positions from XML keyframe '%s'\n",
            key_name ? key_name : "(unnamed)");
  } else {
    std::fill(initial_qpos_.begin(), initial_qpos_.end(), 0.0);
    fprintf(stdout, "[MuJoCoSimulator] No keyframe in XML — using zero initial positions\n");
  }

  for (int i = 0; i < num_robot_joints_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    data_->qpos[joint_qpos_indices_[ui]] = initial_qpos_[ui];
    data_->ctrl[i] = initial_qpos_[ui];
  }
  mj_forward(model_, data_);
  ReadState();

  // ── Physics timestep 검증 ────────────────────────────────────────────────
  xml_timestep_ = static_cast<double>(model_->opt.timestep);

  if (cfg_.physics_timestep > 0.0) {
    constexpr double kEps = 1e-9;
    if (std::abs(cfg_.physics_timestep - xml_timestep_) > kEps) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: physics_timestep 불일치 — "
              "yaml=%.6f s (%.1f Hz)  XML=%.6f s (%.1f Hz) → XML 값 적용\n",
              cfg_.physics_timestep, 1.0 / cfg_.physics_timestep,
              xml_timestep_,         1.0 / xml_timestep_);
      // model_->opt.timestep은 XML 값 유지 (변경 없음)
    } else {
      fprintf(stdout,
              "[MuJoCoSimulator] physics_timestep OK: %.6f s (%.1f Hz)\n",
              xml_timestep_, 1.0 / xml_timestep_);
    }
  }

  // ── YAML servo gain 크기 검증 + 사전 계산 ──────────────────────────────────
  if (cfg_.use_yaml_servo_gains) {
    if (static_cast<int>(cfg_.servo_kp.size()) != num_robot_joints_ ||
        static_cast<int>(cfg_.servo_kd.size()) != num_robot_joints_) {
      fprintf(stderr,
              "[MuJoCoSimulator] ERROR: servo_kp/kd size (%zu/%zu) != "
              "XML robot joints (%d). Set use_yaml_servo_gains: false or "
              "adjust servo_kp/kd array size.\n",
              cfg_.servo_kp.size(), cfg_.servo_kd.size(), num_robot_joints_);
      return false;
    }
  }

  // gainprm = servo_kp / timestep  →  force = servo_kp * dq_cmd - servo_kd * dq_actual
  for (std::size_t i = 0; i < nj; ++i) {
    if (i < cfg_.servo_kp.size()) {
      gainprm_yaml_[i]  = cfg_.servo_kp[i] / xml_timestep_;
    }
    if (i < cfg_.servo_kd.size()) {
      biasprm2_yaml_[i] = -cfg_.servo_kd[i];
    }
  }

  // ── Initial gravity state: position servo → gravity OFF (잠금) ──────────
  gravity_locked_by_servo_.store(true,  std::memory_order_relaxed);
  gravity_enabled_.store(false, std::memory_order_relaxed);

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d  nv=%d  nu=%d  nbody=%d"
          "  robot_joints=%d  dt=%.4f s  mode=%s\n"
          "[MuJoCoSimulator] Servo gains: %s  "
          "kp0=%.1f kd0=%.1f  gainprm0=%.1f\n",
          cfg_.model_path.c_str(),
          model_->nq, model_->nv, model_->nu, model_->nbody,
          num_robot_joints_,
          xml_timestep_,
          cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step",
          cfg_.use_yaml_servo_gains ? "YAML" : "XML",
          cfg_.servo_kp.empty() ? 0.0 : cfg_.servo_kp[0],
          cfg_.servo_kd.empty() ? 0.0 : cfg_.servo_kd[0],
          gainprm_yaml_.empty() ? 0.0 : gainprm_yaml_[0]);
  return true;
}

// ── Thread lifecycle ───────────────────────────────────────────────────────────

void MuJoCoSimulator::Start() noexcept {
  if (running_.exchange(true)) { return; }

  if (cfg_.mode == SimMode::kFreeRun) {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopFreeRun(st); });
  } else {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopSyncStep(st); });
  }
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

// ── Command / State I/O ────────────────────────────────────────────────────────

void MuJoCoSimulator::SetCommand(const std::vector<double>& cmd) noexcept {
  { std::lock_guard lock(cmd_mutex_); pending_cmd_ = cmd; }
  cmd_pending_.store(true, std::memory_order_release);
  if (cfg_.mode == SimMode::kSyncStep) { sync_cv_.notify_one(); }
}

void MuJoCoSimulator::SetStateCallback(StateCallback cb) noexcept {
  state_cb_ = std::move(cb);
}

std::vector<double> MuJoCoSimulator::GetPositions() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_positions_;
}
std::vector<double> MuJoCoSimulator::GetVelocities() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_velocities_;
}
std::vector<double> MuJoCoSimulator::GetEfforts() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_efforts_;
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

}  // namespace ur5e_rt_controller
