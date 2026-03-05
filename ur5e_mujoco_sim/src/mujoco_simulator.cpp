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

void MuJoCoSimulator::ResolveJointIndices() noexcept {
  for (std::size_t i = 0; i < 6; ++i) {
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, kMjJointNames[i]);
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] Joint '%s' not found — using static index %zu\n",
              kMjJointNames[i], i);
      joint_qpos_indices_[i] = static_cast<int>(i);
      joint_qvel_indices_[i] = static_cast<int>(i);
    } else {
      joint_qpos_indices_[i] = model_->jnt_qposadr[jnt_id];
      joint_qvel_indices_[i] = model_->jnt_dofadr[jnt_id];
      fprintf(stdout, "[MuJoCoSimulator] '%s' → qpos[%d]  qvel[%d]\n",
              kMjJointNames[i], joint_qpos_indices_[i], joint_qvel_indices_[i]);
    }
  }
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

  // Apply initial solver configuration from Config
  model_->opt.integrator  = static_cast<mjtIntegrator>(cfg_.integrator_type);
  model_->opt.solver      = static_cast<mjtSolver>(cfg_.solver_type);
  model_->opt.iterations  = cfg_.solver_iterations;
  model_->opt.tolerance   = static_cast<mjtNum>(cfg_.solver_tolerance);
  // Sync atomics with config values
  solver_integrator_.store(cfg_.integrator_type,  std::memory_order_relaxed);
  solver_type_.store(cfg_.solver_type,             std::memory_order_relaxed);
  solver_iterations_.store(cfg_.solver_iterations, std::memory_order_relaxed);
  solver_tolerance_.store(cfg_.solver_tolerance,   std::memory_order_relaxed);

  // Pre-size external force buffer
  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);
  ext_xfrc_.assign(static_cast<std::size_t>(model_->nbody) * 6, 0.0);

  ResolveJointIndices();

  const int njoints = std::min(6, model_->nq);
  for (int i = 0; i < njoints; ++i) {
    const double q0 = cfg_.initial_qpos[static_cast<std::size_t>(i)];
    data_->qpos[joint_qpos_indices_[static_cast<std::size_t>(i)]] = q0;
    data_->ctrl[i] = q0;
  }
  mj_forward(model_, data_);
  ReadState();

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d  nv=%d  nu=%d  nbody=%d"
          "  dt=%.4f s  mode=%s\n",
          cfg_.model_path.c_str(),
          model_->nq, model_->nv, model_->nu, model_->nbody,
          static_cast<double>(model_->opt.timestep),
          cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step");
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

void MuJoCoSimulator::SetCommand(const std::array<double, 6>& cmd) noexcept {
  { std::lock_guard lock(cmd_mutex_); pending_cmd_ = cmd; }
  cmd_pending_.store(true, std::memory_order_release);
  if (cfg_.mode == SimMode::kSyncStep) { sync_cv_.notify_one(); }
}

void MuJoCoSimulator::SetStateCallback(StateCallback cb) noexcept {
  state_cb_ = std::move(cb);
}

std::array<double, 6> MuJoCoSimulator::GetPositions() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_positions_;
}
std::array<double, 6> MuJoCoSimulator::GetVelocities() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_velocities_;
}
std::array<double, 6> MuJoCoSimulator::GetEfforts() const noexcept {
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
