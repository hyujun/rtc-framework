// ── mujoco_sim_loop.cpp ────────────────────────────────────────────────────────
// Physics helpers (ReadState, PreparePhysicsStep, RTF, etc.) and the
// synchronous simulation loop (SimLoop).
// Multi-group: iterates over robot groups for command/state/actuator ops.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace rtc {

// ── Private helpers ────────────────────────────────────────────────────────────

void MuJoCoSimulator::ApplyCommand() noexcept {
  if (!model_) { return; }
  for (auto& g : groups_) {
    if (!g->is_robot) continue;
    if (!g->cmd_pending.load(std::memory_order_acquire)) continue;

    std::lock_guard lock(g->cmd_mutex);
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_command_joints); ++i) {
      if (i < g->pending_cmd.size() && i < g->actuator_indices.size()) {
        data_->ctrl[g->actuator_indices[i]] = g->pending_cmd[i];
      }
    }
    g->cmd_pending.store(false, std::memory_order_release);
  }
}

void MuJoCoSimulator::ReadState() noexcept {
  if (!model_ || !data_) { return; }
  for (auto& g : groups_) {
    if (!g->is_robot) continue;
    std::lock_guard lock(g->state_mutex);
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_state_joints); ++i) {
      g->positions[i]  = data_->qpos[g->state_qpos_indices[i]];
      g->velocities[i] = data_->qvel[g->state_qvel_indices[i]];
      g->efforts[i]    = data_->qfrc_actuator[g->state_qvel_indices[i]];
    }
  }
}

void MuJoCoSimulator::ReadSolverStats() noexcept {
  if (!data_) { return; }
  SolverStats s{};
  s.ncon = data_->ncon;
  const int nisland = data_->nisland;
  for (int k = 0; k < nisland; ++k) {
    s.iter += data_->solver_niter[k];
  }
  if (nisland > 0 && s.iter > 0) {
    s.improvement = static_cast<double>(data_->solver[0].improvement);
    s.gradient    = static_cast<double>(data_->solver[0].gradient);
  }
  std::lock_guard lock(solver_stats_mutex_);
  latest_solver_stats_ = s;
}

void MuJoCoSimulator::InvokeStateCallback() noexcept {
  for (auto& g : groups_) {
    if (!g->is_robot) continue;
    if (!g->state_cb) continue;
    std::vector<double> pos, vel, eff;
    {
      std::lock_guard lock(g->state_mutex);
      pos = g->positions;
      vel = g->velocities;
      eff = g->efforts;
    }
    g->state_cb(pos, vel, eff);
  }
}

void MuJoCoSimulator::UpdateVizBuffer() noexcept {
  if (viz_mutex_.try_lock()) {
    std::memcpy(viz_qpos_.data(), data_->qpos,
                static_cast<std::size_t>(model_->nq) * sizeof(double));
    viz_ncon_  = data_->ncon;
    viz_dirty_ = true;
    viz_mutex_.unlock();
  }
}

void MuJoCoSimulator::UpdateRtf(uint64_t step) noexcept {
  if (step % 200 != 0) { return; }
  const auto   wall_now = std::chrono::steady_clock::now();
  const double wall_dt  =
      std::chrono::duration<double>(wall_now - rtf_wall_start_).count();
  const double sim_dt   = data_->time - rtf_sim_start_;
  if (wall_dt > 0.01) {
    rtf_.store(sim_dt / wall_dt, std::memory_order_relaxed);
    rtf_wall_start_ = wall_now;
    rtf_sim_start_  = data_->time;
  }
}

void MuJoCoSimulator::ThrottleIfNeeded() noexcept {
  const double max_rtf = current_max_rtf_.load(std::memory_order_relaxed);
  if (max_rtf != throttle_rtf_) {
    throttle_wall_start_ = std::chrono::steady_clock::now();
    throttle_sim_start_  = data_->time;
    throttle_rtf_        = max_rtf;
  }
  if (max_rtf <= 0.0) { return; }
  const double sim_elapsed = data_->time - throttle_sim_start_;
  const double target_wall = sim_elapsed / max_rtf;
  const double actual_wall = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - throttle_wall_start_).count();
  if (actual_wall < target_wall) {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(target_wall - actual_wall));
  }
}

// ── PreparePhysicsStep ─────────────────────────────────────────────────────────

void MuJoCoSimulator::PreparePhysicsStep() noexcept {
  // 0. Per-group actuator mode switch (torque ↔ position servo)
  for (auto& g : groups_) {
    if (!g->is_robot) continue;
    if (!g->control_mode_pending.exchange(false, std::memory_order_acq_rel)) continue;

    const bool torque = g->torque_mode.load(std::memory_order_relaxed);
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_command_joints); ++i) {
      const int act = g->actuator_indices[i];
      if (torque) {
        model_->actuator_gainprm[act * mjNGAIN + 0] = static_cast<mjtNum>(1.0);
        model_->actuator_biasprm[act * mjNBIAS + 0] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[act * mjNBIAS + 1] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[act * mjNBIAS + 2] = static_cast<mjtNum>(0.0);
      } else if (cfg_.use_yaml_servo_gains) {
        const double kp = g->gainprm_yaml[i];
        model_->actuator_gainprm[act * mjNGAIN + 0] = static_cast<mjtNum>(kp);
        model_->actuator_biasprm[act * mjNBIAS + 0] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[act * mjNBIAS + 1] = static_cast<mjtNum>(-kp);
        model_->actuator_biasprm[act * mjNBIAS + 2] =
            static_cast<mjtNum>(g->biasprm2_yaml[i]);
      } else {
        const auto& p = orig_actuator_params_[static_cast<std::size_t>(act)];
        model_->actuator_gainprm[act * mjNGAIN + 0] = static_cast<mjtNum>(p.gainprm0);
        model_->actuator_biasprm[act * mjNBIAS + 0] = static_cast<mjtNum>(p.biasprm0);
        model_->actuator_biasprm[act * mjNBIAS + 1] = static_cast<mjtNum>(p.biasprm1);
        model_->actuator_biasprm[act * mjNBIAS + 2] = static_cast<mjtNum>(p.biasprm2);
      }
    }
  }

  // 1. Physics solver parameters
  model_->opt.integrator =
      static_cast<mjtIntegrator>(solver_integrator_.load(std::memory_order_relaxed));
  model_->opt.solver =
      static_cast<mjtSolver>(solver_type_.load(std::memory_order_relaxed));
  model_->opt.iterations = solver_iterations_.load(std::memory_order_relaxed);
  model_->opt.tolerance  =
      static_cast<mjtNum>(solver_tolerance_.load(std::memory_order_relaxed));

  // 2. Contact enable / disable
  if (contacts_enabled_.load(std::memory_order_relaxed)) {
    model_->opt.disableflags &= ~mjDSBL_CONTACT;
  } else {
    model_->opt.disableflags |= mjDSBL_CONTACT;
  }

  // 3. Gravity toggle
  model_->opt.gravity[2] =
      gravity_enabled_.load(std::memory_order_relaxed)
      ? static_cast<mjtNum>(original_gravity_z_)
      : static_cast<mjtNum>(0.0);

  // 4. External forces and perturbation
  if (pert_mutex_.try_lock()) {
    if (ext_xfrc_dirty_) {
      const std::size_t n = static_cast<std::size_t>(model_->nbody) * 6;
      std::memcpy(data_->xfrc_applied, ext_xfrc_.data(), n * sizeof(double));
    } else {
      mju_zero(data_->xfrc_applied, model_->nbody * 6);
    }
    if (pert_active_ && shared_pert_.select > 0) {
      mjv_applyPerturbForce(model_, data_, &shared_pert_);
    }
    pert_mutex_.unlock();
  }
}

void MuJoCoSimulator::ClearContactForces() noexcept {
  if (!pert_active_ && !ext_xfrc_dirty_) {
    mju_zero(data_->xfrc_applied, model_->nbody * 6);
  }
}

// ── HandleReset ───────────────────────────────────────────────────────────────

void MuJoCoSimulator::HandleReset() noexcept {
  mj_resetData(model_, data_);

  for (auto& g : groups_) {
    if (!g->is_robot) continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_command_joints); ++i) {
      data_->qpos[g->qpos_indices[i]] = g->initial_qpos[i];
      data_->ctrl[g->actuator_indices[i]] = g->initial_qpos[i];
    }
  }

  model_->opt.gravity[2] =
      gravity_enabled_.load(std::memory_order_relaxed)
      ? static_cast<mjtNum>(original_gravity_z_)
      : static_cast<mjtNum>(0.0);
  mj_forward(model_, data_);
  {
    std::lock_guard lock(pert_mutex_);
    mjv_defaultPerturb(&shared_pert_);
    pert_active_ = false;
    std::fill(ext_xfrc_.begin(), ext_xfrc_.end(), 0.0);
    ext_xfrc_dirty_ = false;
  }
  step_count_.store(0,   std::memory_order_relaxed);
  sim_time_sec_.store(0.0, std::memory_order_relaxed);
  rtf_.store(0.0, std::memory_order_relaxed);
  const auto now       = std::chrono::steady_clock::now();
  rtf_wall_start_      = now;  rtf_sim_start_      = data_->time;
  throttle_wall_start_ = now;  throttle_sim_start_ = data_->time;
  throttle_rtf_        = current_max_rtf_.load(std::memory_order_relaxed);
  ReadState();
  fprintf(stdout, "[MuJoCoSimulator] Reset to initial pose\n");
}

// ── SimLoop ─────────────────────────────────────────────────────────────────────
// Synchronous simulation: publish state → wait for command → step → throttle.

void MuJoCoSimulator::SimLoop(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(cfg_.sync_timeout_ms > 0.0 ? cfg_.sync_timeout_ms : 50.0));
  uint64_t step = 0;

  // Find primary group index
  std::size_t primary_idx = 0;
  for (std::size_t i = 0; i < groups_.size(); ++i) {
    if (groups_[i]->is_primary) { primary_idx = i; break; }
  }

  const auto loop_start = std::chrono::steady_clock::now();
  rtf_wall_start_      = loop_start;  rtf_sim_start_      = data_->time;
  throttle_wall_start_ = loop_start;  throttle_sim_start_ = data_->time;
  throttle_rtf_        = current_max_rtf_.load(std::memory_order_relaxed);

  while (!stop.stop_requested() && running_.load()) {
    // ── Pause (with step_once support) ────────────────────────────────────
    if (paused_.load(std::memory_order_relaxed)) {
      {
        std::unique_lock lock(sync_mutex_);
        sync_cv_.wait_for(lock, std::chrono::milliseconds(10), [this, &stop] {
          return !paused_.load(std::memory_order_relaxed)
              || step_once_.load(std::memory_order_relaxed)
              || stop.stop_requested()
              || !running_.load();
        });
      }
      if (!step_once_.exchange(false, std::memory_order_acq_rel)) {
        continue;
      }
    }
    // ── Reset ─────────────────────────────────────────────────────────────
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // 1. Publish current state for ALL robot groups
    ReadState();
    InvokeStateCallback();

    // 2. Wait for command from PRIMARY group
    {
      std::unique_lock lock(sync_mutex_);
      sync_cv_.wait_for(lock, timeout, [this, &stop, primary_idx] {
        return groups_[primary_idx]->cmd_pending.load(std::memory_order_relaxed)
            || stop.stop_requested()
            || !running_.load()
            || reset_requested_.load(std::memory_order_relaxed);
      });
    }
    if (stop.stop_requested() || !running_.load()) { break; }
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // 3. Apply commands from ALL robot groups and step
    ApplyCommand();
    PreparePhysicsStep();
    mj_step(model_, data_);
    ClearContactForces();
    ReadSolverStats();

    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    UpdateRtf(step);
    ThrottleIfNeeded();
    if ((step % 8 == 0) && cfg_.enable_viewer) { UpdateVizBuffer(); }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] SimLoop exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()), sim_time_sec_.load());
}

}  // namespace rtc
