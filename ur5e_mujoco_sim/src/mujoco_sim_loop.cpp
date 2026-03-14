// ── mujoco_sim_loop.cpp ────────────────────────────────────────────────────────
// Physics helpers (ReadState, PreparePhysicsStep, RTF, etc.) and both
// simulation loops: SimLoopFreeRun and SimLoopSyncStep.
// ──────────────────────────────────────────────────────────────────────────────
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace ur5e_rt_controller {

// ── Private helpers ────────────────────────────────────────────────────────────

void MuJoCoSimulator::ApplyCommand() noexcept {
  std::lock_guard lock(cmd_mutex_);
  if (!model_) { return; }
  const int nact = std::min(num_robot_joints_, model_->nu);
  for (int i = 0; i < nact; ++i) {
    data_->ctrl[i] = pending_cmd_[static_cast<std::size_t>(i)];
  }
}

void MuJoCoSimulator::ReadState() noexcept {
  if (!model_ || !data_) { return; }
  std::lock_guard lock(state_mutex_);
  const auto nj = static_cast<std::size_t>(num_robot_joints_);
  for (std::size_t i = 0; i < nj; ++i) {
    latest_positions_[i]  = data_->qpos[joint_qpos_indices_[i]];
    latest_velocities_[i] = data_->qvel[joint_qvel_indices_[i]];
    // qfrc_actuator: net actuator force on each DOF (Nm for revolute joints).
    latest_efforts_[i]    = data_->qfrc_actuator[joint_qvel_indices_[i]];
  }
}

void MuJoCoSimulator::ReadSolverStats() noexcept {
  if (!data_) { return; }
  SolverStats s{};
  s.ncon = data_->ncon;
  // solver_niter is int* (one entry per constraint island in MuJoCo 3.x).
  // Sum across all islands for the total iteration count.
  const int nisland = data_->nisland;
  for (int k = 0; k < nisland; ++k) {
    s.iter += data_->solver_niter[k];
  }
  // mjSolverStat[0] holds aggregate stats for the first island.
  if (nisland > 0 && s.iter > 0) {
    s.improvement = static_cast<double>(data_->solver[0].improvement);
    s.gradient    = static_cast<double>(data_->solver[0].gradient);
  }
  std::lock_guard lock(solver_stats_mutex_);
  latest_solver_stats_ = s;
}

MuJoCoSimulator::SolverStats MuJoCoSimulator::GetSolverStats() const noexcept {
  std::lock_guard lock(solver_stats_mutex_);
  return latest_solver_stats_;
}

void MuJoCoSimulator::InvokeStateCallback() noexcept {
  if (!state_cb_) { return; }
  const auto nj = static_cast<std::size_t>(num_robot_joints_);
  std::vector<double> pos(nj), vel(nj), eff(nj);
  {
    std::lock_guard lock(state_mutex_);
    pos = latest_positions_;
    vel = latest_velocities_;
    eff = latest_efforts_;
  }
  state_cb_(pos, vel, eff);
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
//
// Called on the sim thread immediately before mj_step():
//   1. Apply gravity state (toggle on/off).
//   2. Apply user-specified external forces (SetExternalForce).
//   3. Apply mjvPerturb spring force (viewer Ctrl+drag).
//
void MuJoCoSimulator::PreparePhysicsStep() noexcept {
  // 0. Actuator mode switch (torque ↔ position servo)
  if (control_mode_pending_.exchange(false, std::memory_order_acq_rel)) {
    const bool torque = torque_mode_.load(std::memory_order_relaxed);
    const int nact = std::min(num_robot_joints_, model_->nu);
    for (int i = 0; i < nact; ++i) {
      const std::size_t ui = static_cast<std::size_t>(i);
      if (torque) {
        // Direct torque: gainprm=1, bias=0 → ctrl[i] = raw torque (Nm)
        model_->actuator_gainprm[i * mjNGAIN + 0] = static_cast<mjtNum>(1.0);
        model_->actuator_biasprm[i * mjNBIAS + 0] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[i * mjNBIAS + 1] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[i * mjNBIAS + 2] = static_cast<mjtNum>(0.0);
      } else if (cfg_.use_yaml_servo_gains) {
        // Position servo with YAML velocity gains:
        //   gainprm = servo_kp / physics_timestep
        //   force   = gainprm*(ctrl-q) - servo_kd*dq
        //           = servo_kp * dq_cmd - servo_kd * dq_actual
        const double kp = gainprm_yaml_[ui];
        model_->actuator_gainprm[i * mjNGAIN + 0] = static_cast<mjtNum>(kp);
        model_->actuator_biasprm[i * mjNBIAS + 0] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[i * mjNBIAS + 1] = static_cast<mjtNum>(-kp);
        model_->actuator_biasprm[i * mjNBIAS + 2] =
            static_cast<mjtNum>(biasprm2_yaml_[ui]);
      } else {
        // Position servo with original XML gainprm/biasprm
        const auto & p = orig_actuator_params_[ui];
        model_->actuator_gainprm[i * mjNGAIN + 0] = static_cast<mjtNum>(p.gainprm0);
        model_->actuator_biasprm[i * mjNBIAS + 0] = static_cast<mjtNum>(p.biasprm0);
        model_->actuator_biasprm[i * mjNBIAS + 1] = static_cast<mjtNum>(p.biasprm1);
        model_->actuator_biasprm[i * mjNBIAS + 2] = static_cast<mjtNum>(p.biasprm2);
      }
    }
  }

  // 1. Physics solver parameters (integrator, solver type, iterations, tolerance)
  model_->opt.integrator =
      static_cast<mjtIntegrator>(solver_integrator_.load(std::memory_order_relaxed));
  model_->opt.solver =
      static_cast<mjtSolver>(solver_type_.load(std::memory_order_relaxed));
  model_->opt.iterations = solver_iterations_.load(std::memory_order_relaxed);
  model_->opt.tolerance  =
      static_cast<mjtNum>(solver_tolerance_.load(std::memory_order_relaxed));

  // 2. Contact enable / disable (mjDSBL_CONTACT flag)
  if (contacts_enabled_.load(std::memory_order_relaxed)) {
    model_->opt.disableflags &= ~mjDSBL_CONTACT;
  } else {
    model_->opt.disableflags |= mjDSBL_CONTACT;
  }

  // 3. Gravity toggle (cheap relaxed load)
  model_->opt.gravity[2] =
      gravity_enabled_.load(std::memory_order_relaxed)
      ? static_cast<mjtNum>(original_gravity_z_)
      : static_cast<mjtNum>(0.0);

  // 4. External forces and perturbation (under pert_mutex_)
  if (pert_mutex_.try_lock()) {
    // User-specified wrench via SetExternalForce()
    if (ext_xfrc_dirty_) {
      const std::size_t n = static_cast<std::size_t>(model_->nbody) * 6;
      std::memcpy(data_->xfrc_applied, ext_xfrc_.data(), n * sizeof(double));
    } else {
      mju_zero(data_->xfrc_applied, model_->nbody * 6);
    }
    // 5. Viewer perturbation spring (additive on top of ext_xfrc_)
    if (pert_active_ && shared_pert_.select > 0) {
      mjv_applyPerturbForce(model_, data_, &shared_pert_);
    }
    pert_mutex_.unlock();
  }
}

void MuJoCoSimulator::ClearContactForces() noexcept {
  // xfrc_applied is re-set each step from the external force buffer.
  // Clear it here so stale forces don't persist if pert_mutex_ was not acquired.
  if (!pert_active_ && !ext_xfrc_dirty_) {
    mju_zero(data_->xfrc_applied, model_->nbody * 6);
  }
}

// ── HandleReset ───────────────────────────────────────────────────────────────
void MuJoCoSimulator::HandleReset() noexcept {
  mj_resetData(model_, data_);
  for (int i = 0; i < num_robot_joints_; ++i) {
    const std::size_t ui = static_cast<std::size_t>(i);
    data_->qpos[joint_qpos_indices_[ui]] = initial_qpos_[ui];
    data_->ctrl[i]                       = initial_qpos_[ui];
  }
  // Restore gravity (may have been zeroed before reset)
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

// ── SimLoopFreeRun ─────────────────────────────────────────────────────────────
void MuJoCoSimulator::SimLoopFreeRun(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto decim = static_cast<uint64_t>(
      cfg_.publish_decimation > 0 ? cfg_.publish_decimation : 1);
  uint64_t step = 0;

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
        continue;  // still paused, no step requested
      }
      // step_once was true: fall through and execute exactly one physics step
    }
    // ── Reset ─────────────────────────────────────────────────────────────
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // ── Command (lock-free fast path) ─────────────────────────────────────
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }
    // ── Physics step ──────────────────────────────────────────────────────
    PreparePhysicsStep();
    mj_step(model_, data_);
    ClearContactForces();
    ReadSolverStats();

    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    if (step % decim == 0) {
      ReadState();
      InvokeStateCallback();
    }
    UpdateRtf(step);
    ThrottleIfNeeded();
    if ((step % 8 == 0) && cfg_.enable_viewer) { UpdateVizBuffer(); }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] FreeRun exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()), sim_time_sec_.load());
}

// ── SimLoopSyncStep ────────────────────────────────────────────────────────────
void MuJoCoSimulator::SimLoopSyncStep(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(cfg_.sync_timeout_ms > 0.0 ? cfg_.sync_timeout_ms : 50.0));
  uint64_t step = 0;

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
        continue;  // still paused, no step requested
      }
      // step_once was true: fall through and execute exactly one physics step
    }
    // ── Reset ─────────────────────────────────────────────────────────────
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // 1. Publish current state
    ReadState();
    InvokeStateCallback();

    // 2. Wait for command (or timeout / stop / resume / reset)
    {
      std::unique_lock lock(sync_mutex_);
      sync_cv_.wait_for(lock, timeout, [this, &stop] {
        return cmd_pending_.load(std::memory_order_relaxed)
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
    // 3. Apply command and step
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }
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
          "[MuJoCoSimulator] SyncStep exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()), sim_time_sec_.load());
}

}  // namespace ur5e_rt_controller
