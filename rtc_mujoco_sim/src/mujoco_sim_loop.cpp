// ── mujoco_sim_loop.cpp ────────────────────────────────────────────────────────
// Physics helpers (ReadState, PreparePhysicsStep, RTF, etc.) and the
// synchronous simulation loop (SimLoop).
// Multi-group: iterates over robot groups for command/state/actuator ops.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include <rtc_base/threading/thread_utils.hpp>
#include <rtc_base/tracing/trace_scope.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace rtc {

// ── Private helpers ────────────────────────────────────────────────────────────

void MuJoCoSimulator::ApplyCommand() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::ApplyCommand");
  if (!model_) {
    return;
  }
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (!g->cmd_pending.load(std::memory_order_acquire))
      continue;

    std::lock_guard lock(g->cmd_mutex);
    const JointControlMode mode = g->control_mode.load(std::memory_order_relaxed);
    const auto ncj = static_cast<std::size_t>(g->num_command_joints);

    for (std::size_t i = 0; i < ncj; ++i) {
      if (i < g->pending_cmd.size() && i < g->actuator_indices.size()) {
        data_->ctrl[g->actuator_indices[i]] = g->pending_cmd[i];
      }
    }

    // Feedforward torque: inject in kPdFeedforward, otherwise zero the group's
    // dofs every tick. mj_step never auto-clears qfrc_applied, so a stale tau_ff
    // would keep being integrated after a mode switch (impulse with n_substeps>1).
    for (std::size_t i = 0; i < ncj; ++i) {
      if (i >= g->qvel_indices.size())
        break;
      const int dof = g->qvel_indices[i];
      data_->qfrc_applied[dof] =
          (mode == JointControlMode::kPdFeedforward && i < g->pending_feedforward.size())
              ? g->pending_feedforward[i]
              : 0.0;
    }

    // Runtime PD gains: stage→gainprm_yaml here (SimLoop-only) and flag a
    // re-apply so this same tick's PreparePhysicsStep pushes them into mjModel.
    if (g->staged_has_gains) {
      for (std::size_t i = 0; i < ncj; ++i) {
        if (i < g->staged_kp.size() && i < g->gainprm_yaml.size())
          g->gainprm_yaml[i] = g->staged_kp[i];
        if (i < g->staged_kd.size() && i < g->biasprm2_yaml.size())
          g->biasprm2_yaml[i] = -g->staged_kd[i];
      }
      g->gains_overridden = true;
      g->staged_has_gains = false;
      g->control_mode_pending.store(true, std::memory_order_release);
    }

    g->cmd_pending.store(false, std::memory_order_release);
  }
}

void MuJoCoSimulator::ReadState() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::ReadState");
  if (!model_ || !data_) {
    return;
  }
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    std::lock_guard lock(g->state_mutex);
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_state_joints); ++i) {
      g->positions[i] = data_->qpos[g->state_qpos_indices[i]];
      g->velocities[i] = data_->qvel[g->state_qvel_indices[i]];
      // Total joint generalized force = actuator (PD) + applied (feedforward).
      // qfrc_applied is solely our pd_feedforward injection (external Cartesian
      // perturbation uses xfrc_applied), so this is 0 in position/torque modes.
      g->efforts[i] = data_->qfrc_actuator[g->state_qvel_indices[i]] +
                      data_->qfrc_applied[g->state_qvel_indices[i]];
    }
  }
}

void MuJoCoSimulator::ReadSensors() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::ReadSensors");
  if (!model_ || !data_ || model_->nsensor <= 0) {
    return;
  }
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (g->sensor_infos.empty())
      continue;
    std::size_t offset = 0;
    for (const auto& si : g->sensor_infos) {
      for (int d = 0; d < si.dim; ++d) {
        g->sensor_buffer[offset++] = data_->sensordata[si.adr + d];
      }
    }
  }
}

void MuJoCoSimulator::InvokeSensorCallback() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::InvokeSensorCallback");
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (g->sensor_infos.empty() || !g->sensor_cb)
      continue;
    g->sensor_cb(g->sensor_infos, g->sensor_buffer);
  }
}

namespace {

// MJCF <contact> sensor data layout (data="found force torque dist pos normal
// tangent", num=1, reduce="netforce" → mjModel.sensor_dim==17). Offsets index
// into mjData.sensordata starting at mjModel.sensor_adr[sensor_id].
constexpr int kContactSensorFoundOffset = 0;
constexpr int kContactSensorForceOffset = 1;   // force[3]
constexpr int kContactSensorTorqueOffset = 4;  // torque[3] (about contact point)
constexpr int kContactSensorDistOffset = 7;
constexpr int kContactSensorPosOffset = 8;  // pos[3] (contact point in world)
constexpr int kRotationMatrixStride = 9;    // mjData.xmat per body (row-major 3x3)
constexpr int kSitePosStride = 3;           // mjData.site_xpos per site

// Apply transpose of body rotation matrix R_WB (row-major 9 elements) to a
// world vector — produces the vector expressed in the body frame.
// v_B = R_WB^T * v_W. noexcept, heap-free.
inline void WorldVecToBody(const mjtNum* rwb_rowmajor, const std::array<double, 3>& v_world,
                           std::array<double, 3>& v_body) noexcept {
  v_body[0] =
      rwb_rowmajor[0] * v_world[0] + rwb_rowmajor[3] * v_world[1] + rwb_rowmajor[6] * v_world[2];
  v_body[1] =
      rwb_rowmajor[1] * v_world[0] + rwb_rowmajor[4] * v_world[1] + rwb_rowmajor[7] * v_world[2];
  v_body[2] =
      rwb_rowmajor[2] * v_world[0] + rwb_rowmajor[5] * v_world[1] + rwb_rowmajor[8] * v_world[2];
}

inline std::array<double, 3> Cross3(const std::array<double, 3>& lhs,
                                    const std::array<double, 3>& rhs) noexcept {
  return {lhs[1] * rhs[2] - lhs[2] * rhs[1], lhs[2] * rhs[0] - lhs[0] * rhs[2],
          lhs[0] * rhs[1] - lhs[1] * rhs[0]};
}

}  // namespace

// Read mjData.sensordata for each registered contact sensor, shift the torque
// from the reported contact point to the ft_site origin, then express both
// vectors in the ft_site's body frame. Heap-free, noexcept.
void MuJoCoSimulator::ReadContactWrenches() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::ReadContactWrenches");
  if (!model_ || !data_)
    return;
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (g->contact_wrench_infos.empty())
      continue;
    for (std::size_t i = 0; i < g->contact_wrench_infos.size(); ++i) {
      const auto& info = g->contact_wrench_infos[i];
      auto& sample = g->contact_wrench_buffer[i];

      const mjtNum* sensor_data = data_->sensordata + info.sensor_adr;
      const bool found = sensor_data[kContactSensorFoundOffset] > 0.0;
      sample.found = found;
      if (!found) {
        sample.force.fill(0.0);
        sample.torque.fill(0.0);
        sample.point_world.fill(0.0);
        sample.dist = 0.0;
        continue;
      }

      // MuJoCo's netforce contact sensor reports the world-frame wrench in
      // the "geom1-on-environment" convention (force/torque that geom1 applies
      // to whatever it contacts). ROS WrenchStamped convention used here is
      // "environment-on-link" (so the published force is the load felt by the
      // fingertip body). We therefore negate at read time.
      const std::array<double, 3> f_w = {
          -static_cast<double>(sensor_data[kContactSensorForceOffset + 0]),
          -static_cast<double>(sensor_data[kContactSensorForceOffset + 1]),
          -static_cast<double>(sensor_data[kContactSensorForceOffset + 2])};
      const std::array<double, 3> tau_pc_w = {
          -static_cast<double>(sensor_data[kContactSensorTorqueOffset + 0]),
          -static_cast<double>(sensor_data[kContactSensorTorqueOffset + 1]),
          -static_cast<double>(sensor_data[kContactSensorTorqueOffset + 2])};
      sample.dist = static_cast<double>(sensor_data[kContactSensorDistOffset]);
      sample.point_world[0] = static_cast<double>(sensor_data[kContactSensorPosOffset + 0]);
      sample.point_world[1] = static_cast<double>(sensor_data[kContactSensorPosOffset + 1]);
      sample.point_world[2] = static_cast<double>(sensor_data[kContactSensorPosOffset + 2]);

      // Torque shift to ft_site origin: τ_link_world = τ_pc + (p_c − p_L) × f_W.
      const mjtNum* p_link_w =
          data_->site_xpos + (static_cast<std::ptrdiff_t>(kSitePosStride) * info.ft_site_id);
      const std::array<double, 3> r_w = {sample.point_world[0] - static_cast<double>(p_link_w[0]),
                                         sample.point_world[1] - static_cast<double>(p_link_w[1]),
                                         sample.point_world[2] - static_cast<double>(p_link_w[2])};
      const auto r_cross_f = Cross3(r_w, f_w);
      const std::array<double, 3> tau_link_w = {
          tau_pc_w[0] + r_cross_f[0], tau_pc_w[1] + r_cross_f[1], tau_pc_w[2] + r_cross_f[2]};

      // Transform world-frame vectors into the ft_site's body frame.
      const mjtNum* rwb =
          data_->xmat + (static_cast<std::ptrdiff_t>(kRotationMatrixStride) * info.body_id);
      WorldVecToBody(rwb, f_w, sample.force);
      WorldVecToBody(rwb, tau_link_w, sample.torque);
    }
  }
}

void MuJoCoSimulator::InvokeContactWrenchCallback() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::InvokeContactWrenchCallback");
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (g->contact_wrench_infos.empty() || !g->contact_wrench_cb)
      continue;
    g->contact_wrench_cb(g->contact_wrench_infos, g->contact_wrench_buffer);
  }
}

void MuJoCoSimulator::ReadSolverStats() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::ReadSolverStats");
  if (!data_) {
    return;
  }
  SolverStats s{};
  s.ncon = data_->ncon;
  const int nisland = data_->nisland;
  for (int k = 0; k < nisland; ++k) {
    s.iter += data_->solver_niter[k];
  }
  if (nisland > 0 && s.iter > 0) {
    s.improvement = static_cast<double>(data_->solver[0].improvement);
    s.gradient = static_cast<double>(data_->solver[0].gradient);
  }
  std::lock_guard lock(solver_stats_mutex_);
  latest_solver_stats_ = s;
}

void MuJoCoSimulator::InvokeStateCallback() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::InvokeStateCallback");
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (!g->state_cb)
      continue;
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
  RTC_TRACE_SCOPE("MuJoCoSimulator::UpdateVizBuffer");
  if (viz_mutex_.try_lock()) {
    std::memcpy(viz_qpos_.data(), data_->qpos,
                static_cast<std::size_t>(model_->nq) * sizeof(double));
    viz_ncon_ = data_->ncon;
    viz_dirty_ = true;
    viz_mutex_.unlock();
  }
}

void MuJoCoSimulator::UpdateRtf(uint64_t step) noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::UpdateRtf");
  constexpr uint64_t kRtfUpdateInterval = 200;
  if (step % kRtfUpdateInterval != 0) {
    return;
  }
  const auto wall_now = std::chrono::steady_clock::now();
  const double wall_dt = std::chrono::duration<double>(wall_now - rtf_wall_start_).count();
  const double sim_dt = data_->time - rtf_sim_start_;
  if (wall_dt > 0.01) {
    rtf_.store(sim_dt / wall_dt, std::memory_order_relaxed);
    rtf_wall_start_ = wall_now;
    rtf_sim_start_ = data_->time;
  }
}

void MuJoCoSimulator::ThrottleIfNeeded() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::ThrottleIfNeeded");
  const double max_rtf = current_max_rtf_.load(std::memory_order_relaxed);
  if (max_rtf != throttle_rtf_) {
    throttle_wall_start_ = std::chrono::steady_clock::now();
    throttle_sim_start_ = data_->time;
    throttle_rtf_ = max_rtf;
  }
  if (max_rtf <= 0.0) {
    return;
  }
  const double sim_elapsed = data_->time - throttle_sim_start_;
  const double target_wall = sim_elapsed / max_rtf;
  const double actual_wall =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - throttle_wall_start_)
          .count();
  if (actual_wall < target_wall) {
    std::this_thread::sleep_for(std::chrono::duration<double>(target_wall - actual_wall));
  }
}

// ── PreparePhysicsStep ─────────────────────────────────────────────────────────

void MuJoCoSimulator::PreparePhysicsStep() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::PreparePhysicsStep");
  // 0. Per-group actuator mode switch (torque / position-servo / pd_feedforward)
  bool gravcomp_dirty = false;
  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    if (!g->control_mode_pending.exchange(false, std::memory_order_acq_rel))
      continue;

    const JointControlMode mode = g->control_mode.load(std::memory_order_relaxed);
    const bool torque = (mode == JointControlMode::kTorque);

    // Per-body gravity compensation toggle (same gate as actuator mode flip,
    // so all mjModel mutations stay on the SimLoop thread). Only kPosition lets
    // MuJoCo cancel gravity; kTorque and kPdFeedforward leave it to the
    // controller's torque (feedforward must include gravity).
    const mjtNum gravcomp =
        (mode == JointControlMode::kPosition) ? static_cast<mjtNum>(1.0) : static_cast<mjtNum>(0.0);
    for (int body_id : g->body_indices) {
      if (body_id > 0 && body_id < model_->nbody)
        model_->body_gravcomp[body_id] = gravcomp;
    }
    gravcomp_dirty = true;

    // kPosition and kPdFeedforward share the same affine PD actuator params;
    // kPdFeedforward only differs by gravcomp (above) + qfrc_applied (ApplyCommand).
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_command_joints); ++i) {
      const int act = g->actuator_indices[i];
      if (torque) {
        model_->actuator_gainprm[act * mjNGAIN + 0] = static_cast<mjtNum>(1.0);
        model_->actuator_biasprm[act * mjNBIAS + 0] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[act * mjNBIAS + 1] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[act * mjNBIAS + 2] = static_cast<mjtNum>(0.0);
      } else if (g->gains_overridden || cfg_.use_yaml_servo_gains) {
        const double kp = g->gainprm_yaml[i];
        model_->actuator_gainprm[act * mjNGAIN + 0] = static_cast<mjtNum>(kp);
        model_->actuator_biasprm[act * mjNBIAS + 0] = static_cast<mjtNum>(0.0);
        model_->actuator_biasprm[act * mjNBIAS + 1] = static_cast<mjtNum>(-kp);
        model_->actuator_biasprm[act * mjNBIAS + 2] = static_cast<mjtNum>(g->biasprm2_yaml[i]);
      } else {
        const auto& p = orig_actuator_params_[static_cast<std::size_t>(act)];
        model_->actuator_gainprm[act * mjNGAIN + 0] = static_cast<mjtNum>(p.gainprm0);
        model_->actuator_biasprm[act * mjNBIAS + 0] = static_cast<mjtNum>(p.biasprm0);
        model_->actuator_biasprm[act * mjNBIAS + 1] = static_cast<mjtNum>(p.biasprm1);
        model_->actuator_biasprm[act * mjNBIAS + 2] = static_cast<mjtNum>(p.biasprm2);
      }
    }
  }
  if (gravcomp_dirty) {
    // Re-count nonzero entries so mj_passive() doesn't early-out on ngravcomp==0.
    RefreshNgravcomp();
  }

  // 1. Physics solver parameters
  model_->opt.integrator =
      static_cast<mjtIntegrator>(solver_integrator_.load(std::memory_order_relaxed));
  model_->opt.solver = static_cast<mjtSolver>(solver_type_.load(std::memory_order_relaxed));
  model_->opt.iterations = solver_iterations_.load(std::memory_order_relaxed);
  model_->opt.tolerance = static_cast<mjtNum>(solver_tolerance_.load(std::memory_order_relaxed));
  model_->opt.cone = static_cast<mjtCone>(solver_cone_.load(std::memory_order_relaxed));
  model_->opt.jacobian = static_cast<mjtJacobian>(solver_jacobian_.load(std::memory_order_relaxed));
  model_->opt.ls_iterations = solver_ls_iterations_.load(std::memory_order_relaxed);
  model_->opt.ls_tolerance =
      static_cast<mjtNum>(solver_ls_tolerance_.load(std::memory_order_relaxed));
  model_->opt.noslip_iterations = solver_noslip_iterations_.load(std::memory_order_relaxed);
  model_->opt.noslip_tolerance =
      static_cast<mjtNum>(solver_noslip_tolerance_.load(std::memory_order_relaxed));
  model_->opt.impratio = static_cast<mjtNum>(solver_impratio_.load(std::memory_order_relaxed));

  // 2. Contact enable / disable
  if (contacts_enabled_.load(std::memory_order_relaxed)) {
    model_->opt.disableflags &= ~mjDSBL_CONTACT;
  } else {
    model_->opt.disableflags |= mjDSBL_CONTACT;
  }

  // 3. World gravity toggle (debugging only — robot groups handle their own
  //    gravity compensation via per-body body_gravcomp set in SetControlMode).
  model_->opt.gravity[2] = world_gravity_enabled_.load(std::memory_order_relaxed)
                               ? static_cast<mjtNum>(original_gravity_z_)
                               : static_cast<mjtNum>(0.0);

  // 4. External forces and perturbation
  if (pert_mutex_.try_lock()) {
    if (ext_xfrc_dirty_) {
      const std::size_t n = static_cast<std::size_t>(model_->nbody) * 6;
      std::memcpy(data_->xfrc_applied, ext_xfrc_.data(), n * sizeof(double));
    } else {
      mju_zero(data_->xfrc_applied, static_cast<int>(model_->nbody) * 6);
    }
    if (pert_active_ && shared_pert_.select > 0) {
      mjv_applyPerturbForce(model_, data_, &shared_pert_);
    }
    pert_mutex_.unlock();
  }
}

void MuJoCoSimulator::ClearContactForces() noexcept {
  RTC_TRACE_SCOPE("MuJoCoSimulator::ClearContactForces");
  if (!pert_active_ && !ext_xfrc_dirty_) {
    mju_zero(data_->xfrc_applied, static_cast<int>(model_->nbody) * 6);
  }
}

// ── HandleReset ───────────────────────────────────────────────────────────────

void MuJoCoSimulator::HandleReset() noexcept {
  mj_resetData(model_, data_);

  for (auto& g : groups_) {
    if (!g->is_robot)
      continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_command_joints); ++i) {
      data_->qpos[g->qpos_indices[i]] = g->initial_qpos[i];
      data_->ctrl[g->actuator_indices[i]] = g->initial_qpos[i];
    }
  }

  model_->opt.gravity[2] = world_gravity_enabled_.load(std::memory_order_relaxed)
                               ? static_cast<mjtNum>(original_gravity_z_)
                               : static_cast<mjtNum>(0.0);
  // body_gravcomp persists across mj_resetData; no need to re-write here.
  mj_forward(model_, data_);
  {
    std::lock_guard lock(pert_mutex_);
    mjv_defaultPerturb(&shared_pert_);
    pert_active_ = false;
    std::fill(ext_xfrc_.begin(), ext_xfrc_.end(), 0.0);
    ext_xfrc_dirty_ = false;
  }
  step_count_.store(0, std::memory_order_relaxed);
  sim_time_sec_.store(0.0, std::memory_order_relaxed);
  rtf_.store(0.0, std::memory_order_relaxed);
  const auto now = std::chrono::steady_clock::now();
  rtf_wall_start_ = now;
  rtf_sim_start_ = data_->time;
  throttle_wall_start_ = now;
  throttle_sim_start_ = data_->time;
  throttle_rtf_ = current_max_rtf_.load(std::memory_order_relaxed);
  ReadState();
  fprintf(stdout, "[SimLoop] Reset to initial pose\n");
}

// ── SimLoop ─────────────────────────────────────────────────────────────────────
// Synchronous simulation: publish state → wait for command → step → throttle.

void MuJoCoSimulator::SimLoop(std::stop_token stop) noexcept {
  if (!model_ || !data_) {
    return;
  }

  // Apply SystemThreadConfigs.sim_thread (taskset pin + SCHED_OTHER nice 0 +
  // thread name). The ThreadConfig may carry cpu_core == -1 on small tiers
  // (≤ 6-core), in which case ApplyThreadConfig skips the affinity call and
  // leaves MuJoCo to roam under CFS — the cpu_shield --sim path is what
  // releases those cores in the first place.
  (void)rtc::ApplyThreadConfigVerbose(rtc::SelectThreadConfigs().sim_thread);

  const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(cfg_.sync_timeout_ms > 0.0 ? cfg_.sync_timeout_ms : 50.0));
  uint64_t step = 0;

  // Find primary group index
  std::size_t primary_idx = 0;
  for (std::size_t i = 0; i < groups_.size(); ++i) {
    if (groups_[i]->is_primary) {
      primary_idx = i;
      break;
    }
  }

  const auto loop_start = std::chrono::steady_clock::now();
  rtf_wall_start_ = loop_start;
  rtf_sim_start_ = data_->time;
  throttle_wall_start_ = loop_start;
  throttle_sim_start_ = data_->time;
  throttle_rtf_ = current_max_rtf_.load(std::memory_order_relaxed);

  while (!stop.stop_requested() && running_.load()) {
    // ── Pause (with step_once support) ────────────────────────────────────
    if (paused_.load(std::memory_order_relaxed)) {
      {
        std::unique_lock lock(sync_mutex_);
        sync_cv_.wait_for(lock, std::chrono::milliseconds(10), [this, &stop] {
          return !paused_.load(std::memory_order_relaxed) ||
                 step_once_.load(std::memory_order_relaxed) || stop.stop_requested() ||
                 !running_.load();
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
    // 1. Publish current state (and sensors) for ALL robot groups
    // Per-iteration span (one sim step of the raw sim_thread). All the
    // MuJoCoSimulator member-function spans below nest under this; the
    // sync_cv_ wait for the controller's command (sim lock-step) shows up as
    // the explicit sim_wait_command span between the state-publish spans and
    // the ApplyCommand/substep spans.
    RTC_TRACE_SCOPE("sim_step");
    ReadState();
    ReadSensors();
    ReadContactWrenches();
    InvokeStateCallback();
    InvokeSensorCallback();
    InvokeContactWrenchCallback();

    // 2. Wait for command from PRIMARY group
    {
      RTC_TRACE_SCOPE("sim_wait_command");
      std::unique_lock lock(sync_mutex_);
      sync_cv_.wait_for(lock, timeout, [this, &stop, primary_idx] {
        return groups_[primary_idx]->cmd_pending.load(std::memory_order_relaxed) ||
               stop.stop_requested() || !running_.load() ||
               reset_requested_.load(std::memory_order_relaxed);
      });
    }
    if (stop.stop_requested() || !running_.load()) {
      break;
    }
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }
    // 3. Apply commands from ALL robot groups and substep
    ApplyCommand();
    const auto step_start = std::chrono::steady_clock::now();
    for (int sub = 0; sub < cfg_.n_substeps; ++sub) {
      RTC_TRACE_SCOPE("sim_substep");
      PreparePhysicsStep();
      {
        RTC_TRACE_SCOPE("mj_step");
        mj_step(model_, data_);
      }
      ClearContactForces();
    }
    const double step_wall_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start).count();
    physics_load_.store(step_wall_sec / xml_timestep_, std::memory_order_relaxed);
    ReadSolverStats();

    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    UpdateRtf(step);
    ThrottleIfNeeded();
    if ((step % viz_update_interval_ == 0) && cfg_.enable_viewer) {
      UpdateVizBuffer();
    }
  }

  fprintf(stdout, "[SimLoop] Exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()), sim_time_sec_.load());
}

}  // namespace rtc
