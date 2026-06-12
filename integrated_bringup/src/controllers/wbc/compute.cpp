#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <span>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Phase 1: Read state ─────────────────────────────────────────────────────
void DemoWbcController::ReadState(const ControllerState& state) noexcept {
  // Read fingertip data from inference_data: slots 1..3 (fx/fy/fz) always,
  // slot 0 (native contact probability) only when has_native_contact_=true
  // (sensor A path; otherwise backend zero-fills the slot). Slots 4..6
  // (displacement) remain unconsumed — deformation guard is stubbed pending
  // sensor HW upgrade. ft.in_contact here decides *grasp* detection (uses
  // gains.grasp_force_threshold + grasp_contact_threshold via capability-
  // aware AND) — separate from phase.cpp's FSM threshold force_contact_threshold_
  // which gates the kClosure→kHold transition on raw force_magnitude directly.
  num_active_fingertips_ = 0;
  if (state.num_devices <= 1 || !state.devices[1].valid) {
    return;
  }

  const auto& dev1 = state.devices[1];
  // Prefer num_inference_groups (set by mujoco_native via ReadSensorState +
  // udp_hand via ReadSensorState). Fall back to deriving from sensor channel
  // stride for the older udp_hand-style mocks that only fill the sensor lane.
  const int num_groups =
      (dev1.num_inference_groups > 0)
          ? dev1.num_inference_groups
          : (kHandSensorValuesPerFingertipCapacity > 0
                 ? static_cast<int>(dev1.num_sensor_channels /
                                    static_cast<int>(kHandSensorValuesPerFingertipCapacity))
                 : 0);
  num_active_fingertips_ = std::min(num_groups, static_cast<int>(rtc::kMaxSensorGroups));

  const double inv_dt = (state.dt > 0.0) ? (1.0 / state.dt) : 500.0;
  const auto gains_now = gains_lock_.Load();
  const float force_threshold = gains_now.grasp_force_threshold;
  const float contact_threshold = gains_now.grasp_contact_threshold;

  for (int f = 0; f < num_active_fingertips_; ++f) {
    auto& ft = fingertip_data_[static_cast<std::size_t>(f)];

    ft.valid = dev1.inference_enable[static_cast<std::size_t>(f)];
    if (ft.valid) {
      const int ft_base = f * kHandInferenceValuesPerFingertipCapacity;
      for (int j = 0; j < 3; ++j) {
        ft.force[static_cast<std::size_t>(j)] =
            dev1.inference_data[static_cast<std::size_t>(ft_base + 1 + j)];
      }
      const bool native_path =
          has_native_contact_ && dev1.inference_enable[static_cast<std::size_t>(f)];
      ft.contact_flag = native_path ? dev1.inference_data[static_cast<std::size_t>(ft_base)] : 0.0F;
      const float fx = ft.force[0];
      const float fy = ft.force[1];
      const float fz = ft.force[2];
      ft.force_magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
      const bool force_active = (ft.force_magnitude > force_threshold);
      ft.in_contact =
          native_path ? (ft.contact_flag > contact_threshold && force_active) : force_active;
    } else {
      ft.force = {};
      ft.contact_flag = 0.0F;
      ft.force_magnitude = 0.0f;
      ft.in_contact = false;
    }

    // df/dt with EMA smoothing; skip on first tick to avoid startup spike
    if (force_rate_initialized_) {
      const float raw_rate =
          static_cast<float>((ft.force_magnitude - ft.prev_force_magnitude) * inv_dt);
      ft.force_rate = force_rate_alpha_ * raw_rate + (1.0f - force_rate_alpha_) * ft.force_rate;
    } else {
      ft.force_rate = 0.0f;
    }
    ft.prev_force_magnitude = ft.force_magnitude;
  }
  force_rate_initialized_ = true;
}

// ── Phase 2: Compute control ─────────────────────────────────────────────────

// ── Phase 2: Compute control (phase dispatch) ───────────────────────────────

void DemoWbcController::ComputeControl(const ControllerState& state, double dt) noexcept {
  UpdatePhase(state);

  // MPC WriteState moved into ComputeTSIDPosition (next to its
  // ExtractFullState), which is reached every tick by all TSID-routing
  // phases (kIdle/kApproach/kPreGrasp/kClosure/kHold and kRelease via
  // ComputeReleaseMode → ComputeTSIDPosition). kFallback no longer pushes
  // state; the MPC retains its last snapshot until recovery to kIdle, which
  // is safe because MPC output is not driving control during fallback and
  // the dim-mismatch gate (state.nq == model_->nq()) keys on nq, not
  // staleness. Eliminates the per-tick ExtractFullState double-call (this
  // top-level + ComputeTSIDPosition's own call).
  switch (phase_) {
    case WbcPhase::kIdle:
    case WbcPhase::kApproach:
    case WbcPhase::kPreGrasp:
    case WbcPhase::kClosure:
    case WbcPhase::kHold:
      if (tsid_initialized_) {
        ComputeTSIDPosition(state, dt);
      } else {
        ComputePositionMode(dt);  // Fallback if TSID not available
      }
      break;

    case WbcPhase::kRelease:
      ComputeReleaseMode(state, dt);
      break;

    case WbcPhase::kFallback:
      ComputeFallback();
      break;
  }
}

// ── FSM ──────────────────────────────────────────────────────────────────────

// ── Position-mode and TSID solvers ──────────────────────────────────────────

void DemoWbcController::ComputePositionMode(double dt) noexcept {
  // Robot arm trajectory
  robot_trajectory_time_ += dt;
  const auto rstate =
      robot_trajectory_.compute(std::min(robot_trajectory_time_, robot_trajectory_.duration()));
  for (int i = 0; i < arm_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    robot_computed_.positions[idx] = rstate.positions[idx];
    robot_computed_.velocities[idx] = rstate.velocities[idx];
  }

  // Hand trajectory
  hand_trajectory_time_ += dt;
  const auto hstate =
      hand_trajectory_.compute(std::min(hand_trajectory_time_, hand_trajectory_.duration()));
  for (int i = 0; i < hand_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    hand_computed_.positions[idx] = hstate.positions[idx];
    hand_computed_.velocities[idx] = hstate.velocities[idx];
  }

  // This path bypasses the TSID integrator; drop any pending re-seed so it
  // cannot fire stale once TSID becomes available.
  reseed_integration_pending_ = false;
}

// Static, pure semi-implicit Euler step shared by both integration modes (see
// ComputeTSIDPosition step 7). Operates in place on the seed (q, v); the seed
// is the previous command (carry-forward) or the measured state (measured-
// feedback) — chosen by the caller. RT-safe: fixed-size Eigen ops, no alloc /
// throw. The .noalias() self-assignments are coefficient-wise (element i reads
// only element i), so aliasing q/v with themselves is well-defined.
void DemoWbcController::IntegrateAccelStep(Eigen::VectorXd& q, Eigen::VectorXd& v,
                                           const Eigen::VectorXd& a, double dt,
                                           const Eigen::VectorXd& v_limit,
                                           const Eigen::VectorXd& q_min,
                                           const Eigen::VectorXd& q_max) noexcept {
  // v ← v + a · dt, then velocity clamp.
  v.noalias() = v + a * dt;
  v = v.cwiseMax(-v_limit).cwiseMin(v_limit);

  // q ← q + v · dt.
  q.noalias() = q + v * dt;

  // Position clamp (safety net). Where it saturates, zero the velocity if it is
  // still driving further into the violated limit — open-loop carry-forward
  // would otherwise keep pushing. Sign-gated so a TSID-commanded retraction is
  // not suppressed.
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    if (q[i] < q_min[i]) {
      q[i] = q_min[i];
      if (v[i] < 0.0) {
        v[i] = 0.0;
      }
    } else if (q[i] > q_max[i]) {
      q[i] = q_max[i];
      if (v[i] > 0.0) {
        v[i] = 0.0;
      }
    }
  }
}

void DemoWbcController::ComputeTSIDPosition(const ControllerState& state, double dt) noexcept {
  // 1. Extract full state (sensor values, every tick)
  ExtractFullState(state);

  // 1a. Push fresh (q, v) to the MPC thread so HandlerMPCThread::Solve does
  // not reject the snapshot via its dim-mismatch / staleness gates.
  // Lives here (not at ComputeControl's top) because ExtractFullState +
  // pinocchio_cache_.Update already need fresh state at this exact point;
  // co-locating the WriteState avoids a per-tick double extraction.
  if (mpc_enabled_ && mpc_manager_.Enabled()) {
    const uint64_t now_ns = static_cast<uint64_t>(state.iteration) * 2'000'000ULL;
    mpc_manager_.WriteState(q_curr_full_, v_curr_full_, now_ns);
  }

  // Stage A-5b: progress per-contact activation ramp by dt. ContactState
  // auto-flips the legacy `active : bool` once s_i crosses kActivationDeadband
  // so downstream skip paths (active_contact_vars, task/constraint early
  // outs) stay coherent.
  contact_state_.UpdateActivation(dt);
  contact_state_.RecomputeActive(contact_mgr_config_);

  // 2. Update Pinocchio cache (M, h, g, Jacobians)
  pinocchio_cache_.Update(q_curr_full_, v_curr_full_, contact_state_);

  // 2a. Stage B-5: populate the once-per-tick grasp cache shared across the
  // three object-level tasks (ObjectWrenchTask / InternalForceTask /
  // ObjectSE3Task). Order is invariant per the plan handoff:
  //   cache.Update → UpdateActivation → RecomputeActive →
  //   ActiveLambdaDim → ComputeGraspMatrix(G_workspace_) → grasp_cache_.Compute
  // Tasks must never re-Compute on their own — they only read GPinv/GTPinv/
  // ProjN/Rank from grasp_cache_. n_active=0 (idle/pre_grasp) leaves the
  // cache empty (Rank()=0); object-level tasks then report ResidualDim=0.
  const int n_lambda_active = contact_mgr_.ActiveLambdaDim(contact_state_);
  if (n_lambda_active > 0) {
    auto G_view = grasp_G_workspace_.leftCols(n_lambda_active);
    contact_mgr_.ComputeGraspMatrix(pinocchio_cache_, contact_state_, object_frame_, G_view);
    grasp_cache_.Compute(G_view, n_lambda_active);
  } else {
    grasp_cache_.Compute(grasp_G_workspace_.leftCols(0), 0);
  }

  // 2b. MPC reference injection.
  //
  // Publish the current RT state to the MPC thread, then try to consume
  // the freshest MPC solution. If we have a valid interpolated reference,
  // it replaces the self-regularising hold target on the next line. If
  // not (MPC disabled / not yet publishing / too many stale cycles), we
  // fall through to the TSID self-hold behaviour.
  bool mpc_ref_valid = false;
  if (mpc_enabled_ && mpc_manager_.Enabled()) {
    const uint64_t now_ns = static_cast<uint64_t>(state.iteration) * 2'000'000ULL;  // 500 Hz tick
    rtc::mpc::InterpMeta meta;
    mpc_manager_.WriteState(q_curr_full_, v_curr_full_, now_ns);
    mpc_ref_valid =
        mpc_manager_.ComputeReference(q_curr_full_, v_curr_full_, now_ns, mpc_q_ref_, mpc_v_ref_,
                                      mpc_a_ff_, mpc_lambda_ref_, mpc_u_fb_, meta);
  }

  // 3. Set posture reference (regularization toward MPC q_ref if valid,
  //    else toward current position for self-holding behaviour).
  if (mpc_ref_valid) {
    control_ref_.q_des = mpc_q_ref_;
    control_ref_.v_des = mpc_v_ref_;
    // TSID will combine a_ff with task PD correction. u_fb from Riccati
    // is additive acceleration feedback on the actuated joints only.
    control_ref_.a_des = mpc_a_ff_;
    const int n_fb =
        std::min(static_cast<int>(mpc_u_fb_.size()), static_cast<int>(control_ref_.a_des.size()));
    control_ref_.a_des.head(n_fb) += mpc_u_fb_.head(n_fb);
  } else {
    // Phase-scoped posture reference. kIdle regulates toward a fixed init
    // snapshot (q_des_target_full_, re-seeded from measured on each idle entry
    // by SeedHoldFromMeasured) so a disturbed joint returns to the held pose
    // instead of drifting — the InitPositionHold semantics. kRelease keeps
    // measured self-hold so the finger-open ramp is not fought by a stiff arm
    // target. Active driving phases track the external joint target snapshot
    // (q_des_target_full_, built at phase entry alongside the SE3 goal).
    // Falls back to measured self-hold when the reorder map is unavailable
    // (unit-test bypass) so q_des is never the zero vector.
    if (phase_ == WbcPhase::kRelease) {
      control_ref_.q_des = q_curr_full_;
    } else if (phase_ == WbcPhase::kIdle) {
      control_ref_.q_des = joint_reorder_valid_ ? q_des_target_full_ : q_curr_full_;
    } else {
      control_ref_.q_des = q_des_target_full_;
    }
    control_ref_.v_des.setZero();
    control_ref_.a_des.setZero();
  }

  // 3b. Stage A-3 — per-tick ForceReferenceUpdater (kClosure/kHold only).
  // Closes the loop on per-fingertip normal force: λ_des_i_n = PI(f_des - |f_meas_i|).
  // Stage A-4: push along the contact normal (world frame) so the reference
  // and FrictionConeConstraint share the same R_c basis. Default normal=+Z
  // is byte-identical to the original Stage A-3 +Z push.
  if (phase_ == WbcPhase::kClosure || phase_ == WbcPhase::kHold) {
    auto* force_task =
        tsid_initialized_ ? tsid_controller_.Formulation().GetTask("force") : nullptr;
    const int n_lambda = contact_mgr_config_.max_contact_vars;
    if (force_task && n_lambda > 0 && force_lambda_des_.size() == n_lambda) {
      const auto gains_now = gains_lock_.Load();
      const double f_des = gains_now.grasp_target_force;
      force_lambda_des_.setZero();
      int offset = 0;
      int contact_idx = 0;
      for (const auto& c : contact_mgr_config_.contacts) {
        const int cdim = c.contact_dim;
        if (offset + cdim > n_lambda) {
          break;
        }
        // Map contact_idx → fingertip sensor slot 1:1 (configured ordering).
        const bool valid = (contact_idx < num_active_fingertips_) &&
                           fingertip_data_[static_cast<std::size_t>(contact_idx)].valid;
        const double f_meas =
            valid ? static_cast<double>(
                        fingertip_data_[static_cast<std::size_t>(contact_idx)].force_magnitude)
                  : 0.0;
        const double lambda_n = force_ref_updater_.Update(contact_idx, valid, f_des, f_meas, dt);
        // Stage A-4: 3D push along world-frame contact normal n.
        // λ_world_force = lambda_n · n. Moment block (surface) stays zero.
        if (cdim >= 3 && contact_idx < static_cast<int>(contact_state_.contacts.size())) {
          const Eigen::Vector3d& n =
              contact_state_.contacts[static_cast<std::size_t>(contact_idx)].normal;
          force_lambda_des_[offset + 0] = lambda_n * n.x();
          force_lambda_des_[offset + 1] = lambda_n * n.y();
          force_lambda_des_[offset + 2] = lambda_n * n.z();
        }
        offset += cdim;
        ++contact_idx;
      }
      static_cast<rtc::tsid::ForceTask*>(force_task)->SetForceReferences(force_lambda_des_);
    }
  }

  // 3c. TCP SE3 trajectory tick (MPC-disabled mode only). Gated by both
  // tcp_trajectory_active_ (init flag from OnPhaseEnter) AND the per-phase
  // SE3-task-active cache so we don't push references in phases where
  // se3_tcp is YAML-deactivated. Per-tick push of (pose, v, a) replaces the
  // single step-on-entry SetSe3Reference path so the SE3Task sees a smooth
  // ramp instead of a large initial error → aggressive PD acceleration.
  if (tcp_trajectory_active_ && se3_task_active_in_phase_[static_cast<std::size_t>(phase_)]) {
    tcp_trajectory_time_ += dt;
    if (has_pending_tcp_segment_ && tcp_trajectory_time_ >= tcp_trajectory_.duration()) {
      const pinocchio::SE3 mid = tcp_trajectory_.compute(tcp_trajectory_.duration()).pose;
      tcp_trajectory_.initialize(mid, pinocchio::Motion::Zero(), pending_tcp_goal_,
                                 pinocchio::Motion::Zero(), pending_tcp_duration_);
      tcp_trajectory_time_ = 0.0;
      has_pending_tcp_segment_ = false;
    }
    tcp_traj_state_ =
        tcp_trajectory_.compute(std::min(tcp_trajectory_time_, tcp_trajectory_.duration()), dt);
    if (auto* se3 = tsid_controller_.Formulation().GetTask("se3_tcp"); se3) {
      static_cast<rtc::tsid::SE3Task*>(se3)->SetSe3Reference(
          tcp_traj_state_.pose, tcp_traj_state_.velocity.toVector(),
          tcp_traj_state_.acceleration.toVector());
    }
  }

  // 4. Build ControlState
  ctrl_state_.q = q_curr_full_;
  ctrl_state_.v = v_curr_full_;
  ctrl_state_.timestamp_ns = state.iteration;

  // 5. TSID solve
  tsid_output_ =
      tsid_controller_.Compute(ctrl_state_, control_ref_, pinocchio_cache_, contact_state_);

  // 6. QP failure handling
  if (!tsid_output_.qp_converged) {
    ++qp_fail_count_;
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                         "[wbc] QP failed (%d/%d), solve=%.0fus", qp_fail_count_,
                         max_qp_fail_before_fallback_, tsid_output_.solve_time_us);

    if (qp_fail_count_ >= max_qp_fail_before_fallback_) {
      // kFallback bypasses the integrator; recovery re-seeds via
      // reseed_on_fallback_exit_. Drop any pending target re-seed so it cannot
      // fire from a stale target on the recovery tick.
      reseed_integration_pending_ = false;
      phase_ = WbcPhase::kFallback;
      ComputeFallback();
      return;
    }
    // Hold last valid command this tick. Re-seed on the next successful tick so
    // the integrator resumes from fresh measured state (no stale-gap jump).
    reseed_integration_pending_ = true;
    return;
  }
  qp_fail_count_ = 0;

  // 7. Semi-implicit Euler integration: a → v → q.
  //
  // Seed selection decides where the integral accumulates from this tick:
  //   - measured-feedback mode (integrate_from_measured_): re-seed from the
  //     freshly measured (q_curr, v_curr) every tick → v_next = v_curr + a·dt,
  //     q_next = q_curr + v_next·dt. The carry-forward continuity machinery
  //     below is bypassed because there is no prior command state to preserve.
  //   - carry-forward mode (default): the integral accumulates from the
  //     PREVIOUS integrated (q_next, v_next); it re-seeds from the measured
  //     state only on discrete events:
  //       · hard re-seed (= measured exactly): fallback recovery / first tick /
  //         non-finite guard — no command continuity to preserve.
  //       · target re-seed (jerk-bounded): a new target arrived mid-motion;
  //         seed from measured q̇ but bound the one-tick step against the
  //         previous command velocity to avoid a discontinuity.
  const auto& a = tsid_output_.a_opt;

  const bool hard_reseed =
      reseed_on_fallback_exit_ || !q_next_full_.allFinite() || !v_next_full_.allFinite();
  if (integrate_from_measured_ || hard_reseed) {
    q_next_full_ = q_curr_full_;
    v_next_full_ = v_curr_full_;
  } else if (reseed_integration_pending_) {
    const double dv = v_jerk_limit_ * dt;
    v_seed_ = v_next_full_;  // previous command velocity (distinct buffer → no aliasing)
    q_next_full_ = q_curr_full_;
    v_next_full_ =
        v_curr_full_.array().max(v_seed_.array() - dv).min(v_seed_.array() + dv).matrix();
  }
  reseed_integration_pending_ = false;
  reseed_on_fallback_exit_ = false;

  // Semi-implicit Euler step + velocity/position clamps (mode-agnostic given
  // the seed already in q_next_full_/v_next_full_).
  IntegrateAccelStep(q_next_full_, v_next_full_, a, dt, v_limit_, q_min_clamped_, q_max_clamped_);

  // ── 7b. Stage C-2: CLIK reference path + A/B shadow ─────────────────────
  // The integrator candidate is now in q_next_full_/v_next_full_. Compute the
  // CLIK candidate (measured-anchored on the cache updated at step 2 — no
  // extra FK) every tick. command_source selects which drives; the other is
  // logged as a Δ shadow. When CLIK drives we overwrite q_next_full_/
  // v_next_full_ — they double as the carry-forward seed, and CLIK output is
  // measured-anchored, so a later switch back to the integrator resumes
  // without a jump. On CLIK failure in clik mode the integrator candidate
  // stands (throttled WARN). When command_source=integrator (default) the
  // command buffers are never overwritten → integrator output is unchanged;
  // only the shadow diagnostics are computed.
  const auto gains_now = gains_lock_.Load();
  const CommandSource src = clik_enabled_ ? gains_now.command_source : CommandSource::kIntegrator;
  active_command_source_ = CommandSource::kIntegrator;
  clik_compute_ok_ = false;
  shadow_valid_ = false;
  clik_tcp_err_ = 0.0;
  clik_manip_ = 0.0;
  shadow_pos_delta_ = 0.0;
  shadow_vel_delta_ = 0.0;

  if (clik_enabled_ && tcp_goal_valid_) {
    // Track the same SE3 reference the SE3Task sees this tick: the quintic
    // ramp setpoint while se3_tcp is active in this phase, else the held goal.
    const auto phase_idx = static_cast<std::size_t>(phase_);
    const bool se3_active =
        tcp_trajectory_active_ && phase_idx < kNumPhases && se3_task_active_in_phase_[phase_idx];
    const pinocchio::SE3& clik_target = se3_active ? tcp_traj_state_.pose : tcp_goal_;

    // Forward the live CLIK gains (kx broadcast as [pos×3, rot×3]).
    clik_kx_ << gains_now.clik_kx_pos, gains_now.clik_kx_pos, gains_now.clik_kx_pos,
        gains_now.clik_kx_rot, gains_now.clik_kx_rot, gains_now.clik_kx_rot;
    clik_.SetTaskGain(clik_kx_);
    clik_.SetPostureGains(gains_now.clik_ka, gains_now.clik_kh);

    clik_compute_ok_ = clik_.Compute(pinocchio_cache_, clik_tcp_frame_idx_, clik_base_frame_idx_,
                                     clik_target, control_ref_.q_des, dt);
    clik_tcp_err_ = clik_.TcpErrorNorm();
    clik_manip_ = clik_.Manipulability();

    if (clik_compute_ok_) {
      // Δ between the two command candidates (integrator output is still in
      // q_next_full_/v_next_full_ at this point — capture before any overwrite).
      shadow_pos_delta_ = (clik_.QRef() - q_next_full_).norm();
      shadow_vel_delta_ = (clik_.VRef() - v_next_full_).norm();
      shadow_valid_ = true;
      if (src == CommandSource::kClik) {
        q_next_full_ = clik_.QRef();
        v_next_full_ = clik_.VRef();
        active_command_source_ = CommandSource::kClik;
      }
    } else if (src == CommandSource::kClik) {
      RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                           "[wbc] CLIK Compute failed — integrator fallback this tick");
    }
  }

  // 8. Map Pinocchio order → device order
  for (int i = 0; i < arm_dof_; ++i) {
    const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[static_cast<std::size_t>(i)]);
    robot_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    robot_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }
  for (int i = 0; i < hand_dof_; ++i) {
    const auto ext_i = static_cast<std::size_t>(arm_dof_ + i);
    const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[ext_i]);
    hand_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    hand_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }

  // Stage C-3: hand feedforward torque (kPdFeedforward). Model-based τ_ff that
  // the PD position backbone (hand_computed_.positions = hold/closure pose)
  // rides on. τ_ff = gravity_gain · src[hand] + closure_bias, where src is the
  // hand gravity vector g[nv] (kGravityComp, default) or the TSID-solved
  // actuated torque (kTsidTau, computed-torque FF) per hand_tauff_source. Active
  // only in closure/hold, clamped to ±tauff_max. Any non-finite → zero the whole
  // hand and fall back to a plain position hold this tick (conservative,
  // throttled WARN). Opt-in: hand_tauff_enable. The gravity vector g[nv] was
  // filled by pinocchio_cache_.Update() and tsid_output_.tau by the solve above,
  // both earlier this tick.
  // hand_tauff_active_ is reset to false once per tick at Compute() entry (#1,
  // single owner) — not here. This loop only clears stale feedforward indices
  // before the (possible) re-fill below; the flag is set true at :539 only when
  // every clamped torque is finite.
  for (int i = 0; i < hand_dof_; ++i)
    hand_computed_.feedforward[static_cast<std::size_t>(i)] = 0.0;
  if (gains_now.hand_tauff_enable && (phase_ == WbcPhase::kClosure || phase_ == WbcPhase::kHold)) {
    const double gain = gains_now.hand_tauff_gravity_gain;
    const double bias = gains_now.hand_tauff_closure_bias;
    // #3: clamp at point-of-use so a negative hand_tauff_max (e.g. a hand-edited
    // YAML reaching the declare path, which OnSet's std::max does not cover) can
    // never make std::clamp(tau, -tmax, tmax) have lo > hi (UB). 0 disables τ_ff.
    const double tmax = std::max(0.0, gains_now.hand_tauff_max);
    // τ_ff source, both indexed by the pinocchio v-index. kGravityComp = g[nv]
    // (pure gravity comp); kTsidTau = the TSID-solved actuated torque (already
    // QP-converged this tick, computed-torque FF). For the fixed-base control
    // model na==nv so tsid_output_.tau aligns with g[nv] on the same vidx; the
    // bounds guard below keeps either source safe if that assumption breaks.
    const Eigen::VectorXd& src_vec = (gains_now.hand_tauff_source == HandTauffSource::kTsidTau)
                                         ? tsid_output_.tau
                                         : pinocchio_cache_.g;
    bool finite_ok = true;
    for (int i = 0; i < hand_dof_ && finite_ok; ++i) {
      const auto ext_i = static_cast<std::size_t>(arm_dof_ + i);
      const auto vidx = static_cast<Eigen::Index>(ext_to_pin_v_[ext_i]);
      double tau = 0.0;
      if (vidx >= 0 && vidx < src_vec.size())
        tau = gain * src_vec[vidx] + bias;
      if (!std::isfinite(tau)) {
        finite_ok = false;
        break;
      }
      hand_computed_.feedforward[static_cast<std::size_t>(i)] = std::clamp(tau, -tmax, tmax);
    }
    if (finite_ok) {
      hand_tauff_active_ = true;
    } else {
      for (int i = 0; i < hand_dof_; ++i)
        hand_computed_.feedforward[static_cast<std::size_t>(i)] = 0.0;
      RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                           "[wbc] hand τ_ff non-finite — position-hold fallback this tick");
    }
  }

  // Stage B-5: WBC diagnostic (RT-safe throttled INFO). WBCDiagnostic struct
  // (separate SeqLock publisher) is deferred to a future stage — this single
  // throttled line is the interim observability surface. Format uses only
  // %d / %.2f / %.0f (no fmt::format / to_string / string concat — RT-3
  // throttle exception applies).
  RCLCPP_INFO_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                       "[wbc] solve=%.0fus phase=%d n_act=%d rank_G=%d", tsid_output_.solve_time_us,
                       static_cast<int>(phase_), n_lambda_active, grasp_cache_.Rank());
}

void DemoWbcController::ComputeReleaseMode(const ControllerState& state, double dt) noexcept {
  // Stage 0: TSID holds the arm SE3 at current TCP while ContactState ramps
  // activation toward 0 (set in OnPhaseEnter(kRelease)). Posture preset
  // damps hand at its current pose during this brief window.
  // Stage 1: lazily initialise the finger-open joint trajectory once the
  // contact ramp window has elapsed, then overlay it onto hand_computed_
  // after TSID's tick mapping. The arm continues on TSID SE3 hold.
  release_elapsed_s_ += dt;
  // Guard against unbounded accumulation if release_done_ never trips
  // (e.g., hand device drops mid-stage-1 → time advance loop is skipped).
  constexpr double kReleaseElapsedCapSec = 60.0;
  if (release_elapsed_s_ > kReleaseElapsedCapSec) {
    release_elapsed_s_ = kReleaseElapsedCapSec;
  }

  if (tsid_initialized_) {
    ComputeTSIDPosition(state, dt);
  } else {
    // !tsid_initialized_ implies LoadConfig was skipped (unit tests, init
    // failure). Hold the current sensed pose with zero velocity instead of
    // calling ComputePositionMode — that path reads robot_trajectory_,
    // which is only seeded by the Position-controller-style preset path
    // and never initialised on a kRelease entry, so it would feed stale
    // joint targets into the wire output. Production reaches this branch
    // only after init failure, but the unit-test path exercises it on
    // every preempt-into-kRelease, so the fresh-hold guard is mandatory.
    if (state.num_devices > 0 && state.devices[0].valid) {
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        robot_computed_.positions[idx] = state.devices[0].positions[idx];
        robot_computed_.velocities[idx] = 0.0;
      }
    }
    if (state.num_devices > 1 && state.devices[1].valid) {
      for (int i = 0; i < hand_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        hand_computed_.positions[idx] = state.devices[1].positions[idx];
        hand_computed_.velocities[idx] = 0.0;
      }
    }
  }

  if (release_stage_ == 0 && release_elapsed_s_ >= release_ramp_sec_) {
    if (state.num_devices > 1 && state.devices[1].valid) {
      const auto gains = gains_lock_.Load();
      trajectory::JointSpaceTrajectory<kMaxHandDof>::State hstart{};
      trajectory::JointSpaceTrajectory<kMaxHandDof>::State hgoal{};
      double hmax = 0.0;
      for (int i = 0; i < hand_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        hstart.positions[idx] = hand_computed_.positions[idx];
        hgoal.positions[idx] = 0.0;
        const double hd = std::abs(hstart.positions[idx]);
        if (hd > hmax) {
          hmax = hd;
        }
      }
      const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
      hand_trajectory_.initialize(hstart, hgoal, hdur);
      hand_trajectory_time_ = 0.0;
    } else {
      // No hand device → ramp window is the entire release sequence.
      release_done_ = true;
    }
    release_stage_ = 1;
  }

  if (release_stage_ == 1 && state.num_devices > 1 && state.devices[1].valid) {
    hand_trajectory_time_ += dt;
    const auto hstate =
        hand_trajectory_.compute(std::min(hand_trajectory_time_, hand_trajectory_.duration()));
    for (int i = 0; i < hand_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      hand_computed_.positions[idx] = hstate.positions[idx];
      hand_computed_.velocities[idx] = hstate.velocities[idx];
    }
    if (hand_trajectory_time_ >= hand_trajectory_.duration()) {
      release_done_ = true;
    }
  }
}

void DemoWbcController::ComputeFallback() noexcept {
  // Hold last computed positions (already in robot_computed_/hand_computed_)
  // Set velocities to zero
  for (int i = 0; i < arm_dof_; ++i) {
    robot_computed_.velocities[static_cast<std::size_t>(i)] = 0.0;
  }
  for (int i = 0; i < hand_dof_; ++i) {
    hand_computed_.velocities[static_cast<std::size_t>(i)] = 0.0;
  }
}

// ── Phase 3a: Write joint command (wire-bound only) ──────────────────────────

ControllerOutput DemoWbcController::WriteJointCommand(const ControllerState& state) noexcept {
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;
  output.valid = true;

  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = robot_computed_.positions[i];
  }
  rtc::utils::ClampRange(out0.commands, nc0, std::span<const double>(device_position_lower_[0]),
                         std::span<const double>(device_position_upper_[0]), -6.2832, 6.2832);

  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto& out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.commands[i] = hand_computed_.positions[i];
    }
    rtc::utils::ClampRange(out1.commands, nc1, std::span<const double>(device_position_lower_[1]),
                           std::span<const double>(device_position_upper_[1]), -6.2832, 6.2832);
    // Stage C-3: when hand τ_ff is active this tick, drive the hand device via
    // kPdFeedforward — the position commands above are the PD target (hold pose)
    // and feedforward carries the model torque. The arm device leaves its
    // per-device command_type unset (inherits the global kPosition default).
    if (hand_tauff_active_) {
      out1.command_type = CommandType::kPdFeedforward;
      // #9: hand_computed_.feedforward holds hand_dof_ model torques; copy only
      // that many. If the hand device reports nc1 > hand_dof_ channels, the tail
      // of out1.feedforward stays fresh-zero (no stale read past hand_dof_).
      const auto nff =
          std::min(static_cast<std::size_t>(nc1), static_cast<std::size_t>(std::max(hand_dof_, 0)));
      for (std::size_t i = 0; i < nff; ++i) {
        out1.feedforward[i] = hand_computed_.feedforward[i];
      }
    }
  }

  return output;
}

// ── Phase 3b: Fill log output ────────────────────────────────────────────────

void DemoWbcController::FillLogOutput(const ControllerState& state,
                                      ControllerOutput& output) noexcept {
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    out0.goal_positions[i] = current_target_slot_.targets[0][i];
  }

  // FK + actual_task_positions + task_goal_positions (log POD reads both).
  if (arm_handle_) {
    std::span<const double> q_span(dev0.positions.data(), static_cast<std::size_t>(nc0));
    arm_handle_->ComputeForwardKinematics(q_span);
    pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      tcp = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp);
    }
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
    output.actual_task_positions[0] = tcp.translation().x();
    output.actual_task_positions[1] = tcp.translation().y();
    output.actual_task_positions[2] = tcp.translation().z();
    output.actual_task_positions[3] = rpy[0];
    output.actual_task_positions[4] = rpy[1];
    output.actual_task_positions[5] = rpy[2];

    if (tcp_goal_valid_) {
      Eigen::Vector3d grpy = pinocchio::rpy::matrixToRpy(tcp_goal_.rotation());
      output.task_goal_positions[0] = tcp_goal_.translation().x();
      output.task_goal_positions[1] = tcp_goal_.translation().y();
      output.task_goal_positions[2] = tcp_goal_.translation().z();
      output.task_goal_positions[3] = grpy[0];
      output.task_goal_positions[4] = grpy[1];
      output.task_goal_positions[5] = grpy[2];
    } else {
      output.task_goal_positions = output.actual_task_positions;
    }
  }

  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto& out1 = output.devices[1];
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }

  // WBC state aggregates (per-fingertip + FSM phase). Staging buffer feeds
  // both the SeqLock publish path and GetWbcStateForTesting().
  {
    auto& ws = wbc_state_;
    ws.phase = static_cast<uint8_t>(phase_);
    ws.num_fingertips = num_active_fingertips_;
    int active_count = 0;
    float max_force = 0.0F;
    const auto g = gains_lock_.Load();
    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto idx = static_cast<std::size_t>(f);
      const auto& ft = fingertip_data_[idx];
      const float mag = ft.force_magnitude;
      ws.force_magnitude[idx] = mag;
      // contact_flag: native sigmoid probability if backend provides it
      // (sensor A path), else derived binary 1/0 from ft.in_contact.
      // displacement: native (slots 4..6) if backend provides it (sensor A);
      // else 0 — controller-side deformation derive is stubbed.
      ws.contact_flag[idx] = has_native_contact_ ? ft.contact_flag : (ft.in_contact ? 1.0F : 0.0F);
      ws.displacement[idx] = 0.0F;
      if (ft.in_contact) {
        ++active_count;
      }
      if (mag > max_force) {
        max_force = mag;
      }
    }
    ws.num_active_contacts = active_count;
    ws.max_force = max_force;
    ws.grasp_target_force = static_cast<float>(g.grasp_target_force);
    ws.min_fingertips_for_grasp = g.grasp_min_fingertips;
    ws.grasp_detected = (active_count >= ws.min_fingertips_for_grasp);
    ws.tsid_solver_ok = tsid_initialized_ && (qp_fail_count_ == 0);
    ws.qp_fail_count = qp_fail_count_;
  }
  // SeqLock store = two atomic stores + memcpy (wait-free, RT-safe).
  // Read by PublishNonRtSnapshot.
  wbc_state_lock_.Store(wbc_state_);
}

// ── Phase 3c: Fill publish output ────────────────────────────────────────────
//
// WBC's publish surface is narrower than joint/task: only arm_tip_pose for
// kRobotTransforms (no fingertip / virtual_tcp poses — WBC compute doesn't
// produce them). target_*/trajectory_*/goal_positions are also mirrored
// here so the publish snapshot reflects the canonical RT-tick values.

void DemoWbcController::FillPublishOutput(const ControllerState& state,
                                          ControllerOutput& output) noexcept {
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = robot_computed_.positions[i];
    out0.target_velocities[i] = robot_computed_.velocities[i];
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    out0.goal_positions[i] = current_target_slot_.targets[0][i];
  }

  if (arm_handle_) {
    pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      tcp = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp);
    }
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
    output.actual_task_positions[0] = tcp.translation().x();
    output.actual_task_positions[1] = tcp.translation().y();
    output.actual_task_positions[2] = tcp.translation().z();
    output.actual_task_positions[3] = rpy[0];
    output.actual_task_positions[4] = rpy[1];
    output.actual_task_positions[5] = rpy[2];

    if (tcp_goal_valid_) {
      Eigen::Vector3d grpy = pinocchio::rpy::matrixToRpy(tcp_goal_.rotation());
      output.task_goal_positions[0] = tcp_goal_.translation().x();
      output.task_goal_positions[1] = tcp_goal_.translation().y();
      output.task_goal_positions[2] = tcp_goal_.translation().z();
      output.task_goal_positions[3] = grpy[0];
      output.task_goal_positions[4] = grpy[1];
      output.task_goal_positions[5] = grpy[2];
    } else {
      output.task_goal_positions = output.actual_task_positions;
    }

    // TF source: arm tip only. Fingertip / virtual_tcp frames are not
    // produced by WBC compute — those slots stay invalid.
    {
      const Eigen::Vector3d& trans = tcp.translation();
      const Eigen::Quaterniond quat(tcp.rotation());
      output.arm_tip_pose.position = {trans.x(), trans.y(), trans.z()};
      output.arm_tip_pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
      output.arm_tip_pose_valid = true;
    }
  }

  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto& out1 = output.devices[1];
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.target_positions[i] = hand_computed_.positions[i];
      out1.target_velocities[i] = hand_computed_.velocities[i];
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = current_target_slot_.targets[1][i];
    }
  }
}

// ── Target management ────────────────────────────────────────────────────────

// ── E-STOP compute path ──────────────────────────────────────────────────────

ControllerOutput DemoWbcController::ComputeEstop(const ControllerState& state) noexcept {
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.valid = true;
  output.command_type = command_type_;

  // Arm: ramp toward safe position with per-joint velocity limit (matches
  // DemoJoint/DemoTask ComputeEstop pattern — instant jump risks hardware
  // damage on real UR5e at high E-STOP delta).
  auto& out0 = output.devices[0];
  out0.num_channels = dev0.num_channels;
  out0.goal_type = GoalType::kJoint;
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  for (int i = 0; i < arm_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const double lim = (idx < device_max_velocity_[0].size()) ? device_max_velocity_[0][idx] : 2.0;
    out0.commands[idx] =
        dev0.positions[idx] + std::clamp(safe_position_[idx] - dev0.positions[idx], -lim, lim) * dt;
    out0.target_positions[idx] = out0.commands[idx];
  }

  // Hold current position (hand). E-8: this is a fresh ControllerOutput, so the
  // hand device's per-device command_type stays unset (→ plain position hold)
  // and feedforward stays zero — the hand τ_ff path is never engaged under
  // E-STOP regardless of phase/enable. Do NOT set kPdFeedforward or copy
  // feedforward here; zeroed torque on E-STOP is the safety contract.
  if (state.num_devices > 1 && state.devices[1].valid) {
    auto& out1 = output.devices[1];
    out1.num_channels = state.devices[1].num_channels;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(state.devices[1].num_channels) && i < kMaxDeviceChannels;
         ++i) {
      out1.commands[i] = state.devices[1].positions[i];
      out1.target_positions[i] = state.devices[1].positions[i];
    }
  }

  return output;
}

// ── Utility ──────────────────────────────────────────────────────────────────

// ── Helpers (full state extraction, TCP error, MPC timing log) ──────────────

void DemoWbcController::ExtractFullState(const ControllerState& state) noexcept {
  if (!joint_reorder_valid_) {
    return;
  }

  const auto& dev0 = state.devices[0];
  // Arm joints: external [0..arm_dof_-1] → Pinocchio order
  for (int i = 0; i < arm_dof_; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
    const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
    q_curr_full_[pq] = dev0.positions[eidx];
    v_curr_full_[pv] = dev0.velocities[eidx];
  }

  // Hand joints: external [arm_dof_..full_dof_-1] → Pinocchio order
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    for (int i = 0; i < hand_dof_; ++i) {
      const auto eidx = static_cast<std::size_t>(arm_dof_ + i);
      const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
      const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
      q_curr_full_[pq] = dev1.positions[static_cast<std::size_t>(i)];
      v_curr_full_[pv] = dev1.velocities[static_cast<std::size_t>(i)];
    }
  }
}

void DemoWbcController::BuildTargetPosture(const ControllerState& state) noexcept {
  if (!joint_reorder_valid_) {
    return;
  }
  // Arm target: external [0..arm_dof_-1] → Pinocchio order (mirrors ExtractFullState).
  for (int i = 0; i < arm_dof_; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
    q_des_target_full_[pq] = current_target_slot_.targets[0][eidx];
  }
  // Hand target: external [arm_dof_..full_dof_-1] → Pinocchio order.
  if (state.num_devices > 1 && state.devices[1].valid) {
    for (int i = 0; i < hand_dof_; ++i) {
      const auto eidx = static_cast<std::size_t>(arm_dof_ + i);
      const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
      q_des_target_full_[pq] = current_target_slot_.targets[1][static_cast<std::size_t>(i)];
    }
  }
}

void DemoWbcController::SeedHoldFromMeasured(const ControllerState& state) noexcept {
  // joint_goal mirror = current measured config (arm + hand). This makes the
  // logged joint_goal match the posture reference idle actually regulates to.
  const auto& dev0 = state.devices[0];
  for (int i = 0; i < arm_dof_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    current_target_slot_.targets[0][idx] = dev0.positions[idx];
  }
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    for (int i = 0; i < hand_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      current_target_slot_.targets[1][idx] = dev1.positions[idx];
    }
  }
  // Posture reference snapshot (external order → Pinocchio order). No-op until
  // joint_reorder_valid_; the kIdle posture-ref fallback covers that window.
  BuildTargetPosture(state);
  // SE3 hold pose at the current measured FK (base_frame → tip), persisted to
  // the SeqLock POD so the next-tick DrainTargetSlot restore keeps it valid.
  if (arm_handle_) {
    std::span<const double> q_arm(dev0.positions.data(), static_cast<std::size_t>(arm_dof_));
    arm_handle_->ComputeForwardKinematics(q_arm);
    tcp_goal_ = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      tcp_goal_ = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_goal_);
    }
    tcp_goal_valid_ = true;
    current_target_slot_.tcp_goal_valid = true;
    std::memcpy(current_target_slot_.tcp_goal_rot.data(), tcp_goal_.rotation().data(),
                sizeof(current_target_slot_.tcp_goal_rot));
    std::memcpy(current_target_slot_.tcp_goal_t.data(), tcp_goal_.translation().data(),
                sizeof(current_target_slot_.tcp_goal_t));
  }
}

double DemoWbcController::ComputeTcpError(const pinocchio::SE3& target) noexcept {
  if (!arm_handle_) {
    return 1e10;
  }
  const pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
  return (tcp.translation() - target.translation()).norm();
}

// ── Controller-owned topic lifecycle ──────────────────────────────────────

void DemoWbcController::LogMpcSolveTimingTick() noexcept {
  if (!mpc_manager_.Enabled()) {
    return;
  }

  // Drain every pending per-MPC-tick sample into the CSV. One row per
  // tick preserves full sampling-rate granularity; the file grows at
  // (MPC frequency × session_seconds) rows. Producer is the MPCThread
  // main loop (see rtc_mpc/thread/mpc_thread.cpp::OnTick).
  if (mpc_thread_) {
    mpc_thread_->TimingProducer().Drain(
        [this](const rtc::RtTickTimingSample& s) { mpc_timing_logger_.Log(s); });
  }

  // Periodic aggregate INFO so tmux-watchers see progress without
  // tail-ing the CSV. The window is computed by MPCSolutionManager over
  // its 256-sample ring (handler-side solve_duration_ns); 10 s INFO
  // cadence keeps the console readable across a 10-minute pilot session.
  const auto stats = mpc_manager_.GetSolveStats();
  static constexpr std::uint32_t kInfoEveryNTicks = 10;
  if (++mpc_timing_tick_ % kInfoEveryNTicks == 0) {
    RCLCPP_INFO(logger_,
                "[mpc_timing] count=%lu window=%u p50=%.2fms p99=%.2fms "
                "max=%.2fms",
                static_cast<unsigned long>(stats.count), static_cast<unsigned>(stats.window),
                static_cast<double>(stats.p50_ns) / 1e6, static_cast<double>(stats.p99_ns) / 1e6,
                static_cast<double>(stats.max_ns) / 1e6);
  }
}

// ── WBC CSV fill (controller-private) ──────────────────────────────────────
// Mirrors FillDeviceStateLogPod for the shared joint-space block, then adds
// the WBC-specific fields that ControllerOutput does not carry: TSID a_opt
// acceleration (device slice), SE3 quintic-ramp setpoint (arm role), and
// per-fingertip |F| (hand role). RT-safe: bounded loops, no alloc, no throw.
void DemoWbcController::FillDeviceWbcLogPod(
    const ControllerState& state, const ControllerOutput& output, std::size_t device_idx,
    std::uint8_t role, ::integrated_bringup::DeviceWbcLogPod& pod) const noexcept {
  pod.role = role;
  pod.t_relative_s = state.t_relative_s;
  if (static_cast<std::size_t>(state.num_devices) <= device_idx) {
    return;
  }
  const auto& dev = state.devices[device_idx];
  const auto& out = output.devices[device_idx];
  const auto n = std::min(static_cast<std::size_t>(dev.num_channels),
                          ::integrated_bringup::DeviceWbcLogPod::kMaxJoints);
  pod.num_joints = static_cast<std::uint8_t>(n);
  for (std::size_t i = 0; i < n; ++i) {
    pod.actual_positions[i] = dev.positions[i];
    pod.actual_velocities[i] = dev.velocities[i];
    pod.efforts[i] = dev.efforts[i];
    pod.commands[i] = out.commands[i];
    pod.joint_goal[i] = out.goal_positions[i];
    pod.trajectory_positions[i] = out.trajectory_positions[i];
    pod.trajectory_velocities[i] = out.trajectory_velocities[i];
  }

  // accelerations: TSID solution a_opt sliced to this device. External order
  // [arm0.., hand0..] → Pinocchio index via ext_to_pin_q_, reusing the exact
  // mapping the integrator applies to v_next_full_ (compute.cpp:413-427). For
  // these fixed-base demos q-index == v-index per joint, so the q-order map
  // indexes the nv-sized a_opt correctly.
  const int dof = (role == 0) ? arm_dof_ : hand_dof_;
  const int ext_base = (role == 0) ? 0 : arm_dof_;
  const auto& a = tsid_output_.a_opt;
  if (a.size() > 0) {
    const auto count = std::min(static_cast<std::size_t>(std::max(dof, 0)), n);
    for (std::size_t i = 0; i < count; ++i) {
      const auto ext_i = static_cast<std::size_t>(ext_base + static_cast<int>(i));
      const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[ext_i]);
      if (static_cast<Eigen::Index>(pin_idx) < a.size()) {
        pod.accelerations[i] = a[static_cast<Eigen::Index>(pin_idx)];
      }
    }
  }

  if (role == 0) {
    // arm: SE3 task block (goal + actual from output, ramp setpoint from
    // tcp_traj_state_). Order [x,y,z,roll,pitch,yaw] matches Motion toVector
    // [linear; angular].
    for (std::size_t i = 0; i < ::integrated_bringup::DeviceWbcLogPod::kTaskDim; ++i) {
      pod.task_goal[i] = output.task_goal_positions[i];
      pod.actual_task_positions[i] = output.actual_task_positions[i];
    }
    const auto& tpose = tcp_traj_state_.pose;
    const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tpose.rotation());
    pod.trajectory_task_positions[0] = tpose.translation().x();
    pod.trajectory_task_positions[1] = tpose.translation().y();
    pod.trajectory_task_positions[2] = tpose.translation().z();
    pod.trajectory_task_positions[3] = rpy[0];
    pod.trajectory_task_positions[4] = rpy[1];
    pod.trajectory_task_positions[5] = rpy[2];
    const auto tvel = tcp_traj_state_.velocity.toVector();
    for (std::size_t i = 0; i < ::integrated_bringup::DeviceWbcLogPod::kTaskDim; ++i) {
      pod.trajectory_task_velocities[i] = tvel[static_cast<Eigen::Index>(i)];
    }
  } else {
    // hand: motor state + per-fingertip |F| from the wbc_state_ staging buffer
    // (filled earlier this tick in ComputeControl).
    const auto nm = std::min(static_cast<std::size_t>(dev.num_motor_channels),
                             ::integrated_bringup::DeviceWbcLogPod::kMaxMotors);
    pod.num_motors = static_cast<std::uint8_t>(nm);
    for (std::size_t i = 0; i < nm; ++i) {
      pod.motor_positions[i] = dev.motor_positions[i];
      pod.motor_velocities[i] = dev.motor_velocities[i];
      pod.motor_efforts[i] = dev.motor_efforts[i];
    }
    const auto nf = std::min(static_cast<std::size_t>(std::max(num_active_fingertips_, 0)),
                             ::integrated_bringup::DeviceWbcLogPod::kMaxFingertips);
    pod.num_fingertips = static_cast<std::uint8_t>(nf);
    for (std::size_t i = 0; i < nf; ++i) {
      pod.fingertip_force[i] = static_cast<double>(wbc_state_.force_magnitude[i]);
    }
  }

  // #4: log the PER-DEVICE command type (out.command_type), not the global
  // ControllerOutput default — otherwise the hand's kPdFeedforward never shows
  // up in the diag (goal_type already uses the per-device out.goal_type). Falls
  // back to the global default when the device leaves command_type unset.
  const auto dev_ct = out.command_type.value_or(output.command_type);
  pod.command_type = (dev_ct == rtc::CommandType::kPdFeedforward) ? 2
                     : (dev_ct == rtc::CommandType::kTorque)      ? 1
                                                                  : 0;
  pod.goal_type = (out.goal_type == rtc::GoalType::kTask) ? 1 : 0;
}

void DemoWbcController::FillWbcDiagLogPod(const ControllerState& state,
                                          ::integrated_bringup::WbcDiagLogPod& pod) const noexcept {
  pod.t_relative_s = state.t_relative_s;
  pod.phase = static_cast<std::uint8_t>(phase_);
  pod.solve_time_us = tsid_output_.solve_time_us;
  pod.solve_levels = tsid_output_.solve_levels;
  pod.qp_fail_count = qp_fail_count_;
  pod.qp_converged = tsid_output_.qp_converged;
  pod.num_active_contacts = wbc_state_.num_active_contacts;
  pod.grasp_detected = wbc_state_.grasp_detected;
  pod.max_force = wbc_state_.max_force;
  // Stage C-2: command source + CLIK A/B shadow (set in ComputeTSIDPosition
  // step 7b this tick; zero/false on non-TSID phases — kFallback never reaches
  // FillWbcDiagLogPod via a fresh solve).
  pod.command_source = static_cast<std::uint8_t>(active_command_source_);
  pod.clik_valid = clik_compute_ok_;
  pod.shadow_valid = shadow_valid_;
  pod.clik_tcp_err = clik_tcp_err_;
  pod.clik_manipulability = clik_manip_;
  pod.shadow_pos_delta = shadow_pos_delta_;
  pod.shadow_vel_delta = shadow_vel_delta_;
  // lambda_opt is the fixed-dim QP contact solution (size == max_contact_vars,
  // matching the header's λ column count registered in on_configure).
  const auto& lam = tsid_output_.lambda_opt;
  const auto nlam = std::min(static_cast<std::size_t>(lam.size()),
                             ::integrated_bringup::WbcDiagLogPod::kMaxContactVars);
  pod.num_contact_vars = static_cast<std::uint8_t>(nlam);
  for (std::size_t i = 0; i < nlam; ++i) {
    pod.lambda_opt[i] = lam[static_cast<Eigen::Index>(i)];
  }
}

}  // namespace integrated_bringup
