#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/controllers/fingertip_counts.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_controller_interface/device_readability.hpp"
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
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Phase 1: Read state ─────────────────────────────────────────────────────
void DemoWbcController::ReadState(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ReadState");
  // ── Unified kin&dyn (option C): scatter measured joint pos/vel into Pinocchio
  // order (combined_cache_.q()/combined_cache_.v()). A raw read + reindex, so it belongs to
  // ReadState; ComputeControl Stage 1 consumes it via combined_cache_.Update().
  // Gated on cache-readiness (tsid_initialized_ && combined_cache_.reorder_valid()), matching
  // the prior ComputeControl gate. ReadState runs before the E-STOP /
  // !target_initialized early-returns, so this may scatter on those ticks too, but
  // every RT reader of combined_cache_.q()/combined_cache_.v() lives in ComputeControl's FSM
  // subtree (never reached on those paths) — controller output stays byte-for-byte,
  // and the next FSM tick re-scatters fresh values.
  if (tsid_initialized_ && combined_cache_.reorder_valid()) {
    combined_cache_.ExtractFullState(state, arm_dof_, hand_dof_);
  }

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
  // Shared derivation (joint / task / wbc): prefer the inference-group count,
  // fall back to the sensor-channel stride. See DeriveFingertipCounts. The wbc
  // controller has no ToF snapshot, so only the active (inference) count is used.
  num_active_fingertips_ = DeriveFingertipCounts(dev1).active;

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
  RTC_TRACE_SCOPE("DemoWbcController::ComputeControl");
  // ── Stage 1 (compute model): per-tick unconditional cache refresh. ExtractFullState
  // (raw read + reindex → combined_cache_.q()/combined_cache_.v()) ran in ReadState; here we
  // consume it to refresh FK/J/M/h/g/oMf so arm/contact oMf follow measured on kFallback and every
  // non-TSID tick too (pose fresh; command is held by each phase). estop and !target_initialized
  // seed are handled by Compute()'s early-returns, so gate on cache-readiness (tsid_initialized_ &&
  // combined_cache_.reorder_valid()) only. UpdatePhase (FSM; its ComputeTcpError reads the PREVIOUS
  // tick's oMf — one-tick lag contract) ran in Compute() BEFORE this Stage-1 Update, so the lag is
  // preserved → TSID-routing phase byte-for-byte. cache.Update is ContactState-independent (Phase 1
  // always-compute) so it may precede the contact_state_ update below (option C, hoisted
  // UpdatePhase).
  if (tsid_initialized_ && combined_cache_.reorder_valid()) {
    combined_cache_.Update();
  }

  // MPC WriteState stays in ComputeWbcCommon (TSID paths only): kFallback still
  // does not push MPC state; the MPC retains its last snapshot until recovery to
  // kIdle, which is safe because MPC output is not driving control during
  // fallback and the dim-mismatch gate (state.nq == model_->nq()) keys on nq,
  // not staleness.
  switch (phase_) {
    case WbcPhase::kIdle:
    case WbcPhase::kApproach:
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

  // #167: after the FSM dispatch so TSID ticks see this tick's contact
  // frames; non-TSID ticks decay the estimate via invalid inputs.
  UpdatePullEstimate(dt);

  // ── Hand fingertip FK (publish-surface TF) ───────────────────────────────
  // Computed here (not in FillPublishOutput) so the publish path only READS
  // cached poses, never solves kinematics — matching joint/task. Uses the arm
  // TCP oMf refreshed by the Stage-1 Update above (clik_tcp / clik_base
  // registered frames). fingertip_pose_valid_ is reset each tick so a failed
  // hand-FK tick withholds every fingertip TF (FillPublishOutput now gates on
  // these flags, not on ComputeHandFingertipFk's return).
  fingertip_pose_valid_.fill(false);
  if (clik_tcp_frame_idx_ >= 0) {
    pinocchio::SE3 tcp = combined_cache_.cache()
                             .registered_frames[static_cast<std::size_t>(clik_tcp_frame_idx_)]
                             .oMf;
    if (clik_base_frame_idx_ >= 0) {
      tcp = combined_cache_.cache()
                .registered_frames[static_cast<std::size_t>(clik_base_frame_idx_)]
                .oMf.actInv(tcp);
    }
    (void)ComputeHandFingertipFk(state, tcp);
  }
}

// ── In-plane pull-force estimate (#167) ──────────────────────────────────────
//
// Measured per-fingertip forces rotated by the TSID contact-frame geometry —
// deliberately NOT the QP's λ_opt, which reflects the commanded force
// distribution rather than the sensed external load. Contact k's slot indexes
// both tsid.contacts (→ combined_cache_.cache().contact_frames) and the fingertip
// sensor lanes (documented 1:1 mapping, see ComputeWbcCommon Stage A-3).
// Gating: contact_geometry_fresh_ (a TSID-routing tick ran ComputeWbcCommon —
// the trust signal for the pull estimate, NOT oMf freshness: Task B refreshes
// contact_frames[].oMf every tick incl. kFallback) AND the per-contact active
// flag. cache.Update always-computes ALL registered contact frames (Phase 1), so
// inactive frames are geometrically fresh too; the active gate is a policy choice.

void DemoWbcController::UpdatePullEstimate(double dt) noexcept {
  if (!pull_wiring_.enabled()) {
    return;
  }
  RTC_TRACE_SCOPE("DemoWbcController::UpdatePullEstimate");
  for (int k = 0; k < pull_wiring_.num_contacts; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    const int ci = pull_wiring_.slot[ki];
    const auto cs = static_cast<std::size_t>(ci);
    auto& in = pull_wiring_.inputs[ki];
    const bool geom_ok = contact_geometry_fresh_ && cs < contact_state_.contacts.size() &&
                         contact_state_.contacts[cs].active &&
                         cs < combined_cache_.cache().contact_frames.size();
    const auto& ft = fingertip_data_[cs];
    const bool sensor_ok = ci < num_active_fingertips_ && ft.valid && std::isfinite(ft.force[0]) &&
                           std::isfinite(ft.force[1]) && std::isfinite(ft.force[2]);
    in.valid = geom_ok && sensor_ok;
    pull_wiring_.position_valid[ki] = geom_ok;
    if (geom_ok) {
      const pinocchio::SE3& oMf = combined_cache_.cache().contact_frames[cs].oMf;
      pull_wiring_.positions[ki] = oMf.translation();
      if (in.valid) {
        in.rotation = oMf.rotation();
        in.force =
            Eigen::Vector3d(static_cast<double>(ft.force[0]), static_cast<double>(ft.force[1]),
                            static_cast<double>(ft.force[2]));
      }
    }
  }

  // Baseline arming edge mirrors ws.grasp_detected (FillLogOutput): active
  // in_contact count vs the shared grasp_min_fingertips threshold.
  const auto g = gains_lock_.Load();
  int active_count = 0;
  for (int f = 0; f < num_active_fingertips_; ++f) {
    if (fingertip_data_[static_cast<std::size_t>(f)].in_contact) {
      ++active_count;
    }
  }
  const bool grasp_detected = active_count >= g.grasp_min_fingertips;

  const rtc::grasp::PullEstimate& pull_est = UpdatePullEstimator(pull_wiring_, grasp_detected, dt);
  rtc::grasp::FillPullEstimateData(pull_est, wbc_state_.pull);
}

// ── FSM ──────────────────────────────────────────────────────────────────────

// ── Position-mode and TSID solvers ──────────────────────────────────────────

void DemoWbcController::ComputePositionMode(double dt) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputePositionMode");
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
}

void DemoWbcController::ComputeTSIDPosition(const ControllerState& state, double dt) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputeTSIDPosition");
  // Orchestrator: common stage (references) → Kinematic WBC (CLIK-QP position
  // backbone) → Dynamic WBC (TSID solve + hand τ_ff). One gains snapshot is
  // shared by all three so the whole tick sees a consistent set of runtime
  // gains. The two QPs consume the same Common-stage references independently
  // (decision 5): Kinematic owns position, Dynamic owns the hand τ_ff overlay,
  // so neither depends on the other's output. A Dynamic QP failure is
  // non-critical (decision 6: τ_ff drop only); position criticality lives in
  // ComputeKinematicWbc (CLIK is the sole position backbone).
  const Gains gains_now = gains_lock_.Load();
  ComputeWbcCommon(state, dt, gains_now);
  ComputeKinematicWbc(dt, gains_now);
  ComputeDynamicWbc(gains_now);
}

// ComputeWbcCommon — shared per-tick stage (decision 5): state extraction,
// pinocchio + contact/grasp cache, MPC reference, and the joint/SE3 references
// both QPs consume. Produces no command and runs no solve; ComputeDynamicWbc
// and ComputeKinematicWbc read its outputs (tsid cache, control_ref_,
// tcp_traj_state_, n_lambda_active_).
void DemoWbcController::ComputeWbcCommon(const ControllerState& state, double dt,
                                         const Gains& gains_now) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputeWbcCommon");
  // 1. State extraction + Pinocchio cache refresh moved to ComputeControl (Task B,
  //    옵션 B): ExtractFullState + combined_cache_.Update() now run once per tick
  //    for ALL phases just before this stage, so combined_cache_.q()/combined_cache_.v() and the
  //    cache (M/h/g/Jacobians/contact & registered frame oMf) are already fresh here.
  //    This stage only CONSUMES them (grasp/MPC/references). #unified-kindyn Phase 1:
  //    cache is ContactState-independent (always-compute), so running Update before the
  //    contact_state_ update below is safe.

  // Stage A-5b: progress per-contact activation ramp by dt. ContactState
  // auto-flips the legacy `active : bool` once s_i crosses kActivationDeadband
  // so downstream skip paths (active_contact_vars, task/constraint early
  // outs) stay coherent.
  contact_state_.UpdateActivation(dt);
  contact_state_.RecomputeActive(contact_mgr_config_);

  // 2. #167: mark contact geometry trusted for THIS (TSID-routing) tick so the
  // pull estimator consumes it. cache.Update already refreshed contact_frames[].oMf
  // for ALL contacts in ComputeControl; non-TSID ticks leave this false (bounded
  // decay). UpdatePullEstimate additionally gates per contact on the active flag.
  contact_geometry_fresh_ = true;

  // 2a. Stage B-5: populate the once-per-tick grasp cache shared across the
  // three object-level tasks (ObjectWrenchTask / InternalForceTask /
  // ObjectSE3Task). Order is invariant per the plan handoff:
  //   cache.Update → UpdateActivation → RecomputeActive →
  //   ActiveLambdaDim → ComputeGraspMatrix(G_workspace_) → grasp_cache_.Compute
  // Tasks must never re-Compute on their own — they only read GPinv/GTPinv/
  // ProjN/Rank from grasp_cache_. n_active=0 (idle/approach) leaves the
  // cache empty (Rank()=0); object-level tasks then report ResidualDim=0.
  n_lambda_active_ = contact_mgr_.ActiveLambdaDim(contact_state_);
  if (n_lambda_active_ > 0) {
    auto G_view = grasp_G_workspace_.leftCols(n_lambda_active_);
    contact_mgr_.ComputeGraspMatrix(combined_cache_.cache(), contact_state_, object_frame_, G_view);
    grasp_cache_.Compute(G_view, n_lambda_active_);
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
    // Synthetic monotonic tick clock for the MPC staleness / dim-mismatch gates:
    // iteration × tick period, derived from dt so it tracks the configured
    // control_rate (100 Hz–5 kHz) instead of a hardcoded 500 Hz. Pushes fresh
    // (q, v) to the MPC thread, then consumes the freshest interpolated solution.
    const uint64_t now_ns =
        static_cast<uint64_t>(state.iteration) * static_cast<uint64_t>(dt * 1e9);
    rtc::mpc::InterpMeta meta;
    mpc_manager_.WriteState(combined_cache_.q(), combined_cache_.v(), now_ns);
    mpc_ref_valid =
        mpc_manager_.ComputeReference(combined_cache_.q(), combined_cache_.v(), now_ns, mpc_q_ref_,
                                      mpc_v_ref_, mpc_a_ff_, mpc_lambda_ref_, mpc_u_fb_, meta);
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
      control_ref_.q_des = combined_cache_.q();
    } else if (phase_ == WbcPhase::kIdle) {
      control_ref_.q_des =
          combined_cache_.reorder_valid() ? q_des_target_full_ : combined_cache_.q();
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

  // 4. Build ControlState (consumed by the Dynamic-WBC solve next).
  ctrl_state_.q = combined_cache_.q();
  ctrl_state_.v = combined_cache_.v();
  ctrl_state_.timestamp_ns = state.iteration;
}

// ComputeKinematicWbc — Kinematic WBC: arm/hand position via the CLIK-QP
// reference (q_ref = q + v_ref·dt, measured-anchored on the cache updated in
// ComputeWbcCommon), then Pinocchio→device order mapping. CLIK is the SOLE
// position backbone (the integrator A/B shadow was removed once its numerical
// equivalence was verified): a CLIK failure is therefore CRITICAL.
void DemoWbcController::ComputeKinematicWbc(double dt, const Gains& gains_now) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputeKinematicWbc");
  clik_compute_ok_ = false;
  clik_tcp_err_ = 0.0;
  clik_manip_ = 0.0;

  // clik_enabled_ is guaranteed true on the accepted config path: InitClik
  // enforces the nq==nv CLIK contract and on_configure FAILS the lifecycle
  // transition otherwise (DEC-1 ⓐ — no integrator fallback remains). The guard
  // is kept defensive; if it ever does not hold, hold last (early return below).
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

    // Re-anchor q_ref to measured on a goal/phase edge (clik_reseed_pending_);
    // otherwise CLIK carries forward from the previous desired. Consume the flag
    // only when CLIK actually solves so a skipped tick keeps the pending reseed.
    const bool reseed_anchor = clik_reseed_pending_;
    clik_reseed_pending_ = false;
    clik_compute_ok_ =
        clik_.Compute(combined_cache_.cache(), clik_tcp_frame_idx_, clik_base_frame_idx_,
                      clik_target, control_ref_.q_des, dt, reseed_anchor);
    clik_tcp_err_ = clik_.TcpErrorNorm();
    clik_manip_ = clik_.Manipulability();

    if (!clik_compute_ok_) {
      // CLIK is the position backbone (decision 6): a failure is CRITICAL.
      // robot_computed_/hand_computed_ still hold the previous tick's command
      // (the Pinocchio→device mapping below has not run yet), so an early return
      // holds last this tick. After max_qp_fail_before_fallback_ consecutive
      // fails, trip kFallback (ComputeFallback zeroes velocity on the held pose).
      ++kin_qp_fail_count_;
      RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                           "[wbc] CLIK QP failed (%d/%d) — holding last command this tick",
                           kin_qp_fail_count_, max_qp_fail_before_fallback_);
      if (kin_qp_fail_count_ >= max_qp_fail_before_fallback_) {
        phase_ = WbcPhase::kFallback;
        ComputeFallback();
      }
      return;  // skip the device mapping → robot_computed_/hand_computed_ hold last.
    }
    kin_qp_fail_count_ = 0;
  } else {
    // No CLIK / no valid goal: nothing to drive position → hold last command.
    return;
  }

  // 8. Map Pinocchio order → device order (CLIK reference is the position).
  const auto& q_ref = clik_.QRef();
  const auto& v_ref = clik_.VRef();
  for (int i = 0; i < arm_dof_; ++i) {
    const auto pin_idx = static_cast<std::size_t>(combined_cache_.ext_to_pin_q(i));
    robot_computed_.positions[static_cast<std::size_t>(i)] =
        q_ref[static_cast<Eigen::Index>(pin_idx)];
    robot_computed_.velocities[static_cast<std::size_t>(i)] =
        v_ref[static_cast<Eigen::Index>(pin_idx)];
  }
  for (int i = 0; i < hand_dof_; ++i) {
    const auto ext_i = static_cast<std::size_t>(arm_dof_ + i);
    const auto pin_idx =
        static_cast<std::size_t>(combined_cache_.ext_to_pin_q(static_cast<int>(ext_i)));
    hand_computed_.positions[static_cast<std::size_t>(i)] =
        q_ref[static_cast<Eigen::Index>(pin_idx)];
    hand_computed_.velocities[static_cast<std::size_t>(i)] =
        v_ref[static_cast<Eigen::Index>(pin_idx)];
  }
}

// ComputeDynamicWbc — Dynamic WBC (TSID-ID QP): solves the whole-body
// acceleration/torque QP (fills tsid_output_.a_opt / tau), then overlays the
// hand feedforward torque (kPdFeedforward). A QP failure is NON-critical
// (decision 6): position is owned by the Kinematic CLIK-QP, so a failure only
// drops the hand τ_ff this tick (throttled WARN) and returns — it never trips
// kFallback nor skips the Kinematic layer. Runs before ComputeKinematicWbc
// (decision 7) so the integrator A/B shadow consumes this-tick a_opt; on a
// failed solve that a_opt is garbage, but the shadow is non-driving (CLIK is
// primary) so only the diagnostic Δ spikes.
void DemoWbcController::ComputeDynamicWbc(const Gains& gains_now) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputeDynamicWbc");
  // ── Dynamic WBC QP solve (was the SolveWbcQp tail) ────────────────────────
  // 5. TSID solve — consumes the common-stage ctrl_state_/control_ref_/cache.
  tsid_output_ =
      tsid_controller_.Compute(ctrl_state_, control_ref_, combined_cache_.cache(), contact_state_);

  // 6. Dynamic QP failure handling — non-critical (τ_ff drop only).
  if (!tsid_output_.qp_converged) {
    ++dyn_qp_fail_count_;
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                         "[wbc] Dynamic QP failed (%d), τ_ff dropped this tick, solve=%.0fus",
                         dyn_qp_fail_count_, tsid_output_.solve_time_us);
    // Drop the hand feedforward this tick; the Kinematic CLIK-QP still produces
    // the position command (hand falls back to its plain PD position hold).
    for (int i = 0; i < hand_dof_; ++i)
      hand_computed_.feedforward[static_cast<std::size_t>(i)] = 0.0;
    hand_tauff_active_ = false;
    return;
  }
  dyn_qp_fail_count_ = 0;

  // Stage B-5: WBC diagnostic (RT-safe throttled INFO) — QP solve summary.
  // Format uses only %d / %.0f (no fmt::format / to_string / string concat —
  // RT-3 throttle exception applies). n_lambda_active_ from the common stage.
  RCLCPP_INFO_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                       "[wbc] solve=%.0fus phase=%d n_act=%d rank_G=%d", tsid_output_.solve_time_us,
                       static_cast<int>(phase_), n_lambda_active_, grasp_cache_.Rank());

  // ── Dynamic WBC (hand) — feedforward torque overlay ───────────────────────
  // Stage C-3: hand feedforward torque (kPdFeedforward). The "Dynamic WBC" path
  // (hand-only): a model-based τ_ff overlaid on the kinematic/PD position
  // backbone (hand_computed_.positions = hold/closure pose), distinct from the
  // Kinematic WBC (CLIK) arm path above.
  // τ_ff = gravity_gain · src[hand] + closure_bias, where src is the
  // hand gravity vector g[nv] (kGravityComp, default) or the TSID-solved
  // actuated torque (kTsidTau, computed-torque FF) per hand_tauff_source. Active
  // only in closure/hold, clamped to ±tauff_max. Any non-finite → zero the whole
  // hand and fall back to a plain position hold this tick (conservative,
  // throttled WARN). Opt-in: hand_tauff_enable. The gravity vector g[nv] was
  // filled by combined_cache_.Update() and tsid_output_.tau by the solve above,
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
                                         : combined_cache_.cache().g;
    bool finite_ok = true;
    for (int i = 0; i < hand_dof_ && finite_ok; ++i) {
      const auto ext_i = static_cast<std::size_t>(arm_dof_ + i);
      const auto vidx =
          static_cast<Eigen::Index>(combined_cache_.ext_to_pin_v(static_cast<int>(ext_i)));
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
}

void DemoWbcController::ComputeReleaseMode(const ControllerState& state, double dt) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputeReleaseMode");
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
    // IsDeviceReadable, not `valid` (#265 audit W5): this loop runs arm_dof_
    // deep, so a device that reported fewer channels would hold the unreported
    // joints at 0 — a "fresh hold" at the origin.
    if (state.num_devices > 0 && arm_readable_) {
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        robot_computed_.positions[idx] = state.devices[0].positions[idx];
        robot_computed_.velocities[idx] = 0.0;
      }
    }
    // Same on the hand axis (#291): hand_dof_ deep, so an unreported finger
    // would be recorded as a "fresh hold" at the origin — the same defect the
    // arm gate above closes, on the device this branch also touches.
    if (hand_readable_) {
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
  RTC_TRACE_SCOPE("DemoWbcController::ComputeFallback");
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
  RTC_TRACE_SCOPE("DemoWbcController::WriteJointCommand");
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;
  output.valid = true;

  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.goal_type = GoalType::kJoint;
  if (arm_readable_) {
    out0.num_channels = nc0;
    // robot_computed_ is only written [0, arm_dof_) by every FSM branch, so
    // channels the device reports beyond the model were left at a fresh zero —
    // "go to the origin" on a position lane (#265 audit W6). The tail policy is
    // what they actually get.
    const int nq = rtc::ModelChannelBound(nc0, arm_dof_);
    for (std::size_t i = 0; i < static_cast<std::size_t>(nq); ++i) {
      out0.commands[i] = robot_computed_.positions[i];
    }
    rtc::FillCommandTail(out0.commands, nq, nc0, command_type_, dev0.positions);
    rtc::utils::ClampRange(out0.commands, nc0, std::span<const double>(device_position_lower_[0]),
                           std::span<const double>(device_position_upper_[0]),
                           -kJointLimitFallbackRad, kJointLimitFallbackRad);
  } else {
    // F5: no honest arm command this tick, so none is issued. Zero-length is
    // "no update" — the drive holds its previous setpoint (§3.7).
    rtc::SilenceDeviceOutput(out0);
  }

  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto& out1 = output.devices[1];
    out1.goal_type = GoalType::kJoint;
    if (!hand_readable_) {
      // F5 on the SECONDARY axis (#291), the same answer the arm gets above and
      // for the same reason: hand_computed_ was withheld this tick, so the only
      // thing left to command would be a stale or zero-init buffer. Zero-length
      // is "no update" — the hand holds its previous setpoint. nc1 zeros would
      // be a real command to the origin, i.e. every finger slammed open. The
      // τ_ff branch is skipped with it: a feedforward torque is only meaningful
      // alongside the PD target it was computed against, and there is no
      // command lane to attach it to on a silenced tick.
      rtc::SilenceDeviceOutput(out1);
      // The log lane is bounded by the DEVICE's channel count, so an untouched
      // reference row would be recorded as zeros and read as exactly that
      // origin command. Report where the fingers are parked instead.
      rtc::HoldTelemetryAtMeasured(out1, nc1, dev1.positions);
    } else {
      out1.num_channels = nc1;
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
        out1.commands[i] = hand_computed_.positions[i];
      }
      rtc::utils::ClampRange(out1.commands, nc1, std::span<const double>(device_position_lower_[1]),
                             std::span<const double>(device_position_upper_[1]),
                             -kJointLimitFallbackRad, kJointLimitFallbackRad);
      // Stage C-3: when hand τ_ff is active this tick, drive the hand device via
      // kPdFeedforward — the position commands above are the PD target (hold
      // pose) and feedforward carries the model torque. The arm device leaves
      // its per-device command_type unset (inherits the global kPosition
      // default).
      if (hand_tauff_active_) {
        out1.command_type = CommandType::kPdFeedforward;
        // #9: hand_computed_.feedforward holds hand_dof_ model torques; copy
        // only that many. If the hand device reports nc1 > hand_dof_ channels,
        // the tail of out1.feedforward stays fresh-zero (no stale read past
        // hand_dof_).
        const auto nff = std::min(static_cast<std::size_t>(nc1),
                                  static_cast<std::size_t>(std::max(hand_dof_, 0)));
        for (std::size_t i = 0; i < nff; ++i) {
          out1.feedforward[i] = hand_computed_.feedforward[i];
        }
      }
    }
  }

  return output;
}

// Shared device joint-space fill: trajectory_* reference + goal_positions for
// one device. target_* (publish-only) stays in FillPublishOutput.
void DemoWbcController::FillDeviceTrajectoryPods(rtc::DeviceOutput& out, int num_channels,
                                                 const ComputedTrajectory& computed,
                                                 int target_slot) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::FillDeviceTrajectoryPods");
  const auto slot = static_cast<std::size_t>(target_slot);
  for (std::size_t i = 0; i < static_cast<std::size_t>(num_channels); ++i) {
    out.trajectory_positions[i] = computed.positions[i];
    out.trajectory_velocities[i] = computed.velocities[i];
    out.goal_positions[i] = current_target_slot_.targets[slot][i];
  }
}

// Shared TCP → task-space pods: fills actual_task_positions + task_goal_positions
// from the shared PinocchioCache's clik_tcp/clik_base registered-frame oMf
// (#unified-kindyn Phase 2). Replaces the arm-only arm_handle_ FK recompute —
// value-identical on serial arms and on the closed-chain arm TCP (upstream of
// the hand loop), proven by test_wbc_arm_tcp_cache_equivalence. Precondition:
// clik_tcp_frame_idx_ >= 0 (guaranteed on the activated path via InitClik +
// on_configure gate) and cache.Update ran this TSID tick. Returns the TCP SE3
// so the publish path can reuse it for arm_tip_pose.
pinocchio::SE3 DemoWbcController::FillTaskPosePods(ControllerOutput& output) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::FillTaskPosePods");
  pinocchio::SE3 tcp =
      combined_cache_.cache().registered_frames[static_cast<std::size_t>(clik_tcp_frame_idx_)].oMf;
  if (clik_base_frame_idx_ >= 0) {
    tcp = combined_cache_.cache()
              .registered_frames[static_cast<std::size_t>(clik_base_frame_idx_)]
              .oMf.actInv(tcp);
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
  return tcp;
}

// ── #123 Phase 2: per-tick hand fingertip FK (RT-safe, publish-surface data) ──
// Runs the closed-chain projection (extended hands) or serial hand FK, then
// composes each fingertip's hand-root-relative pose to the base frame via the
// arm TCP placement (@p tcp = base→tool0; base_adapter ≡ tool0 identity mount).
// Caches into fingertip_positions_/rotations_/pose_valid_; FillPublishOutput
// only reads them. Called from ComputeControl (after the Stage-1 cache Update)
// so the publish path never solves kinematics — mirrors DemoTaskController's
// compute.cpp fingertip loop. noexcept, no allocation: the SE3 temporaries are
// stack locals and the dispatch helpers are RT-safe.
bool DemoWbcController::ComputeHandFingertipFk(const ControllerState& state,
                                               const pinocchio::SE3& tcp) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputeHandFingertipFk");
  if (closed_hand_fk_.active() && !closed_hand_fk_.owns_projection()) {
    // #175 borrowed: 사영은 이번 tick 의 combined_cache_.Update() 안에서 provider 가 이미 돌렸다.
    // 여기서 하는 일은 그 결과를 채택할지 판정하는 것뿐이고, 두 축을 AND 한다:
    //
    //  (a) 실행 — provider 의 사영 카운터가 이번 tick 에 증가했는가. Stage-1 게이트가 닫혀
    //      cache.Update 가 아예 안 돈 tick 에는 증가하지 않으므로, 직전 tick 형상을 이번 tick
    //      것으로 오인하지 않는다.
    //  (b) 입력 provenance — 그 사영의 **입력 q** 가 전부 이번 tick 측정값인가. cache 는 블록
    //      단위로 hold 하므로(arm 게이트가 닫히면 전량, hand 게이트가 닫히면 hand 블록) 사영은
    //      돌았는데 입력이 직전 tick 값인 경우가 있다. (a) 는 이것을 구분하지 못한다.
    //
    // (b) 의 술어는 cache 가 블록을 갱신하는 조건 그 자체다 — 같은 IsDeviceReadable 3항(valid ·
    // 폭 · per-slot freshness)이라 폭 축과 구멍 축이 함께 닫힌다. owning 모드의 per-slot 브릿지
    // 판정보다 **엄격**하다 (읽지 않는 슬롯의 구멍도 게이트를 닫는다): 채택하던 것을 hold 할 수는
    // 있어도, hold 하던 것을 채택하지는 않는 방향이다.
    const std::uint32_t seq = wbc_reduced_dynamics_.projection_seq();
    const bool projected_this_tick = (seq != last_projection_seq_);
    last_projection_seq_ = seq;
    closed_hand_fk_.UpdateFromProjection(projected_this_tick && arm_readable_ && hand_readable_,
                                         wbc_reduced_dynamics_.kinematic_status());
  } else if (!RunHandForwardKinematics(closed_hand_fk_, hand_handle_.get(), hand_q_, state)) {
    return false;
  }
  for (std::size_t f = 0; f < kNumFingertips; ++f) {
    pinocchio::SE3 T_hand_ft;
    const bool produced =
        HandFingertipPoseDispatch(closed_hand_fk_, hand_handle_.get(), fingertip_frame_ids_,
                                  use_hand_root_frame_, hand_root_frame_id_, f, T_hand_ft);
    fingertip_pose_valid_[f] = produced;
    if (produced) {
      const pinocchio::SE3 T_base_ft = tcp.act(T_hand_ft);
      fingertip_positions_[f] = T_base_ft.translation();
      fingertip_rotations_[f] = T_base_ft.rotation();
    }
  }
  return true;
}

// ── Phase 3b: Fill log output ────────────────────────────────────────────────

void DemoWbcController::FillLogOutput(const ControllerState& state,
                                      ControllerOutput& output) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::FillLogOutput");
  const auto& dev0 = state.devices[0];
  const int nc0 = dev0.num_channels;
  // Same gate as FillPublishOutput (#236 S7b): on a silenced tick robot_computed_
  // still holds the last readable tick's values, and recording them here while
  // the publish lane withholds them would make the CSV and the topic disagree
  // about the same tick. Withholding alone is not enough either — the log POD is
  // bounded by the DEVICE's channel count, so an untouched row is written as
  // zeros and reads as a command to the origin; report the parked position.
  if (arm_readable_) {
    FillDeviceTrajectoryPods(output.devices[0], nc0, robot_computed_, 0);
  } else {
    rtc::HoldTelemetryAtMeasured(output.devices[0], nc0, dev0.positions);
    // The goal survives the gate, as in demo_joint / demo_task.
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
      output.devices[0].goal_positions[i] = current_target_slot_.targets[0][i];
    }
  }

  // actual_task_positions + task_goal_positions from the shared cache oMf (log
  // POD reads both). #unified-kindyn Phase 2: no arm_handle_ FK recompute here —
  // cache.Update already produced clik_tcp/clik_base oMf this TSID tick.
  if (clik_tcp_frame_idx_ >= 0) {
    FillTaskPosePods(output);
  }

  // Same on the hand axis (#291), same shape as the arm above.
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    if (hand_readable_) {
      FillDeviceTrajectoryPods(output.devices[1], nc1, hand_computed_, 1);
    } else {
      rtc::HoldTelemetryAtMeasured(output.devices[1], nc1, dev1.positions);
      // The goal survives the gate, as on the arm.
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
        output.devices[1].goal_positions[i] = current_target_slot_.targets[1][i];
      }
    }
  }

  // WBC state aggregates (per-fingertip + FSM phase). Staging buffer feeds
  // both the SeqLock publish path and GetWbcStateForTesting().
  FillWbcSensorAggregates();
  {
    auto& ws = wbc_state_;
    // tsid_solver_ok / qp_fail_count are the Dynamic (TSID) QP health — the
    // rtc_msgs/WbcState fields always referred to the TSID solve. Filled only
    // on the path that actually ran a solve; FillEstopPublishState reports
    // "not solved this tick" instead of replaying these.
    ws.tsid_solver_ok = tsid_initialized_ && (dyn_qp_fail_count_ == 0);
    ws.qp_fail_count = dyn_qp_fail_count_;
    ws.tsid_solve_us = static_cast<float>(tsid_output_.solve_time_us);
  }
  // SeqLock store = two atomic stores + memcpy (wait-free, RT-safe).
  // Read by PublishNonRtSnapshot.
  wbc_state_lock_.Store(wbc_state_);
}

// Sensor-derived aggregates only — see the header for why these are shared
// with the E-STOP path (fingertip_data_ is refreshed by ReadState every tick).
void DemoWbcController::FillWbcSensorAggregates() noexcept {
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
}

// ── E-STOP publish state (#234 P-1) ─────────────────────────────────────────
//
// Rationale in the header.

void DemoWbcController::FillEstopPublishState(double dt) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::FillEstopPublishState");
  FillWbcSensorAggregates();
  auto& ws = wbc_state_;
  // No TSID solve ran this tick. Reporting the previous solve's health here is
  // exactly the stale-payload republish this path exists to stop; qp_fail_count
  // is a lifetime counter, not a per-tick derivation, so it stays.
  ws.tsid_solver_ok = false;
  ws.tsid_solve_us = 0.0F;
  ws.qp_fail_count = dyn_qp_fail_count_;
  StageEstopPullTick(pull_wiring_, dt, ws.pull);
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
  RTC_TRACE_SCOPE("DemoWbcController::FillPublishOutput");
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  // Telemetry follows the wire (#236 S7b): on a tick that issued no arm command
  // robot_computed_ still holds the last readable tick's values, so publishing
  // them would date-stamp stale numbers as this tick's reference.
  // WriteJointCommand silenced device 0 for the same reason.
  if (arm_readable_) {
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
      out0.target_positions[i] = robot_computed_.positions[i];
      out0.target_velocities[i] = robot_computed_.velocities[i];
    }
    FillDeviceTrajectoryPods(out0, nc0, robot_computed_, 0);
  } else {
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
    for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
      out0.goal_positions[i] = current_target_slot_.targets[0][i];
    }
  }

  // #unified-kindyn Phase 2: arm TCP from the shared cache oMf (clik_tcp/base),
  // not an arm_handle_ FK recompute. clik_tcp_frame_idx_ >= 0 ⇒ arm model present.
  if (clik_tcp_frame_idx_ >= 0) {
    const pinocchio::SE3 tcp = FillTaskPosePods(output);

    // TF source: arm tip.
    const Eigen::Vector3d& trans = tcp.translation();
    const Eigen::Quaterniond quat(tcp.rotation());
    output.arm_tip_pose.position = {trans.x(), trans.y(), trans.z()};
    output.arm_tip_pose.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
    // Withheld on a silenced tick: the cache was not refreshed from this tick's
    // state, so the pose is history, not a measurement (#125 F1's rule).
    output.arm_tip_pose_valid = arm_readable_;

    // #123 Phase 2: fingertip TF source — poses were computed + cached in
    // ComputeControl (fingertip_positions_/rotations_/pose_valid_) this tick; the
    // publish path only READS them, never re-solves hand FK (matches joint/task).
    for (std::size_t f = 0; f < kNumFingertips; ++f) {
      // Gate on both a resolved serial frame id AND a pose actually produced
      // this tick: a downstream (loop) tip holds no pose until the closed
      // chain first converges, so publishing its zero-init cache would snap
      // the fingertip TF to the base origin.
      if (fingertip_frame_ids_[f] != 0 && fingertip_pose_valid_[f]) {
        const Eigen::Vector3d& ft_trans = fingertip_positions_[f];
        const Eigen::Quaterniond ft_quat(fingertip_rotations_[f]);
        output.task_link_poses[f].position = {ft_trans.x(), ft_trans.y(), ft_trans.z()};
        output.task_link_poses[f].quaternion = {ft_quat.w(), ft_quat.x(), ft_quat.y(), ft_quat.z()};
        output.task_link_pose_valid[f] = true;
      } else {
        output.task_link_pose_valid[f] = false;
      }
    }
  }

  // Same on the hand axis (#291), same shape as the arm above.
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto& out1 = output.devices[1];
    if (hand_readable_) {
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
        out1.target_positions[i] = hand_computed_.positions[i];
        out1.target_velocities[i] = hand_computed_.velocities[i];
      }
      FillDeviceTrajectoryPods(out1, nc1, hand_computed_, 1);
    } else {
      rtc::HoldTelemetryAtMeasured(out1, nc1, dev1.positions);
      for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
        out1.goal_positions[i] = current_target_slot_.targets[1][i];
      }
    }
  }
}

// ── Target management ────────────────────────────────────────────────────────

// ── E-STOP compute path ──────────────────────────────────────────────────────

ControllerOutput DemoWbcController::ComputeEstop(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::ComputeEstop");
  const auto& dev0 = state.devices[0];
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.valid = true;
  output.command_type = command_type_;

  // Arm: ramp toward safe position with per-joint velocity limit (matches
  // DemoJoint/DemoTask ComputeEstop pattern — instant jump risks hardware
  // damage on real UR5e at high E-STOP delta).
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.goal_type = GoalType::kJoint;
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  // The ramp loop is arm_dof_ deep but num_channels was declared nc0, so
  // channels in [arm_dof_, nc0) went out as a fresh zero — the arm's extra
  // channels commanded to the origin under E-STOP (#265 audit W7). And the ramp
  // itself is only a ramp toward safety while q is a real measurement, so the
  // gate applies here exactly as it does on the normal lane (#236 E-8).
  const int nq = rtc::ModelChannelBound(nc0, arm_dof_);
  if (arm_readable_) {
    out0.num_channels = nc0;
    for (int i = 0; i < nq; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      const double lim =
          (idx < device_max_velocity_[0].size()) ? device_max_velocity_[0][idx] : 2.0;
      out0.commands[idx] = dev0.positions[idx] +
                           std::clamp(safe_position_[idx] - dev0.positions[idx], -lim, lim) * dt;
      out0.target_positions[idx] = out0.commands[idx];
    }
    // No configured safe position past the model: hold where the joint is.
    rtc::FillCommandTail(out0.commands, nq, nc0, command_type_, dev0.positions);
    for (std::size_t i = static_cast<std::size_t>(nq); i < static_cast<std::size_t>(nc0); ++i) {
      out0.target_positions[i] = out0.commands[i];
    }
  } else {
    rtc::SilenceDeviceOutput(out0);
    // A silenced E-STOP tick is still a logged tick — no Fill* runs on this
    // lane, so the parked-position fill has to happen here.
    rtc::HoldTelemetryAtMeasured(out0, nc0, dev0.positions);
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

// ── Helpers (TCP error, MPC timing log) ─────────────────────────────────────
// (ExtractFullState — per-tick measured state scatter → Pinocchio order — now
//  lives in CombinedModelCache; ReadState/OnPhaseEnter call combined_cache_.
//  ExtractFullState(state, arm_dof_, hand_dof_).)

void DemoWbcController::BuildTargetPosture(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::BuildTargetPosture");
  if (!combined_cache_.reorder_valid()) {
    return;
  }
  // Arm target: external [0..arm_dof_-1] → Pinocchio order (mirrors ExtractFullState).
  for (int i = 0; i < arm_dof_; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(combined_cache_.ext_to_pin_q(static_cast<int>(eidx)));
    q_des_target_full_[pq] = current_target_slot_.targets[0][eidx];
  }
  // Hand target — shared with the always-live mid-phase hand jog (Compute()).
  BuildHandTargetPosture(state);
}

bool DemoWbcController::BuildHandTargetPosture(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::BuildHandTargetPosture");
  // Fold the current hand joint target (current_target_slot_.targets[1]) into the
  // posture reference's hand block (external [arm_dof_..full_dof_-1] → Pinocchio
  // order). Called at phase entry via BuildTargetPosture AND per-tick from
  // Compute() when a fresh hand target arrives, so the hand joint target is a
  // live command in every driving phase — not consumed only on a closure edge.
  // Returns true iff the fold applied (reorder map ready + hand device valid) so
  // the caller can keep a target pending across a transient hand-device dropout.
  if (!combined_cache_.reorder_valid()) {
    return false;
  }
  if (state.num_devices > 1 && state.devices[1].valid) {
    for (int i = 0; i < hand_dof_; ++i) {
      const auto eidx = static_cast<std::size_t>(arm_dof_ + i);
      const auto pq =
          static_cast<Eigen::Index>(combined_cache_.ext_to_pin_q(static_cast<int>(eidx)));
      q_des_target_full_[pq] = current_target_slot_.targets[1][static_cast<std::size_t>(i)];
    }
    return true;
  }
  return false;
}

void DemoWbcController::SeedHoldFromMeasured(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::SeedHoldFromMeasured");
  // joint_goal mirror = current measured config (arm + hand). This makes the
  // logged joint_goal match the posture reference idle actually regulates to.
  const auto& dev0 = state.devices[0];
  // Only mirror a measurement that exists: an unreadable arm would seed the
  // joint_goal — and through BuildTargetPosture the posture REFERENCE — with
  // zeros for the joints nobody reported.
  if (arm_readable_) {
    for (int i = 0; i < arm_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      current_target_slot_.targets[0][idx] = dev0.positions[idx];
    }
  }
  // Same on the hand axis (#291). This one latches further than most: targets[1]
  // feeds BuildHandTargetPosture and through it the posture REFERENCE, so a
  // phantom 0 per unreported finger becomes what TSID actively regulates the
  // hand toward — not a one-tick blemish.
  if (hand_readable_) {
    const auto& dev1 = state.devices[1];
    for (int i = 0; i < hand_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      current_target_slot_.targets[1][idx] = dev1.positions[idx];
    }
  }
  // Posture reference snapshot (external order → Pinocchio order). No-op until
  // combined_cache_.reorder_valid(); the kIdle posture-ref fallback covers that window.
  BuildTargetPosture(state);
  // SE3 hold pose at the current measured FK (base_frame → tip), persisted to
  // the SeqLock POD so the next-tick DrainTargetSlot restore keeps it valid.
  if (arm_readable_ && arm_handle_) {
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
  // #unified-kindyn Phase 2: world tip from the shared cache clik_tcp oMf — the
  // exact value arm_handle_->GetFramePlacement(tip_frame_id_) produced. UpdatePhase
  // (this caller) runs before the same-tick cache.Update, so it reads the previous
  // tick's oMf — the SAME one-tick lag the old path had (it read arm_handle_ FK
  // last refreshed by the previous tick's FillLogOutput). target (tcp_goal_) is
  // compared in the same world frame as before (no base actInv here).
  if (clik_tcp_frame_idx_ < 0) {
    return 1e10;
  }
  const pinocchio::SE3& tcp =
      combined_cache_.cache().registered_frames[static_cast<std::size_t>(clik_tcp_frame_idx_)].oMf;
  return (tcp.translation() - target.translation()).norm();
}

// ── Controller-owned topic lifecycle ──────────────────────────────────────

void DemoWbcController::LogMpcSolveTimingTick() noexcept {
  // L2 under the 1 Hz aux-timer executor callback (mpc_timing_cb_group_) — NOT
  // an RT-tick span. Attributes the MPC timing-CSV drain + aggregate INFO.
  RTC_TRACE_SCOPE("DemoWbcController::LogMpcSolveTimingTick");
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
  RTC_TRACE_SCOPE("DemoWbcController::FillDeviceWbcLogPod");
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
  // [arm0.., hand0..] → Pinocchio index via combined_cache_.ext_to_pin_q, the same mapping
  // ComputeKinematicWbc applies to the CLIK reference. For these fixed-base
  // demos q-index == v-index per joint, so the q-order map indexes the
  // nv-sized a_opt correctly. (a_opt is now log-only — Dynamic WBC produces it
  // but no longer feeds position; the Kinematic CLIK-QP owns position.)
  const int dof = (role == 0) ? arm_dof_ : hand_dof_;
  const int ext_base = (role == 0) ? 0 : arm_dof_;
  const auto& a = tsid_output_.a_opt;
  if (a.size() > 0) {
    const auto count = std::min(static_cast<std::size_t>(std::max(dof, 0)), n);
    for (std::size_t i = 0; i < count; ++i) {
      const auto ext_i = static_cast<std::size_t>(ext_base + static_cast<int>(i));
      const auto pin_idx =
          static_cast<std::size_t>(combined_cache_.ext_to_pin_q(static_cast<int>(ext_i)));
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
      // Force vector (link frame) straight from the per-tick fingertip staging
      // buffer; wbc_state_ only carries the |F| magnitude aggregate.
      const auto& fvec = fingertip_data_[i].force;
      pod.fingertip_force_x[i] = static_cast<double>(fvec[0]);
      pod.fingertip_force_y[i] = static_cast<double>(fvec[1]);
      pod.fingertip_force_z[i] = static_cast<double>(fvec[2]);
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
  RTC_TRACE_SCOPE("DemoWbcController::FillWbcDiagLogPod");
  pod.t_relative_s = state.t_relative_s;
  pod.phase = static_cast<std::uint8_t>(phase_);
  pod.solve_time_us = tsid_output_.solve_time_us;
  pod.solve_levels = tsid_output_.solve_levels;
  pod.qp_fail_count = dyn_qp_fail_count_;
  pod.kin_qp_fail_count = kin_qp_fail_count_;
  pod.qp_converged = tsid_output_.qp_converged;
  pod.num_active_contacts = wbc_state_.num_active_contacts;
  pod.grasp_detected = wbc_state_.grasp_detected;
  pod.max_force = wbc_state_.max_force;
  // CLIK-QP (Kinematic WBC) diagnostics, set in ComputeKinematicWbc this tick;
  // zero/false on non-TSID phases (kFallback never reaches FillWbcDiagLogPod via
  // a fresh solve).
  pod.clik_valid = clik_compute_ok_;
  pod.clik_tcp_err = clik_tcp_err_;
  pod.clik_manipulability = clik_manip_;
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
