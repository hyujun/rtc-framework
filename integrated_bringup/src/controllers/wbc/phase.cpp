#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/internal_force_task.hpp"
#include "rtc_tsid/tasks/object_se3_task.hpp"
#include "rtc_tsid/tasks/object_wrench_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

#include <algorithm>
#include <cmath>

namespace integrated_bringup {

// ── 7-state grasp FSM (Idle → Approach → PreGrasp → Closure → Hold →
//                       Release → Fallback; slot 5 reserved) ───────────────
//
// RELEASE preempt: grasp_cmd=2 jumps to kRelease from any active grasp phase
// (kApproach/kPreGrasp/kClosure/kHold). Terminal/safe phases — kIdle (no
// contacts, hand already open), kRelease (already releasing), kFallback
// (latched safe state) — are exempt to keep the guard a no-op there.
// Abort (cmd=0) returns to kIdle from the same active set. Both preempt the
// per-case transitions below.
void DemoWbcController::UpdatePhase(const ControllerState& state) noexcept {
  const int cmd = grasp_cmd_.load(std::memory_order_acquire);
  WbcPhase next = phase_;

  // Top-level preempt guards: RELEASE > abort > case-internal transitions.
  if (cmd == 2 && phase_ != WbcPhase::kIdle && phase_ != WbcPhase::kRelease &&
      phase_ != WbcPhase::kFallback) {
    next = WbcPhase::kRelease;
  } else if (cmd == 0 && phase_ != WbcPhase::kIdle && phase_ != WbcPhase::kRelease &&
             phase_ != WbcPhase::kFallback) {
    next = WbcPhase::kIdle;
  } else {
    switch (phase_) {
      case WbcPhase::kIdle:
        // grasp_cmd=1 + valid target → approach
        if (cmd == 1 && robot_new_target_pending_) {
          next = WbcPhase::kApproach;
        }
        break;

      case WbcPhase::kApproach: {
        // TCP close enough to approach goal → fine positioning. epsilon_approach_
        // (formerly unused) is the loose threshold; pre_grasp tightens to
        // epsilon_pregrasp_ for closure entry.
        if (tcp_goal_valid_ && ComputeTcpError(tcp_goal_) < epsilon_approach_) {
          if (tsid_initialized_) {
            next = WbcPhase::kPreGrasp;
          } else {
            next = WbcPhase::kIdle;
          }
        }
        break;
      }

      case WbcPhase::kPreGrasp: {
        // TCP close enough to goal → closure
        if (tcp_goal_valid_ && ComputeTcpError(tcp_goal_) < epsilon_pregrasp_) {
          next = WbcPhase::kClosure;
        }
        break;
      }

      case WbcPhase::kClosure: {
        // Count active contacts from parsed fingertip forces
        int active_contacts = 0;
        for (int f = 0; f < num_active_fingertips_; ++f) {
          const auto& ft = fingertip_data_[static_cast<std::size_t>(f)];
          if (ft.valid && ft.force_magnitude > force_contact_threshold_) {
            ++active_contacts;
          }
        }
        if (active_contacts >= min_contacts_for_hold_) {
          next = WbcPhase::kHold;
        }
        break;
      }

      case WbcPhase::kHold: {
        // Anomaly detection: slip (|df/dt|) only. Deformation guard is
        // TODO(layer-d) — current fingertip sensors do not publish per-tip
        // displacement, so the kFallback ramp on |d| > deformation_threshold_
        // is suppressed. `deformation_threshold_` member + YAML key kept for
        // backward compatibility and Layer D restoration.
        for (int f = 0; f < num_active_fingertips_; ++f) {
          const auto& ft = fingertip_data_[static_cast<std::size_t>(f)];
          if (!ft.valid) {
            continue;
          }
          if (std::abs(ft.force_rate) > slip_rate_threshold_) {
            RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                                 "[wbc] slip detected f=%d df/dt=%.2f N/s > %.2f", f,
                                 static_cast<double>(ft.force_rate), slip_rate_threshold_);
            next = WbcPhase::kFallback;
            break;
          }
        }
        break;
      }

      case WbcPhase::kRelease:
        // Stage 1 finger-open completed → idle. release_done_ is set by
        // ComputeReleaseMode when hand_trajectory_ runs out.
        if (release_done_) {
          next = WbcPhase::kIdle;
        }
        break;

      case WbcPhase::kFallback:
        // Manual recovery only: grasp_cmd=0 → idle (exempt from top-level guard)
        if (cmd == 0) {
          next = WbcPhase::kIdle;
        }
        break;
    }
  }

  if (next != phase_) {
    RCLCPP_INFO_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleFastMs,
                         "[wbc] phase %d -> %d", static_cast<int>(phase_), static_cast<int>(next));
    prev_phase_ = phase_;
    OnPhaseEnter(next, state);
    phase_ = next;
  }
}

void DemoWbcController::OnPhaseEnter(WbcPhase new_phase, const ControllerState& state) noexcept {
  // Atomic gains snapshot — phase transitions are RT-safe reads.
  const auto gains = gains_lock_.Load();
  const auto& dev0 = state.devices[0];
  const auto& dev1 = state.devices[1];

  // ── TCP SE3 trajectory edge handling (MPC-disabled mode only) ──────────
  // SE3 task activity is the gate (YAML phase_presets[<phase>].tasks.se3_tcp.
  // active). MPC-enabled mode keeps the legacy step-on-entry path (`tcp_goal_`
  // assigned once below); MPC-disabled mode drives a quintic ramp every tick.
  {
    const bool se3_now = Se3TaskActiveInPhase(new_phase);
    const bool se3_prev = Se3TaskActiveInPhase(prev_phase_);
    const bool mpc_on = mpc_enabled_ && mpc_manager_.Enabled();
    if (se3_now && !se3_prev && tcp_goal_valid_ && !mpc_on) {
      InitTcpTrajectory(state);
    } else if (!se3_now && se3_prev) {
      tcp_trajectory_active_ = false;
      has_pending_tcp_segment_ = false;
    }
  }

  switch (new_phase) {
    case WbcPhase::kIdle: {
      // TSID drives SE3 hold at current TCP + posture hold at current q.
      // No phase-entry integrator seed (carry-forward is event-driven only).
      // ComputeTSIDPosition fills robot_computed_/hand_computed_ from
      // q_next_full_ per tick. No direct command-buffer writes here — they
      // would be overwritten by the TSID mapping the same tick.
      ExtractFullState(state);
      // kFallback→kIdle recovery: ComputeFallback bypasses the integrator, so
      // its state is cold. Hard re-seed from measured on the recovery edge.
      if (prev_phase_ == WbcPhase::kFallback) {
        reseed_on_fallback_exit_ = true;
      }

      const auto idx = static_cast<std::size_t>(WbcPhase::kIdle);
      if (phase_preset_valid_[idx]) {
        tsid_controller_.ApplyPhasePreset(phase_presets_[idx]);
      }

      // Re-snapshot the idle hold target from the current measured config:
      // posture reference (q_des_target_full_), joint_goal mirror, and the SE3
      // hold pose at current TCP. Re-seeding on each idle entry (e.g. a
      // release→idle return) holds where the robot is *now* rather than the
      // stale grasp target. The SeqLock store persists the refreshed targets[]
      // + SE3 POD so the next-tick DrainTargetSlot restore keeps them.
      SeedHoldFromMeasured(state);
      target_seqlock_.Store(current_target_slot_);
      // SE3 hold at current TCP (zero-displacement quintic).
      if (arm_handle_) {
        InitTcpTrajectory(state);
      }

      qp_fail_count_ = 0;
      // Stage A-5b: kIdle ramps activation down to 0 (gentle release).
      for (int i = 0; i < static_cast<int>(contact_state_.contacts.size()); ++i) {
        contact_state_.SetActivationTarget(i, 0.0, contact_ramp_sec_);
      }
      contact_state_.RecomputeActive(contact_mgr_config_);
      break;
    }

    case WbcPhase::kApproach: {
      // TSID drives the arm via an SE3 quintic ramp (current FK → pregrasp
      // pose). The hand stays at its current pose throughout Approach: the
      // TSID posture task is rewritten to q_curr_full_ each tick inside
      // ComputeTSIDPosition's MPC-disabled branch (q_des = q_curr_full_),
      // so the user-provided hand close pose in current_target_slot_.
      // targets[1] is NOT applied here. That target is consumed by the
      // hand_trajectory_ ramp initialised on kClosure/kHold entry below,
      // which interpolates from the current hand pose toward the user
      // target over `hand_trajectory_speed`-shaped duration.
      const auto idx = static_cast<std::size_t>(WbcPhase::kApproach);
      if (phase_preset_valid_[idx]) {
        tsid_controller_.ApplyPhasePreset(phase_presets_[idx]);
      }

      ExtractFullState(state);
      robot_new_target_pending_ = false;
      hand_new_target_pending_ = false;

      // Compute FK of arm target for SE3 goal, then re-FK current pose so
      // InitTcpTrajectory's start = current FK, goal = tcp_goal_.
      if (arm_handle_) {
        std::span<const double> q_target(current_target_slot_.targets[0].data(),
                                         static_cast<std::size_t>(arm_dof_));
        arm_handle_->ComputeForwardKinematics(q_target);
        tcp_goal_ = arm_handle_->GetFramePlacement(tip_frame_id_);
        if (use_root_frame_) {
          tcp_goal_ = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_goal_);
        }
        tcp_goal_valid_ = true;
        // Mirror the freshly-FK'd SE3 into the SeqLock POD so subsequent
        // ticks can restore tcp_goal_ from current_target_slot_ without
        // touching another writer. Single-writer (RT) invariant preserved.
        std::memcpy(current_target_slot_.tcp_goal_rot.data(), tcp_goal_.rotation().data(),
                    sizeof(current_target_slot_.tcp_goal_rot));
        std::memcpy(current_target_slot_.tcp_goal_t.data(), tcp_goal_.translation().data(),
                    sizeof(current_target_slot_.tcp_goal_t));
        current_target_slot_.tcp_goal_valid = true;
        target_seqlock_.Store(current_target_slot_);

        // Reset FK to current pose so InitTcpTrajectory's start = current.
        std::span<const double> q_arm(dev0.positions.data(),
                                      static_cast<std::size_t>(arm_dof_));
        arm_handle_->ComputeForwardKinematics(q_arm);
        InitTcpTrajectory(state);
      }

      // Posture reference tracks the same targets[] snapshot the SE3 goal was
      // built from above (single consistent target per phase entry).
      BuildTargetPosture(state);

      qp_fail_count_ = 0;
      break;
    }

    case WbcPhase::kPreGrasp:
    case WbcPhase::kClosure:
    case WbcPhase::kHold: {
      // Apply TSID phase preset (RT-safe: uses pre-resolved PhasePreset)
      const auto idx = static_cast<std::size_t>(new_phase);
      if (phase_preset_valid_[idx]) {
        tsid_controller_.ApplyPhasePreset(phase_presets_[idx]);
      }

      // No phase-entry integrator seed (carry-forward is event-driven). Refresh
      // the posture target snapshot for this phase (idempotent if targets[]
      // unchanged since approach).
      ExtractFullState(state);
      BuildTargetPosture(state);

      // Set SE3Task reference (TCP goal). MPC-enabled mode pushes the step
      // here once on entry — MPC drives PostureTask smoothly so the SE3 step
      // is acceptable. MPC-disabled mode lets ComputeTSIDPosition push a
      // quintic-shaped (pose, v, a) every tick via tcp_trajectory_ instead;
      // a step push here would defeat the trajectory's smoothing on entry.
      if (tcp_goal_valid_ && mpc_enabled_ && mpc_manager_.Enabled()) {
        auto* se3_task = tsid_controller_.Formulation().GetTask("se3_tcp");
        if (se3_task) {
          static_cast<rtc::tsid::SE3Task*>(se3_task)->SetSe3Reference(tcp_goal_);
        }
      }

      // Contact activation for closure/hold phases. Stage A-5b: drive the
      // continuous activation ramp toward 1.0 over `contact_ramp_sec_`; the
      // legacy `active : bool` flips automatically once s_i crosses the
      // deadband inside ContactState::UpdateActivation.
      if (new_phase == WbcPhase::kClosure || new_phase == WbcPhase::kHold) {
        for (int i = 0; i < static_cast<int>(contact_state_.contacts.size()); ++i) {
          contact_state_.SetActivationTarget(i, 1.0, contact_ramp_sec_);
        }
        contact_state_.RecomputeActive(contact_mgr_config_);

        // Stage A-3: reset the per-contact PI integrator on the closure
        // edge so a fresh grasp starts from a known state. Hold reuses the
        // same updater state — the kClosure→kHold transition is a smooth
        // continuation of the same closed loop, not a new control episode.
        if (new_phase == WbcPhase::kClosure) {
          force_ref_updater_.Reset();
        }

        // Stage B-5: seed the three object-level task references on
        // closure/hold entry. tsid_initialized_ gate avoids a null
        // formulation_ deref when LoadConfig was skipped (unit tests).
        //   ObjectWrenchTask:   w_obj_des = [0, 0, m·g, 0, 0, 0]  (gravity
        //                       support; m=0 leaves it a no-op residual).
        //   InternalForceTask:  λ_squeeze_des ≡ 0 (placeholder; future
        //                       dynamic squeeze planner writes here).
        //   ObjectSE3Task:      placement_des = object_frame_ (identity-only
        //                       provider; pose is static for Stage B-5).
        // Tasks lookup by Name(); silent skip when YAML didn't register them.
        if (tsid_initialized_) {
          if (auto* ow = tsid_controller_.Formulation().GetTask("object_wrench"); ow) {
            constexpr double kGravity = 9.81;
            Eigen::Matrix<double, 6, 1> w_obj_des = Eigen::Matrix<double, 6, 1>::Zero();
            w_obj_des(2) = object_mass_kg_ * kGravity;
            static_cast<rtc::tsid::ObjectWrenchTask*>(ow)->SetDesiredWrench(w_obj_des);
          }
          if (auto* in = tsid_controller_.Formulation().GetTask("internal_force"); in) {
            squeeze_lambda_des_.setZero();
            static_cast<rtc::tsid::InternalForceTask*>(in)->SetSqueezeReference(
                squeeze_lambda_des_);
          }
          if (auto* os = tsid_controller_.Formulation().GetTask("object_se3"); os) {
            pinocchio::SE3 placement_des(object_frame_.R_w, object_frame_.p_w);
            static_cast<rtc::tsid::ObjectSE3Task*>(os)->SetSe3Reference(placement_des);
          }
        }

        // Ramp hand joint target toward stored target (user-provided close pose)
        if (state.num_devices > 1 && dev1.valid) {
          trajectory::JointSpaceTrajectory<kMaxHandDof>::State hstart{};
          trajectory::JointSpaceTrajectory<kMaxHandDof>::State hgoal{};
          double hmax = 0.0;
          for (int i = 0; i < hand_dof_; ++i) {
            const auto hidx = static_cast<std::size_t>(i);
            hstart.positions[hidx] = dev1.positions[hidx];
            hgoal.positions[hidx] = current_target_slot_.targets[1][hidx];
            const double hd = std::abs(hgoal.positions[hidx] - hstart.positions[hidx]);
            if (hd > hmax) {
              hmax = hd;
            }
          }
          const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
          hand_trajectory_.initialize(hstart, hgoal, hdur);
          hand_trajectory_time_ = 0.0;
        }
      }

      qp_fail_count_ = 0;
      break;
    }

    case WbcPhase::kRelease: {
      // 2-stage release. Stage 0: contact activation_target → 0 over
      // release_ramp_sec_ while TSID holds the arm SE3 at current TCP and
      // posture damps the hand at its current pose. Stage 1: finger-open
      // trajectory plays after the ramp window (initialised lazily in
      // ComputeReleaseMode so hstart reflects the post-ramp pose).
      release_stage_ = 0;
      release_elapsed_s_ = 0.0;
      release_done_ = false;

      for (int i = 0; i < static_cast<int>(contact_state_.contacts.size()); ++i) {
        contact_state_.SetActivationTarget(i, 0.0, release_ramp_sec_);
      }
      contact_state_.RecomputeActive(contact_mgr_config_);

      const auto idx = static_cast<std::size_t>(WbcPhase::kRelease);
      if (phase_preset_valid_[idx]) {
        tsid_controller_.ApplyPhasePreset(phase_presets_[idx]);
      }

      // Release uses self-hold posture (q_des = measured) — see
      // ComputeTSIDPosition; the finger-open is driven by hand_trajectory_ in
      // ComputeReleaseMode, so posture must not pull the hand toward targets[1].
      // Re-anchor the integrator on entry: release is a fresh safe-hold episode
      // (like fallback recovery), so it should not carry a drifted q_next.
      ExtractFullState(state);
      reseed_integration_pending_ = true;

      // SE3 hold at current TCP (zero-displacement quintic).
      if (arm_handle_) {
        std::span<const double> q_arm(dev0.positions.data(),
                                      static_cast<std::size_t>(arm_dof_));
        arm_handle_->ComputeForwardKinematics(q_arm);
        tcp_goal_ = arm_handle_->GetFramePlacement(tip_frame_id_);
        if (use_root_frame_) {
          tcp_goal_ = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_goal_);
        }
        tcp_goal_valid_ = true;
        InitTcpTrajectory(state);
      }

      qp_fail_count_ = 0;
      break;
    }

    case WbcPhase::kFallback: {
      // Hold current position, deactivate contacts
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        robot_computed_.positions[idx] = dev0.positions[idx];
        robot_computed_.velocities[idx] = 0.0;
      }
      if (state.num_devices > 1 && dev1.valid) {
        for (int i = 0; i < hand_dof_; ++i) {
          const auto idx = static_cast<std::size_t>(i);
          hand_computed_.positions[idx] = dev1.positions[idx];
          hand_computed_.velocities[idx] = 0.0;
        }
      }
      // Stage A-5b: fallback rapidly ramps activation down (10 ms) so QP
      // cost / constraints release contact within a few ticks.
      for (int i = 0; i < static_cast<int>(contact_state_.contacts.size()); ++i) {
        contact_state_.SetActivationTarget(i, 0.0, 0.01);
      }
      contact_state_.RecomputeActive(contact_mgr_config_);
      qp_fail_count_ = 0;
      break;
    }
  }

  // ── GraspPhaseManager bridge (handler mode only) ──────────────────────────
  //
  // WBC FSM is authoritative for the demo; the grasp phase manager mirrors
  // it via ForcePhase so rtc_mpc picks up the matching OCP type
  // (contact_light vs contact_rich) on every WBC edge. `ForcePhase` is
  // atomic and RT-safe; `SetTaskTarget` uses SeqLock::Store (wait-free,
  // RT-4 safe). See grasp_phase_manager.hpp thread-safety notes.
  // WBC has no direct MANIPULATE analogue — kClosure maps to CLOSURE and
  // kHold to HOLD; MANIPULATE is reserved for a future WBC extension.
  if (phase_manager_ptr_) {
    namespace phase = integrated_bringup::phase;
    int grasp_id = static_cast<int>(phase::GraspPhaseId::kIdle);
    switch (new_phase) {
      case WbcPhase::kIdle:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kIdle);
        break;
      case WbcPhase::kApproach:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kApproach);
        if (tcp_goal_valid_) {
          phase::GraspTarget gt{};
          gt.grasp_pose = tcp_goal_;
          gt.pregrasp_pose = tcp_goal_;
          gt.approach_start = tcp_goal_;
          phase_manager_ptr_->SetTaskTarget(gt);
        }
        break;
      case WbcPhase::kPreGrasp:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kPreGrasp);
        break;
      case WbcPhase::kClosure:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kClosure);
        break;
      case WbcPhase::kHold:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kHold);
        break;
      case WbcPhase::kRelease:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kRelease);
        break;
      case WbcPhase::kFallback:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kIdle);
        break;
    }
    phase_manager_ptr_->ForcePhase(grasp_id);
  }
}

// ── TCP SE3 trajectory init (MPC-disabled SE3 ramp) ─────────────────────────
//
// Mirrors the demo_task_controller TaskSpaceTrajectory pattern: quintic
// rest-to-rest interpolation in SE3 with a π-rotation defense (mid-pose
// split). Called from OnPhaseEnter on SE3-inactive → SE3-active edges
// when MPC is disabled. tcp_goal_valid_ is the caller's precondition.
void DemoWbcController::InitTcpTrajectory(const ControllerState& /*state*/) noexcept {
  if (!arm_handle_) {
    tcp_trajectory_active_ = false;
    has_pending_tcp_segment_ = false;
    return;
  }

  // Current TCP via FK (already populated by ReadState / FillLogOutput path
  // earlier this tick — but be defensive: arm_handle_ keeps last computed
  // FK so GetFramePlacement is RT-safe even if no FK was issued this tick).
  pinocchio::SE3 start_pose = arm_handle_->GetFramePlacement(tip_frame_id_);
  if (use_root_frame_) {
    start_pose = arm_handle_->GetFramePlacement(root_frame_id_).actInv(start_pose);
  }
  const pinocchio::SE3 goal_pose = tcp_goal_;

  const auto gains = gains_lock_.Load();

  const Eigen::Vector3d start_pos = start_pose.translation();
  const Eigen::Vector3d goal_pos = goal_pose.translation();
  const double trans_dist = (goal_pos - start_pos).norm();

  // Quintic rest-to-rest peak velocity = 1.875 · d / T.
  const double T_speed_trans = trans_dist / gains.tcp_trajectory_speed;
  const double T_vel_trans = (gains.tcp_max_traj_velocity > 0.0)
                                 ? (1.875 * trans_dist / gains.tcp_max_traj_velocity)
                                 : 0.0;

  // Angular distance via AngleAxisd (stable at θ → π, unlike log3).
  const Eigen::AngleAxisd aa(start_pose.rotation().transpose() * goal_pose.rotation());
  const double angular_dist = aa.angle();  // [0, π]
  const Eigen::Vector3d rot_axis = aa.axis();

  const double T_speed_rot = angular_dist / gains.tcp_trajectory_angular_speed;
  const double T_vel_rot = (gains.tcp_max_traj_angular_velocity > 0.0)
                               ? (1.875 * angular_dist / gains.tcp_max_traj_angular_velocity)
                               : 0.0;
  const double duration = std::max({0.01, T_speed_trans, T_vel_trans, T_speed_rot, T_vel_rot});

  const bool split_trajectory = (angular_dist > M_PI - gains.pi_rotation_margin);

  if (split_trajectory) {
    // ── π-rotation defense: split into 2 rest-to-rest segments ──────────
    const double half_angle = angular_dist * 0.5;
    const Eigen::Matrix3d R_mid =
        start_pose.rotation() * Eigen::AngleAxisd(half_angle, rot_axis).toRotationMatrix();

    pinocchio::SE3 mid_pose;
    mid_pose.translation() = 0.5 * (start_pos + goal_pos);
    mid_pose.rotation() = R_mid;

    const double half_trans = trans_dist * 0.5;
    const double T1_speed_t = half_trans / gains.tcp_trajectory_speed;
    const double T1_vel_t = (gains.tcp_max_traj_velocity > 0.0)
                                ? (1.875 * half_trans / gains.tcp_max_traj_velocity)
                                : 0.0;
    const double T1_speed_r = half_angle / gains.tcp_trajectory_angular_speed;
    const double T1_vel_r = (gains.tcp_max_traj_angular_velocity > 0.0)
                                ? (1.875 * half_angle / gains.tcp_max_traj_angular_velocity)
                                : 0.0;
    const double dur1 = std::max({0.01, T1_speed_t, T1_vel_t, T1_speed_r, T1_vel_r});

    tcp_trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), mid_pose,
                               pinocchio::Motion::Zero(), dur1);
    pending_tcp_goal_ = goal_pose;
    pending_tcp_duration_ = dur1;  // symmetric split
    has_pending_tcp_segment_ = true;
  } else {
    tcp_trajectory_.initialize(start_pose, pinocchio::Motion::Zero(), goal_pose,
                               pinocchio::Motion::Zero(), duration);
    has_pending_tcp_segment_ = false;
  }

  tcp_trajectory_time_ = 0.0;
  tcp_trajectory_active_ = true;
}

// ── Control modes ────────────────────────────────────────────────────────────

}  // namespace integrated_bringup
