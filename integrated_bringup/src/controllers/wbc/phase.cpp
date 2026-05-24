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

// ── 8-state grasp FSM (Idle → Approach → PreGrasp → Closure → Hold →
//                       Retreat → Release → Fallback) ───────────────────────
void DemoWbcController::UpdatePhase(const ControllerState& state) noexcept {
  const int cmd = grasp_cmd_.load(std::memory_order_acquire);
  WbcPhase next = phase_;

  switch (phase_) {
    case WbcPhase::kIdle:
      // grasp_cmd=1 + valid target → approach
      if (cmd == 1 && robot_new_target_pending_) {
        next = WbcPhase::kApproach;
      }
      break;

    case WbcPhase::kApproach: {
      // Trajectory complete → pre-grasp (TSID)
      if (robot_trajectory_time_ >= robot_trajectory_.duration()) {
        if (tsid_initialized_) {
          next = WbcPhase::kPreGrasp;
        } else {
          next = WbcPhase::kIdle;  // No TSID, stay in position mode
        }
      }
      // Abort
      if (cmd == 0) {
        next = WbcPhase::kIdle;
      }
      break;
    }

    case WbcPhase::kPreGrasp: {
      // TCP close enough to goal → closure
      if (tcp_goal_valid_) {
        const double err = ComputeTcpError(tcp_goal_);
        if (err < epsilon_pregrasp_) {
          next = WbcPhase::kClosure;
        }
      }
      // Abort
      if (cmd == 0) {
        next = WbcPhase::kIdle;
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
      if (cmd == 0) {
        next = WbcPhase::kIdle;
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
      if (cmd == 2) {
        next = WbcPhase::kRetreat;
      }
      if (cmd == 0) {
        next = WbcPhase::kIdle;
      }
      break;
    }

    case WbcPhase::kRetreat:
      // Trajectory complete → release
      if (robot_trajectory_time_ >= robot_trajectory_.duration()) {
        next = WbcPhase::kRelease;
      }
      if (cmd == 0) {
        next = WbcPhase::kIdle;
      }
      break;

    case WbcPhase::kRelease:
      // Hand open complete → idle
      if (hand_trajectory_time_ >= hand_trajectory_.duration()) {
        next = WbcPhase::kIdle;
      }
      break;

    case WbcPhase::kFallback:
      // Manual recovery only: grasp_cmd=0 → idle
      if (cmd == 0) {
        next = WbcPhase::kIdle;
      }
      break;
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

  switch (new_phase) {
    case WbcPhase::kIdle: {
      // Hold current position
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
      tcp_goal_valid_ = false;
      qp_fail_count_ = 0;
      // Stage A-5b: kIdle ramps activation down to 0 (gentle release).
      for (int i = 0; i < static_cast<int>(contact_state_.contacts.size()); ++i) {
        contact_state_.SetActivationTarget(i, 0.0, contact_ramp_sec_);
      }
      contact_state_.RecomputeActive(contact_mgr_config_);
      break;
    }

    case WbcPhase::kApproach: {
      // Build quintic trajectory: current → target (arm). current_target_slot_
      // was refreshed at the top of Compute() by DrainTargetSlot; the RT
      // thread is still the sole SeqLock writer.
      trajectory::JointSpaceTrajectory<kMaxArmDof>::State start{};
      trajectory::JointSpaceTrajectory<kMaxArmDof>::State goal{};
      double max_delta = 0.0;
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        start.positions[idx] = dev0.positions[idx];
        q_approach_start_[idx] = dev0.positions[idx];  // save for kRetreat
        goal.positions[idx] = current_target_slot_.targets[0][idx];
        const double delta = std::abs(goal.positions[idx] - start.positions[idx]);
        if (delta > max_delta) {
          max_delta = delta;
        }
      }
      const double duration = std::max(max_delta / gains.arm_trajectory_speed, 0.1);
      robot_trajectory_.initialize(start, goal, duration);
      robot_trajectory_time_ = 0.0;
      robot_new_target_pending_ = false;

      // Compute FK of arm target for SE3Task reference in kPreGrasp
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
      }

      // Hand trajectory (pre-shape)
      if (hand_new_target_pending_ && state.num_devices > 1 && dev1.valid) {
        trajectory::JointSpaceTrajectory<kMaxHandDof>::State hstart{};
        trajectory::JointSpaceTrajectory<kMaxHandDof>::State hgoal{};
        double hmax = 0.0;
        for (int i = 0; i < hand_dof_; ++i) {
          const auto idx = static_cast<std::size_t>(i);
          hstart.positions[idx] = dev1.positions[idx];
          hgoal.positions[idx] = current_target_slot_.targets[1][idx];
          const double hd = std::abs(hgoal.positions[idx] - hstart.positions[idx]);
          if (hd > hmax) {
            hmax = hd;
          }
        }
        const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
        hand_trajectory_.initialize(hstart, hgoal, hdur);
        hand_trajectory_time_ = 0.0;
        hand_new_target_pending_ = false;
      }
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

      // Set TSID integration initial conditions from current state
      ExtractFullState(state);
      q_next_full_ = q_curr_full_;
      v_next_full_ = v_curr_full_;

      // Set SE3Task reference (TCP goal)
      if (tcp_goal_valid_) {
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

    case WbcPhase::kRetreat: {
      // Stage A-5b: ramp contact activation down to 0 over contact_ramp_sec_.
      for (int i = 0; i < static_cast<int>(contact_state_.contacts.size()); ++i) {
        contact_state_.SetActivationTarget(i, 0.0, contact_ramp_sec_);
      }
      contact_state_.RecomputeActive(contact_mgr_config_);

      // Arm trajectory: current → saved approach-start pose
      trajectory::JointSpaceTrajectory<kMaxArmDof>::State start{};
      trajectory::JointSpaceTrajectory<kMaxArmDof>::State goal{};
      double max_delta = 0.0;
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        start.positions[idx] = dev0.positions[idx];
        goal.positions[idx] = q_approach_start_[idx];
        const double delta = std::abs(goal.positions[idx] - start.positions[idx]);
        if (delta > max_delta) {
          max_delta = delta;
        }
      }
      const double duration = std::max(max_delta / gains.arm_trajectory_speed, 0.1);
      robot_trajectory_.initialize(start, goal, duration);
      robot_trajectory_time_ = 0.0;
      tcp_goal_valid_ = false;
      break;
    }

    case WbcPhase::kRelease: {
      // Hand open: all motors → 0
      if (state.num_devices > 1 && dev1.valid) {
        trajectory::JointSpaceTrajectory<kMaxHandDof>::State hstart{};
        trajectory::JointSpaceTrajectory<kMaxHandDof>::State hgoal{};
        double hmax = 0.0;
        for (int i = 0; i < hand_dof_; ++i) {
          const auto idx = static_cast<std::size_t>(i);
          hstart.positions[idx] = dev1.positions[idx];
          hgoal.positions[idx] = 0.0;
          const double hd = std::abs(hstart.positions[idx]);
          if (hd > hmax) {
            hmax = hd;
          }
        }
        const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
        hand_trajectory_.initialize(hstart, hgoal, hdur);
        hand_trajectory_time_ = 0.0;
      }
      // Arm holds current pose during release
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        robot_computed_.positions[idx] = dev0.positions[idx];
        robot_computed_.velocities[idx] = 0.0;
      }
      // Freeze arm trajectory (duration=0 so ComputePositionMode clamps)
      trajectory::JointSpaceTrajectory<kMaxArmDof>::State hold{};
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        hold.positions[idx] = dev0.positions[idx];
      }
      robot_trajectory_.initialize(hold, hold, 0.01);
      robot_trajectory_time_ = 0.0;
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
  // atomic and RT-safe (see grasp_phase_manager.hpp thread-safety notes);
  // `SetTaskTarget` uses a non-RT mutex but fires at most once per WBC edge
  // (not per 500 Hz tick), which is acceptable off the TSID hot path.
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
      case WbcPhase::kRetreat:
        grasp_id = static_cast<int>(phase::GraspPhaseId::kRetreat);
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

// ── Control modes ────────────────────────────────────────────────────────────

}  // namespace integrated_bringup
