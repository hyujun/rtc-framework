#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"

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
      if (cmd == 1 && robot_new_target_.load(std::memory_order_acquire)) {
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
      // Anomaly detection: slip (|df/dt|) or excessive deformation
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
        const float dmag = std::sqrt(ft.displacement[0] * ft.displacement[0] +
                                     ft.displacement[1] * ft.displacement[1] +
                                     ft.displacement[2] * ft.displacement[2]);
        if (dmag > deformation_threshold_) {
          RCLCPP_WARN_THROTTLE(logger_, log_clock_, integrated_bringup::logging::kThrottleSlowMs,
                               "[wbc] deformation detected f=%d |d|=%.3f > %.3f", f,
                               static_cast<double>(dmag), deformation_threshold_);
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
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        robot_computed_.positions[i] = dev0.positions[i];
        robot_computed_.velocities[i] = 0.0;
      }
      if (state.num_devices > 1 && dev1.valid) {
        for (std::size_t i = 0; i < kNumHandMotors; ++i) {
          hand_computed_.positions[i] = dev1.positions[i];
          hand_computed_.velocities[i] = 0.0;
        }
      }
      tcp_goal_valid_ = false;
      qp_fail_count_ = 0;
      // Deactivate all contacts
      for (auto& c : contact_state_.contacts) {
        c.active = false;
      }
      contact_state_.recompute_active(contact_mgr_config_);
      break;
    }

    case WbcPhase::kApproach: {
      // Build quintic trajectory: current → target (arm)
      std::lock_guard lock(target_mutex_);
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start{};
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal{};
      double max_delta = 0.0;
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        start.positions[i] = dev0.positions[i];
        q_approach_start_[i] = dev0.positions[i];  // save for kRetreat
        goal.positions[i] = device_targets_[0][i];
        const double delta = std::abs(goal.positions[i] - start.positions[i]);
        if (delta > max_delta) {
          max_delta = delta;
        }
      }
      const double duration = std::max(max_delta / gains.arm_trajectory_speed, 0.1);
      robot_trajectory_.initialize(start, goal, duration);
      robot_trajectory_time_ = 0.0;
      robot_new_target_.store(false, std::memory_order_relaxed);

      // Compute FK of arm target for SE3Task reference in kPreGrasp
      if (arm_handle_) {
        std::span<const double> q_target(device_targets_[0].data(), kNumRobotJoints);
        arm_handle_->ComputeForwardKinematics(q_target);
        tcp_goal_ = arm_handle_->GetFramePlacement(tip_frame_id_);
        if (use_root_frame_) {
          tcp_goal_ = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_goal_);
        }
        tcp_goal_valid_ = true;
      }

      // Hand trajectory (pre-shape)
      if (hand_new_target_.load(std::memory_order_acquire) && state.num_devices > 1 && dev1.valid) {
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State hstart{};
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State hgoal{};
        double hmax = 0.0;
        for (std::size_t i = 0; i < kNumHandMotors; ++i) {
          hstart.positions[i] = dev1.positions[i];
          hgoal.positions[i] = device_targets_[1][i];
          const double hd = std::abs(hgoal.positions[i] - hstart.positions[i]);
          if (hd > hmax) {
            hmax = hd;
          }
        }
        const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
        hand_trajectory_.initialize(hstart, hgoal, hdur);
        hand_trajectory_time_ = 0.0;
        hand_new_target_.store(false, std::memory_order_relaxed);
      }
      break;
    }

    case WbcPhase::kPreGrasp:
    case WbcPhase::kClosure:
    case WbcPhase::kHold: {
      // Apply TSID phase preset (RT-safe: uses pre-resolved PhasePreset)
      const auto idx = static_cast<std::size_t>(new_phase);
      if (phase_preset_valid_[idx]) {
        tsid_controller_.apply_phase_preset(phase_presets_[idx]);
      }

      // Set TSID integration initial conditions from current state
      ExtractFullState(state);
      q_next_full_ = q_curr_full_;
      v_next_full_ = v_curr_full_;

      // Set SE3Task reference (TCP goal)
      if (tcp_goal_valid_) {
        auto* se3_task = tsid_controller_.formulation().get_task("se3_tcp");
        if (se3_task) {
          static_cast<rtc::tsid::SE3Task*>(se3_task)->set_se3_reference(tcp_goal_);
        }
      }

      // Contact activation for closure/hold phases
      if (new_phase == WbcPhase::kClosure || new_phase == WbcPhase::kHold) {
        for (auto& c : contact_state_.contacts) {
          c.active = true;
        }
        contact_state_.recompute_active(contact_mgr_config_);

        // Set per-contact force reference: +Z normal = gains.grasp_target_force
        auto* force_task =
            tsid_initialized_ ? tsid_controller_.formulation().get_task("force") : nullptr;
        if (force_task) {
          const int n = contact_mgr_config_.max_contact_vars;
          if (n > 0) {
            Eigen::VectorXd lambda_des = Eigen::VectorXd::Zero(n);
            int offset = 0;
            for (const auto& c : contact_mgr_config_.contacts) {
              const int cdim = c.contact_dim;
              if (offset + cdim > n) {
                break;
              }
              // Point contact: lambda = [fx, fy, fz]; push +Z target force
              if (cdim >= 3) {
                lambda_des[offset + 2] = gains.grasp_target_force;
              }
              offset += cdim;
            }
            static_cast<rtc::tsid::ForceTask*>(force_task)->set_force_references(lambda_des);
          }
        }

        // Ramp hand joint target toward stored target (user-provided close pose)
        if (state.num_devices > 1 && dev1.valid) {
          trajectory::JointSpaceTrajectory<kNumHandMotors>::State hstart{};
          trajectory::JointSpaceTrajectory<kNumHandMotors>::State hgoal{};
          double hmax = 0.0;
          for (std::size_t i = 0; i < kNumHandMotors; ++i) {
            hstart.positions[i] = dev1.positions[i];
            hgoal.positions[i] = device_targets_[1][i];
            const double hd = std::abs(hgoal.positions[i] - hstart.positions[i]);
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
      // Deactivate contacts
      for (auto& c : contact_state_.contacts) {
        c.active = false;
      }
      contact_state_.recompute_active(contact_mgr_config_);

      // Arm trajectory: current → saved approach-start pose
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start{};
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal{};
      double max_delta = 0.0;
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        start.positions[i] = dev0.positions[i];
        goal.positions[i] = q_approach_start_[i];
        const double delta = std::abs(goal.positions[i] - start.positions[i]);
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
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State hstart{};
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State hgoal{};
        double hmax = 0.0;
        for (std::size_t i = 0; i < kNumHandMotors; ++i) {
          hstart.positions[i] = dev1.positions[i];
          hgoal.positions[i] = 0.0;
          const double hd = std::abs(hstart.positions[i]);
          if (hd > hmax) {
            hmax = hd;
          }
        }
        const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
        hand_trajectory_.initialize(hstart, hgoal, hdur);
        hand_trajectory_time_ = 0.0;
      }
      // Arm holds current pose during release
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        robot_computed_.positions[i] = dev0.positions[i];
        robot_computed_.velocities[i] = 0.0;
      }
      // Freeze arm trajectory (duration=0 so ComputePositionMode clamps)
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State hold{};
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        hold.positions[i] = dev0.positions[i];
      }
      robot_trajectory_.initialize(hold, hold, 0.01);
      robot_trajectory_time_ = 0.0;
      break;
    }

    case WbcPhase::kFallback: {
      // Hold current position, deactivate contacts
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        robot_computed_.positions[i] = dev0.positions[i];
        robot_computed_.velocities[i] = 0.0;
      }
      if (state.num_devices > 1 && dev1.valid) {
        for (std::size_t i = 0; i < kNumHandMotors; ++i) {
          hand_computed_.positions[i] = dev1.positions[i];
          hand_computed_.velocities[i] = 0.0;
        }
      }
      for (auto& c : contact_state_.contacts) {
        c.active = false;
      }
      contact_state_.recompute_active(contact_mgr_config_);
      qp_fail_count_ = 0;
      break;
    }
  }

  // ── GraspPhaseManager bridge (handler mode only) ──────────────────────────
  //
  // WBC FSM is authoritative for the demo; the grasp phase manager mirrors
  // it via ForcePhase so rtc_mpc picks up the matching OCP type
  // (light_contact vs contact_rich) on every WBC edge. `ForcePhase` is
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
