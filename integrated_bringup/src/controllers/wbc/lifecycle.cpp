#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────
RTControllerInterface::CallbackReturn DemoWbcController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // F-2: surface base_frame ↔ urdf.root_link mismatch detected in
  // OnDeviceConfigsSet. RCLCPP_ERROR was already emitted there; this
  // gate fails the lifecycle transition so the controller never becomes
  // active with a quietly broken SE3 reference frame.
  if (base_frame_mismatch_) {
    RCLCPP_ERROR(logger_, "DemoWbcController on_configure: %s",
                 base_frame_mismatch_detail_.c_str());
    return CallbackReturn::FAILURE;
  }
  try {
    CreateOwnedTopics(*this, owned_topics_);

    // ── Controller-owned non-RT publisher (no YAML role mapping) ─────────
    // WbcState rides under the secondary device's namespace (e.g.
    // "p1a/wbc_state" for ur5e_p1a, "leap/wbc_state" for iiwa7_leap).
    // Robot-agnostic — derives the prefix from GetSecondaryDeviceName().
    {
      const auto secondary = GetSecondaryDeviceName();
      if (!secondary.empty()) {
        SetupWbcStatePublisher(*this, owned_topics_, secondary + "/wbc_state", secondary);
      }
    }

    mpc_timing_cb_group_ =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Reference frame for the vector payloads (pull estimate force / plane
    // normal / basis). Same arm root the TSID contact geometry is expressed
    // in, read from the system URDF YAML so no robot name appears here
    // (#234 P-5).
    {
      const auto* sys_cfg = GetSystemModelConfig();
      if (sys_cfg != nullptr && !sys_cfg->sub_models.empty()) {
        SetOwnedStateFrameId(owned_topics_, sys_cfg->sub_models.front().root_link);
      }
    }

    // ── kRobotTransforms: register frame slots ─────────────────────────────
    // DemoWbc broadcasts arm tip + 4 fingertip frames + an alpha placeholder.
    // (#123 Phase 2) The fingertip poses are produced by ComputeHandFingertipFk
    // on the publish side — loop-consistent (extended hands, DIP-responsive) or
    // serial tree-model FK — and fed via task_link_poses (kHandTip). This is the
    // observation surface only; the TSID/MPC control model is untouched. The
    // placeholder slot reserves a future alpha frame (D-5) — slot_valid=false,
    // publish path skips it.
    if (owned_topics_.tf_pub) {
      const auto* sys_cfg = GetSystemModelConfig();
      if (sys_cfg && !sys_cfg->sub_models.empty()) {
        const auto& submodel = sys_cfg->sub_models.front();
        AppendArmTipSlot(owned_topics_, submodel.root_link, submodel.tip_link, /*group_idx=*/0);
      }
      // Parent = ARM root (sub_models[0].root_link, e.g. base), NOT the hand tree
      // root: ComputeHandFingertipFk composes each fingertip pose to the arm base
      // frame via the TCP placement (tcp = base→tool0), so the published
      // translation is base-relative. Labelling the parent as the hand root
      // (base_adapter / hand_base_link) double-counts the arm reach in RViz.
      if (sys_cfg && !sys_cfg->sub_models.empty()) {
        const auto secondary = GetSecondaryDeviceName();
        if (!secondary.empty()) {
          for (const auto& tm : sys_cfg->tree_models) {
            if (tm.name == secondary) {
              // Register only as many hand-tip slots as ComputeHandFingertipFk
              // fills (kNumFingertips): extra tip_links would register slots
              // that never receive a pose and silently never broadcast (#125 F4).
              AppendHandTipSlots(owned_topics_, sys_cfg->sub_models.front().root_link, tm.tip_links,
                                 /*group_idx=*/0,
                                 /*max_tips=*/kNumFingertips);
              break;
            }
          }
        }
      }
      AppendCustomPlaceholderSlot(owned_topics_, "base", "wbc_alpha_actual");
    }

    // ── PR2 (U3) Lift: Phase C controller-owned CSV log registration ──────
    // Log instance keys derived from device names so YAML `instance:` values
    // track the active config_variant (robot-agnostic, ARCH-1).
    const auto primary = GetPrimaryDeviceName();
    const auto secondary = GetSecondaryDeviceName();
    const auto primary_state_key = primary + "_state";
    const auto secondary_state_key = secondary.empty() ? std::string{} : secondary + "_state";
    const auto secondary_sensor_key = secondary.empty() ? std::string{} : secondary + "_sensor";
    const std::string wbc_diag_key = "wbc_diag";

    // WBC arm/hand state channels use DeviceWbcLog (superset POD) — role 0 =
    // arm (SE3 task block), role 1 = hand (motor + fingertip-force block).
    // wbc_diag carries the per-tick TSID/QP solution; λ column count = the
    // fixed QP contact dim (clamped to the POD capacity).
    const auto num_contact_vars =
        std::min(static_cast<std::size_t>(std::max(contact_mgr_config_.max_contact_vars, 0)),
                 integrated_bringup::WbcDiagLogPod::kMaxContactVars);

    LogRegistrationContext ctx{logger_, log_set_, {}, {}, {}, {}, {}};
    ctx.sensor_logs = {{secondary_sensor_key, secondary_sensor_names_}};
    ctx.wbc_state_logs = {
        {primary_state_key, {/*role=*/0, primary_joint_names_, {}, {}}},
        {secondary_state_key,
         {/*role=*/1, secondary_joint_names_, secondary_motor_names_, secondary_sensor_names_}},
    };
    ctx.wbc_diag_logs = {{wbc_diag_key, num_contact_vars}};
    ctx.pull_estimator_enabled = pull_wiring_.enabled();
    ctx.pull_estimator_roles = pull_wiring_.roles;

    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      return CallbackReturn::FAILURE;
    }
    // LogHandle is a trivially-copyable pointer wrapper — plain assignment.
    if (auto it = reg.handles.wbc_state.find(primary_state_key);
        it != reg.handles.wbc_state.end()) {
      primary_wbc_log_handle_ = it->second;
    }
    if (!secondary_state_key.empty()) {
      if (auto it = reg.handles.wbc_state.find(secondary_state_key);
          it != reg.handles.wbc_state.end()) {
        secondary_wbc_log_handle_ = it->second;
      }
    }
    if (!secondary_sensor_key.empty()) {
      if (auto it = reg.handles.sensor.find(secondary_sensor_key); it != reg.handles.sensor.end()) {
        secondary_sensor_log_handle_ = it->second;
      }
    }
    if (auto it = reg.handles.wbc_diag.find(wbc_diag_key); it != reg.handles.wbc_diag.end()) {
      wbc_diag_log_handle_ = it->second;
    }
    pull_estimator_log_handle_ = reg.handles.pull_estimator;
    if (!log_set_.empty() && node) {
      log_drain_cb_group_ =
          node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    // Phase D: declare tunable gains as ROS 2 parameters seeded from
    // gains_lock_ (populated by LoadConfig from YAML), register the
    // set-parameters callback, and create the WBC FSM grasp_command srv.
    DeclareGainParameters();
    param_callback_handle_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
          return OnGainParametersSet(params);
        });

    // Force-PI grasp_command srv: WBC has no grasp_controller_ — the srv
    // handler updates grasp_cmd_ atomic + gains.grasp_target_force, which
    // the WBC FSM (UpdatePhase) reads to drive kApproach/kRelease.
    grasp_command_srv_ = node_->create_service<rtc_msgs::srv::GraspCommand>(
        "grasp_command", [this](const std::shared_ptr<rtc_msgs::srv::GraspCommand::Request> req,
                                std::shared_ptr<rtc_msgs::srv::GraspCommand::Response> resp) {
          if (estopped_.load(std::memory_order_acquire)) {
            resp->ok = false;
            resp->message = "E-STOP active";
            return;
          }
          using Req = rtc_msgs::srv::GraspCommand::Request;
          if (req->command == Req::GRASP) {
            if (!(req->target_force > 0.0)) {
              resp->ok = false;
              resp->message = "GRASP requires target_force > 0";
              return;
            }
            auto g = gains_lock_.Load();
            g.grasp_target_force = req->target_force;
            gains_lock_.Store(g);
            grasp_cmd_.store(static_cast<int>(Req::GRASP), std::memory_order_release);
            RCLCPP_INFO(logger_, "[grasp_command] GRASP target=%.2fN (WBC FSM kApproach)",
                        req->target_force);
            resp->ok = true;
            resp->message = "grasp started @ " + std::to_string(req->target_force) + " N";
          } else if (req->command == Req::RELEASE) {
            grasp_cmd_.store(static_cast<int>(Req::RELEASE), std::memory_order_release);
            RCLCPP_INFO(logger_, "[grasp_command] RELEASE (WBC FSM kRelease)");
            resp->ok = true;
            resp->message = "release accepted";
          } else {
            resp->ok = false;
            resp->message = "command must be GRASP or RELEASE";
          }
        });

    // Apply the posture arm/hand gain split now that LoadConfig built the
    // PostureTask and OnDeviceConfigsSet built the joint reorder map. No-op
    // when the YAML omits the split (PostureTask::Init's kp/kd then stands).
    ApplyPostureGains();

    // Stage C-2: initialise the CLIK reference generator (registers the
    // se3_tcp tip/base frames on the shared cache + Init's clik_ with the
    // arm/hand v-index sets). Must run here — after LoadConfig (TSID built,
    // frames registered by SE3Task) and OnDeviceConfigsSet (reorder map +
    // frame ids) — and before on_activate spins up the RT loop whose first
    // cache.Update() locks frame registration.
    InitClik();

    // DEC-1 ⓐ: CLIK-QP is the SOLE position backbone (the integrator was
    // removed). A TSID-initialised controller that could not enable CLIK (the
    // dominant cause being a non-reduced nq != nv model; InitClik logs the
    // specific reason) has no position backbone — fail the transition rather
    // than activate a controller that would hold-last forever.
    if (tsid_initialized_ && !clik_enabled_) {
      RCLCPP_ERROR(logger_,
                   "DemoWbcController on_configure: CLIK could not be enabled but is the "
                   "required position backbone (see prior error) — refusing to configure.");
      return CallbackReturn::FAILURE;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "DemoWbcController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(logger_, "DemoWbcController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoWbcController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);

  // Pull-estimator latches (grasp edge, contact/touch hysteresis, filter tail,
  // baseline) are otherwise only cleared at configure, so a deactivate/activate
  // cycle — or the E-STOP path, which early-returns before the estimator tick —
  // would resume mid-grasp state against a possibly different object.
  ResetPullEstimatorRtState(pull_wiring_);

  // MPC tick-timing CSV + 1 Hz aux timer: one-shot setup per controller
  // lifetime (gated on mpc_timing_initialized_). Re-activation after a
  // deactivate must NOT re-Open the CSV (truncates accumulated rows) or
  // re-register the timer (executor churn).
  if (mpc_enabled_ && !mpc_timing_initialized_) {
    if (!mpc_timing_logger_.Open()) {
      RCLCPP_WARN(logger_, "MpcTimingLogger::Open() failed — MPC timing CSV disabled");
    } else {
      RCLCPP_INFO(logger_, "MPC tick-timing CSV: %s", mpc_timing_logger_.Path().c_str());
    }
    auto node = get_lifecycle_node();
    if (node && mpc_timing_cb_group_) {
      mpc_timing_timer_ = node->create_wall_timer(
          std::chrono::seconds(1), [this]() { LogMpcSolveTimingTick(); }, mpc_timing_cb_group_);
    }
    mpc_timing_initialized_ = true;
  }

  // Spawn the MPC thread on the aux thread (heap-allocating; must not run on
  // the RT path). Idempotent — re-activation reuses the existing thread.
  // Trajectory / FSM / target-slot init happens lazily in Compute() via
  // DrainTargetSlot's first-tick path (needs the current device state).
  SpawnMpcThreadIfNeeded();

  // Base bumps the activation generation + calls ResetTargetInitialization().
  const auto rc = RTControllerInterface::on_activate(prev);

  // Resume the MPC solve loop if the thread has already been spawned.
  if (mpc_thread_) {
    mpc_thread_->Resume();
  }

  return rc;
}

RTControllerInterface::CallbackReturn DemoWbcController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // Pause the MPC solve loop so it stops burning CPU while this controller
  // is inactive. The timing logger / timer keep running — the MPCThread
  // TimingProducer drain naturally yields zero rows while paused, and the
  // aggregate stats are skipped via the mpc_manager_.Enabled() guard at
  // the top of LogMpcSolveTimingTick.
  if (mpc_thread_) {
    mpc_thread_->Pause();
  }
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();  // flush in-flight log SPSC residue
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoWbcController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  mpc_timing_timer_.reset();
  mpc_timing_cb_group_.reset();
  mpc_timing_initialized_ = false;
  mpc_timing_tick_ = 0;
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetOwnedTopics(owned_topics_);
  grasp_command_srv_.reset();
  param_callback_handle_.reset();
  return RTControllerInterface::on_cleanup(prev);
}

// ── Phase D: gain → ROS 2 parameter declaration & callback ────────────────
//
// Mirrors DemoTask/DemoJoint Phase D pattern but with WBC-specific keys:
// arm_*/hand_* trajectory, TSID weights, MPC runtime gates. mpc_enable and
// riccati_gain_scale are forwarded into mpc_manager_ at set time so they
// take effect immediately (no controller restart required).

}  // namespace integrated_bringup
