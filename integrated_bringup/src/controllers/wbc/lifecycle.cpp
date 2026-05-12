#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"

#include <chrono>
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
    mpc_timing_cb_group_ =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ── kRobotTransforms: register frame slots (Phase 3) ──────────────────
    // DemoWbc broadcasts arm tip + alpha placeholder for now. The
    // integrated `tree_models.wbc` (root=base, tip_links=4 fingertips)
    // includes the fingertip frames but WBC compute does not yet do
    // per-fingertip FK on the publish side; adding that is deferred to a
    // follow-up so this phase lands the cutover plumbing without touching
    // the TSID/MPC fast path. The placeholder slot reserves a future
    // alpha frame (D-5) — slot_valid=false, publish path skips it.
    if (owned_topics_.tf_pub) {
      const auto* sys_cfg = GetSystemModelConfig();
      if (sys_cfg && !sys_cfg->sub_models.empty()) {
        const auto& submodel = sys_cfg->sub_models.front();
        AppendArmTipSlot(owned_topics_, submodel.root_link, submodel.tip_link, /*group_idx=*/0);
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

    LogRegistrationContext ctx{
        logger_,
        log_set_,
        {
            {primary_state_key, {primary_joint_names_, std::vector<std::string>{}}},
            {secondary_state_key, {secondary_joint_names_, secondary_motor_names_}},
        },
        {
            {secondary_sensor_key, secondary_sensor_names_},
        }};
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      return CallbackReturn::FAILURE;
    }
    if (auto it = reg.handles.state.find(primary_state_key); it != reg.handles.state.end()) {
      primary_state_log_handle_ = std::move(it->second);
    }
    if (!secondary_state_key.empty()) {
      if (auto it = reg.handles.state.find(secondary_state_key); it != reg.handles.state.end()) {
        secondary_state_log_handle_ = std::move(it->second);
      }
    }
    if (!secondary_sensor_key.empty()) {
      if (auto it = reg.handles.sensor.find(secondary_sensor_key); it != reg.handles.sensor.end()) {
        secondary_sensor_log_handle_ = std::move(it->second);
      }
    }
    if (!log_set_.empty() && node) {
      log_drain_cb_group_ =
          node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node->create_wall_timer(
          std::chrono::milliseconds(100), [this]() { log_set_.DrainAll(); }, log_drain_cb_group_);
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
    const rclcpp_lifecycle::State& prev, const rtc::ControllerState& device_snapshot) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);

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

  // Delegate to base for hold-init (InitializeHoldPosition lazy-spawns the
  // MPC thread on the first call). Snapshot is empty when CM activates the
  // initial controller before any RT tick — RT loop's auto-hold path then
  // takes over and triggers spawn on the first sensor read.
  const auto rc = RTControllerInterface::on_activate(prev, device_snapshot);

  // Resume the MPC solve loop if the thread has already been spawned (either
  // by the base on_activate above, or by a previous activation cycle). Calling
  // Resume on a freshly-spawned thread that was never paused is a no-op.
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
