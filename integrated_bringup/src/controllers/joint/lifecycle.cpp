#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────
// Delegates to integrated_bringup::owned_topics helpers so the 3 demo controllers
// share a single implementation; only storage lives in the subclass.

RTControllerInterface::CallbackReturn DemoJointController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    CreateOwnedTopics(*this, owned_topics_);

    // ── Controller-owned non-RT publishers (no YAML role mapping) ─────────
    // GraspState rides under the secondary device's namespace, robot-agnostic
    // (e.g. "p1a/grasp_state" for ur5e_p1a). The full topic path is
    // /<config_key>/<secondary>/<topic> because the controller's
    // LifecycleNode lives under /<config_key>/. Only created when the
    // secondary device has fingertip sensors (sensor_names non-empty) —
    // robots without tactile sensing (e.g. iiwa7_leap) skip these.
    {
      const auto secondary = GetSecondaryDeviceName();
      if (!secondary.empty()) {
        const auto* secondary_cfg = GetDeviceNameConfig(secondary);
        if (secondary_cfg != nullptr && !secondary_cfg->sensor_names.empty()) {
          SetupGraspStatePublisher(*this, owned_topics_, secondary + "/grasp_state", secondary);
          // ToF lives under "tof/snapshot" (legacy from before per-controller
          // namespacing — BT subscribes to /<ctrl>/tof/snapshot). Independent
          // of the secondary device name.
          SetupToFSnapshotPublisher(*this, owned_topics_, "tof/snapshot");
        }
      }
    }

    // Reference frame for the vector payloads (pull estimate force / plane
    // normal / basis). Same arm root the fingertip FK is composed into below,
    // read from the system URDF YAML so no robot name appears here (#234 P-5).
    {
      const auto* sys_cfg = GetSystemModelConfig();
      if (sys_cfg != nullptr && !sys_cfg->sub_models.empty()) {
        SetOwnedStateFrameId(owned_topics_, sys_cfg->sub_models.front().root_link);
      }
    }

    // ── kRobotTransforms: register frame slots from system URDF YAML ──────
    // DemoJoint frame layout (D-3 _actual suffix convention), robot-agnostic:
    //   sub_models[0] (primary device):     base → <tip>_actual        (group 0)
    //   tree_models[secondary device name]: base → <fingertip>_actual ×N (group 1)
    //     (parent = arm root, NOT hand tree root — poses are base-framed)
    //   virtual TCP:                        base → virtual_tcp_actual (group 0)
    // Slot list is fixed at on_configure; publish thread skips invalid
    // poses via PublishSnapshot::*_valid flags.
    if (owned_topics_.tf_pub) {
      const auto* sys_cfg = GetSystemModelConfig();
      // Primary arm tip — sub_models[0]
      if (sys_cfg && !sys_cfg->sub_models.empty()) {
        const auto& sm = sys_cfg->sub_models.front();
        AppendArmTipSlot(owned_topics_, sm.root_link, sm.tip_link, /*group_idx=*/0);
      }
      // Secondary hand fingertips — tree_models[GetSecondaryDeviceName()].
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
              // Cap slots to the fingertip count the compute side fills (#125 F4).
              AppendHandTipSlots(owned_topics_, sys_cfg->sub_models.front().root_link, tm.tip_links,
                                 /*group_idx=*/1,
                                 /*max_tips=*/kNumFingertips);
              break;
            }
          }
        }
      }
      // Virtual TCP — broadcast under arm group, parent = arm root_link
      if (sys_cfg && !sys_cfg->sub_models.empty()) {
        AppendVirtualTcpSlot(owned_topics_, sys_cfg->sub_models.front().root_link,
                             /*group_idx=*/0);
      }
    }

    // ── PR2 (U3) Lift: Phase C controller-owned CSV log registration ──────
    // Caller maps instance strings → (joint_names, motor_names, sensor_names).
    // The helper iterates parsed_log_entries_ once, returns instance-keyed
    // handles, and we plug the ones we own into typed members below.
    // Log instance keys are derived from device names so YAML `instance:`
    // values track the active config_variant (e.g. ur5e_state/p1a_state
    // for ur5e_p1a vs iiwa7_state/leap_state for iiwa7_leap).
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
            {secondary_sensor_key, {secondary_sensor_names_, secondary_sensor_values_per_group_}},
        },
        {},                      // wbc_state_logs — WBC controller only
        {},                      // wbc_diag_logs  — WBC controller only
        pull_wiring_.enabled(),  // pull_estimator_enabled
        pull_wiring_.roles,      // pull_estimator_roles (mask bit order)
    };
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      ResetLogState();  // roll back channels registered before the missing one (#238)
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
    pull_estimator_log_handle_ = std::move(reg.handles.pull_estimator);

    // Drain timer on a non-RT callback group (10 Hz). Single-threaded —
    // executor's MutuallyExclusive group is sufficient.
    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    // Phase D: declare tunable gains as ROS 2 parameters seeded from
    // gains_lock_ (populated by LoadConfig from YAML), register the
    // set-parameters callback, and create the Force-PI grasp_command srv.
    DeclareGainParameters();
    param_callback_handle_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
          return OnGainParametersSet(params);
        });

    grasp_command_srv_ = node_->create_service<rtc_msgs::srv::GraspCommand>(
        "grasp_command", [this](const std::shared_ptr<rtc_msgs::srv::GraspCommand::Request> req,
                                std::shared_ptr<rtc_msgs::srv::GraspCommand::Response> resp) {
          if (estopped_.load(std::memory_order_acquire)) {
            resp->ok = false;
            resp->message = "E-STOP active";
            return;
          }
          // Activity, before any mode question: this service outlives
          // on_deactivate, so an Inactive controller used to answer "grasp
          // started" and arm a request that no tick of its own would consume —
          // then fire it on the first tick after the next activation. on_activate
          // now Resets() the FSM, which disarms that; this stops the lie.
          if (!active_.load(std::memory_order_acquire)) {
            resp->ok = false;
            resp->message = "controller is not active";
            return;
          }
          // ONE snapshot for the whole callback: the decision and the log line
          // that reports it must name the same mode. Three separate Loads could
          // refuse on one mode and log another if a set landed in between.
          const auto mode = gains_lock_.Load().grasp_hand_mode;
          // Mode, not just nullness (#327 B-3). BuildGraspController no longer
          // consults the mode, so a non-null controller no longer means the PI law
          // is the one driving the hand — accepting here would answer "grasp
          // started" and step an FSM the tick ignores.
          if (const char* why = GraspCommandRejectReason(mode, grasp_controller_ != nullptr);
              why != nullptr) {
            resp->ok = false;
            resp->message = why;
            return;
          }
          using Req = rtc_msgs::srv::GraspCommand::Request;
          if (req->command == Req::GRASP) {
            if (!(req->target_force > 0.0)) {
              resp->ok = false;
              resp->message = "GRASP requires target_force > 0";
              return;
            }
            // The mirror, not phase(): the FSM is mutated by Update() on the RT
            // tick, so a direct read here races it. Tick-old is exactly right for
            // "the phase before this command" — the command has not been applied.
            const auto phase_before =
                static_cast<unsigned>(grasp_phase_pub_.load(std::memory_order_acquire));
            grasp_controller_->CommandGrasp(req->target_force);
            RCLCPP_INFO(logger_, "[grasp_command] GRASP target=%.2fN type=%s phase_before=%u",
                        req->target_force, GraspHandModeName(mode),
                        phase_before);
            resp->ok = true;
            resp->message = "grasp started @ " + std::to_string(req->target_force) + " N";
          } else if (req->command == Req::RELEASE) {
            const auto phase_before =
                static_cast<unsigned>(grasp_phase_pub_.load(std::memory_order_acquire));
            grasp_controller_->CommandRelease();
            RCLCPP_INFO(logger_, "[grasp_command] RELEASE type=%s phase_before=%u",
                        GraspHandModeName(mode), phase_before);
            resp->ok = true;
            resp->message = "release accepted";
          } else {
            resp->ok = false;
            resp->message = "command must be GRASP or RELEASE";
          }
        });
  } catch (const std::exception& e) {
    ResetLogState();  // don't leave channels registered on a failed configure (#238)
    RCLCPP_ERROR(logger_, "DemoJointController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    ResetLogState();
    RCLCPP_ERROR(logger_, "DemoJointController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

void DemoJointController::ResetLogState() noexcept {
  // Close open channels and return every typed handle to unbound. Without
  // this, a cleanup→configure cycle re-enters RegisterLog() with the old
  // channels still present, so the Q-MSG-3 duplicate guard hands back unbound
  // handles and CSV logging silently dies (#238).
  log_set_.Reset();
  // Reset() destroys every channel's drop counter, so TotalDropCount() restarts
  // at 0; drop the stale high-water mark in lockstep or DrainControllerLogs
  // silently swallows the fresh session's first drop burst (#238).
  log_drops_reported_ = 0;
  primary_state_log_handle_ = {};
  secondary_state_log_handle_ = {};
  secondary_sensor_log_handle_ = {};
  pull_estimator_log_handle_ = {};
}

RTControllerInterface::CallbackReturn DemoJointController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);
  // Pull-estimator latches (grasp edge, contact/touch hysteresis, filter tail,
  // baseline) are otherwise only cleared at configure, so a deactivate/activate
  // cycle would resume mid-grasp state against a possibly different object.
  ResetPullEstimatorRtState(pull_wiring_);
  // The identical argument, applied to the grasp FSM — which it was not, until a
  // review found the gap. A deactivate mid-grasp freezes the FSM (nothing steps
  // it while Inactive) with its request flags still armed, so the first tick after
  // re-activation resumes a squeeze against whatever the hand now holds, and the
  // quiet gate's phase mirror stays non-Idle in the meantime. Safe here: the CM
  // activates before it ticks this controller, so no RT tick is in flight — the
  // same window ResetPullEstimatorRtState relies on.
  if (grasp_controller_) {
    grasp_controller_->Reset();
  }
  grasp_phase_pub_.store(static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle),
                         std::memory_order_release);
  // Gates the grasp_command service (below Inactive it would arm a request no
  // tick of this controller consumes and answer "grasp started").
  active_.store(true, std::memory_order_release);
  // The base bumps the activation generation (invalidating targets queued while
  // Inactive) and calls ResetTargetInitialization(), which forces a fresh
  // self-init on the first Compute() tick. Single-writer invariant preserved —
  // the RT thread is still the only one that stores into target_seqlock_.
  return RTControllerInterface::on_activate(prev);
}

RTControllerInterface::CallbackReturn DemoJointController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // First, before anything else can block: the service stays alive across
  // deactivation, so this is what stops it accepting commands for a controller
  // that has stopped ticking.
  active_.store(false, std::memory_order_release);
  DeactivateOwnedTopics(prev, owned_topics_);
  // Flush any in-flight log samples — controller switch leaves SPSC residue
  // that would otherwise replay on the next on_activate.
  log_set_.DrainAll();
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoJointController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  ResetOwnedTopics(owned_topics_);
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  // Close log channels + unbind handles so a re-configure re-registers cleanly
  // instead of hitting the Q-MSG-3 duplicate guard (#238). Order: after the
  // drain timer is torn down, so no drain runs concurrently with Reset().
  ResetLogState();
  grasp_command_srv_.reset();
  param_callback_handle_.reset();
  return RTControllerInterface::on_cleanup(prev);
}

}  // namespace integrated_bringup
