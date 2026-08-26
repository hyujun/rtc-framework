#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────
RTControllerInterface::CallbackReturn DemoComplianceController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    // ── §7.3 joint tail: refuse a margin that INVERTS a joint's clamp band ──
    //
    // `joint_limit_margin` is applied to both ends of every arm joint's band, so
    // a δ past half the narrowest band leaves `lo >= hi` and the tail SKIPS the
    // position clamp for that joint — a silent way to turn the guard off while
    // the YAML still reads as though it were on. Same call #473 made for the
    // sibling WBC binding's `integration.position_margin`.
    //
    // HERE, not in LoadConfig, and the difference is not stylistic. LoadConfig
    // runs in the CM's Pass 1 (`PreConfigure`), and the band does not exist yet:
    // device configs arrive in Pass 2 and `OnDeviceConfigsSet` is what fills
    // `device_position_lower_/upper_`. A check written next to the parser would
    // have validated the ±2π fallback every time and passed for any δ below 2π.
    // Pass 2 is also the wrong home — `SetDeviceNameConfigs` is called outside
    // any try/catch, so a throw there escapes the configure path entirely
    // instead of becoming the FAILURE this block turns it into.
    //
    // Resolved with the SAME fallbacks the tail uses, so the check and the RT
    // path cannot disagree about which joints they are talking about.
    {
      const auto& lower = device_position_lower_[0];
      const auto& upper = device_position_upper_[0];
      const std::size_t n = std::max({lower.size(), upper.size(), std::size_t{1}});
      const double margin = admittance_params_.joint_limit_margin;
      for (std::size_t i = 0; i < n; ++i) {
        const double raw_lo = (i < lower.size()) ? lower[i] : -6.2832;
        const double raw_hi = (i < upper.size()) ? upper[i] : 6.2832;
        if (raw_lo + margin >= raw_hi - margin) {
          throw std::runtime_error(
              "demo_compliance_controller: 'joint_limit_margin' (" + std::to_string(margin) +
              " rad) inverts the q_cmd clamp band on arm joint index " + std::to_string(i) +
              " (limits [" + std::to_string(raw_lo) + ", " + std::to_string(raw_hi) +
              "]) — it must stay below half the band width, or the §7.3 position clamp is "
              "silently skipped for that joint");
        }
      }
    }

    CreateOwnedTopics(*this, owned_topics_);

    // ── Controller-owned non-RT publishers (no YAML role mapping) ─────────
    // GraspState rides under the secondary device's namespace (e.g.
    // "p1a/grasp_state" for ur5e_p1a). Only created when the secondary
    // device has fingertip sensors (sensor_names non-empty) — robots
    // without tactile sensing skip these.
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
    // normal / basis). Same arm root the fingertip FK is composed into, read
    // from the system URDF YAML so no robot name appears here (#234 P-5).
    {
      const auto* sys_cfg = GetSystemModelConfig();
      if (sys_cfg != nullptr && !sys_cfg->sub_models.empty()) {
        SetOwnedStateFrameId(owned_topics_, sys_cfg->sub_models.front().root_link);
      }
    }

    // ── kRobotTransforms: register frame slots (Phase 3) ──────────────────
    // DemoCompliance shares the DemoJoint frame layout — arm tip + 4 fingertip +
    // virtual_tcp = 6 frames. See joint/lifecycle.cpp for rationale.
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
              // Cap slots to the fingertip count the compute side fills (#125 F4).
              AppendHandTipSlots(owned_topics_, sys_cfg->sub_models.front().root_link, tm.tip_links,
                                 /*group_idx=*/1,
                                 /*max_tips=*/kNumFingertips);
              break;
            }
          }
        }
      }
      if (sys_cfg && !sys_cfg->sub_models.empty()) {
        AppendVirtualTcpSlot(owned_topics_, sys_cfg->sub_models.front().root_link,
                             /*group_idx=*/0);
      }
    }

    // ── PR2 (U3) Lift: Phase C controller-owned CSV log registration ──────
    // Log instance keys derived from device names so YAML `instance:` values
    // track the active config_variant (robot-agnostic, ARCH-1).
    const auto primary = GetPrimaryDeviceName();
    const auto secondary = GetSecondaryDeviceName();
    const auto primary_state_key = primary + "_state";
    const auto secondary_state_key = secondary.empty() ? std::string{} : secondary + "_state";
    const auto secondary_sensor_key = secondary.empty() ? std::string{} : secondary + "_sensor";

    // ── #135 Layer 1b: consume the observer's configure verdict ──────────
    // The wiring itself is built in OnDeviceConfigsSet (that is where the arm
    // device's joint order first exists, and it has to precede the log
    // registration below, which gates on whether the observer is enabled). That
    // hook cannot fail a configure on its own, so a config error arrives here.
    if (!momentum_config_error_.empty()) {
      RCLCPP_ERROR(logger_, "momentum_observer configuration failed: %s",
                   momentum_config_error_.c_str());
      return CallbackReturn::FAILURE;
    }

    // ── #135 D12: PayloadEstimate topic ──────────────────────────────────
    // Gated on the OBSERVER, not on the payload estimator. The residual half
    // of the message stands on its own, and the shipped config has
    // payload_estimator.enabled: false — a payload-gated publisher would take
    // the residual off the wire along with it. `payload_frame` is left empty
    // unless the estimator actually configured, so a consumer can tell "no
    // estimator" from "estimator held this tick" without reading the reason.
    if (momentum_wiring_.enabled()) {
      SetupPayloadEstimatePublisher(
          *this, owned_topics_, "payload_estimate", primary_joint_names_,
          momentum_wiring_.payload_enabled() ? momentum_params_.payload.frame : std::string{});
    }

    LogRegistrationContext ctx{
        .logger = logger_,
        .log_set = log_set_,
        .state_logs =
            {
                {primary_state_key, {primary_joint_names_, std::vector<std::string>{}}},
                {secondary_state_key, {secondary_joint_names_, secondary_motor_names_}},
            },
        .sensor_logs =
            {
                {secondary_sensor_key,
                 {secondary_sensor_names_, secondary_sensor_values_per_group_}},
            },
        // wbc_state_logs / wbc_diag_logs — WBC controller only.
        .pull_estimator_enabled = pull_wiring_.enabled(),
        .pull_estimator_roles = pull_wiring_.roles,
        .task_diag_enabled = true,  // (#310) — §6.5 σ_min/λ² lane
        .grasp_diag_enabled = grasp_controller_ != nullptr,
        .grasp_diag_finger_names =
            GraspDiagFingerNames(secondary_sensor_names_, num_grasp_fingers_),
        // (#469 S4) — §7 admittance lane. Unconditional because the source key
        // is required with no default (D-A12): a controller that configured has
        // one, and it cannot change without a re-configure.
        .compliance_diag_enabled = true,
        .momentum_observer_enabled = momentum_wiring_.enabled(),
        .momentum_observer_joint_names = primary_joint_names_,
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
    momentum_observer_log_handle_ = std::move(reg.handles.momentum_observer);
    task_diag_log_handle_ = std::move(reg.handles.task_diag);
    grasp_diag_log_handle_ = std::move(reg.handles.grasp_diag);
    compliance_diag_log_handle_ = std::move(reg.handles.compliance_diag);
    LogGraspDiagWiring(logger_, grasp_controller_ != nullptr, ctx.grasp_diag_finger_names);
    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    // Phase B: declare tunable gains as ROS 2 parameters on the controller's
    // own LifecycleNode and register the set-parameters callback. LoadConfig
    // (run from the base class on_configure above) already seeded gains_lock_
    // from YAML; declare uses those as defaults so YAML-loaded values are the
    // initial parameter values.
    DeclareGainParameters();
    param_callback_handle_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
          return OnGainParametersSet(params);
        });

    // Phase B: Force-PI grasp command channel (one-shot event, NOT a gain).
    // Resolves to /<config_key>/grasp_command via the LifecycleNode's
    // namespace.
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
                        req->target_force, GraspHandModeName(mode), phase_before);
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
    RCLCPP_ERROR(logger_, "DemoComplianceController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    ResetLogState();
    RCLCPP_ERROR(logger_, "DemoComplianceController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

void DemoComplianceController::ResetLogState() noexcept {
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
  momentum_observer_log_handle_ = {};
  task_diag_log_handle_ = {};
  // #238: an unbound handle is what stops a re-configure from logging into a
  // channel whose file the previous configure owned.
  grasp_diag_log_handle_ = {};
  compliance_diag_log_handle_ = {};
}

RTControllerInterface::CallbackReturn DemoComplianceController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);
  // Pull-estimator latches (grasp edge, contact/touch hysteresis, filter tail,
  // baseline) are otherwise only cleared at configure, so a deactivate/activate
  // cycle would resume mid-grasp state against a possibly different object.
  ResetPullEstimatorRtState(pull_wiring_);
  // Same argument for the momentum observer: its integrator, momentum reference
  // and residual are per-tick latches. Resuming across a gap without dropping
  // them bills the momentum change accumulated while nothing was ticking as an
  // external torque — the residual would report a load that never existed and
  // then decay over ~1/K_I, which reads exactly like a real transient.
  ResetMomentumObserverRtState(momentum_wiring_);
  // #484: the joint-tail totals are per-ACTIVATION. Carrying them across a
  // deactivate/activate cycle would bill the previous session's clamping to this
  // one, and the deactivation summary's tick window — the half that lets an
  // operator line a pin up against what the arm was doing — would span a gap
  // during which nothing ticked at all.
  joint_tail_stats_.Reset();
  // #469 S3, §10.7: the compliant frame is a deviation from the reference, and
  // an activation must inherit none of it — otherwise the first tick asks the
  // arm to realise a displacement accrued before anyone was watching, which is
  // the jump §10.7 exists to forbid. The wrench goes with it: re-dating the
  // sample present at reset would revive a dead producer's last reading as
  // fresh (§10.6 says an expired wrench fades to ZERO, never holds).
  admittance_.Reset();
  wrench_pipeline_.ResetForActivation();
  // The FSM is NOT reset here (#469 review, E-8). A latched SAFE_STOP is the
  // one piece of state an activation must inherit: §10.6 forbids automatic
  // recovery, and a deactivate/activate cycle — which a BT or a
  // `switch_controller` round-trip performs on its own — is not an operator
  // re-authorising a fault. Laundering it here would also make the latch
  // unobservable after the fact, since nothing else records that it fired.
  // `/rtc_cm/reset_fault` (ResetFault() above) is the only exit, and it is
  // wired for exactly this reason.
  compliance_state_ = compliance_fsm_.state();
  compliance_engaged_ = false;
  compliance_ramp_elapsed_ = 0.0;
  wrench_quality_low_ = false;
  wrench_invalid_reason_ = 0;
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
  // Base bumps the activation generation + calls ResetTargetInitialization().
  return RTControllerInterface::on_activate(prev);
}

RTControllerInterface::CallbackReturn DemoComplianceController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // First, before anything else can block: the service stays alive across
  // deactivation, so this is what stops it accepting commands for a controller
  // that has stopped ticking.
  active_.store(false, std::memory_order_release);
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();  // flush in-flight log SPSC residue
  // #484: the §7.3 tail's activation summary. Silent unless the band actually
  // engaged — see joint_tail_stats.hpp.
  //
  // Reached on a controller SWITCH and on an explicit lifecycle
  // deactivate/shutdown, NOT on SIGINT: the CM's main() waits for
  // `rclcpp::ok()` to fall, cancels its executors and returns without driving
  // the lifecycle down. That is why the per-tick throttled WARN at the call site
  // is the primary channel and this line is the recap — a Ctrl-C'd bring-up
  // still has the WARNs in its terminal scrollback.
  //
  // Counters are final by the time this runs: on the switch path the CM has
  // already moved `active_controller_idx_` away and waited a tick (D-A1 step 4),
  // and on the lifecycle path it has stopped the RT loop outright. Worst case
  // under a future faster switch, the recap misses the last tick's increment —
  // the members are atomic, so that is a lost count, never a torn read.
  LogJointTailSummary(logger_, "[compliance]", joint_tail_stats_);
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoComplianceController::on_cleanup(
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
