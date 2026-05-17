#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────
RTControllerInterface::CallbackReturn DemoTaskController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    CreateOwnedTopics(*this, owned_topics_);

    // ── Controller-owned non-RT publishers (no YAML role mapping) ─────────
    // GraspState rides under the secondary device's namespace (e.g.
    // "hand/grasp_state" for ur5e_hand). Only created when the secondary
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

    // ── kRobotTransforms: register frame slots (Phase 3) ──────────────────
    // DemoTask shares the DemoJoint frame layout — arm tip + 4 fingertip +
    // virtual_tcp = 6 frames. See joint/lifecycle.cpp for rationale.
    if (owned_topics_.tf_pub) {
      const auto* sys_cfg = GetSystemModelConfig();
      if (sys_cfg && !sys_cfg->sub_models.empty()) {
        const auto& submodel = sys_cfg->sub_models.front();
        AppendArmTipSlot(owned_topics_, submodel.root_link, submodel.tip_link, /*group_idx=*/0);
      }
      if (sys_cfg) {
        const auto secondary = GetSecondaryDeviceName();
        if (!secondary.empty()) {
          for (const auto& tm : sys_cfg->tree_models) {
            if (tm.name == secondary) {
              AppendHandTipSlots(owned_topics_, tm.root_link, tm.tip_links, /*group_idx=*/1);
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
    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(100), [this]() { log_set_.DrainAll(); }, log_drain_cb_group_);
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
          if (!grasp_controller_) {
            resp->ok = false;
            resp->message =
                "grasp_controller unavailable (set 'grasp_controller_type: "
                "force_pi' in YAML to enable Grasp/Release)";
            return;
          }
          using Req = rtc_msgs::srv::GraspCommand::Request;
          if (req->command == Req::GRASP) {
            if (!(req->target_force > 0.0)) {
              resp->ok = false;
              resp->message = "GRASP requires target_force > 0";
              return;
            }
            const auto phase_before = static_cast<unsigned>(grasp_controller_->phase());
            grasp_controller_->CommandGrasp(req->target_force);
            RCLCPP_INFO(logger_, "[grasp_command] GRASP target=%.2fN type=%s phase_before=%u",
                        req->target_force, grasp_controller_type_.c_str(), phase_before);
            resp->ok = true;
            resp->message = "grasp started @ " + std::to_string(req->target_force) + " N";
          } else if (req->command == Req::RELEASE) {
            const auto phase_before = static_cast<unsigned>(grasp_controller_->phase());
            grasp_controller_->CommandRelease();
            RCLCPP_INFO(logger_, "[grasp_command] RELEASE type=%s phase_before=%u",
                        grasp_controller_type_.c_str(), phase_before);
            resp->ok = true;
            resp->message = "release accepted";
          } else {
            resp->ok = false;
            resp->message = "command must be GRASP or RELEASE";
          }
        });
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "DemoTaskController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(logger_, "DemoTaskController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoTaskController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);
  target_initialized_.store(false, std::memory_order_release);
  return RTControllerInterface::on_activate(prev);
}

RTControllerInterface::CallbackReturn DemoTaskController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();  // flush in-flight log SPSC residue
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoTaskController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  ResetOwnedTopics(owned_topics_);
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  grasp_command_srv_.reset();
  param_callback_handle_.reset();
  return RTControllerInterface::on_cleanup(prev);
}

}  // namespace integrated_bringup
