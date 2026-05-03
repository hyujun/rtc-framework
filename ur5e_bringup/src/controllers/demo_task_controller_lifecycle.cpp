#include "ur5e_bringup/controllers/demo_task_controller.hpp"

#include "ur5e_bringup/controllers/owned_topics.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace ur5e_bringup {

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

    // ── Phase C: register controller-owned CSV log channels ─────────────
    for (const auto& entry : parsed_log_entries_) {
      if (entry.instance.empty()) {
        RCLCPP_ERROR(logger_, "logs entry msg_type=%s missing required `instance:` field",
                     entry.msg_type.c_str());
        return CallbackReturn::FAILURE;
      }
      if (entry.msg_type == "rtc_msgs/DeviceStateLog") {
        // Q-MSG-3: ur5e_state vs hand_state instance names.
        const bool is_hand = (entry.instance == "hand_state");
        const std::vector<std::string> joint_names_copy =
            is_hand ? hand_joint_names_ : ur5e_joint_names_;
        const std::vector<std::string> motor_names_copy =
            is_hand ? hand_motor_names_ : std::vector<std::string>{};
        auto handle = log_set_.RegisterLog<ur5e::DeviceStateLogPod>(
            entry.instance,
            [joint_names_copy, motor_names_copy](std::ostream& os) {
              ur5e::WriteDeviceStateLogHeader(os, joint_names_copy, motor_names_copy);
            },
            [](std::ostream& os, const ur5e::DeviceStateLogPod& p) {
              ur5e::WriteDeviceStateLogRow(os, p);
            });
        if (!handle) {
          RCLCPP_WARN(logger_, "Failed to open device_state CSV for instance=%s",
                      entry.instance.c_str());
        } else if (entry.instance == "ur5e_state") {
          ur5e_state_log_handle_ = handle;
        } else if (is_hand) {
          hand_state_log_handle_ = handle;
        }
      } else if (entry.msg_type == "rtc_msgs/DeviceSensorLog") {
        const std::vector<std::string> sensor_names_copy = hand_sensor_names_;
        auto handle = log_set_.RegisterLog<ur5e::DeviceSensorLogPod>(
            entry.instance,
            [sensor_names_copy](std::ostream& os) {
              ur5e::WriteDeviceSensorLogHeader(os, sensor_names_copy);
            },
            [](std::ostream& os, const ur5e::DeviceSensorLogPod& p) {
              ur5e::WriteDeviceSensorLogRow(os, p);
            });
        if (!handle) {
          RCLCPP_WARN(logger_, "Failed to open device_sensor CSV for instance=%s",
                      entry.instance.c_str());
        } else if (entry.instance == "hand_sensor") {
          hand_sensor_log_handle_ = handle;
        }
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
    const rclcpp_lifecycle::State& prev, const rtc::ControllerState& device_snapshot) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);
  return RTControllerInterface::on_activate(prev, device_snapshot);
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

}  // namespace ur5e_bringup
