// ── DemoDualArmController: lifecycle (non-RT) ───────────────────────────────

#include "integrated_bringup/controllers/demo_dualarm_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"

#include <rclcpp/logging.hpp>

#include <chrono>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

using CallbackReturn = rtc::RTControllerInterface::CallbackReturn;

CallbackReturn DemoDualArmController::on_configure(const rclcpp_lifecycle::State& prev,
                                                   rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                                   const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    // The runtime (model, frames, solve) is built where the device configs
    // arrive; that hook cannot refuse a configure, so its verdict is read
    // here. A task frame the model does not carry, a joint a posture group
    // names that the body group does not have, a limit margin that leaves a
    // joint no range — each is a wiring error of the shipped config, and a
    // controller that came up without its solve would look configured and
    // hold forever.
    if (!config_error_.empty()) {
      RCLCPP_ERROR(logger_, "DemoDualArmController on_configure refused: %s",
                   config_error_.c_str());
      return CallbackReturn::FAILURE;
    }

    CreateOwnedTopics(*this, owned_topics_);
    {
      std::vector<TaskGoalSubscriptionRequest> requests;
      for (std::size_t k = 0; k < num_tasks_; ++k) {
        requests.push_back({task_names_[k], &ingress_[k]});
      }
      CreateTaskGoalSubscriptions(*this, owned_topics_, requests, target_frame_names_);
    }

    // ── Transform slots ─────────────────────────────────────────────────────
    // Every pose is the MEASURED one, in the body group's root link — the
    // parent the joint controller publishes under on the same robot. The
    // source fields are the output's global pose slots, so the slot order
    // here is the order the tick fills them in: fingertips first, then one
    // per task frame.
    if (owned_topics_.tf_pub && !root_link_name_.empty()) {
      if (has_tip_) {
        AppendArmTipSlot(owned_topics_, root_link_name_, tip_link_name_,
                         /*group_idx=*/kDualArmBodyDeviceIdx);
      }
      std::vector<std::string> links = fingertip_names_;
      for (std::size_t k = 0; k < num_tasks_; ++k) {
        links.push_back(cfg_.tasks[k].frame);
      }
      AppendHandTipSlots(owned_topics_, root_link_name_, links,
                         /*group_idx=*/kDualArmBodyDeviceIdx, /*max_tips=*/links.size());
    }

    // ── CSV logs ────────────────────────────────────────────────────────────
    const std::string body_key = GetPrimaryDeviceName() + "_state";
    const std::string secondary = GetSecondaryDeviceName();
    const std::string hand_key = secondary.empty() ? std::string{} : secondary + "_state";
    LogRegistrationContext ctx{
        .logger = logger_,
        .log_set = log_set_,
        .state_logs =
            {
                {body_key, {body_joint_names_, std::vector<std::string>{}}},
                {hand_key, {hand_joint_names_, hand_motor_names_}},
            },
        .dualarm_diag_enabled = true,
        .dualarm_diag_task_names = task_names_,
        .dualarm_diag_joint_names = body_joint_names_,
    };
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      TearDownConfigured();
      return CallbackReturn::FAILURE;
    }
    if (auto it = reg.handles.state.find(body_key); it != reg.handles.state.end()) {
      body_state_log_handle_ = std::move(it->second);
    }
    if (!hand_key.empty()) {
      if (auto it = reg.handles.state.find(hand_key); it != reg.handles.state.end()) {
        hand_state_log_handle_ = std::move(it->second);
      }
    }
    diag_log_handle_ = std::move(reg.handles.dualarm_diag);

    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    DeclareParameters();
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return OnParametersSet(params); });
  } catch (const std::exception& e) {
    TearDownConfigured();
    RCLCPP_ERROR(logger_, "DemoDualArmController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    TearDownConfigured();
    RCLCPP_ERROR(logger_, "DemoDualArmController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

void DemoDualArmController::TearDownConfigured() noexcept {
  // What on_configure created, in the order that is safe to undo it: the
  // drain timer first (it runs on the executor and reads the log channels),
  // then the subscriptions and publishers, then the channels themselves. A
  // configure that fails half-way goes through here too — it may already have
  // created the timer, and a retry must not find the previous attempt's
  // subscriptions still attached.
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  param_callback_handle_.reset();
  ResetOwnedTopics(owned_topics_);
  ResetLogState();
}

void DemoDualArmController::ResetLogState() noexcept {
  // Close the channels and unbind every handle, so a cleanup → configure cycle
  // registers afresh instead of meeting the duplicate guard with stale handles.
  log_set_.Reset();
  log_drops_reported_ = 0;
  body_state_log_handle_ = {};
  hand_state_log_handle_ = {};
  diag_log_handle_ = {};
}

CallbackReturn DemoDualArmController::on_activate(const rclcpp_lifecycle::State& prev) noexcept {
  // A fault reset no tick consumed dies here: carried across, the first tick
  // of this activation would release a latch nobody re-authorised in it.
  // Stored before the generation moves, so the tick that sees the activation
  // also sees this.
  fault_reset_floor_.store(fault_reset_epoch_.load(std::memory_order_acquire),
                           std::memory_order_release);
  // The base first: it bumps the activation generation, which is what makes
  // the first tick re-seed its command from the measurement and drop every
  // goal that was sent while the controller was Inactive.
  const auto ret = RTControllerInterface::on_activate(prev);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  ActivateOwnedTopics(prev, owned_topics_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DemoDualArmController::on_deactivate(const rclcpp_lifecycle::State& prev) noexcept {
  DeactivateOwnedTopics(prev, owned_topics_);
  // Flush in-flight samples: a controller switch leaves residue in the rings
  // that would otherwise replay on the next activation.
  log_set_.DrainAll();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DemoDualArmController::on_cleanup(const rclcpp_lifecycle::State& prev) noexcept {
  TearDownConfigured();
  return RTControllerInterface::on_cleanup(prev);
}

void DemoDualArmController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
}

}  // namespace integrated_bringup
