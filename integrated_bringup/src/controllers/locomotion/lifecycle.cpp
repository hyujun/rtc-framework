// locomotion_controller: lifecycle hooks. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace {
constexpr int kLogDrainPeriodMs = 100;

const char* CauseName(LocomotionDiagLogPod::FaultCause cause) noexcept {
  switch (cause) {
    case LocomotionDiagLogPod::FaultCause::kNone:
      return "none";
    case LocomotionDiagLogPod::FaultCause::kSolveHealth:
      return "solve_health";
    case LocomotionDiagLogPod::FaultCause::kSource:
      return "source";
    case LocomotionDiagLogPod::FaultCause::kNonFinite:
      return "non_finite";
    case LocomotionDiagLogPod::FaultCause::kModelEpoch:
      return "model_epoch";
  }
  return "unknown";
}
}  // namespace

rtc::RTControllerInterface::CallbackReturn LocomotionController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = rtc::RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    if (!config_error_.empty()) {
      RCLCPP_ERROR(logger_, "[locomotion] configure refused: %s", config_error_.c_str());
      return CallbackReturn::FAILURE;
    }
    CreateOwnedTopics(*this, owned_topics_, 1 + cfg_.model.hand_groups.size());

    std::vector<std::string> groups{cfg_.model.body_group};
    groups.insert(groups.end(), cfg_.model.hand_groups.begin(), cfg_.model.hand_groups.end());
    LogRegistrationContext ctx{
        .logger = logger_,
        .log_set = log_set_,
        .joint_ff_enabled = true,
        .joint_ff_joint_names = body_joint_names_,
        .locomotion_diag_enabled = true,
        .locomotion_diag_contact_names = contact_names_,
        .locomotion_diag_joint_names = body_joint_names_,
    };
    ctx.state_logs[groups[0] + "_state"] = {body_joint_names_, {}};
    for (std::size_t h = 0; h < hand_joint_names_.size(); ++h) {
      ctx.state_logs[groups[h + 1] + "_state"] = {hand_joint_names_[h], {}};
    }
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      ResetLogState();
      return CallbackReturn::FAILURE;
    }
    state_log_handles_.clear();
    for (const auto& g : groups) {
      auto it = reg.handles.state.find(g + "_state");
      state_log_handles_.push_back(it != reg.handles.state.end()
                                       ? std::move(it->second)
                                       : rtc::LogHandle<DeviceStateLogPod>{});
    }
    joint_log_handle_ = std::move(reg.handles.joint_ff);
    diag_log_handle_ = std::move(reg.handles.locomotion_diag);

    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(kLogDrainPeriodMs),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }
  } catch (const std::exception& e) {
    ResetLogState();
    RCLCPP_ERROR(logger_, "LocomotionController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    ResetLogState();
    RCLCPP_ERROR(logger_, "LocomotionController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger_,
              "[locomotion] configured: body '%s' (%zu joints), %zu hand group(s), %zu contact(s); "
              "full model nv %d (kinematics), reduced model nv %d (dynamics), mass %.3f kg",
              cfg_.model.body_group.c_str(), nb_, cfg_.model.hand_groups.size(),
              contact_names_.size(), full_model_->nv, reduced_model_->nv, total_mass_);
  return CallbackReturn::SUCCESS;
}

void LocomotionController::ResetLogState() noexcept {
  log_set_.Reset();
  log_drops_reported_ = 0;
  state_log_handles_.clear();
  joint_log_handle_ = {};
  diag_log_handle_ = {};
}

rtc::RTControllerInterface::CallbackReturn LocomotionController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // The first tick of this activation drops a latch left from the last one
  // (header); say so here, off the RT thread.
  if (fault_latched_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(logger_,
                "[locomotion] a latched fault (cause %s) is dropped by this activation; the "
                "reference re-latches from the measured state",
                CauseName(fault_cause_));
  }
  ActivateOwnedTopics(prev, owned_topics_);
  return rtc::RTControllerInterface::on_activate(prev);
}

rtc::RTControllerInterface::CallbackReturn LocomotionController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();
  return CallbackReturn::SUCCESS;
}

rtc::RTControllerInterface::CallbackReturn LocomotionController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  ResetOwnedTopics(owned_topics_);
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetLogState();
  return rtc::RTControllerInterface::on_cleanup(prev);
}

}  // namespace integrated_bringup
