#include "integrated_bringup/controllers/hand_joint_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace {
/// CSV drain timer period (10 Hz), the same cadence as the other bindings.
constexpr int kLogDrainPeriodMs = 100;
}  // namespace

rtc::RTControllerInterface::CallbackReturn HandJointController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = rtc::RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    // The model could not be tied to the device (OnDeviceConfigsSet latched
    // it): a loop whose independent joint this group does not drive, a plain
    // tree the device does not cover, a frame that is not on the model. Every
    // one of those would be finite and wrong on the tick, so refuse here.
    if (!config_error_.empty()) {
      RCLCPP_ERROR(logger_, "[hand_joint] configure refused: %s", config_error_.c_str());
      return CallbackReturn::FAILURE;
    }

    // One group: the hand. A longer `topics:` list throws here, before any
    // subscription exists, and the catch below turns it into FAILURE.
    CreateOwnedTopics(*this, owned_topics_, /*max_groups=*/1);

    // kRobotTransforms: `<root>` → `<tip>_actual` per fingertip, read from
    // group_commands[0].task_link_poses — the poses FillOutput writes
    // root-relative. Parent = the tree root, which is the frame they are
    // expressed in.
    if (owned_topics_.tf_pub) {
      AppendHandTipSlots(owned_topics_, root_link_, tip_links_, /*group_idx=*/0,
                         /*max_tips=*/num_tips_);
    }

    const auto group = GetPrimaryDeviceName();
    const std::string state_key = group + "_state";
    LogRegistrationContext ctx{
        .logger = logger_,
        .log_set = log_set_,
        .state_logs = {{state_key, {joint_names_, std::vector<std::string>{}}}},
        .joint_ff_enabled = true,
        .joint_ff_joint_names = joint_names_,
    };
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      ResetLogState();
      return CallbackReturn::FAILURE;
    }
    if (auto it = reg.handles.state.find(state_key); it != reg.handles.state.end()) {
      state_log_handle_ = it->second;
    }
    ff_log_handle_ = reg.handles.joint_ff;

    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(kLogDrainPeriodMs),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    DeclareGainParameters();
    param_callback_handle_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
          return OnGainParametersSet(params);
        });
  } catch (const std::exception& e) {
    ResetLogState();
    RCLCPP_ERROR(logger_, "HandJointController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    ResetLogState();
    RCLCPP_ERROR(logger_, "HandJointController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger_,
              "[hand_joint] configured: group '%s' (%d joints), model %s, %zu fingertip frame(s) "
              "under '%s', gravity_ff %s (gain %.3f, max_abs %.4f N·m)",
              GetPrimaryDeviceName().c_str(), dof_,
              closed_handle_ ? "closed-chain (reduced g_a)" : "plain tree", num_tips_,
              root_link_.c_str(), cfg_.ff.enabled ? "on" : "off", cfg_.ff.gravity_gain,
              cfg_.ff.max_abs);
  return CallbackReturn::SUCCESS;
}

void HandJointController::ResetLogState() noexcept {
  // Close open channels and unbind the handles so a cleanup→configure cycle
  // re-registers cleanly (#238); drop the drop high-water mark in lockstep.
  log_set_.Reset();
  log_drops_reported_ = 0;
  state_log_handle_ = {};
  ff_log_handle_ = {};
}

rtc::RTControllerInterface::CallbackReturn HandJointController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);
  // The base bumps the activation generation (invalidating targets queued
  // while Inactive) and calls ResetTargetInitialization(), which re-seeds the
  // reference from the measurement on the first tick.
  return rtc::RTControllerInterface::on_activate(prev);
}

rtc::RTControllerInterface::CallbackReturn HandJointController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();
  return CallbackReturn::SUCCESS;
}

rtc::RTControllerInterface::CallbackReturn HandJointController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  ResetOwnedTopics(owned_topics_);
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetLogState();
  param_callback_handle_.reset();
  return rtc::RTControllerInterface::on_cleanup(prev);
}

}  // namespace integrated_bringup
