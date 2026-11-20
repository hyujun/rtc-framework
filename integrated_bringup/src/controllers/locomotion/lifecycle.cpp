// locomotion_controller: lifecycle hooks. See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/layout_profile.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/threading/thread_utils.hpp"

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace {
constexpr int kLogDrainPeriodMs = 100;

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
        .locomotion_truth_enabled = true,
        .locomotion_truth_contact_names = contact_names_,
    };
    ctx.state_logs[groups[0] + "_state"] = {body_joint_names_, {}};
    for (std::size_t h = 0; h < hand_joint_names_.size(); ++h) {
      ctx.state_logs[groups[h + 1] + "_state"] = {hand_joint_names_[h], {}};
    }
    ctx.session_logging_enabled = SessionLoggingEnabled();
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
    truth_log_handle_ = std::move(reg.handles.locomotion_truth);

    layout_profile_drops_mpc_ = false;
    if (mpc_enabled_ && node) {
      const std::string profile = ReadLayoutProfile(*node);
      layout_profile_drops_mpc_ = LayoutProfileDropsMpc(profile);
      mpc_worker_slots_ = rtc::SelectMpcWorkerSlots(profile);
    }

    // The status topic, with or without an MPC.
    SetupLocomotionStatusPublisher(*this, owned_topics_,
                                   cfg_.model.body_group + "/locomotion_status",
                                   "locomotion_controller", cfg_.model.body_group);

    // The velocity command: a subscription only with a consumer (the plan).
    velocity_sub_.reset();
    joy_sub_.reset();
    const std::string& topic = cfg_.velocity_command_topic;
    if (topic.empty()) {
      // no command path
    } else if (!mpc_enabled_ || !node) {
      RCLCPP_INFO(logger_,
                  "[locomotion] velocity_command.topic '%s' is not subscribed: no MPC in this "
                  "configuration (mpc.enabled false or `mpc:` absent)",
                  topic.c_str());
    } else {
      // ARCH-6: depth 1 — a latest-wins lane; reliability as the node's default.
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      if (cfg_.velocity_command_joy) {
        // Full deflection = the filter's own bounds (u_max = [v_max…, hdot_max]),
        // fixed here: the callback only multiplies.
        const auto& u_max = command_filter_.config().u_max;
        joy_scale_ = {u_max[0], u_max[1], u_max[2], u_max[3]};
        joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
            topic, qos, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) { OnJoyCommand(*msg); });
      } else {
        velocity_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
            topic, qos,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { OnVelocityCommand(*msg); });
      }
      RCLCPP_INFO(logger_, "[locomotion] velocity command: %s (%s, %.0f Hz filter)", topic.c_str(),
                  cfg_.velocity_command_joy ? "Joy" : "Twist",
                  command_filter_.config().sample_rate_hz);
    }
    // After the subscription: the rest sample is tagged joy only with a Joy
    // subscription (status and diag log agree on "no input" otherwise).
    raw_command_.Store(RestCommandSample());
    command_out_.Store(
        LocomotionCommandOut{rtc::mpc::FilteredCommand{}, RestCommandSample().source});

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
  truth_log_handle_ = {};
}

rtc::RTControllerInterface::CallbackReturn LocomotionController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // First: a refused activation leaves nothing behind (the CM keeps the
  // previous controller and never deactivates this one).
  if (layout_profile_drops_mpc_) {  // only set with mpc.enabled
    RCLCPP_ERROR(logger_,
                 "[locomotion] activation refused: this launch ran with layout profile '%s' "
                 "(enable_mpc:=false), which returned the MPC cores to the system cpuset, but "
                 "mpc.enabled is true. Relaunch with enable_mpc:=true, or set mpc.enabled: false.",
                 std::string(kMpcOffLayoutProfile).c_str());
    return CallbackReturn::FAILURE;
  }
  if (mpc_enabled_) {
    if (!SpawnMpcThread(nullptr)) {
      return CallbackReturn::FAILURE;
    }
    // The 1 Hz timer reads the thread: created once it exists. The controller
    // node is spun by the CM's single-threaded nrt_callback executor, which
    // also serves the switch / lifecycle calls — the timer never runs beside
    // them.
    if (!mpc_timing_timer_ && node_) {
      mpc_timing_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      mpc_timing_timer_ = node_->create_wall_timer(
          std::chrono::seconds(1), [this]() { LogMpcTick(); }, mpc_timing_cb_group_);
    }
    // Not with session logging off (the manager's enable_logging); Log() on a
    // closed logger is a no-op.
    if (SessionLoggingEnabled() && !mpc_timing_logger_.IsOpen()) {
      if (mpc_timing_logger_.Open()) {
        RCLCPP_INFO(logger_, "[locomotion] MPC tick-timing CSV: %s",
                    mpc_timing_logger_.Path().c_str());
      } else {
        RCLCPP_WARN(logger_, "[locomotion] MPC tick-timing CSV disabled (Open failed)");
      }
    }
    // A plan from before this activation is not sampled (the manager's
    // re-enable fence); the first tick restarts the state stream.
    mpc_manager_.SetEnabled(true);
    StartCommandTimer();
  }
  // The first tick of this activation drops a latch left from the last one
  // (header); say so here, off the RT thread.
  if (fault_latched_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(logger_,
                "[locomotion] a latched fault (cause %s) is dropped by this activation; the "
                "reference re-latches from the measured state",
                LocomotionFaultCauseName(
                    static_cast<std::uint8_t>(fault_cause_.load(std::memory_order_relaxed))));
  }
  ActivateOwnedTopics(prev, owned_topics_);
  const auto rc = rtc::RTControllerInterface::on_activate(prev);
  if (mpc_thread_) {
    if (rc == CallbackReturn::SUCCESS) {
      mpc_thread_->Resume();
    } else {
      PauseMpc();  // nothing will deactivate a refused activation
    }
  }
  return rc;
}

rtc::RTControllerInterface::CallbackReturn LocomotionController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  PauseMpc();
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();
  return CallbackReturn::SUCCESS;
}

rtc::RTControllerInterface::CallbackReturn LocomotionController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  velocity_sub_.reset();
  joy_sub_.reset();
  StopMpcThread();
  ResetOwnedTopics(owned_topics_);
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetLogState();
  return rtc::RTControllerInterface::on_cleanup(prev);
}

}  // namespace integrated_bringup
