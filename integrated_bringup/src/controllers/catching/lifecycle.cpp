#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <chrono>
#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace {

/// Human-readable tag for one validation entry. Static strings only — this
/// runs in a lifecycle callback, but the report itself is allocation-free by
/// contract and there is no reason to make its rendering the exception.
[[nodiscard]] const char* ReasonText(rtc::catching::CatchingValidationReason reason) noexcept {
  using R = rtc::catching::CatchingValidationReason;
  switch (reason) {
    case R::kActiveConfigTbd:
      return "still TBD in the active configuration";
    case R::kControlRateOutOfRange:
      return "control_rate out of range";
    case R::kRangeViolation:
      return "value outside its documented range";
    case R::kZetaNotCriticallyDamped:
      return "reference.zeta must be exactly 1 in v1";
    case R::kEtaVOutOfRange:
      return "eta_v must be in (0, 1]";
    case R::kDecelExceedsAMax:
      return "a_dec exceeds reference.a_max";
    case R::kUnstableDiscretization:
      return "omega*h is at or above the discrete stability limit";
    case R::kDiscretizationAccuracy:
      return "omega*h exceeds the accuracy recommendation";
    case R::kHandCagingGapTooSmall:
      return "caging joint moves by less than rho_eps between q_pre and q_close";
    case R::kProvisionalOnRealArm:
      return "provisional value blocks a real-arm configuration";
    case R::kProvisionalWarning:
      return "provisional value (sim only)";
  }
  return "unknown";
}

/// Whether a validation entry names a key this S4.0 skeleton actually
/// consumes.
///
/// G0-C (L0 §5.3) blocks ARMING on a TBD active key, and S5's controller gates
/// on the whole report for exactly that reason. This one never arms: it holds
/// the arm still and steps the hand so S4.2 can time the closure. Refusing to
/// run that measurement because `reference.v_max` (TBD-ARM-02) or
/// `supervisor.decel.a_dec` is still open would invert the dependency — those
/// values are decided BY the measurement chain this controller feeds (plan
/// §4.2 DAG: S4.2 → S4.4 → S3.5b). So the gate is the consumed subset and
/// everything else is reported as a warning naming the step that owns it.
///
/// `robot.hand.T_close_e2e` is the sharpest case of that inversion and is
/// excluded by name: it is the very number S4.2 produces with this controller,
/// so gating on it would make the measurement its own precondition. S7.1 (the
/// sequencer, which derives T_close_timeout from it) is the first consumer.
[[nodiscard]] bool ConsumedByCatchingSkeleton(const char* key) noexcept {
  const std::string_view k{key};
  if (k == "control_rate") {
    return true;
  }
  return k.starts_with("robot.hand.") && k != "robot.hand.T_close_e2e";
}

/// Read-only descriptor for the mirrored profile parameters. They exist so an
/// off-process reader (the S4.2 runner, the GUI panel) uses what the CONTROLLER
/// loaded instead of re-reading the YAML — a run whose controller read
/// something else must not be analysed against the file on disk.
[[nodiscard]] rcl_interfaces::msg::ParameterDescriptor ReadOnlyDescriptor(std::string description) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = std::move(description);
  d.read_only = true;
  return d;
}

}  // namespace

void DemoCatchingController::DeclareProfileParameters() {
  if (!node_) {
    return;
  }
  const auto& hand = params_.hand;
  const auto dof = static_cast<std::size_t>(hand.dof);

  std::vector<double> q_open(hand.q_open.begin(), hand.q_open.begin() + dof);
  std::vector<double> q_pre(hand.q_pre.begin(), hand.q_pre.begin() + dof);
  std::vector<double> q_close(hand.q_close.begin(), hand.q_close.begin() + dof);
  std::vector<bool> caging(hand.caging_mask.begin(), hand.caging_mask.begin() + dof);

  node_->declare_parameter("hand.q_open", q_open, ReadOnlyDescriptor("L6 §5.1 open pose [rad]"));
  node_->declare_parameter("hand.q_pre", q_pre, ReadOnlyDescriptor("L6 §5.1 preshape pose [rad]"));
  node_->declare_parameter("hand.q_close", q_close,
                           ReadOnlyDescriptor("L6 §5.1 closed pose [rad]"));
  node_->declare_parameter("hand.caging_mask", caging,
                           ReadOnlyDescriptor("L6 §4.2 caging joint set C"));
  node_->declare_parameter("hand.eta_close", hand.eta_close.value,
                           ReadOnlyDescriptor("L6 §4.2 closure threshold eta"));
  node_->declare_parameter("hand.rho_eps", hand.rho_eps,
                           ReadOnlyDescriptor("L6 §4.2 caging gap floor [rad]"));
  node_->declare_parameter("hand.T_close_e2e", hand.T_close_e2e.value,
                           ReadOnlyDescriptor("L6 §4.2 end-to-end closure time [s]"));
  node_->declare_parameter("diagnostic.hand_step", hand_step_enabled_,
                           ReadOnlyDescriptor("accept unshaped hand step targets (S4a)"));
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    // CM calls SetDeviceNameConfigs between PreConfigure and here, so the hook
    // has already run with the groups resolved. A fixture that calls
    // on_configure directly sets the configs FIRST, so the hook then saw no
    // groups — re-run rather than trust the ordering (idempotent).
    ResolveDevices();

    if (topic_config_.groups.size() < 2) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: this controller claims an arm group and a hand group, "
                   "but `topics:` declares %zu",
                   topic_config_.groups.size());
      return CallbackReturn::FAILURE;
    }

    // ── Sim-only guard (plan §4.4 S4a Q3) ────────────────────────────────
    // Refuse rather than degrade. This controller holds the arm by commanding
    // it every tick; doing that on real hardware is a change on the E-STOP
    // path (E-8), which is pending approval before S5. S5.1 removes this
    // guard together with that approval.
    if (!device_configs_seen_) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: no device config resolved for any declared group, so "
                   "no device has proven it is the simulator (E-8: this controller is sim-only "
                   "until the E-STOP path is approved)");
      return CallbackReturn::FAILURE;
    }
    if (!non_sim_groups_.empty()) {
      for (const auto& group : non_sim_groups_) {
        const auto* cfg = GetDeviceNameConfig(group);
        RCLCPP_ERROR(logger_,
                     "refusing to configure: device group '%s' is bound to backend '%s', not '%s'. "
                     "This controller is SIM-ONLY until E-8 (the E-STOP path) is approved in S5.1 "
                     "— it holds the arm by commanding it every tick.",
                     group.c_str(),
                     (cfg != nullptr && cfg->backend.has_value()) ? cfg->backend->type.c_str()
                                                                  : "<none declared>",
                     kCatchingRequiredBackendType);
      }
      return CallbackReturn::FAILURE;
    }

    // ── Hand profile (G0-C, L0 §5.3) ─────────────────────────────────────
    if (!catching_section_present_) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: no `catching:` section. The hand profile has no "
                   "defensible default — see L6 §6 and plan §4.4 S4.1.");
      return CallbackReturn::FAILURE;
    }
    report_ = rtc::catching::ValidateCatchingParams(params_, 1.0 / GetDefaultDt(),
                                                    /*real_arm_config=*/false);
    for (std::size_t i = 0; i < report_.warning_count; ++i) {
      const auto& w = report_.warnings[i];
      RCLCPP_WARN(logger_, "catching config warning: %s — %s", w.key, ReasonText(w.reason));
    }
    bool consumed_failure = false;
    for (std::size_t i = 0; i < report_.failure_count; ++i) {
      const auto& f = report_.failures[i];
      if (!ConsumedByCatchingSkeleton(f.key)) {
        RCLCPP_WARN(logger_,
                    "catching config: %s — %s. Not consumed by this S4.0 skeleton (it never "
                    "arms); the step that owns the value decides it.",
                    f.key, ReasonText(f.reason));
        continue;
      }
      consumed_failure = true;
      if (f.index >= 0) {
        RCLCPP_ERROR(logger_, "catching config error: %s[%d] — %s", f.key, f.index,
                     ReasonText(f.reason));
      } else {
        RCLCPP_ERROR(logger_, "catching config error: %s — %s", f.key, ReasonText(f.reason));
      }
    }
    if (consumed_failure) {
      return CallbackReturn::FAILURE;
    }
    if (params_.hand.dof != hand_dof_) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: hand profile declares %d joints but the hand device "
                   "reports %d (L6 §5.1: n must equal the hand device's channel count)",
                   params_.hand.dof, hand_dof_);
      return CallbackReturn::FAILURE;
    }

    // ── Controller-owned CSVs ────────────────────────────────────────────
    // Instance keys are derived from the device names so a YAML `instance:`
    // tracks the active config_variant, exactly as the other bindings do.
    const auto arm_key = GetPrimaryDeviceName() + "_state";
    const auto hand_key = GetSecondaryDeviceName() + "_state";
    LogRegistrationContext ctx{
        .logger = logger_,
        .log_set = log_set_,
        .state_logs =
            {
                {arm_key, {arm_joint_names_, std::vector<std::string>{}}},
                {hand_key, {hand_joint_names_, hand_motor_names_}},
            },
    };
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      ResetLogState();
      return CallbackReturn::FAILURE;
    }
    if (auto it = reg.handles.state.find(arm_key); it != reg.handles.state.end()) {
      arm_state_log_handle_ = it->second;
    }
    if (auto it = reg.handles.state.find(hand_key); it != reg.handles.state.end()) {
      hand_state_log_handle_ = it->second;
    }

    // Drain timer on a non-RT callback group (10 Hz), same as every other
    // binding: the RT tick only pushes into the SPSC ring.
    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    DeclareProfileParameters();

    RCLCPP_INFO(logger_,
                "configured: arm=%s(%d) hand=%s(%d), hand_step=%s, backend=%s (sim-only until E-8)",
                GetPrimaryDeviceName().c_str(), arm_dof_, GetSecondaryDeviceName().c_str(),
                hand_dof_, hand_step_enabled_ ? "enabled" : "disabled",
                kCatchingRequiredBackendType);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

void DemoCatchingController::ResetLogState() noexcept {
  log_set_.Reset();
  // Reset() destroys every channel's drop counter, so the high-water mark has
  // to go with it or the next session's first drop burst is swallowed (#238).
  log_drops_reported_ = 0;
  arm_state_log_handle_ = {};
  hand_state_log_handle_ = {};
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  log_set_.DrainAll();
  // Tear the timer down BEFORE Reset() so no drain callback runs against
  // channels that are being destroyed.
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetLogState();
  return RTControllerInterface::on_cleanup(prev);
}

}  // namespace integrated_bringup
