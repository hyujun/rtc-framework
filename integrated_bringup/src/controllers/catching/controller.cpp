#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_controller_interface/device_readability.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace integrated_bringup {

namespace {

// Fallback position bounds for a device whose YAML declares no `joint_limits`
// (LoadDeviceLimitsFromConfig fills the slot with these). ±2π is the same
// envelope the other demo bindings pass — stated here rather than hidden in a
// default argument so this controller's clamp is visible at its call site.
inline constexpr double kFallbackPositionLower = -6.2832;
inline constexpr double kFallbackPositionUpper = 6.2832;
inline constexpr double kFallbackMaxVelocity = 2.0;

}  // namespace

DemoCatchingController::DemoCatchingController(std::string_view urdf_path) : urdf_path_(urdf_path) {
  // Nothing to build: this skeleton runs no model. urdf_path_ is stored so the
  // signature matches the registry factory and S5.1 can build one here without
  // changing the registration.
}

// ── Configuration ───────────────────────────────────────────────────────────

void DemoCatchingController::LoadConfig(const YAML::Node& cfg) {
  // Base first: `topics:` (the group list this controller's device indices are
  // defined against) and `command_type`.
  RTControllerInterface::LoadConfig(cfg);

  if (!cfg || !cfg.IsMap()) {
    throw std::runtime_error(
        "DemoCatchingController: config node is absent or not a map — this controller is "
        "registered as REQUIRING_CONFIG and has no defensible defaults (no hand profile)");
  }

  // ── diagnostic.hand_step ────────────────────────────────────────────────
  // Default false: from S7.1 the hand sequencer owns the hand, and an
  // operator-issued step would then race it. S4a's own YAML turns it on.
  hand_step_enabled_ = false;
  if (const YAML::Node diag = cfg["diagnostic"]; diag) {
    if (!diag.IsMap()) {
      throw std::runtime_error("DemoCatchingController: 'diagnostic' must be a map");
    }
    if (const YAML::Node step = diag["hand_step"]; step) {
      hand_step_enabled_ = step.as<bool>();
    }
  }

  // ── catching: (L0 §5.3 / §9 G0-C subset) ────────────────────────────────
  // Parsed here so a malformed tree refuses the configure through the same
  // try/catch every other controller's LoadConfig uses. The VERDICT is applied
  // in on_configure, which is the hook that can refuse.
  const YAML::Node catching = cfg["catching"];
  catching_section_present_ = static_cast<bool>(catching);
  if (catching_section_present_) {
    params_ = rtc::catching::ParseCatchingParams(catching);
  }

  // ── logs: (Phase C) ─────────────────────────────────────────────────────
  parsed_log_entries_.clear();
  if (const YAML::Node logs = cfg["logs"]; logs) {
    if (!logs.IsSequence()) {
      throw std::runtime_error("DemoCatchingController: 'logs' must be a sequence");
    }
    for (const auto& entry : logs) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoCatchingController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      // Closed set: this controller owns exactly one channel type. A typo is a
      // hard fail at parse time rather than a CSV that never appears.
      if (e.msg_type != "rtc_msgs/DeviceStateLog") {
        throw std::runtime_error("DemoCatchingController: unknown msg_type in `logs`: " +
                                 e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

void DemoCatchingController::OnDeviceConfigsSet() {
  ResolveDevices();
}

void DemoCatchingController::ResolveDevices() {
  const auto arm = GetPrimaryDeviceName();
  const auto hand = GetSecondaryDeviceName();

  arm_dof_ = 0;
  hand_dof_ = 0;
  arm_joint_names_.clear();
  hand_joint_names_.clear();
  hand_motor_names_.clear();
  non_sim_groups_.clear();
  device_configs_seen_ = false;

  if (const auto* cfg = GetDeviceNameConfig(arm); cfg != nullptr) {
    arm_dof_ = static_cast<int>(cfg->joint_state_names.size());
    arm_joint_names_ = cfg->joint_state_names;
  }
  if (!hand.empty()) {
    if (const auto* cfg = GetDeviceNameConfig(hand); cfg != nullptr) {
      hand_dof_ = static_cast<int>(cfg->joint_state_names.size());
      hand_joint_names_ = cfg->joint_state_names;
      hand_motor_names_ = cfg->motor_state_names;
    }
  }
  arm_dof_ = std::min(arm_dof_, kDemoCatchingMaxArmDof);
  hand_dof_ = std::min(hand_dof_, kDemoCatchingMaxHandDof);

  // ── Sim-only guard (Q3) ─────────────────────────────────────────────────
  // Fail-closed on BOTH "wrong backend" and "no backend block": the question
  // is not "is this a real arm" but "has this device PROVEN it is the
  // simulator", and silence does not prove it.
  for (const auto& group : topic_config_.groups) {
    const auto* cfg = GetDeviceNameConfig(group.first);
    if (cfg == nullptr) {
      continue;  // no config for this group yet — nothing claimed, nothing to prove
    }
    device_configs_seen_ = true;
    if (!cfg->backend.has_value() || cfg->backend->type != kCatchingRequiredBackendType) {
      non_sim_groups_.push_back(group.first);
    }
  }

  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             kFallbackPositionLower, kFallbackPositionUpper, kFallbackMaxVelocity);
}

// ── Targets ─────────────────────────────────────────────────────────────────

void DemoCatchingController::SetDeviceTarget(int device_idx,
                                             std::span<const double> target) noexcept {
  // Off-RT (the controller's non-RT callback group). Two refusals happen here
  // rather than on the RT side so the operator's goal is answered where it was
  // issued, and so a refused goal never occupies a mailbox slot.
  if (device_idx != kCatchingHandDeviceIdx) {
    // The arm slot has exactly one writer: the activation latch. S5.1 opens
    // this path together with the E-8 approval.
    arm_target_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "target for device %d refused: this controller holds the arm and accepts "
                         "goals on the hand device (%d) only",
                         device_idx, kCatchingHandDeviceIdx);
    return;
  }
  if (!hand_step_enabled_) {
    hand_step_disabled_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "hand target refused: 'diagnostic.hand_step' is false, so the hand holds "
                         "its activation pose");
    return;
  }
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

void DemoCatchingController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                                bool /*is_task*/) noexcept {
  // RT tick, once per surviving mailbox entry. Entries invalidated by a later
  // activation never reach here (the base's generation gate), which is what
  // keeps a goal queued while Inactive off the first tick after re-activation.
  if (device_idx != kCatchingHandDeviceIdx) {
    return;
  }
  const auto n = std::min(values.size(), hand_target_raw_.size());
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_raw_[i] = values[i];
  }
  hand_target_width_ = static_cast<int>(n);
  hand_step_applied_count_.fetch_add(1, std::memory_order_relaxed);
}

void DemoCatchingController::ResetTargetInitialization() noexcept {
  // Lifecycle thread, from the base's on_activate, while no tick runs. Drop
  // both latches so the first tick after activation re-seeds the hold from the
  // pose the robot is actually in, and drop the step target with them: a step
  // the operator issued before this activation is not a goal for this one.
  arm_hold_ = HoldLatch{};
  hand_hold_ = HoldLatch{};
  hand_target_width_ = 0;
}

// ── RT tick ─────────────────────────────────────────────────────────────────

void DemoCatchingController::ReportGateClosure(const ControllerState& state, int device_idx,
                                               int dof, const char* axis) noexcept {
  if (device_idx >= state.num_devices) {
    return;
  }
  const auto& dev = state.devices[static_cast<std::size_t>(device_idx)];
  // RT-3's throttle exception: primitive-only format, no assembly. `axis` is a
  // string literal selected by the caller, which is on the allowed list.
  //
  // Not an else-if pair: IsGateClosedByHoles requires the width test to have
  // PASSED, so the two are mutually exclusive by construction and chaining them
  // would only hide that.
  if (rtc::IsGateClosedByWidth(dev, dof)) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "F5 gate closed: %s device reports num_channels=%d < dof=%d — that axis "
                         "is silenced every tick; fix the group's 'joint_state_names' or the "
                         "backend's reported channel count",
                         axis, dev.num_channels, dof);
  }
  if (rtc::IsGateClosedByHoles(dev, dof)) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "F5 gate closed: %s device slot %d was never written although "
                         "num_channels=%d >= dof=%d — that axis is silenced every tick; the state "
                         "message carries no joint named at that position of the group's "
                         "'joint_command_names' (defaults to 'joint_state_names')",
                         axis, rtc::FirstHoleSlot(dev, dof), dev.num_channels, dof);
  }
}

void DemoCatchingController::WriteDeviceCommand(const ControllerState& state,
                                                ControllerOutput& output, int device_idx,
                                                bool readable) noexcept {
  const auto idx = static_cast<std::size_t>(device_idx);
  const auto& dev = state.devices[idx];
  auto& out = output.devices[idx];
  out.goal_type = rtc::GoalType::kJoint;

  const bool is_hand = (device_idx == kCatchingHandDeviceIdx);
  HoldLatch& latch = is_hand ? hand_hold_ : arm_hold_;

  // Latch on the first READABLE tick, and never again until the next
  // activation. Seeding from an unreadable device would freeze a hold target
  // whose unreported joints are 0 — i.e. "go to the origin" — and the latch
  // would make that permanent.
  if (!latch.IsLatched() && readable) {
    const int width = std::min(dev.num_channels, static_cast<int>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
      latch.commands[i] = dev.positions[i];
    }
    rtc::utils::ClampRange(latch.commands, width,
                           std::span<const double>(device_position_lower_[idx]),
                           std::span<const double>(device_position_upper_[idx]),
                           kFallbackPositionLower, kFallbackPositionUpper);
    latch.width = width;
  }

  if (!latch.IsLatched()) {
    // Nothing honest to command yet: zero-length is "no update" and the drive
    // holds its own setpoint. nc zeros would be a real command to the origin.
    rtc::SilenceDeviceOutput(out);
    rtc::HoldTelemetryAtMeasured(out, dev.num_channels, std::span<const double>(dev.positions));
    return;
  }

  out.num_channels = latch.width;
  for (std::size_t i = 0; i < static_cast<std::size_t>(latch.width); ++i) {
    out.commands[i] = latch.commands[i];
    // The reference lanes carry the same value: this controller has no
    // trajectory, so the reference IS the command.
    out.target_positions[i] = latch.commands[i];
    out.trajectory_positions[i] = latch.commands[i];
    out.goal_positions[i] = latch.commands[i];
  }

  if (!is_hand || hand_target_width_ == 0) {
    return;
  }

  // ── The step ────────────────────────────────────────────────────────────
  // G6-B's controller half: what this writes into `commands` is what the CM
  // copies to the backend verbatim (it neither clamps nor filters), so the
  // clamp against YAML ∩ URDF limits has to happen HERE. `goal_positions`
  // keeps the raw target so a clamp shows up in the CSV as a divergence
  // instead of silently rewriting what was asked for.
  const int width = std::min(hand_target_width_, latch.width);
  for (std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
    out.commands[i] = hand_target_raw_[i];
    out.goal_positions[i] = hand_target_raw_[i];
    out.target_positions[i] = hand_target_raw_[i];
    out.trajectory_positions[i] = hand_target_raw_[i];
  }
  rtc::utils::ClampRange(out.commands, width, std::span<const double>(device_position_lower_[idx]),
                         std::span<const double>(device_position_upper_[idx]),
                         kFallbackPositionLower, kFallbackPositionUpper);
}

ControllerOutput DemoCatchingController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoCatchingController::Compute");
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = CommandType::kPosition;
  output.valid = true;

  arm_readable_ = state.num_devices > kCatchingArmDeviceIdx &&
                  rtc::IsDeviceReadable(state.devices[kCatchingArmDeviceIdx], arm_dof_);
  hand_readable_ = state.num_devices > kCatchingHandDeviceIdx &&
                   rtc::IsDeviceReadable(state.devices[kCatchingHandDeviceIdx], hand_dof_);
  ReportGateClosure(state, kCatchingArmDeviceIdx, arm_dof_, "arm");
  ReportGateClosure(state, kCatchingHandDeviceIdx, hand_dof_, "hand");

  // Drained before the write, not after: WriteDeviceCommand seeds the hold
  // latch and then overlays the step, so a step arriving on the same tick as
  // the first readable state is honoured on that tick instead of a tick later.
  (void)DrainPendingTargets();

  if (state.num_devices > kCatchingArmDeviceIdx) {
    WriteDeviceCommand(state, output, kCatchingArmDeviceIdx, arm_readable_);
  }
  if (state.num_devices > kCatchingHandDeviceIdx) {
    WriteDeviceCommand(state, output, kCatchingHandDeviceIdx, hand_readable_);
  }

  if (arm_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kCatchingArmDeviceIdx, pod);
    arm_state_log_handle_.Push(pod);
  }
  if (hand_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kCatchingHandDeviceIdx, pod);
    hand_state_log_handle_.Push(pod);
  }

  return output;
}

}  // namespace integrated_bringup
