#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_CATCHING_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_CATCHING_CONTROLLER_H_

// ── Catching controller, S4.0 minimal skeleton (dynamic_catching) ────────────
//
// WHAT THIS IS. The first slice of the catching controller (plan §4.4 S4a):
// the arm holds the pose it was activated in, and the hand accepts UNSHAPED
// step position targets so S4.2 can identify T_close,e2e from the device CSVs
// (L6 §4.2). It exists because no shipped controller can command a step —
// DemoJointController interpolates every goal through a quintic trajectory and
// DemoInference rate-bounds it, so the closing edge would be the controller's
// own, not the hand's.
//
// WHAT THIS IS NOT. There is no catching law here — no CLIK, no trajectory, no
// hand sequencer, no ρ, no state message. Those land with S5 / S7.1. The arm
// never moves under this controller, by construction.
//
// SIM ONLY (2026-09-20 decision, plan §4.4 S4a Q3). on_configure REFUSES unless
// every device this controller claims is bound to the `mujoco_native` backend.
// Commanding a real arm — even to hold it — is a change on the E-STOP path
// (E-8) that is pending approval before S5, and this controller must not be the
// way that path opens by accident. The guard is removed in S5.1 together with
// that approval. For the same reason this class overrides NONE of the E-STOP
// hooks: the base no-ops plus the CM's hold/substitution lane are the whole
// defence here, and adding a hook now would be the E-8 change itself.
//
// THE HAND STEP IS A DIAGNOSTIC MODE. Hand targets are accepted only while the
// YAML says `diagnostic.hand_step: true`. From S7.1 the sequencer owns the
// hand, so the default is false and an operator-issued step is then a refusal
// rather than a race with the sequencer.

#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/catching/catching_params.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace integrated_bringup {

using rtc::CommandType;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::kMaxDeviceChannels;
using rtc::RTControllerInterface;

/// Backend tag every claimed device must carry for this controller to
/// configure (see the sim-only note above). A registry tag, not a device name
/// — robot-agnostic (ARCH-1).
inline constexpr const char* kCatchingRequiredBackendType = "mujoco_native";

/// Controller-local device indices. This controller claims exactly two groups
/// in `topics:` order: the arm it holds and the hand it steps. Indices rather
/// than names because that is what ControllerState / ControllerOutput are
/// indexed by; the NAMES come from YAML, so no robot is hard-coded.
inline constexpr int kCatchingArmDeviceIdx = 0;
inline constexpr int kCatchingHandDeviceIdx = 1;

/// Fixed POD capacity; the runtime widths live in arm_dof_ / hand_dof_,
/// resolved from the device configs (never from a robot constant).
inline constexpr int kDemoCatchingMaxArmDof = 32;
inline constexpr int kDemoCatchingMaxHandDof = static_cast<int>(rtc::catching::kMaxHandDof);

class DemoCatchingController final : public RTControllerInterface {
 public:
  explicit DemoCatchingController(std::string_view urdf_path);

  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override { return "DemoCatchingController"; }

  void LoadConfig(const YAML::Node& cfg) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) noexcept override;

  // ── Observation surface (tools, GUI, tests) ──────────────────────────────
  // The measurement tools read the profile the CONTROLLER loaded, never the
  // shipped YAML directly: a run whose controller read something else must not
  // be analysed against the file on disk. on_configure mirrors these into
  // read-only ROS parameters for the off-process readers.

  [[nodiscard]] const rtc::catching::CatchingParams& GetCatchingParams() const noexcept {
    return params_;
  }

  [[nodiscard]] const rtc::catching::CatchingValidationReport& GetValidationReport()
      const noexcept {
    return report_;
  }

  [[nodiscard]] bool IsHandStepEnabled() const noexcept { return hand_step_enabled_; }

  [[nodiscard]] int GetArmDof() const noexcept { return arm_dof_; }

  [[nodiscard]] int GetHandDof() const noexcept { return hand_dof_; }

  /// Group names whose backend is not `mujoco_native` (or which declare no
  /// `backend:` block at all). Non-empty ⇒ on_configure refuses.
  [[nodiscard]] const std::vector<std::string>& GetNonSimGroups() const noexcept {
    return non_sim_groups_;
  }

  /// Targets refused because they addressed the arm. The arm slot has exactly
  /// one writer here (the activation latch), so a goal sent to it is a mistake
  /// worth counting rather than a command worth obeying.
  [[nodiscard]] std::uint64_t GetArmTargetRejectCount() const noexcept {
    return arm_target_reject_count_.load(std::memory_order_relaxed);
  }

  /// Hand targets refused because `diagnostic.hand_step` is false.
  [[nodiscard]] std::uint64_t GetHandStepDisabledRejectCount() const noexcept {
    return hand_step_disabled_reject_count_.load(std::memory_order_relaxed);
  }

  /// Hand step targets applied on an RT tick since construction.
  [[nodiscard]] std::uint64_t GetHandStepAppliedCount() const noexcept {
    return hand_step_applied_count_.load(std::memory_order_relaxed);
  }

 protected:
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override;
  void OnDeviceConfigsSet() override;
  void ResetTargetInitialization() noexcept override;

 private:
  /// Resolve per-device widths, joint limits, log header names and the backend
  /// check from `device_name_configs_` ∩ `topic_config_.groups`. Idempotent:
  /// CM calls SetDeviceNameConfigs after PreConfigure (so the groups exist),
  /// but a fixture that calls on_configure directly sets the configs first, so
  /// on_configure re-runs this rather than trusting the hook to have seen them.
  void ResolveDevices();

  /// Mirror the loaded profile into read-only ROS parameters. Non-RT.
  void DeclareProfileParameters();

  /// Close every CSV channel and unbind the handles (#238 — a re-configure
  /// must not land in a channel the previous configure owned).
  void ResetLogState() noexcept;

  /// One tick's worth of per-device command writing, shared by both axes.
  /// `latch` supplies the held command; an unreadable device is SILENCED
  /// (zero-length = "no update") rather than commanded to zeros.
  void WriteDeviceCommand(const ControllerState& state, ControllerOutput& output, int device_idx,
                          bool readable) noexcept;

  /// Emit the F5 gate-closure diagnostics for one axis. RT-3's throttle
  /// exception: primitive-only format, no assembly. Expanded per axis so each
  /// gets its own throttle window.
  void ReportGateClosure(const ControllerState& state, int device_idx, int dof,
                         const char* axis) noexcept;

  /// Per-device latch. `width` is the device's channel count at latch time and
  /// 0 while unlatched — one field answers both "is it latched" and "how wide".
  struct HoldLatch {
    std::array<double, kMaxDeviceChannels> commands{};
    int width{0};

    [[nodiscard]] bool IsLatched() const noexcept { return width > 0; }
  };

  /// Duck-typed by RegisterControllerLogs (reads `.instance` / `.msg_type`).
  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  std::string urdf_path_;

  // ── Config (LoadConfig, non-RT) ──────────────────────────────────────────
  rtc::catching::CatchingParams params_{};
  rtc::catching::CatchingValidationReport report_{};
  bool hand_step_enabled_{false};
  bool catching_section_present_{false};
  std::vector<ParsedLogEntry> parsed_log_entries_;

  // ── Devices (ResolveDevices, non-RT) ─────────────────────────────────────
  int arm_dof_{0};
  int hand_dof_{0};
  bool device_configs_seen_{false};
  std::vector<std::string> non_sim_groups_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> hand_joint_names_;
  std::vector<std::string> hand_motor_names_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_lower_{};
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_upper_{};
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_max_velocity_{};

  // ── RT state ─────────────────────────────────────────────────────────────
  // Written and read by the RT tick only, except the latch clear in
  // ResetTargetInitialization (lifecycle thread, while no tick runs).
  HoldLatch arm_hold_{};
  HoldLatch hand_hold_{};
  /// Last accepted step target, UNCLAMPED — the CSV's joint_goal lane. Kept
  /// apart from the clamped command so a clamp is visible in the log instead
  /// of silently rewriting what the operator asked for.
  std::array<double, kMaxDeviceChannels> hand_target_raw_{};
  int hand_target_width_{0};
  bool arm_readable_{false};
  bool hand_readable_{false};

  std::atomic<std::uint64_t> arm_target_reject_count_{0};
  std::atomic<std::uint64_t> hand_step_disabled_reject_count_{0};
  std::atomic<std::uint64_t> hand_step_applied_count_{0};

  // ── Controller-owned topics (`topics:` block) ────────────────────────────
  // The hand step arrives on the group's `joint_goal`, which only exists if
  // CreateOwnedTopics runs — the YAML entry alone creates no endpoint. Holds
  // the target subscription only: this controller publishes nothing (D-20), so
  // there is no lifecycle publisher to gate and no on_activate/on_deactivate
  // pair to add.
  ControllerTopicHandles owned_topics_;

  // ── Logging (Phase C `logs:` block) ──────────────────────────────────────
  rtc::ControllerLogSet log_set_{"demo_catching_controller"};
  rtc::LogHandle<DeviceStateLogPod> arm_state_log_handle_;
  rtc::LogHandle<DeviceStateLogPod> hand_state_log_handle_;
  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
  std::uint64_t log_drops_reported_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("integrated_bringup.demo_catching_controller")};
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
};

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_CONTROLLERS_DEMO_CATCHING_CONTROLLER_H_
