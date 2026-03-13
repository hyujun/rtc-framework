#pragma once

// ── Workspace Analysis ───────────────────────────────────────────────────────
// Package:      ur5e_status_monitor (v5.2.2)
// Convention:   ur5e_ prefix, snake_case, MIT license
// Dependencies: No ROS2 deps in this header — pure C++20
// Reuse:        kNumRobotJoints from ur5e_rt_base/types/types.hpp
// ─────────────────────────────────────────────────────────────────────────────

// ── Project headers ──────────────────────────────────────────────────────────
#include "ur5e_rt_base/types/types.hpp"

// ── C++ stdlib ───────────────────────────────────────────────────────────────
#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace ur5e_status_monitor {

// ── Compile-time constants ───────────────────────────────────────────────────
inline constexpr int kNumJoints = ur5e_rt_controller::kNumRobotJoints;

// ── FailureType ──────────────────────────────────────────────────────────────
/// Enumerates all detectable failure conditions.
/// Stored in std::atomic — must be trivially copyable.
enum class FailureType : uint8_t {
  kNone = 0,
  kEstop,
  kProtectiveStop,
  kSafetyViolation,
  kProgramDisconnected,
  kWatchdogTimeout,
  kControllerInactive,
  kHardwareFault,
  kTrackingError,
  kJointLimitViolation,
};

// ── WarningType ──────────────────────────────────────────────────────────────
/// Warning conditions that do not trigger failure but are logged and reported.
enum class WarningType : uint8_t {
  kNone = 0,
  kJointLimitProximity,
  kTrackingErrorHigh,
  kHighLatency,
};

// ── FailureContext ────────────────────────────────────────────────────────────
/// Snapshot of robot state at the moment a failure is detected.
/// Passed to failure callbacks and written to the failure log file.
struct FailureContext {
  FailureType type{FailureType::kNone};
  int64_t     timestamp_ns{0};               ///< steady_clock nanoseconds since epoch
  int32_t     robot_mode{-1};
  int32_t     safety_mode{0};
  std::string description;
  std::array<double, kNumJoints> q_actual_at_failure{};
  std::array<double, kNumJoints> qd_actual_at_failure{};
  std::array<double, kNumJoints> q_ref_at_failure{};
  std::array<double, kNumJoints> tracking_error_at_failure{};
};

// ── UR robot_mode enum (mirrors ur_robot_driver Int32 values) ────────────────
/// Defined locally to avoid hard dependency on ur_dashboard_msgs.
enum class URRobotMode : int32_t {
  kDisconnected     = -1,
  kConfirmSafety    =  1,
  kBooting          =  2,
  kPowerOff         =  3,
  kPowerOn          =  4,
  kIdle             =  5,
  kBackdrive        =  6,
  kRunning          =  7,
  kUpdatingFirmware =  8,
};

// ── UR safety_mode enum (mirrors ur_robot_driver Int32 values) ───────────────
enum class URSafetyMode : int32_t {
  kNormal              = 1,
  kReduced             = 2,
  kProtectiveStop      = 3,
  kRecovery            = 4,
  kSafeguardStop       = 5,
  kSystemEmergencyStop = 6,
  kRobotEmergencyStop  = 7,
  kViolation           = 8,
  kFault               = 9,
  kValidateJointId     = 10,
  kUndefined           = 11,
};

// ── String conversion helpers (constexpr, no allocation) ─────────────────────

/// Convert FailureType to human-readable string.
[[nodiscard]] constexpr std::string_view FailureTypeToString(FailureType t) noexcept {
  switch (t) {
    case FailureType::kNone:               return "NONE";
    case FailureType::kEstop:              return "ESTOP";
    case FailureType::kProtectiveStop:     return "PROTECTIVE_STOP";
    case FailureType::kSafetyViolation:    return "SAFETY_VIOLATION";
    case FailureType::kProgramDisconnected:return "PROGRAM_DISCONNECTED";
    case FailureType::kWatchdogTimeout:    return "WATCHDOG_TIMEOUT";
    case FailureType::kControllerInactive: return "CONTROLLER_INACTIVE";
    case FailureType::kHardwareFault:      return "HARDWARE_FAULT";
    case FailureType::kTrackingError:      return "TRACKING_ERROR";
    case FailureType::kJointLimitViolation:return "JOINT_LIMIT_VIOLATION";
  }
  return "UNKNOWN";
}

/// Convert WarningType to human-readable string.
[[nodiscard]] constexpr std::string_view WarningTypeToString(WarningType w) noexcept {
  switch (w) {
    case WarningType::kNone:               return "NONE";
    case WarningType::kJointLimitProximity: return "JOINT_LIMIT_PROXIMITY";
    case WarningType::kTrackingErrorHigh:  return "TRACKING_ERROR_HIGH";
    case WarningType::kHighLatency:        return "HIGH_LATENCY";
  }
  return "UNKNOWN";
}

/// Convert URRobotMode to human-readable string.
[[nodiscard]] constexpr std::string_view RobotModeToString(URRobotMode m) noexcept {
  switch (m) {
    case URRobotMode::kDisconnected:     return "DISCONNECTED";
    case URRobotMode::kConfirmSafety:    return "CONFIRM_SAFETY";
    case URRobotMode::kBooting:          return "BOOTING";
    case URRobotMode::kPowerOff:         return "POWER_OFF";
    case URRobotMode::kPowerOn:          return "POWER_ON";
    case URRobotMode::kIdle:             return "IDLE";
    case URRobotMode::kBackdrive:        return "BACKDRIVE";
    case URRobotMode::kRunning:          return "RUNNING";
    case URRobotMode::kUpdatingFirmware: return "UPDATING_FIRMWARE";
  }
  return "UNKNOWN";
}

/// Convert URSafetyMode to human-readable string.
[[nodiscard]] constexpr std::string_view SafetyModeToString(URSafetyMode m) noexcept {
  switch (m) {
    case URSafetyMode::kNormal:              return "NORMAL";
    case URSafetyMode::kReduced:             return "REDUCED";
    case URSafetyMode::kProtectiveStop:      return "PROTECTIVE_STOP";
    case URSafetyMode::kRecovery:            return "RECOVERY";
    case URSafetyMode::kSafeguardStop:       return "SAFEGUARD_STOP";
    case URSafetyMode::kSystemEmergencyStop: return "SYSTEM_EMERGENCY_STOP";
    case URSafetyMode::kRobotEmergencyStop:  return "ROBOT_EMERGENCY_STOP";
    case URSafetyMode::kViolation:           return "VIOLATION";
    case URSafetyMode::kFault:               return "FAULT";
    case URSafetyMode::kValidateJointId:     return "VALIDATE_JOINT_ID";
    case URSafetyMode::kUndefined:           return "UNDEFINED";
  }
  return "UNKNOWN";
}

/// Convert raw int32 to URRobotMode string (convenience for logging).
[[nodiscard]] inline std::string_view RobotModeToString(int32_t val) noexcept {
  return RobotModeToString(static_cast<URRobotMode>(val));
}

/// Convert raw int32 to URSafetyMode string (convenience for logging).
[[nodiscard]] inline std::string_view SafetyModeToString(int32_t val) noexcept {
  return SafetyModeToString(static_cast<URSafetyMode>(val));
}

}  // namespace ur5e_status_monitor
