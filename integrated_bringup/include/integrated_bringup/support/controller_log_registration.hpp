#ifndef UR5E_BRINGUP_SUPPORT_CONTROLLER_LOG_REGISTRATION_HPP_
#define UR5E_BRINGUP_SUPPORT_CONTROLLER_LOG_REGISTRATION_HPP_

// Shared helper for the Phase C `logs:` YAML block. The 3 demo controllers
// (joint, task, wbc) parse the same controller-owned `logs:` schema and
// register typed CSV channels on their own ControllerLogSet. PR2 of the
// demo-controller refactor lifts that ~54-line verbatim block into a single
// templated entry point.
//
// Conservative scope (KUKA + LEAP readiness):
//   - The helper does NOT assume any device name, fingertip count, or
//     instance string mapping. The caller passes:
//       * state_logs   : map<instance, (joint_names, motor_names)>
//       * sensor_logs  : map<instance, sensor_names>
//     and receives back maps keyed by the same instance strings. Each demo
//     controller assigns the entries it cares about into its own typed
//     handle members (e.g. primary_state_log_handle_,
//     secondary_sensor_log_handle_).
//   - ParsedLogEntry is duck-typed (only `.instance` and `.msg_type` are
//     read), so each controller can keep its private nested struct unchanged.
//
// What is NOT lifted (deferred until KUKA + LEAP):
//   - fingertip / sensor-layout parsing
//   - arm + hand model initialisation
//   - virtual-TCP setup
// See agent_docs handoff: `~/.claude/plans/demo-controller-refactor.md`.

#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

// ── Per-instance header buffers (caller-supplied) ──────────────────────────
struct LogRegistrationContext {
  rclcpp::Logger logger;
  rtc::ControllerLogSet& log_set;

  // Map: DeviceStateLog instance string → (joint_names, motor_names) used by
  // the header writer. Entry omitted → that instance is not registered, even
  // if it appears in the parsed YAML (defensive against typos).
  std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>> state_logs;

  // Map: DeviceSensorLog instance string → sensor_names.
  std::map<std::string, std::vector<std::string>> sensor_logs;
};

// ── Returned handles (caller assigns to its own typed members) ─────────────
struct RegisteredLogHandles {
  std::map<std::string, rtc::LogHandle<integrated_bringup::DeviceStateLogPod>> state;
  std::map<std::string, rtc::LogHandle<integrated_bringup::DeviceSensorLogPod>> sensor;
};

// ── Outcome of a single RegisterControllerLogs call ────────────────────────
//
// Distinguishes "missing instance string in YAML" (hard fail — caller should
// return CallbackReturn::FAILURE) from "RegisterLog returned an unbound
// handle" (warning — caller continues but the channel is silently disabled).
enum class LogRegistrationStatus { kSuccess, kMissingInstance };

struct LogRegistrationResult {
  LogRegistrationStatus status{LogRegistrationStatus::kSuccess};
  RegisteredLogHandles handles{};
};

// ── Templated helper (ParsedLogEntry is duck-typed) ────────────────────────
//
// `entries` typically comes from each demo controller's private
// `parsed_log_entries_` vector populated in LoadConfig(). The helper only
// reads `.instance` (string) and `.msg_type` (string).
//
// Returns kMissingInstance the moment any entry has an empty instance —
// caller logs and returns CallbackReturn::FAILURE.
//
// RegisterLog filesystem failures are logged via RCLCPP_WARN here; the
// corresponding map entry is left out so caller's `if (handle)` check on
// the returned map's lookup result still works.
template <typename ParsedLogEntryT>
[[nodiscard]] LogRegistrationResult RegisterControllerLogs(
    const std::vector<ParsedLogEntryT>& entries, const LogRegistrationContext& ctx) {
  LogRegistrationResult result;

  for (const auto& entry : entries) {
    if (entry.instance.empty()) {
      RCLCPP_ERROR(ctx.logger, "logs entry msg_type=%s missing required `instance:` field",
                   entry.msg_type.c_str());
      result.status = LogRegistrationStatus::kMissingInstance;
      return result;
    }

    if (entry.msg_type == "rtc_msgs/DeviceStateLog") {
      auto it = ctx.state_logs.find(entry.instance);
      if (it == ctx.state_logs.end()) {
        // Not a registered DeviceStateLog instance for this controller —
        // silently skip (e.g. controller has no buffer for this name).
        continue;
      }
      // Capture-by-value: header writer may run after the LogRegistrationContext
      // is gone (channel writes header on first Open).
      const auto joint_names = it->second.first;
      const auto motor_names = it->second.second;
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::DeviceStateLogPod>(
          entry.instance,
          [joint_names, motor_names](std::ostream& os) {
            integrated_bringup::WriteDeviceStateLogHeader(os, joint_names, motor_names);
          },
          [](std::ostream& os, const integrated_bringup::DeviceStateLogPod& pod) {
            integrated_bringup::WriteDeviceStateLogRow(os, pod);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open device_state CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.state[entry.instance] = std::move(handle);
    } else if (entry.msg_type == "rtc_msgs/DeviceSensorLog") {
      auto it = ctx.sensor_logs.find(entry.instance);
      if (it == ctx.sensor_logs.end()) {
        continue;
      }
      const auto sensor_names = it->second;
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::DeviceSensorLogPod>(
          entry.instance,
          [sensor_names](std::ostream& os) {
            integrated_bringup::WriteDeviceSensorLogHeader(os, sensor_names);
          },
          [](std::ostream& os, const integrated_bringup::DeviceSensorLogPod& pod) {
            integrated_bringup::WriteDeviceSensorLogRow(os, pod);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open device_sensor CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.sensor[entry.instance] = std::move(handle);
    }
    // Unknown msg_type: LoadConfig() has already validated against the
    // closed set {DeviceStateLog, DeviceSensorLog}; reaching here is a
    // YAML parser bug. Silently ignore — out of scope for this helper.
  }

  return result;
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_CONTROLLER_LOG_REGISTRATION_HPP_
