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
//       * sensor_logs  : map<instance, (sensor_names, values_per_group)>
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
#include "integrated_bringup/logging/device_wbc_log_pod.hpp"
#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/logging/task_diag_log_pod.hpp"
#include "integrated_bringup/logging/wbc_diag_log_pod.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup {

// Drain every registered CSV channel and report ring overflow (#234 P-20).
// Call from the controller's 100 ms drain timer instead of DrainAll() — non-RT
// (executor callback), so the string build + RCLCPP_WARN are allowed here.
//
// A dropped sample is a row that silently never reaches the file, which reads
// downstream exactly like a tick the controller chose not to log. `reported`
// is the caller's monotonic bookkeeping (one member per controller): the warn
// fires once per new burst, not once per drain.
inline void DrainControllerLogs(rtc::ControllerLogSet& log_set, const rclcpp::Logger& logger,
                                std::uint64_t& reported) {
  log_set.DrainAll();
  const std::uint64_t total = log_set.TotalDropCount();
  if (total <= reported) {
    return;
  }
  const std::uint64_t added = total - reported;
  reported = total;
  std::string offenders;
  const auto per_channel = log_set.DropCounts();
  const auto channels = log_set.Channels();
  for (std::size_t i = 0; i < channels.size() && i < per_channel.size(); ++i) {
    if (per_channel[i] == 0) {
      continue;
    }
    if (!offenders.empty()) {
      offenders += ", ";
    }
    offenders += std::string(channels[i].first) + "=" + std::to_string(per_channel[i]);
  }
  RCLCPP_WARN(logger,
              "Controller CSV logging dropped %llu new sample(s) (lifetime %llu) — SPSC ring "
              "overflow, rows are missing from the file(s): %s",
              static_cast<unsigned long long>(added), static_cast<unsigned long long>(total),
              offenders.c_str());
}

// ── Per-instance header buffers (caller-supplied) ──────────────────────────
struct LogRegistrationContext {
  rclcpp::Logger logger;
  rtc::ControllerLogSet& log_set;

  // Map: DeviceStateLog instance string → (joint_names, motor_names) used by
  // the header writer. Entry omitted → that instance is not registered, even
  // if it appears in the parsed YAML (defensive against typos).
  std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>> state_logs;

  // DeviceSensorLog header context per instance.
  struct SensorLogInfo {
    std::vector<std::string> sensor_names;
    // Runtime DeviceSensorLayout stride (0 for a force-only hand). Drives the
    // raw/filt column block so a device with no barometer/ToF lane gets no
    // such columns instead of a permanently-zero block. Defaults to the POD
    // capacity for callers that have no layout.
    std::size_t values_per_group{DeviceSensorLogPod::kSensorValuesPerFingertip};
  };

  // Map: DeviceSensorLog instance string → header context.
  std::map<std::string, SensorLogInfo> sensor_logs;

  // ── WBC-specific channels (Path A: POD-only, integrated_bringup-private) ──
  // DeviceWbcLog header context per instance. role drives the arm/hand column
  // blocks; fingertip_names expands the hand-only force columns (empty for arm).
  struct WbcStateLogInfo {
    std::uint8_t role{0};  // 0=arm (SE3 task block), 1=hand (motor+fingertip block)
    std::vector<std::string> joint_names;
    std::vector<std::string> motor_names;
    std::vector<std::string> fingertip_names;
  };

  std::map<std::string, WbcStateLogInfo> wbc_state_logs;

  // Map: WbcDiagLog instance string → num_contact_vars (λ column count =
  // fixed QP contact dim, contact_mgr_config_.max_contact_vars).
  std::map<std::string, std::size_t> wbc_diag_logs;

  // PullEstimatorLog (#167) — a single fixed instance
  // (kPullEstimatorLogInstance), so one bool gates registration. Caller sets it
  // only when its PullEstimatorWiring is enabled; a YAML entry on a pull-less
  // variant is silently skipped like any unregistered instance.
  bool pull_estimator_enabled{false};
  // Configured tip roles in contact-index order. Stamps the bit order into the
  // mask column names so a stored CSV decodes without that run's ROS log
  // (#234 P-14). Empty is tolerated — the columns then carry no suffix.
  std::vector<std::string> pull_estimator_roles;

  // TaskDiagLog (#310) — §6.5 σ_min/λ² diagnostics for the task controller.
  // A single fixed instance (kTaskDiagLogInstance) with a fixed-width header,
  // so a bool gates it rather than a map carrying header context.
  //
  // ⚠ Appended at the END on purpose: the joint/task controllers brace-init this
  // struct POSITIONALLY, so inserting a field mid-struct silently reassigns
  // their arguments (it did — `pull_estimator_roles` landed in this bool and
  // only failed to compile because the types happened to disagree). New fields
  // go last.
  bool task_diag_enabled{false};
};

// ── Returned handles (caller assigns to its own typed members) ─────────────
struct RegisteredLogHandles {
  std::map<std::string, rtc::LogHandle<integrated_bringup::DeviceStateLogPod>> state;
  std::map<std::string, rtc::LogHandle<integrated_bringup::DeviceSensorLogPod>> sensor;
  std::map<std::string, rtc::LogHandle<integrated_bringup::DeviceWbcLogPod>> wbc_state;
  std::map<std::string, rtc::LogHandle<integrated_bringup::WbcDiagLogPod>> wbc_diag;
  // Single fixed instance (kPullEstimatorLogInstance) — no map keyed on a name
  // the caller already knows.
  rtc::LogHandle<integrated_bringup::PullEstimatorLogPod> pull_estimator;
  // Single fixed instance (kTaskDiagLogInstance), same reason.
  rtc::LogHandle<integrated_bringup::TaskDiagLogPod> task_diag;
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
      const auto sensor_names = it->second.sensor_names;
      // Both writers must agree on the stride or the row stops lining up with
      // the header, so the same value is captured into each.
      const auto values_per_group = it->second.values_per_group;
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::DeviceSensorLogPod>(
          entry.instance,
          [sensor_names, values_per_group](std::ostream& os) {
            integrated_bringup::WriteDeviceSensorLogHeader(os, sensor_names, values_per_group);
          },
          [values_per_group](std::ostream& os, const integrated_bringup::DeviceSensorLogPod& pod) {
            integrated_bringup::WriteDeviceSensorLogRow(os, pod, values_per_group);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open device_sensor CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.sensor[entry.instance] = std::move(handle);
    } else if (entry.msg_type == "integrated_bringup/DeviceWbcLog") {
      auto it = ctx.wbc_state_logs.find(entry.instance);
      if (it == ctx.wbc_state_logs.end()) {
        continue;
      }
      const std::uint8_t role = it->second.role;
      const auto joint_names = it->second.joint_names;
      const auto motor_names = it->second.motor_names;
      const auto fingertip_names = it->second.fingertip_names;
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::DeviceWbcLogPod>(
          entry.instance,
          [role, joint_names, motor_names, fingertip_names](std::ostream& os) {
            integrated_bringup::WriteDeviceWbcLogHeader(os, role, joint_names, motor_names,
                                                        fingertip_names);
          },
          [](std::ostream& os, const integrated_bringup::DeviceWbcLogPod& pod) {
            integrated_bringup::WriteDeviceWbcLogRow(os, pod);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open device_wbc CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.wbc_state[entry.instance] = std::move(handle);
    } else if (entry.msg_type == "integrated_bringup/WbcDiagLog") {
      auto it = ctx.wbc_diag_logs.find(entry.instance);
      if (it == ctx.wbc_diag_logs.end()) {
        continue;
      }
      const std::size_t num_contact_vars = it->second;
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::WbcDiagLogPod>(
          entry.instance,
          [num_contact_vars](std::ostream& os) {
            integrated_bringup::WriteWbcDiagLogHeader(os, num_contact_vars);
          },
          [](std::ostream& os, const integrated_bringup::WbcDiagLogPod& pod) {
            integrated_bringup::WriteWbcDiagLogRow(os, pod);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open wbc_diag CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.wbc_diag[entry.instance] = std::move(handle);
    } else if (entry.msg_type == kPullEstimatorLogMsgType) {
      if (!ctx.pull_estimator_enabled || entry.instance != kPullEstimatorLogInstance) {
        // Pull estimator disabled (or unexpected instance) for this controller —
        // silently skip like any unregistered instance.
        continue;
      }
      const auto pull_roles = ctx.pull_estimator_roles;
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::PullEstimatorLogPod>(
          entry.instance,
          [pull_roles](std::ostream& os) {
            integrated_bringup::WritePullEstimatorLogHeader(os, pull_roles);
          },
          [](std::ostream& os, const integrated_bringup::PullEstimatorLogPod& pod) {
            integrated_bringup::WritePullEstimatorLogRow(os, pod);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open pull_estimator CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.pull_estimator = std::move(handle);
    } else if (entry.msg_type == kTaskDiagLogMsgType) {
      if (!ctx.task_diag_enabled || entry.instance != kTaskDiagLogInstance) {
        // Not a task-controller registration (or unexpected instance) —
        // silently skip like any unregistered instance.
        continue;
      }
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::TaskDiagLogPod>(
          entry.instance, [](std::ostream& os) { integrated_bringup::WriteTaskDiagLogHeader(os); },
          [](std::ostream& os, const integrated_bringup::TaskDiagLogPod& pod) {
            integrated_bringup::WriteTaskDiagLogRow(os, pod);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open task_diag CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.task_diag = std::move(handle);
    }
    // Unknown msg_type: LoadConfig() has already validated against the
    // closed set {DeviceStateLog, DeviceSensorLog, DeviceWbcLog, WbcDiagLog,
    // PullEstimatorLog, TaskDiagLog}; reaching here is a YAML parser bug.
    // Silently ignore.
  }

  return result;
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_CONTROLLER_LOG_REGISTRATION_HPP_
