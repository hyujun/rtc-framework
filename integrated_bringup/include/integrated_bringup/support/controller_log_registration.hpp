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
#include "integrated_bringup/logging/grasp_diag_log_pod.hpp"
#include "integrated_bringup/logging/momentum_observer_log_pod.hpp"
#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/logging/task_diag_log_pod.hpp"
#include "integrated_bringup/logging/wbc_diag_log_pod.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"

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
  std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>> state_logs{};

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
  std::map<std::string, SensorLogInfo> sensor_logs{};

  // ── WBC-specific channels (Path A: POD-only, integrated_bringup-private) ──
  // DeviceWbcLog header context per instance. role drives the arm/hand column
  // blocks; fingertip_names expands the hand-only force columns (empty for arm).
  struct WbcStateLogInfo {
    std::uint8_t role{0};  // 0=arm (SE3 task block), 1=hand (motor+fingertip block)
    std::vector<std::string> joint_names;
    std::vector<std::string> motor_names;
    std::vector<std::string> fingertip_names;
  };

  std::map<std::string, WbcStateLogInfo> wbc_state_logs{};

  // Map: WbcDiagLog instance string → num_contact_vars (λ column count =
  // fixed QP contact dim, contact_mgr_config_.max_contact_vars).
  std::map<std::string, std::size_t> wbc_diag_logs{};

  // PullEstimatorLog (#167) — a single fixed instance
  // (kPullEstimatorLogInstance), so one bool gates registration. Caller sets it
  // only when its PullEstimatorWiring is enabled; a YAML entry on a pull-less
  // variant is silently skipped like any unregistered instance.
  bool pull_estimator_enabled{false};
  // Configured tip roles in contact-index order. Stamps the bit order into the
  // mask column names so a stored CSV decodes without that run's ROS log
  // (#234 P-14). Empty is tolerated — the columns then carry no suffix.
  std::vector<std::string> pull_estimator_roles{};

  // TaskDiagLog (#310) — §6.5 σ_min/λ² diagnostics for the task controller.
  // A single fixed instance (kTaskDiagLogInstance) with a fixed-width header,
  // so a bool gates it rather than a map carrying header context.
  bool task_diag_enabled{false};

  // GraspDiagLog (#428) — per-tick Force-PI servo + stiffness-estimator
  // diagnostics for the joint/task controllers. Single fixed instance
  // (kGraspDiagLogInstance).
  //
  // Gated on the force_pi_grasp BLOCK, not on `grasp_hand_mode` — deliberately
  // the same rule BuildGraspController uses. The mode is runtime-writable
  // through `grasp_controller_type` and controller switching never
  // re-configures, so gating on the mode would silently produce no file for a
  // session that started in contact_stop and switched to force_pi — which is
  // exactly the "the data you needed is not in the session directory" failure
  // this channel exists to remove. Ticks where the PI law was not the one
  // driving the hand are logged as valid=0 rows instead.
  bool grasp_diag_enabled{false};
  // Configured grasp-finger order, stamped into the per-finger column names so
  // a stored CSV decodes without that run's YAML (#234 P-14 rationale). Slot f
  // is the fingertip sensor that feeds grasp finger f, so these are the hand's
  // sensor names truncated to the grasp finger count. Empty is tolerated: the
  // header then emits no per-finger columns at all, which is visible rather
  // than mislabelled.
  std::vector<std::string> grasp_diag_finger_names{};

  // MomentumObserverLog (#135 Layer 1b) — single fixed instance
  // (kMomentumObserverLogInstance). Gated on the wiring being configured, which
  // is the `momentum_observer` block being present and enabled; a YAML entry on
  // a variant without that block is silently skipped like any unregistered
  // instance.
  bool momentum_observer_enabled{false};
  // Arm device joint order — the order the residual columns are in. Stamped
  // into the column names so a stored CSV decodes without that run's YAML.
  // Empty is tolerated: the header then emits no per-joint columns at all,
  // which is visible rather than mislabelled.
  std::vector<std::string> momentum_observer_joint_names{};
};

// ⚠ Two separate hazards, two separate fixes — keep both (#428).
//
//   1. Every field above is reachable by POSITIONAL brace-init, which is how
//      the three demo controllers used to build this struct. Inserting a field
//      mid-struct then silently reassigned their arguments (it did:
//      `pull_estimator_roles` landed in a bool and only failed to compile
//      because the types happened to disagree). All three call sites now use
//      DESIGNATED initializers, so order is not load-bearing and an omission is
//      named rather than shifted. Keep them that way.
//   2. Designated initializers do NOT silence -Wmissing-field-initializers —
//      GCC warns for any omitted member that has no default member
//      initializer, however it was written. That is why every aggregate member
//      above carries an explicit `{}`: a call site that legitimately wants the
//      empty map (the WBC controller wants four of them) can then leave it out
//      instead of spelling it, which is what stops the next field addition from
//      re-growing the warning at three sites.

/// Push one row for this tick (RT tick path — noexcept, heap-free).
///
/// `ran` is whether the Force-PI law actually drove the hand this tick. When it
/// is false — E-STOP, or any tick where another law owns the hand — a valid=0
/// row is still pushed, so the file has no gaps to interpret and stays joinable
/// with `pull_estimator.csv` on `tick`. A gap therefore means exactly one
/// thing: a dropped row (#234 P-20 convention).
inline void PushGraspDiagLog(rtc::LogHandle<GraspDiagLogPod>& handle,
                             const rtc::grasp::GraspController* ctrl, bool ran,
                             double t_relative_s, std::uint64_t tick) noexcept {
  if (!handle) {
    return;
  }
  GraspDiagLogPod pod{};
  if (ran && ctrl != nullptr) {
    FillGraspDiagLogPod(ctrl->finger_states(), ctrl->params(), ctrl->phase(),
                        ctrl->target_force(), t_relative_s, tick, pod);
  } else {
    // Zero rather than freeze — see GraspDiagLogPod::valid.
    pod.t_relative_s = t_relative_s;
    pod.tick = tick;
  }
  handle.Push(pod);
}

// One-shot INFO describing whether the grasp_diag channel registered.
//
// Not an error either way: a hand-less variant, a config with no
// `force_pi_grasp` block, or a hand reporting no fingertip sensors all land in
// the disabled branch deliberately. Say so out loud anyway and name the file —
// a missing grasp_diag.csv otherwise reads as a logging bug, which is the
// failure mode `pull_estimator_wiring.cpp` already had to fix once (#234 P-20).
inline void LogGraspDiagWiring(const rclcpp::Logger& logger, bool has_controller,
                               const std::vector<std::string>& finger_names) {
  if (!has_controller || finger_names.empty()) {
    RCLCPP_INFO(logger,
                "[grasp_diag] disabled — %s. No grasp_diag.csv will be written.",
                !has_controller ? "no 'force_pi_grasp' block in demo_shared.yaml"
                                : "hand reported no fingertip sensors to name the columns");
    return;
  }
  std::string names;
  for (const auto& n : finger_names) {
    if (!names.empty()) {
      names += ", ";
    }
    names += n;
  }
  RCLCPP_INFO(logger,
              "[grasp_diag] enabled — grasp_diag.csv, %zu finger column set(s): %s. Ticks not "
              "driven by the Force-PI law are logged as valid=0 rows.",
              finger_names.size(), names.c_str());
}

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
  // Single fixed instance (kGraspDiagLogInstance), same reason.
  rtc::LogHandle<integrated_bringup::GraspDiagLogPod> grasp_diag;
  // Single fixed instance (kMomentumObserverLogInstance), same reason.
  rtc::LogHandle<integrated_bringup::MomentumObserverLogPod> momentum_observer;
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
      // Both writers must be sized from the same numbers or the row stops
      // lining up with the header (#440), so the geometry is derived once here
      // and captured into each.
      const auto cols = integrated_bringup::DeviceStateLogColumnsFor(joint_names, motor_names);
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::DeviceStateLogPod>(
          entry.instance,
          [joint_names, motor_names, cols](std::ostream& os) {
            integrated_bringup::WriteDeviceStateLogHeader(os, joint_names, motor_names, cols);
          },
          [cols](std::ostream& os, const integrated_bringup::DeviceStateLogPod& pod) {
            integrated_bringup::WriteDeviceStateLogRow(os, pod, cols);
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
      // Both writers must agree on the stride AND on the fingertip-block width
      // or the row stops lining up with the header, so the whole geometry is
      // derived once here and captured into each (#440).
      const auto cols = integrated_bringup::DeviceSensorLogColumnsFor(
          sensor_names, it->second.values_per_group);
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::DeviceSensorLogPod>(
          entry.instance,
          [sensor_names, cols](std::ostream& os) {
            integrated_bringup::WriteDeviceSensorLogHeader(os, sensor_names, cols);
          },
          [cols](std::ostream& os, const integrated_bringup::DeviceSensorLogPod& pod) {
            integrated_bringup::WriteDeviceSensorLogRow(os, pod, cols);
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
      // Same reason as the DeviceStateLog channel above: one derivation, both
      // writers (#440). The fingertip block is already fixed-width.
      const auto cols = integrated_bringup::DeviceWbcLogColumnsFor(joint_names, motor_names);
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::DeviceWbcLogPod>(
          entry.instance,
          [role, joint_names, motor_names, fingertip_names, cols](std::ostream& os) {
            integrated_bringup::WriteDeviceWbcLogHeader(os, role, joint_names, motor_names,
                                                        fingertip_names, cols);
          },
          [cols](std::ostream& os, const integrated_bringup::DeviceWbcLogPod& pod) {
            integrated_bringup::WriteDeviceWbcLogRow(os, pod, cols);
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
          [num_contact_vars](std::ostream& os, const integrated_bringup::WbcDiagLogPod& pod) {
            integrated_bringup::WriteWbcDiagLogRow(os, pod, num_contact_vars);
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
    } else if (entry.msg_type == kGraspDiagLogMsgType) {
      if (!ctx.grasp_diag_enabled || entry.instance != kGraspDiagLogInstance) {
        // No force_pi_grasp block on this variant (or unexpected instance) —
        // silently skip like any unregistered instance. on_configure emits an
        // INFO naming the absent file so the empty session directory does not
        // read as a logging bug.
        continue;
      }
      const auto finger_names = ctx.grasp_diag_finger_names;
      // Both writers must agree on the per-finger column count or the row stops
      // lining up with the header, so the same list sizes each.
      const auto num_columns = finger_names.size();
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::GraspDiagLogPod>(
          entry.instance,
          [finger_names](std::ostream& os) {
            integrated_bringup::WriteGraspDiagLogHeader(os, finger_names);
          },
          [num_columns](std::ostream& os, const integrated_bringup::GraspDiagLogPod& pod) {
            integrated_bringup::WriteGraspDiagLogRow(os, pod, num_columns);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open grasp_diag CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.grasp_diag = std::move(handle);
    } else if (entry.msg_type == kMomentumObserverLogMsgType) {
      if (!ctx.momentum_observer_enabled || entry.instance != kMomentumObserverLogInstance) {
        // No momentum_observer block on this variant (or unexpected instance) —
        // silently skip like any unregistered instance. on_configure emits an
        // INFO naming the absent file so the empty session directory does not
        // read as a logging bug.
        continue;
      }
      const auto joint_names = ctx.momentum_observer_joint_names;
      // Both writers must agree on the per-joint column count or the row stops
      // lining up with the header, so the same list sizes each.
      const auto num_columns = integrated_bringup::MomentumObserverLogColumns(joint_names);
      auto handle = ctx.log_set.RegisterLog<integrated_bringup::MomentumObserverLogPod>(
          entry.instance,
          [joint_names](std::ostream& os) {
            integrated_bringup::WriteMomentumObserverLogHeader(os, joint_names);
          },
          [num_columns](std::ostream& os, const integrated_bringup::MomentumObserverLogPod& pod) {
            integrated_bringup::WriteMomentumObserverLogRow(os, pod, num_columns);
          });
      if (!handle) {
        RCLCPP_WARN(ctx.logger, "Failed to open momentum_observer CSV for instance=%s",
                    entry.instance.c_str());
        continue;
      }
      result.handles.momentum_observer = std::move(handle);
    }
    // Unknown msg_type: LoadConfig() has already validated against the
    // closed set {DeviceStateLog, DeviceSensorLog, DeviceWbcLog, WbcDiagLog,
    // PullEstimatorLog, TaskDiagLog, GraspDiagLog, MomentumObserverLog};
    // reaching here is a YAML parser bug. Silently ignore.
  }

  return result;
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_CONTROLLER_LOG_REGISTRATION_HPP_
