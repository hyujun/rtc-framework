#ifndef UR5E_BRINGUP_BRINGUP_LOGGING_HPP_
#define UR5E_BRINGUP_BRINGUP_LOGGING_HPP_

/// Hierarchical sub-logger factories and standard throttle periods for the
/// ur5e_bringup package.
///
/// Demo controllers run inside the 500 Hz RT control loop, so all hot-path
/// log calls must use the *_THROTTLE variants together with the constants
/// declared below — never the bare INFO/WARN/ERROR macros.
///
/// See README.md "로깅 (Logging)" section for the full doctrine.

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ur5e_bringup::logging {

// ── Standard throttle periods ──────────────────────────────────────────────
//
// Use these constants instead of magic numbers in *_THROTTLE log macros.

inline constexpr int kThrottleFastMs =
    1000; ///< Fast progress (contact freeze, release skip)
inline constexpr int kThrottleSlowMs =
    2000; ///< Generic recurring snapshot/warning
inline constexpr int kThrottleIdleMs =
    10000; ///< Long idle / one-shot transitions

// ── Sub-logger factories ───────────────────────────────────────────────────
//
// Each subsystem owns a stable logger name using the repo-wide convention
// `<package_name>.<controller_key>` — the dot separates the package (left)
// from the controller identity (right), so a log prefix like
// `[ur5e_bringup.demo_joint_controller]` makes both fields explicit. See
// agent_docs/conventions.md "Logging" section.
//
// The rclcpp logger handle is cheap to construct (internally cached), so
// callers may obtain a fresh handle on every log call without measurable
// overhead. Demo controllers may still cache the result in a member to avoid
// repeating the lookup on every cycle.

inline rclcpp::Logger DemoJointLogger() {
  return rclcpp::get_logger("ur5e_bringup.demo_joint_controller");
}

inline rclcpp::Logger DemoTaskLogger() {
  return rclcpp::get_logger("ur5e_bringup.demo_task_controller");
}

inline rclcpp::Logger DemoWbcLogger() {
  return rclcpp::get_logger("ur5e_bringup.demo_wbc_controller");
}

inline rclcpp::Logger SharedConfigLogger() {
  return rclcpp::get_logger("ur5e_bringup.demo_shared_config");
}

} // namespace ur5e_bringup::logging

#endif // UR5E_BRINGUP_BRINGUP_LOGGING_HPP_
