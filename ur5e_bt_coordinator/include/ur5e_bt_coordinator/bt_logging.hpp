#pragma once

/// Hierarchical sub-logger factories and standard throttle periods for the
/// ur5e_bt_coordinator package.
///
/// All BT nodes share a common naming convention so that runtime log filtering
/// can target individual nodes, a whole subsystem, or the entire package via
/// the rcl_interfaces/SetLoggerLevels service. See README.md "로깅" section
/// for the full doctrine.

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <string>
#include <string_view>

namespace rtc_bt::logging {

// ── Standard throttle periods ──────────────────────────────────────────────
//
// Use these constants instead of magic numbers in *_THROTTLE log macros.

inline constexpr int kThrottleFastMs = 500;    ///< Fast polling progress
inline constexpr int kThrottleSlowMs = 2000;   ///< Generic recurring warning
inline constexpr int kThrottleIdleMs = 10000;  ///< Long idle / paused state

// ── Sub-logger factories ───────────────────────────────────────────────────
//
// Each subsystem owns a stable logger name. The rclcpp logger handle is cheap
// to construct (internally cached), so callers may obtain a fresh handle on
// every log call without measurable overhead.

inline rclcpp::Logger CoordLogger() {
  return rclcpp::get_logger("bt.coord");
}

inline rclcpp::Logger BridgeLogger() {
  return rclcpp::get_logger("bt.bridge");
}

inline rclcpp::Logger FailLogger() {
  return rclcpp::get_logger("bt.fail");
}

inline rclcpp::Logger WatchdogLogger() {
  return rclcpp::get_logger("bt.watchdog");
}

inline rclcpp::Logger PosesLogger() {
  return rclcpp::get_logger("bt.poses");
}

/// Per-action-node sub-logger. `node_snake` should be the snake_case form of
/// the BT registration name (e.g. "move_to_pose" for MoveToPose).
inline rclcpp::Logger ActionLogger(std::string_view node_snake) {
  std::string name;
  name.reserve(10 + node_snake.size());
  name.append("bt.action.").append(node_snake);
  return rclcpp::get_logger(name);
}

/// Per-condition-node sub-logger.
inline rclcpp::Logger CondLogger(std::string_view node_snake) {
  std::string name;
  name.reserve(8 + node_snake.size());
  name.append("bt.cond.").append(node_snake);
  return rclcpp::get_logger(name);
}

}  // namespace rtc_bt::logging
