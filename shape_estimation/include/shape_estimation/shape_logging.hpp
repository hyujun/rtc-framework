#pragma once

/// Hierarchical sub-logger factories and standard throttle periods for the
/// shape_estimation package.
///
/// All library modules and the node shell share a common naming convention so
/// that runtime log filtering can target individual subsystems via the
/// rcl_interfaces/SetLoggerLevels service. See README.md "Logging" section for
/// the full doctrine.

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rtc::shape::logging {

// ── Standard throttle periods ──────────────────────────────────────────────
//
// Use these constants instead of magic numbers in *_THROTTLE log macros.

inline constexpr int kThrottleFastMs = 500;    ///< Fast progress (fit results)
inline constexpr int kThrottleSlowMs = 2000;   ///< Generic recurring warning
inline constexpr int kThrottleIdleMs = 10000;  ///< Long idle / stall

// ── Sub-logger factories ───────────────────────────────────────────────────
//
// Each subsystem owns a stable logger name. The rclcpp logger handle is cheap
// to construct (internally cached), so callers may obtain a fresh handle on
// every log call without measurable overhead.

inline rclcpp::Logger NodeLogger() {
  return rclcpp::get_logger("shape.node");
}

inline rclcpp::Logger VoxelLogger() {
  return rclcpp::get_logger("shape.voxel");
}

inline rclcpp::Logger ClassifyLogger() {
  return rclcpp::get_logger("shape.classify");
}

inline rclcpp::Logger FitLogger() {
  return rclcpp::get_logger("shape.fit");
}

inline rclcpp::Logger ProtusLogger() {
  return rclcpp::get_logger("shape.protus");
}

inline rclcpp::Logger ExploreLogger() {
  return rclcpp::get_logger("shape.explore");
}

}  // namespace rtc::shape::logging
