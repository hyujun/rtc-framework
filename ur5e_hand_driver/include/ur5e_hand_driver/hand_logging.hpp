#ifndef UR5E_HAND_DRIVER_HAND_LOGGING_HPP_
#define UR5E_HAND_DRIVER_HAND_LOGGING_HPP_

/// Hierarchical sub-logger factories and standard throttle periods for the
/// ur5e_hand_driver package.
///
/// The HandController EventLoop runs at ~500 Hz UDP polling rate, so the
/// transport, codec, sensor processor, and FT inferencer headers are all on
/// the RT hot path. All hot-path log calls must use the *_THROTTLE variants
/// together with the constants declared below — never the bare INFO/WARN
/// macros, and never `_ONCE` (which still pays the formatting allocation on
/// the first hit and offers no defense against repeated firing).
///
/// See README.md "로깅 (Logging)" section for the full doctrine.

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ur5e_hand_driver::logging {

// ── Standard throttle periods ──────────────────────────────────────────────
//
// Use these constants instead of magic numbers in *_THROTTLE log macros.

inline constexpr int kThrottleFastMs = 500;    ///< Fast progress (recv failures)
inline constexpr int kThrottleSlowMs = 2000;   ///< Generic recurring warning
inline constexpr int kThrottleIdleMs = 10000;  ///< Long idle / one-shot transitions
inline constexpr int kThrottleHotMs  = 5000;   ///< RT hot-path exception path

// ── Sub-logger factories ───────────────────────────────────────────────────
//
// Each subsystem owns a stable logger name. The rclcpp logger handle is cheap
// to construct (internally cached), so callers may obtain a fresh handle on
// every log call without measurable overhead.

inline rclcpp::Logger NodeLogger() {
  return rclcpp::get_logger("hand.node");
}

inline rclcpp::Logger ControllerLogger() {
  return rclcpp::get_logger("hand.ctrl");
}

inline rclcpp::Logger TransportLogger() {
  return rclcpp::get_logger("hand.udp");
}

inline rclcpp::Logger SensorLogger() {
  return rclcpp::get_logger("hand.sensor");
}

inline rclcpp::Logger FailureLogger() {
  return rclcpp::get_logger("hand.fail");
}

inline rclcpp::Logger FtLogger() {
  return rclcpp::get_logger("hand.ft");
}

}  // namespace ur5e_hand_driver::logging

#endif  // UR5E_HAND_DRIVER_HAND_LOGGING_HPP_
