#pragma once

/// Shared fail-closed staleness gate for grasp-reading condition nodes.
///
/// IsGrasped / IsForceAbove / IsGraspPhase all read the 500Hz grasp aggregate
/// and must refuse to trust a frozen last value once its producer dies or
/// pauses. They shared a byte-identical age-check-and-warn block; this collapses
/// it to one call so the staleness policy has a single definition.

#include "ur5e_bt_coordinator/bt_logging.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <chrono>

namespace rtc_bt {

/// Returns true when the cached grasp state stamped `received_at` is older than
/// kGraspStateStaleSec, emitting a throttled fail-closed WARN via `logger` and
/// `clock`. Callers return BT::NodeStatus::FAILURE when this is true.
inline bool GraspStateStale(std::chrono::steady_clock::time_point received_at,
                            const rclcpp::Logger& logger, rclcpp::Clock& clock) {
  const double age_s = CachedStateAgeSec(received_at);
  if (age_s > kGraspStateStaleSec) {
    RCLCPP_WARN_THROTTLE(logger, clock, ::rtc_bt::logging::kThrottleSlowMs,
                         "grasp state stale (%.2fs > %.2fs) — failing closed", age_s,
                         kGraspStateStaleSec);
    return true;
  }
  return false;
}

}  // namespace rtc_bt
