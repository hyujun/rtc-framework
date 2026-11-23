#ifndef INTEGRATED_BRINGUP_SUPPORT_SIM_TIME_SYNC_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_SIM_TIME_SYNC_HPP_

/// @file sim_time_sync.hpp
/// @brief Whether the controller manager that built a controller's node runs
///        its RT tick in simulation lock-step — the fact a controller mode
///        that blocks the tick on a solver (the locomotion controller's MPC
///        lock-step) needs before it may be configured.
///
/// The CM declares `cm_sim_time_sync` on every per-controller LifecycleNode
/// with its own `use_sim_time_sync` (rtc_controller_manager README). This
/// reader is fail-closed, the opposite of `ReadLayoutProfile`: a node without
/// the parameter (no CM built it — a test rig, an older CM), one where it is
/// false, or one where it is not a bool reads as "not sim-synced". Nothing
/// here declares the parameter: a controller cannot grant itself the sim.

#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace integrated_bringup {

/// The per-controller node parameter the CM sets to its own `use_sim_time_sync`.
inline constexpr const char* kCmSimTimeSyncParam = "cm_sim_time_sync";

/// True only when `node` carries `cm_sim_time_sync` as a bool set to true.
/// Non-RT (on_configure).
[[nodiscard]] inline bool CmSimTimeSync(const rclcpp_lifecycle::LifecycleNode& node) noexcept {
  try {
    if (!node.has_parameter(kCmSimTimeSyncParam)) {
      return false;
    }
    const rclcpp::Parameter p = node.get_parameter(kCmSimTimeSyncParam);
    return p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && p.as_bool();
  } catch (...) {
    return false;
  }
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_SIM_TIME_SYNC_HPP_
