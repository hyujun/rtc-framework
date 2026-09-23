#ifndef INTEGRATED_BRINGUP_SUPPORT_LAYOUT_PROFILE_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_LAYOUT_PROFILE_HPP_

// ── Launch layout profile (issue #350), shared by every controller that runs a
// thread on the `mpc` layout role ─────────────────────────────────────────────
//
// Two controllers put a thread on that role: DemoWbcController (the MPC solver)
// and DemoCatchingController (the catching planner — E-7 decision J, it takes
// the MPC role rather than a role of its own). Both must refuse to activate
// under a profile that returned the role's core to the system cpuset, and both
// must read the profile the SAME way, or one of them ends up spawning a
// SCHED_FIFO thread onto an unshielded core. Hence one definition here rather
// than a copy per controller (P5).
//
// Ids mirror repo_scripts/config/thread_layout.yaml's `profiles:` block — the
// launch resolves the id (rtc_tools.launch.cpu_shield::mpc_layout_profile) and
// hands the same string to the cset shield and to the controllers.

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <string>
#include <string_view>

namespace integrated_bringup {

/// The per-controller node parameter carrying the launch's profile id.
inline constexpr const char* kRtLayoutProfileParam = "rt_layout_profile";

inline constexpr std::string_view kDefaultLayoutProfile = "mpc_on";
inline constexpr std::string_view kMpcOffLayoutProfile = "mpc_off";

/// True when `profile` returns the `mpc` role's core to the system cpuset.
///
/// Anything other than the recognised opt-out — an empty or misspelled id
/// included — reads as the default profile, with the core reserved. That
/// direction is deliberate: reserving a core nobody uses wastes it, while
/// treating an unrecognised id as an opt-out would refuse activation on a box
/// whose shield still holds the core.
[[nodiscard]] constexpr bool LayoutProfileDropsMpc(std::string_view profile) noexcept {
  return profile == kMpcOffLayoutProfile;
}

/// The profile id on `node`, declaring the parameter with the default when no
/// one has set it. Non-RT (on_configure). Guarded re-entry: a re-configure on
/// the same node must not throw ParameterAlreadyDeclaredException.
[[nodiscard]] inline std::string ReadLayoutProfile(rclcpp_lifecycle::LifecycleNode& node) {
  if (node.has_parameter(kRtLayoutProfileParam)) {
    return node.get_parameter(kRtLayoutProfileParam).as_string();
  }
  return node.declare_parameter<std::string>(kRtLayoutProfileParam,
                                             std::string(kDefaultLayoutProfile));
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_LAYOUT_PROFILE_HPP_
