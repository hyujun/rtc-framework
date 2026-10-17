// ── Joint groups of the arm device (dynamic_catching E3-F02, #641) ───────────
//
// A segment core's terms and its move blocking can differ per joint group
// (formulation §1.1, §1.3, MD-49): a group's velocity is damped, another's held
// near a rest pose, each with its own block pattern. Which joints form a group
// is the profile's (`planner.groups`, written once at the planner's level next
// to `planner.wait_pose`); what a group is FOR is only the weights the segment
// blocks give it. Nothing here knows a robot or a role.
//
// The groups PARTITION the arm device's joints: every device joint is in
// exactly one group. No `planner.groups` = one group of every joint, which is
// what every core solved before groups existed.
//
// Non-RT: parsing and resolving run at configure; what reaches a planner is the
// fixed-size JointGroups map.
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"  // kMaxPlanNv

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace rtc::catching {

/// Most groups a profile may declare.
inline constexpr int kMaxJointGroups = 4;

/// One `planner.groups` entry as written: a name the segment blocks refer to
/// and device joint names.
struct JointGroupSpec {
  std::string name;
  std::vector<std::string> joints;
};

/// The groups resolved against the arm device (device joint index → group).
struct JointGroups {
  /// 0 = none declared: every joint is in group 0.
  int n_groups{0};
  /// Group of device joint d (read for d < the device's joint count).
  std::array<std::int8_t, kMaxPlanNv> group_of_device{};

  /// Group of device joint d; 0 when no group is declared.
  [[nodiscard]] int GroupOf(int d) const noexcept {
    return n_groups == 0 || d < 0 || d >= kMaxPlanNv ? 0
                                                     : group_of_device[static_cast<std::size_t>(d)];
  }

  /// Groups a core solves with: 1 when none is declared.
  [[nodiscard]] int Count() const noexcept { return n_groups == 0 ? 1 : n_groups; }
};

/// @brief Resolve `planner.groups` against the arm device's joint names (non-RT).
///
/// Every named joint must be one of `device_joints` (a joint of another device,
/// or a misspelt one, is refused by name), and every device joint must be in a
/// group. Empty `specs` gives the no-group map.
/// @param specs         the parsed `planner.groups` (names unique, joints disjoint)
/// @param device_joints the arm device's joint names, device order
/// @param[out] out      written only on success
/// @param[out] error    why, when false (may be nullptr)
[[nodiscard]] bool ResolveJointGroups(std::span<const JointGroupSpec> specs,
                                      std::span<const std::string> device_joints, JointGroups& out,
                                      std::string* error);

/// @brief The group of each model joint m: groups.GroupOf(device_of_model[m]) (non-RT).
/// @return false when an index is outside [0, kMaxPlanNv) or nv is out of range.
[[nodiscard]] bool GroupsOfModel(const JointGroups& groups, std::span<const int> device_of_model,
                                 int nv, std::array<std::int8_t, kMaxPlanNv>& group_of_model);

/// Index of the group named `name` in `specs`, −1 when none is.
[[nodiscard]] int FindJointGroup(std::span<const JointGroupSpec> specs,
                                 const std::string& name) noexcept;

}  // namespace rtc::catching
