#include "rtc_controllers/catching/joint_groups.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <string>

namespace rtc::catching {

bool ResolveJointGroups(std::span<const JointGroupSpec> specs,
                        std::span<const std::string> device_joints, JointGroups& out,
                        std::string* error) {
  const auto fail = [error](const std::string& why) {
    if (error != nullptr) {
      *error = why;
    }
    return false;
  };
  JointGroups g;
  if (specs.empty()) {
    out = g;
    return true;
  }
  if (specs.size() > static_cast<std::size_t>(kMaxJointGroups)) {
    return fail("'planner.groups' lists " + std::to_string(specs.size()) + " groups, at most " +
                std::to_string(kMaxJointGroups) + " (kMaxJointGroups)");
  }
  if (device_joints.empty() || device_joints.size() > static_cast<std::size_t>(kMaxPlanNv)) {
    return fail("the arm device has " + std::to_string(device_joints.size()) +
                " joints; groups need 1.." + std::to_string(kMaxPlanNv) + " (kMaxPlanNv)");
  }
  std::array<bool, kMaxPlanNv> seen{};
  for (std::size_t i = 0; i < specs.size(); ++i) {
    for (const std::string& name : specs[i].joints) {
      const auto it = std::find(device_joints.begin(), device_joints.end(), name);
      if (it == device_joints.end()) {
        return fail("'planner.groups[" + std::to_string(i) + "].joints' names '" + name +
                    "', which is not a joint of the arm device");
      }
      const auto d = static_cast<std::size_t>(std::distance(device_joints.begin(), it));
      if (seen[d]) {
        return fail("'planner.groups' puts joint '" + name +
                    "' in two groups (the groups are disjoint)");
      }
      seen[d] = true;
      g.group_of_device[d] = static_cast<std::int8_t>(i);
    }
  }
  for (std::size_t d = 0; d < device_joints.size(); ++d) {
    if (!seen[d]) {
      return fail("'planner.groups' leaves joint '" + device_joints[d] +
                  "' in no group (the groups partition the arm device's joints)");
    }
  }
  g.n_groups = static_cast<int>(specs.size());
  out = g;
  return true;
}

bool GroupsOfModel(const JointGroups& groups, std::span<const int> device_of_model, int nv,
                   std::array<std::int8_t, kMaxPlanNv>& group_of_model) {
  if (nv < 1 || nv > kMaxPlanNv || device_of_model.size() < static_cast<std::size_t>(nv)) {
    return false;
  }
  std::array<std::int8_t, kMaxPlanNv> out{};
  for (int m = 0; m < nv; ++m) {
    const int d = device_of_model[static_cast<std::size_t>(m)];
    if (d < 0 || d >= kMaxPlanNv) {
      return false;
    }
    out[static_cast<std::size_t>(m)] = static_cast<std::int8_t>(groups.GroupOf(d));
  }
  group_of_model = out;
  return true;
}

int FirstGroupLeftFree(const JointGroups& groups, int n_dev, JointSet plan_joints,
                       std::span<const double> w_rest_of_group) noexcept {
  for (int d = 0; d < n_dev && d < kMaxPlanNv; ++d) {
    if (InJointSet(plan_joints, d)) {
      continue;
    }
    const int g = groups.GroupOf(d);
    const bool held = groups.n_groups > 0 && static_cast<std::size_t>(g) < w_rest_of_group.size() &&
                      w_rest_of_group[static_cast<std::size_t>(g)] > 0.0;
    if (!held) {
      return g;
    }
  }
  return -1;
}

int FindJointGroup(std::span<const JointGroupSpec> specs, const std::string& name) noexcept {
  for (std::size_t i = 0; i < specs.size(); ++i) {
    if (specs[i].name == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace rtc::catching
