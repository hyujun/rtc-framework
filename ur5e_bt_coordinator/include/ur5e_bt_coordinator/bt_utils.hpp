// file: include/ur5e_bt_coordinator/bt_utils.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <sstream>
#include <string>
#include <vector>

namespace rtc_bt {

// ── Time ────────────────────────────────────────────────────────────────────

/// Seconds elapsed since a time point.
inline double ElapsedSeconds(std::chrono::steady_clock::time_point since)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

// ── Map lookup ──────────────────────────────────────────────────────────────

/// Lookup a key in an associative container; throw BT::RuntimeError on miss.
template <typename MapT>
const typename MapT::mapped_type& LookupOrThrow(
    const MapT& map, const std::string& key, const std::string& context)
{
  auto it = map.find(key);
  if (it == map.end()) {
    throw BT::RuntimeError(context + ": unknown key: " + key);
  }
  return it->second;
}

// ── CSV parsing ─────────────────────────────────────────────────────────────

/// Parse a comma-separated string into a vector of T (int or double).
template <typename T>
std::vector<T> ParseCsvList(const std::string& str)
{
  std::vector<T> result;
  std::istringstream ss(str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      if constexpr (std::is_same_v<T, int>) {
        result.push_back(std::stoi(token));
      } else if constexpr (std::is_same_v<T, double>) {
        result.push_back(std::stod(token));
      }
    } catch (...) {}
  }
  return result;
}

// ── Partial hand joint update ───────────────────────────────────────────────

/// Read current hand state, overlay specific finger joints, and publish.
inline void ApplyPartialHandTarget(
    BtRosBridge& bridge,
    const HandPose& target_pose,
    const std::vector<int>& joint_indices)
{
  auto current = bridge.GetHandJointPositions();
  if (current.size() < static_cast<std::size_t>(kHandDofCount)) {
    current.resize(kHandDofCount, 0.0);
  }
  for (int idx : joint_indices) {
    current[idx] = target_pose[idx];
  }
  bridge.PublishHandTarget(current);
}

// ── Force counting ──────────────────────────────────────────────────────────

/// Count fingertips with active contact above a force threshold.
inline int CountActiveContacts(
    const std::vector<FingertipForce>& forces, float threshold)
{
  int count = 0;
  for (const auto& ft : forces) {
    if (ft.inference_enable && ft.contact_flag > 0.5f &&
        ft.Magnitude() > threshold) {
      ++count;
    }
  }
  return count;
}

}  // namespace rtc_bt
