// file: include/ur5e_bt_coordinator/bt_utils.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace rtc_bt {

// ── Hand trajectory gain defaults (match RT controller YAML) ───────────────

inline constexpr double kDefaultHandTrajectorySpeed = 1.0;    // [rad/s]
inline constexpr double kDefaultHandMaxTrajVelocity = 2.0;    // [rad/s]

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
    } catch (...) {
      throw BT::RuntimeError("ParseCsvList: invalid token '" + token +
                             "' in string '" + str + "'");
    }
  }
  return result;
}

// ── Partial hand joint update ───────────────────────────────────────────────

/// Read last hand target (or current position as fallback), overlay specific
/// finger joints, and publish.  Non-target joints keep moving toward their
/// previously commanded targets instead of freezing at the current position.
inline void ApplyPartialHandTarget(
    BtRosBridge& bridge,
    const HandPose& target_pose,
    const std::vector<int>& joint_indices)
{
  auto base = bridge.GetLastHandTarget();
  if (base.size() < static_cast<std::size_t>(kHandDofCount)) {
    // First publish in this session — fall back to current position
    base = bridge.GetHandJointPositions();
    if (base.size() < static_cast<std::size_t>(kHandDofCount)) {
      base.resize(kHandDofCount, 0.0);
    }
  }
  for (int idx : joint_indices) {
    base[idx] = target_pose[idx];
  }
  bridge.PublishHandTarget(base);
}

// ── Trajectory duration estimation ─────────────────────────────────────────

/// RT 컨트롤러와 동일한 공식으로 hand trajectory duration을 추정한다.
/// 특정 관절 인덱스 전용 오버로드.
///
/// RT 공식: duration = max(0.01, max_dist/speed, 1.875*max_dist/max_vel)
/// Quintic rest-to-rest peak velocity = (15/8) * max_dist / T 이므로
/// T >= 1.875 * max_dist / max_vel 조건으로 velocity limit 보장.
inline double EstimateHandTrajectoryDuration(
    const std::vector<double>& current,
    const std::array<double, kHandDofCount>& target,
    const std::vector<int>& indices,
    double speed, double max_vel,
    double margin = 1.1)
{
  double max_dist = 0.0;
  for (int idx : indices) {
    const auto ui = static_cast<std::size_t>(idx);
    if (ui < current.size()) {
      max_dist = std::max(max_dist, std::abs(target[ui] - current[ui]));
    }
  }
  const double T_speed = (speed > 0.0) ? (max_dist / speed) : 0.0;
  const double T_vel = (max_vel > 0.0) ? (1.875 * max_dist / max_vel) : 0.0;
  return std::max(0.01, std::max(T_speed, T_vel)) * margin;
}

/// RT 컨트롤러와 동일한 공식으로 hand trajectory duration을 추정한다.
/// 전체 10-DoF 오버로드.
inline double EstimateHandTrajectoryDuration(
    const std::vector<double>& current,
    const std::vector<double>& target,
    double speed, double max_vel,
    double margin = 1.1)
{
  double max_dist = 0.0;
  const std::size_t n = std::min(current.size(), target.size());
  for (std::size_t i = 0; i < n; ++i) {
    max_dist = std::max(max_dist, std::abs(target[i] - current[i]));
  }
  const double T_speed = (speed > 0.0) ? (max_dist / speed) : 0.0;
  const double T_vel = (max_vel > 0.0) ? (1.875 * max_dist / max_vel) : 0.0;
  return std::max(0.01, std::max(T_speed, T_vel)) * margin;
}

// ── Opposition helper ──────────────────────────────────────────────────────

/// Opposition 전용: thumb + target 손가락만 목표 포즈, 나머지는 home으로 리셋.
/// 비-target 손가락 잔류 문제를 방지한다.
/// Bridge의 pose library에서 home 포즈를 읽는다.
inline void ApplyOppositionTarget(
    BtRosBridge& bridge,
    const HandPose& thumb_pose,
    const HandPose& target_pose,
    const std::vector<int>& target_indices)
{
  const auto& home = bridge.GetHandPose("home");
  std::vector<double> cmd(home.begin(), home.end());
  // thumb 관절 덮어쓰기
  for (int idx : kFingerJointIndices.at("thumb")) {
    cmd[static_cast<std::size_t>(idx)] = thumb_pose[static_cast<std::size_t>(idx)];
  }
  // target 손가락 관절 덮어쓰기
  for (int idx : target_indices) {
    cmd[static_cast<std::size_t>(idx)] = target_pose[static_cast<std::size_t>(idx)];
  }
  bridge.PublishHandTarget(cmd);
}

}  // namespace rtc_bt
