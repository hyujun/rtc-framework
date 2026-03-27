#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace rtc_bt {

// ── 6D Pose (position + RPY orientation) ────────────────────────────────────
struct Pose6D {
  double x{0.0}, y{0.0}, z{0.0};
  double roll{0.0}, pitch{0.0}, yaw{0.0};

  double PositionDistanceTo(const Pose6D& other) const {
    double dx = x - other.x, dy = y - other.y, dz = z - other.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double OrientationDistanceTo(const Pose6D& other) const {
    double dr = roll - other.roll;
    double dp = pitch - other.pitch;
    double dy_ = yaw - other.yaw;
    return std::sqrt(dr * dr + dp * dp + dy_ * dy_);
  }
};

// ── Fingertip force data (parsed from HandSensorState) ──────────────────────
struct FingertipForce {
  std::string name;
  float fx{0.0f}, fy{0.0f}, fz{0.0f};
  float contact_flag{0.0f};
  bool inference_enable{false};

  float Magnitude() const {
    return std::sqrt(fx * fx + fy * fy + fz * fz);
  }
};

// ── Constants ───────────────────────────────────────────────────────────────
inline constexpr int kArmDof = 6;
inline constexpr int kHandDof = 10;

}  // namespace rtc_bt

// ── BehaviorTree.CPP type conversions ─────────────────────────────────────
// Enable Pose6D to be stored/retrieved from BT Blackboard via string conversion.
namespace BT {

template <>
inline rtc_bt::Pose6D convertFromString(StringView str)
{
  // Format: "x;y;z;roll;pitch;yaw"
  auto parts = splitString(str, ';');
  if (parts.size() != 6) {
    throw RuntimeError("Pose6D: expected 6 semicolon-separated values, got ",
                       std::to_string(parts.size()));
  }
  rtc_bt::Pose6D p;
  p.x     = convertFromString<double>(parts[0]);
  p.y     = convertFromString<double>(parts[1]);
  p.z     = convertFromString<double>(parts[2]);
  p.roll  = convertFromString<double>(parts[3]);
  p.pitch = convertFromString<double>(parts[4]);
  p.yaw   = convertFromString<double>(parts[5]);
  return p;
}

template <>
inline std::vector<double> convertFromString(StringView str)
{
  // Format: "0.1;0.2;0.3;..." (semicolon-separated)
  std::vector<double> result;
  auto parts = splitString(str, ';');
  for (const auto& p : parts) {
    result.push_back(convertFromString<double>(p));
  }
  return result;
}

template <>
inline std::vector<rtc_bt::Pose6D> convertFromString(StringView str)
{
  // Format: "x;y;z;r;p;y|x;y;z;r;p;y|..."
  std::vector<rtc_bt::Pose6D> result;
  auto poses = splitString(str, '|');
  for (const auto& ps : poses) {
    result.push_back(convertFromString<rtc_bt::Pose6D>(ps));
  }
  return result;
}

}  // namespace BT
