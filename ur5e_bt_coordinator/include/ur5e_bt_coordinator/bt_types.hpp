#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <cstdint>
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

// ── Cached grasp state (from /hand/grasp_state, computed at 500Hz) ──────────
struct CachedGraspState {
  // Per-fingertip
  struct Fingertip {
    std::string name;
    float force_magnitude{0.0f};  // |F| [N]
    float contact_flag{0.0f};     // contact probability (0.0~1.0)
    bool  inference_valid{false};
  };
  std::vector<Fingertip> fingertips;

  // Aggregate (pre-computed by controller at 500Hz)
  int   num_active_contacts{0};
  float max_force{0.0f};
  bool  grasp_detected{false};
  float force_threshold{1.0f};
  int   min_fingertips{2};

  // Force-PI grasp controller state (grasp_controller_type == "force_pi")
  uint8_t grasp_phase{0};                      // GraspPhase enum (0=Idle..5=Releasing)
  float grasp_target_force{0.0f};              // active target force [N]
  std::vector<float> finger_s;                 // grasp parameter per finger [0,1]
  std::vector<float> finger_filtered_force;    // filtered force per finger [N]
  std::vector<float> finger_force_error;       // force error per finger [N]
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
