#pragma once

#include <shape_estimation_msgs/msg/shape_estimate.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
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
    // contact_flag: producer-capability dependent — sensor A native sigmoid
    // prob [0,1] or sensor B derived binary 1.0/0.0 (bakes in the controller's
    // grasp_force_threshold). IsGrasped/IsForceAbove custom-threshold recounts
    // gate on force_magnitude alone, NOT contact_flag, so a BT threshold below
    // that floor stays reachable; the aggregate path still uses grasp_detected.
    float contact_flag{0.0f};
    bool inference_valid{false};
  };

  std::vector<Fingertip> fingertips;

  // Aggregate (pre-computed by controller at 500Hz)
  int num_active_contacts{0};
  float max_force{0.0f};
  bool grasp_detected{false};
  float force_threshold{1.0f};
  int min_fingertips{2};

  // Force-PI grasp controller state (grasp_controller_type == "force_pi")
  uint8_t grasp_phase{0};                    // GraspPhase enum (0=Idle..5=Releasing)
  float grasp_target_force{0.0f};            // active target force [N]
  std::vector<float> finger_s;               // grasp parameter per finger [0,1]
  std::vector<float> finger_filtered_force;  // filtered force per finger [N]
  std::vector<float> finger_force_error;     // force error per finger [N]

  // steady_clock receive time of the message this state was decoded from.
  // Default (epoch) means never received; the staleness gate treats that as
  // stale so a condition node fails closed instead of trusting a zero aggregate.
  std::chrono::steady_clock::time_point received_at{};
};

// ── Cached WBC state (from /<ctrl>/hand/wbc_state, computed at 500Hz) ───────
// Published by TSID-based whole-body controllers (DemoWbcController). Schema
// parallels CachedGraspState but reflects WBC's TSID-based grasp algorithm
// (no Force-PI fields). BtRosBridge subscribes to both grasp_state and
// wbc_state — only one is populated at a time, depending on which controller
// is active. Health watchdog tracks each separately.
struct CachedWbcState {
  // Per-fingertip
  struct Fingertip {
    std::string name;
    float force_magnitude{0.0f};  // |F| [N]
    // contact_flag: same sensor-A/B semantics as CachedGraspState.
    float contact_flag{0.0f};
    // displacement: native (sensor A slots 4..6) when backend exposes it,
    // 0 otherwise. WBC controller-side deformation guard is stubbed.
    float displacement{0.0f};  // [m]
  };

  std::vector<Fingertip> fingertips;

  // Aggregate (pre-computed by controller at 500Hz)
  int num_active_contacts{0};
  float max_force{0.0f};
  float grasp_target_force{0.0f};
  bool grasp_detected{false};
  int min_fingertips{2};

  // WbcPhase enum (0=Idle, 1=Approach, 3=Closure, 4=Hold, 6=Release,
  // 7=Fallback). Slots 2 (was PreGrasp, merged into Approach) and 5 (was
  // Retreat) are reserved/deprecated and no longer published. Exact enum
  // values match WbcPhase in demo_wbc_controller.hpp and PHASE_* constants
  // in WbcState.msg.
  uint8_t phase{0};

  // TSID solver diagnostics (informational)
  float tsid_solve_us{0.0f};
  bool tsid_solver_ok{true};
  int qp_fail_count{0};

  // See CachedGraspState::received_at.
  std::chrono::steady_clock::time_point received_at{};
};

// Max age (s) a cached grasp/wbc aggregate may reach before condition nodes
// treat it as stale and fail closed. Shorter than the default watchdog_timeout
// (2.0s) so a dead 500Hz producer trips these gates well before the watchdog.
constexpr double kGraspStateStaleSec = 0.5;

/// Age in seconds of a cached state given its `received_at`. Returns +inf when
/// unset (never received) so a staleness comparison fails closed.
inline double CachedStateAgeSec(std::chrono::steady_clock::time_point received_at) {
  if (received_at == std::chrono::steady_clock::time_point{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - received_at).count();
}

}  // namespace rtc_bt

// ── BehaviorTree.CPP type conversions ─────────────────────────────────────
// Enable Pose6D to be stored/retrieved from BT Blackboard via string
// conversion.
namespace BT {

template <>
inline rtc_bt::Pose6D convertFromString(StringView str) {
  // Format: "x;y;z;roll;pitch;yaw"
  auto parts = splitString(str, ';');
  if (parts.size() != 6) {
    throw RuntimeError("Pose6D: expected 6 semicolon-separated values, got ",
                       std::to_string(parts.size()));
  }
  rtc_bt::Pose6D p;
  p.x = convertFromString<double>(parts[0]);
  p.y = convertFromString<double>(parts[1]);
  p.z = convertFromString<double>(parts[2]);
  p.roll = convertFromString<double>(parts[3]);
  p.pitch = convertFromString<double>(parts[4]);
  p.yaw = convertFromString<double>(parts[5]);
  return p;
}

template <>
inline std::vector<double> convertFromString(StringView str) {
  // Format: "0.1;0.2;0.3;..." (semicolon-separated)
  std::vector<double> result;
  auto parts = splitString(str, ';');
  for (const auto& p : parts) {
    result.push_back(convertFromString<double>(p));
  }
  return result;
}

template <>
inline std::vector<rtc_bt::Pose6D> convertFromString(StringView str) {
  // Format: "x;y;z;r;p;y|x;y;z;r;p;y|..."
  std::vector<rtc_bt::Pose6D> result;
  auto poses = splitString(str, '|');
  for (const auto& ps : poses) {
    result.push_back(convertFromString<rtc_bt::Pose6D>(ps));
  }
  return result;
}

}  // namespace BT
