#ifndef RTC_CONTROLLERS_GRASP_GRASP_STATE_HPP_
#define RTC_CONTROLLERS_GRASP_GRASP_STATE_HPP_

// Trivially-copyable POD describing the per-tick state of a grasp pipeline:
// contact detection + per-finger force/inference aggregates + Force-PI phase.
// Lives here (rtc_controllers/grasp) rather than rtc_base because the
// semantics are grasp-domain — rtc_base stays robot-agnostic and primitive.
//
// The capacity bound (kMaxGraspFingertips) is a compile-time upper bound used
// for the RT-path std::array dims; runtime fingertip count comes from each
// controller's YAML / DeviceNameConfig::sensor_names. SeqLock<GraspStateData>
// requires trivially-copyable, hence fixed arrays here even though the
// publish-side ROS message (rtc_msgs/GraspState) is dynamic.

#include <array>
#include <cstdint>
#include <type_traits>

namespace rtc::grasp {

// Compile-time fingertip array capacity for grasp-domain RT POD. Sized to
// match the historical rtc::kMaxSensorGroups / kMaxFingertips bound; raise
// if a future hand exceeds 8 fingertips — never branch on it.
inline constexpr int kMaxGraspFingertips = 8;

// Grasp detection state — trivially copyable, SeqLock-compatible.
struct GraspStateData {
  std::array<float, kMaxGraspFingertips> force_magnitude{};
  std::array<float, kMaxGraspFingertips> contact_flag{};
  std::array<bool, kMaxGraspFingertips> inference_valid{};
  int num_fingertips{0};
  int num_active_contacts{0};
  float max_force{0.0f};
  bool grasp_detected{false};
  float force_threshold{1.0f};
  int min_fingertips_for_grasp{2};

  // Force-PI grasp controller state (grasp_controller_type == "force_pi")
  uint8_t grasp_phase{0};  // GraspPhase enum
  std::array<float, kMaxGraspFingertips> finger_s{};
  std::array<float, kMaxGraspFingertips> finger_filtered_force{};
  std::array<float, kMaxGraspFingertips> finger_force_error{};
  float grasp_target_force{0.0f};
};

static_assert(std::is_trivially_copyable_v<GraspStateData>,
              "GraspStateData must be trivially copyable for SeqLock");

}  // namespace rtc::grasp

#endif  // RTC_CONTROLLERS_GRASP_GRASP_STATE_HPP_
