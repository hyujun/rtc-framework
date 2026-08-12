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

// In-plane pull-force estimate mirror (#167) — trivially copyable float/bool
// mirror of rtc::grasp::PullEstimate (whose Eigen members are not SeqLock-
// compatible). Embedded in the controller-owned state PODs (GraspStateData /
// integrated_bringup WbcStateData); filled via FillPullEstimateData()
// (pull_force_estimator.hpp). Mirrored 1:1 onto the pull_* blocks of the
// rtc_msgs GraspState/WbcState wire messages (#167 Phase-2).
struct PullEstimateData {
  std::array<float, 3> force{};          // filtered F̂, reference frame [N]
  std::array<float, 2> force_inplane{};  // Bᵀ·F̂ plane coordinates [N]
  // Self-description of the plane and its basis (#234 P-5) — force_inplane is
  // meaningless without them, since the basis is recomputed from the observed
  // pinch axis every tick. e_y = plane_normal × basis_x, so the two vectors
  // pin B exactly. Zero ⇒ no plane/basis this tick.
  std::array<float, 3> plane_normal{};  // n̂, reference frame
  std::array<float, 3> basis_x{};       // ê_x, reference frame
  float magnitude{0.0f};                // |F̂| [N]
  float directional{0.0f};              // dᵀ·F̂ (0 unless direction set) [N]
  float friction_utilization{0.0f};     // max_i |f_t,i| / (μ_i f_n,i)
  float leakage_bound{0.0f};            // grip→in-plane leakage bound [N]
  int32_t valid_contact_count{0};
  uint8_t basis_source{0};    // rtc::grasp::PullBasisSource
  uint8_t invalid_reason{0};  // rtc::grasp::PullInvalidReason (0 iff valid)
  uint8_t contact_mask{0};    // bit k = contact k in the force sum
  uint8_t touch_mask{0};      // bit k = contact k pressing (|f_obj| gate)
  bool valid{false};
  bool slip_risk{false};
  bool any_saturated{false};
  bool baseline_applied{false};
};

static_assert(std::is_trivially_copyable_v<PullEstimateData>,
              "PullEstimateData must be trivially copyable for SeqLock PODs");

// Grasp detection state — trivially copyable, SeqLock-compatible.
struct GraspStateData {
  std::array<float, kMaxGraspFingertips> force_magnitude{};
  // contact_flag: backend capability 분기 — sensor A (udp_hand_native +
  // ft_inferencer) → native sigmoid prob [0,1]; sensor B (force-only) →
  // controller-derived binary 1.0F/0.0F. See rtc_msgs/GraspState.msg.
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
  // GraspController::FingerState::K_contact_est mirror (#424) — the estimator
  // had no consumer at all for six months, which is why a latch-order bug kept
  // it pinned at its seed without a single test noticing (#425). 0.0 means not
  // computed this tick (E-STOP / non-Force-PI), never a real estimate.
  std::array<float, kMaxGraspFingertips> finger_stiffness_est{};
  float grasp_target_force{0.0f};

  // In-plane pull-force estimate (#167) — mirrored onto the pull_* block of
  // rtc_msgs/GraspState by the owned-topics publish path.
  PullEstimateData pull{};
};

static_assert(std::is_trivially_copyable_v<GraspStateData>,
              "GraspStateData must be trivially copyable for SeqLock");

}  // namespace rtc::grasp

#endif  // RTC_CONTROLLERS_GRASP_GRASP_STATE_HPP_
