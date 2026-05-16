#ifndef INTEGRATED_BRINGUP_CONTROLLERS_WBC_WBC_STATE_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_WBC_WBC_STATE_HPP_

// WBC state — produced by TSID-based whole-body controllers in
// integrated_bringup. Parallel role to rtc::grasp::GraspStateData but
// reflects WBC's TSID-based grasp algorithm (no Force-PI fields).
// Consumers (BT coordinator / GUI) pick between grasp_state and wbc_state
// based on the active controller name.
//
// Lives in integrated_bringup (not rtc_base) because WBC is a demo-domain
// composition — TSID + per-fingertip aggregates + grasp-style FSM —
// specific to the integrated demo controllers, not a primitive of the
// robot-agnostic framework.

#include <array>
#include <cstdint>
#include <type_traits>

namespace integrated_bringup {

inline constexpr int kMaxWbcFingertips = 8;

// WBC state — RT-safe POD, SeqLock-compatible.
struct WbcStateData {
  uint8_t phase{0};                                        // WbcPhase enum
  std::array<float, kMaxWbcFingertips> force_magnitude{};  // |F| per fingertip [N]
  std::array<float, kMaxWbcFingertips> contact_flag{};     // contact probability [0,1]
  std::array<float, kMaxWbcFingertips> displacement{};     // raw displacement [m]
  int num_fingertips{0};
  int num_active_contacts{0};
  float max_force{0.0f};           // max across fingertips [N]
  float grasp_target_force{0.0f};  // active target force [N]
  bool grasp_detected{false};
  int min_fingertips_for_grasp{2};
  // TSID solver diagnostics (informational — not safety-critical)
  float tsid_solve_us{0.0f};
  bool tsid_solver_ok{true};
  int qp_fail_count{0};
};

static_assert(std::is_trivially_copyable_v<WbcStateData>,
              "WbcStateData must be trivially copyable for SeqLock");

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_WBC_WBC_STATE_HPP_
