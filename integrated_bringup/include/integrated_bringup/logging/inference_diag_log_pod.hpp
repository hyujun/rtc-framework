#ifndef INTEGRATED_BRINGUP_LOGGING_INFERENCE_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_INFERENCE_DIAG_LOG_POD_HPP_

// Per-tick diagnostics for the learned-policy controller (demo_inference_
// controller): one `inference_diag.csv` row per tick, the same single-instance
// shape as `task_diag_log_pod.hpp`.
//
// Why this exists. The controller's failure policy is "hold everything", and a
// hold is indistinguishable from a policy that chose to stand still unless the
// REASON is recorded: an unreadable lane, a closed-chain projection still
// walking in, a stale object pose and a failed Run() all command the same
// latched position. The reasons below are exactly the early returns of the
// tick, one code each, so a session answers "why did it hold?" without a
// debugger. The rest of the row is what the policy was shown on its last
// accepted step (reach gate, object pose) and what the robot did with the
// action (arm lag), so a run can be judged from this file plus the device state
// files alone.
//
// Timing columns are deliberately absent: the CM's cm_timing_log already times
// every Compute(), and `iteration` here joins onto it.
//
// SPSC constraint: trivially copyable. Path A: no rtc_msgs/.msg — controller-
// internal state, not device state. YAML `msg_type` id is
// "integrated_bringup/InferenceDiagLog".

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

inline constexpr std::string_view kInferenceDiagLogMsgType = "integrated_bringup/InferenceDiagLog";
inline constexpr std::string_view kInferenceDiagLogInstance = "inference_diag";

/// Why a tick shipped the latched position instead of a policy action. One
/// value per early return of `DemoInferenceController::Compute()`; kNone on a
/// tick that emitted an action (fresh or replayed between policy steps).
enum class InferenceHoldReason : std::uint8_t {
  kNone = 0,
  kNotReady,        ///< hold mode (no model), no engine, or a device group missing
  kUnreadable,      ///< a device's position lane failed the readability gate
  kTensorMismatch,  ///< engine buffers disagree with the declared schema
  kSeed,            ///< a recurrent seed feature was unavailable at reset
  kClosedChain,     ///< closed-chain hand FK held / singular / not converged
  kLinkPose,        ///< any other link pose unavailable (model cache not valid)
  kObject,          ///< object pose missing, ambiguous or stale
  kFeature,         ///< any other observation unavailable (e.g. a velocity lane)
  kRunFailed,       ///< the engine's Run() returned false
  kOutput,          ///< an output head was short or non-finite
};
inline constexpr std::size_t kNumInferenceHoldReasons = 11;

[[nodiscard]] constexpr std::string_view InferenceHoldReasonName(InferenceHoldReason r) noexcept {
  switch (r) {
    case InferenceHoldReason::kNone:
      return "none";
    case InferenceHoldReason::kNotReady:
      return "not_ready";
    case InferenceHoldReason::kUnreadable:
      return "unreadable";
    case InferenceHoldReason::kTensorMismatch:
      return "tensor_mismatch";
    case InferenceHoldReason::kSeed:
      return "seed";
    case InferenceHoldReason::kClosedChain:
      return "closed_chain";
    case InferenceHoldReason::kLinkPose:
      return "link_pose";
    case InferenceHoldReason::kObject:
      return "object";
    case InferenceHoldReason::kFeature:
      return "feature";
    case InferenceHoldReason::kRunFailed:
      return "run_failed";
    case InferenceHoldReason::kOutput:
      return "output";
  }
  return "unknown";
}

struct InferenceDiagLogPod {
  static constexpr std::size_t kMaxTips = 8;
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  // ── Timestamp (CM-provided) ───────────────────────────────────────────────
  double t_relative_s{0.0};
  /// `ControllerState::iteration` — the CM loop counter, which is what joins a
  /// row onto cm_timing_log (Compute() time of the same tick).
  std::uint64_t iteration{0};

  // ── What the tick decided ─────────────────────────────────────────────────
  bool held{false};
  std::uint8_t hold_reason{0};  ///< InferenceHoldReason
  /// An action was produced AND accepted on this tick (a policy step). Ticks
  /// between steps replay it and read 0 here with held = 0.
  bool policy_step{false};
  /// Run() calls since activation, including ones whose output was refused.
  std::uint64_t inference_count{0};

  // ── What the policy was shown on its last ACCEPTED step ───────────────────
  // Latched: rows between steps repeat the step's values, NaN before the first.
  double reach_phase{kNaN};
  double tip_distance{kNaN};  ///< mean fingertip ↔ contact-point distance [m]
  bool reach_hold{false};     ///< tactile hold latch of the reach gate

  // ── Object lane (this tick's snapshot, policy frame) ──────────────────────
  bool object_valid{false};
  double object_age_s{kNaN};
  std::array<double, 3> object_position{kNaN, kNaN, kNaN};

  // ── Closed-chain hand FK (this tick; zeros when the tick did not run it) ──
  bool closed_held{false};
  std::int32_t closed_held_ticks{0};
  bool closed_singular{false};
  double closure_error{0.0};

  // ── Tracking ──────────────────────────────────────────────────────────────
  /// max over arm joints of |policy target − measured| [rad]. NaN on a held
  /// tick, where the target is the latch rather than anything the policy said.
  double arm_lag_max{kNaN};

  // ── Fingertip force the reach gate reads, ‖F‖ per tip [N] ─────────────────
  std::uint8_t num_tips{0};
  std::array<double, kMaxTips> tip_force{};
};

static_assert(std::is_trivially_copyable_v<InferenceDiagLogPod>,
              "InferenceDiagLogPod must be trivially copyable for SPSC ring");

/// Emit the CSV header. `tip_names` label the per-tip force columns (the reach
/// gate's force groups, in its tip order) and fix their count; the row writer
/// must be given the same count. The logger appends '\n'.
inline void WriteInferenceDiagLogHeader(std::ostream& os,
                                        const std::vector<std::string>& tip_names) {
  os << "t_relative_s,iteration,held,hold_code,hold_reason,policy_step,inference_count";
  os << ",reach_phase,tip_distance,reach_hold";
  os << ",object_valid,object_age_s,object_x,object_y,object_z";
  os << ",closed_held,closed_held_ticks,closed_singular,closure_error";
  os << ",arm_lag_max";
  const std::size_t n = std::min(tip_names.size(), InferenceDiagLogPod::kMaxTips);
  for (std::size_t i = 0; i < n; ++i) {
    os << ",force_" << tip_names[i];
  }
}

/// Emit one row with exactly `num_tip_columns` force columns — padded with 0
/// past the POD's own count, so a short row can never shift under the header.
inline void WriteInferenceDiagLogRow(std::ostream& os, const InferenceDiagLogPod& p,
                                     std::size_t num_tip_columns) {
  const auto reason = static_cast<InferenceHoldReason>(p.hold_reason);
  os << p.t_relative_s;
  os << ',' << p.iteration;
  os << ',' << (p.held ? 1 : 0);
  os << ',' << static_cast<int>(p.hold_reason);
  os << ',' << InferenceHoldReasonName(reason);
  os << ',' << (p.policy_step ? 1 : 0);
  os << ',' << p.inference_count;
  os << ',' << p.reach_phase;
  os << ',' << p.tip_distance;
  os << ',' << (p.reach_hold ? 1 : 0);
  os << ',' << (p.object_valid ? 1 : 0);
  os << ',' << p.object_age_s;
  os << ',' << p.object_position[0] << ',' << p.object_position[1] << ',' << p.object_position[2];
  os << ',' << (p.closed_held ? 1 : 0);
  os << ',' << p.closed_held_ticks;
  os << ',' << (p.closed_singular ? 1 : 0);
  os << ',' << p.closure_error;
  os << ',' << p.arm_lag_max;
  const std::size_t cols = std::min(num_tip_columns, InferenceDiagLogPod::kMaxTips);
  for (std::size_t i = 0; i < cols; ++i) {
    os << ',' << (i < p.num_tips ? p.tip_force[i] : 0.0);
  }
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_INFERENCE_DIAG_LOG_POD_HPP_
