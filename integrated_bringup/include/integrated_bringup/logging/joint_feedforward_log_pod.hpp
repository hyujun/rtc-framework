#ifndef INTEGRATED_BRINGUP_LOGGING_JOINT_FEEDFORWARD_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_JOINT_FEEDFORWARD_LOG_POD_HPP_

// Per-tick record of a feedforward-torque joint controller (HandJointController):
// what the model evaluation did this tick, and per joint the measured position,
// the position reference on the wire, the model gravity term and the
// feedforward torque that went out.
//
// One row per tick this controller RUNS, every such tick — an E-STOP or
// unreadable tick still writes its row with `ff_finite = 0` / `readable = 0`
// and the torque columns at the zeros (or the held values) the wire carried.
// `tick` is the controller manager's loop counter, which keeps counting while
// another controller is the active one, so a gap in it is either rows the
// logger dropped or a stretch this controller was not running.
//
// What the judge reads (rtc_tools analyze_hand_gravity_ff): `ff_enabled` to
// split an A/B/A run into segments, and `q_ref_*` − `q_*` for the steady-state
// error per joint (meaningful in mode 0 only — in the damping modes the
// reference IS the measurement; plot_rtc_log shades those stretches). The raw model term `g_*` is
// logged next to `tau_ff_*` so a clamp or a gain below 1 is visible in the file rather than
// inferred.
//
// SPSC constraint: trivially copyable. No rtc_msgs/.msg — this is
// controller-internal. YAML `msg_type` id is
// "integrated_bringup/JointFeedforwardLog".

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

inline constexpr std::string_view kJointFeedforwardLogMsgType =
    "integrated_bringup/JointFeedforwardLog";
/// One feedforward lane per controller: a single fixed instance.
inline constexpr std::string_view kJointFeedforwardLogInstance = "joint_ff";

struct JointFeedforwardLogPod {
  static constexpr std::size_t kMaxJoints = 32;

  // ── Stamps (true of every tick) ───────────────────────────────────────────
  double t_relative_s{0.0};
  std::uint64_t tick{0};

  // ── What the tick did ─────────────────────────────────────────────────────
  bool estop{false};        ///< the measured pose was held and the torque zeroed
  bool readable{false};     ///< the device reported every joint this tick
  bool seeded{false};       ///< the reference has been seeded since activation
  bool traj_active{false};  ///< a joint goal is still being followed
  bool ff_enabled{false};   ///< the lane was switched on (gravity_ff.enabled)
  /// The controller's mode this tick: 0 tracking, 1 damping (reference =
  /// measurement, torque 0), 2 damping_feedforward (reference = measurement,
  /// lane on). The enum is HandJointMode; the codes are append-only.
  std::uint8_t mode{0};
  /// The torque on the wire came out of a finite model term this tick. False
  /// on E-STOP, before the first valid projection, and when the law retracted
  /// a non-finite term.
  bool ff_finite{false};
  /// The model term was re-evaluated this tick (not a held / last-good one).
  bool model_fresh{false};
  // Closed-chain projection status of this tick (0 on a plain model).
  bool held{false};
  std::uint32_t held_ticks{0};
  bool singular{false};
  double closure_error{0.0};
  /// Bit i: joint i's torque was clamped to its bound this tick.
  std::uint64_t clamped_mask{0};

  // ── Per joint ─────────────────────────────────────────────────────────────
  std::uint8_t num_joints{0};
  std::array<double, kMaxJoints> q{};       ///< measured [rad]
  std::array<double, kMaxJoints> q_ref{};   ///< position target on the wire [rad]
  std::array<double, kMaxJoints> g{};       ///< model gravity term, device order [N·m]
  std::array<double, kMaxJoints> tau_ff{};  ///< feedforward on the wire [N·m]
};

static_assert(std::is_trivially_copyable_v<JointFeedforwardLogPod>,
              "JointFeedforwardLogPod must be trivially copyable for SPSC ring");

/// Column geometry, derived ONCE from the configured names and captured by
/// both writers: the header is written before any pod exists, so a row sized
/// from the pod's runtime width would stop lining up with it (#440).
struct JointFeedforwardLogColumns {
  std::size_t joints{0};
};

[[nodiscard]] inline JointFeedforwardLogColumns JointFeedforwardLogColumnsFor(
    const std::vector<std::string>& joint_names) {
  return {std::min(joint_names.size(), JointFeedforwardLogPod::kMaxJoints)};
}

/// Emit the CSV header. The logger appends '\n'.
inline void WriteJointFeedforwardLogHeader(std::ostream& os,
                                           const std::vector<std::string>& joint_names,
                                           const JointFeedforwardLogColumns& cols) {
  os << "t_relative_s,tick,estop,readable,seeded,traj_active,ff_enabled,mode,ff_finite,"
        "model_fresh,held,held_ticks,singular,closure_error,clamped_mask,num_joints";
  for (const char* prefix : {"q_", "q_ref_", "g_", "tau_ff_"}) {
    for (std::size_t i = 0; i < cols.joints; ++i) {
      os << ',' << prefix << joint_names[i];
    }
  }
}

/// Emit one row at the SAME width as the header; joints the device did not
/// report this tick carry 0 and `num_joints` says how many were real. The
/// logger appends '\n' + flush.
inline void WriteJointFeedforwardLogRow(std::ostream& os, const JointFeedforwardLogPod& p,
                                        const JointFeedforwardLogColumns& cols) {
  os << p.t_relative_s << ',' << p.tick << ',' << (p.estop ? 1 : 0) << ',' << (p.readable ? 1 : 0)
     << ',' << (p.seeded ? 1 : 0) << ',' << (p.traj_active ? 1 : 0) << ',' << (p.ff_enabled ? 1 : 0)
     << ',' << static_cast<unsigned>(p.mode) << ',' << (p.ff_finite ? 1 : 0) << ','
     << (p.model_fresh ? 1 : 0) << ',' << (p.held ? 1 : 0) << ',' << p.held_ticks << ','
     << (p.singular ? 1 : 0) << ',' << p.closure_error << ',' << p.clamped_mask << ','
     << static_cast<unsigned>(p.num_joints);
  const std::size_t reported =
      std::min(static_cast<std::size_t>(p.num_joints), JointFeedforwardLogPod::kMaxJoints);
  auto emit = [&](const std::array<double, JointFeedforwardLogPod::kMaxJoints>& a) {
    for (std::size_t i = 0; i < cols.joints; ++i) {
      os << ',' << (i < reported ? a[i] : 0.0);
    }
  };
  emit(p.q);
  emit(p.q_ref);
  emit(p.g);
  emit(p.tau_ff);
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_JOINT_FEEDFORWARD_LOG_POD_HPP_
