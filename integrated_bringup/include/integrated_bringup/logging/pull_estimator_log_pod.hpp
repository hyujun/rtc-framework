#ifndef INTEGRATED_BRINGUP_LOGGING_PULL_ESTIMATOR_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_PULL_ESTIMATOR_LOG_POD_HPP_

// Per-controller in-plane pull-force estimator log POD (#167 Phase-2). One
// per-tick row (`pull_estimator.csv`) shared by the three demo controllers
// (joint / task / wbc) — the estimator is a single shared component
// (PullEstimatorWiring), so the channel shape is controller-independent.
//
// Filled from rtc::grasp::PullEstimate (the Eigen-typed estimator output).
// The shared filtered-block + diagnostics + gates are stored in an embedded
// rtc::grasp::PullEstimateData (filled via the single FillPullEstimateData);
// the CSV additionally carries force_raw (pre-filter) for filter transient
// analysis, which the wire/SeqLock surface deliberately omits. Path A: no
// rtc_msgs/.msg for this channel. YAML `msg_type` id is kPullEstimatorLogMsgType.

#include "rtc_controllers/grasp/grasp_state.hpp"           // PullEstimateData
#include "rtc_controllers/grasp/pull_force_estimator.hpp"  // PullEstimate, FillPullEstimateData

#include <array>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>

namespace integrated_bringup {

// Closed-set YAML `msg_type` id and the fixed instance key for this channel —
// referenced by the LoadConfig validators, the log registration dispatch, and
// the lifecycle wiring so the string lives in exactly one place (#167).
inline constexpr std::string_view kPullEstimatorLogMsgType = "integrated_bringup/PullEstimatorLog";
inline constexpr std::string_view kPullEstimatorLogInstance = "pull_estimator";

struct PullEstimatorLogPod {
  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // CM RT-loop tick index (ControllerState::iteration). Rows are pushed one
  // per tick, so a jump here is a *dropped* row (SPSC ring overflow) rather
  // than a skipped tick — the only remaining way a pull row can go missing now
  // that E-STOP ticks log valid=0 (#234 P-20, carried over from #235). The
  // third cause, a disabled wiring, produces no file at all.
  std::uint64_t tick{0};

  // Pre-filter estimate [N] — CSV-only surface (the wire/SeqLock mirror
  // deliberately omits this series; it exists here for filter transient
  // analysis). Named `force_prefilter_*` in the CSV rather than `force_raw_*`
  // (#234 P-13): this is -P∥(Σ f_obj + m·g) *after* projection, gravity and
  // baseline subtraction, so the old name read as a raw sensor sum and made
  // the baseline-capture step look like a filter artefact.
  std::array<float, 3> force_prefilter{};

  // Filtered estimate + slip diagnostics + validity gates — byte-identical to
  // the SeqLock/wire mirror, filled via rtc::grasp::FillPullEstimateData so the
  // double→float casts live in exactly one place.
  rtc::grasp::PullEstimateData estimate{};

  // Observed grasp shape (CSV-only, stamped by PushPullEstimatorLog from the
  // wiring — the estimator core is told the axis, not how it was chosen).
  // Bit k = contact k opposed the thumb this tick, so thumb+index vs
  // thumb+middle vs tripod is recoverable post-hoc; a normal that jumps
  // between grasp shapes is otherwise indistinguishable from a force step.
  // Zero means no tip was touching: the pinch axis is then degenerate and the
  // row is already marked invalid, so no separate "provisional axis" flag is
  // needed — the estimator never publishes against a guessed plane.
  std::uint8_t opposing_mask{0};
};

static_assert(std::is_trivially_copyable_v<PullEstimatorLogPod>,
              "PullEstimatorLogPod must be trivially copyable for SPSC ring");

/// Bit-order stamp appended to every mask column name, e.g.
/// "[thumb|index|middle]" — bit k of that column is `roles[k]`.
///
/// The masks are indexed by *contact* index, and the contact→role mapping is a
/// YAML ordering that used to live only in a one-shot startup INFO line: a
/// stored CSV could not be decoded without that run's ROS log (#234 P-14).
/// Stamping the order into the header keeps the file self-describing without a
/// second sidecar artefact to keep in sync, and reordering `tip_names` changes
/// the header rather than silently re-meaning the bits. Empty when the caller
/// has no role list (the loader then falls back to bit indices).
inline std::string PullMaskRoleSuffix(std::span<const std::string> roles) {
  if (roles.empty()) {
    return {};
  }
  std::string out = "[";
  for (std::size_t i = 0; i < roles.size(); ++i) {
    if (i != 0) {
      out += '|';
    }
    out += roles[i];
  }
  out += ']';
  return out;
}

/// Emit the CSV header. `roles` is the estimator's configured tip order and
/// only stamps the mask column names (see PullMaskRoleSuffix); the column set
/// itself is fixed. The logger appends '\n'.
inline void WritePullEstimatorLogHeader(std::ostream& os, std::span<const std::string> roles = {}) {
  const std::string m = PullMaskRoleSuffix(roles);
  os << "t_relative_s,tick";
  os << ",force_prefilter_x,force_prefilter_y,force_prefilter_z";
  os << ",force_x,force_y,force_z";
  os << ",inplane_x,inplane_y";
  os << ",plane_normal_x,plane_normal_y,plane_normal_z";
  os << ",basis_x_x,basis_x_y,basis_x_z";
  os << ",basis_source,invalid_reason";
  os << ",magnitude,directional";
  os << ",friction_utilization,leakage_bound";
  os << ",valid_contact_count,valid,slip_risk,any_saturated,baseline_applied";
  os << ",contact_mask" << m << ",touch_mask" << m << ",opposing_mask" << m;
}

/// Emit one row. The logger appends '\n' + flush.
inline void WritePullEstimatorLogRow(std::ostream& os, const PullEstimatorLogPod& p) {
  const auto& e = p.estimate;
  os << p.t_relative_s;
  os << ',' << p.tick;
  os << ',' << p.force_prefilter[0] << ',' << p.force_prefilter[1] << ',' << p.force_prefilter[2];
  os << ',' << e.force[0] << ',' << e.force[1] << ',' << e.force[2];
  os << ',' << e.force_inplane[0] << ',' << e.force_inplane[1];
  os << ',' << e.plane_normal[0] << ',' << e.plane_normal[1] << ',' << e.plane_normal[2];
  os << ',' << e.basis_x[0] << ',' << e.basis_x[1] << ',' << e.basis_x[2];
  os << ',' << static_cast<unsigned>(e.basis_source);
  os << ',' << static_cast<unsigned>(e.invalid_reason);
  os << ',' << e.magnitude << ',' << e.directional;
  os << ',' << e.friction_utilization << ',' << e.leakage_bound;
  os << ',' << e.valid_contact_count;
  os << ',' << (e.valid ? 1 : 0);
  os << ',' << (e.slip_risk ? 1 : 0);
  os << ',' << (e.any_saturated ? 1 : 0);
  os << ',' << (e.baseline_applied ? 1 : 0);
  os << ',' << static_cast<unsigned>(e.contact_mask);
  os << ',' << static_cast<unsigned>(e.touch_mask);
  os << ',' << static_cast<unsigned>(p.opposing_mask);
}

/// Mirror one PullEstimate tick into the log POD (RT tick path — noexcept,
/// heap-free). `t_relative_s` is the session-relative CM timestamp
/// (ControllerState::t_relative_s), per the ThreadCsvProducer-family
/// first-column convention.
inline void FillPullEstimatorLogPod(const rtc::grasp::PullEstimate& est, double t_relative_s,
                                    std::uint64_t tick, PullEstimatorLogPod& pod) noexcept {
  pod.t_relative_s = t_relative_s;
  pod.tick = tick;
  pod.force_prefilter = {static_cast<float>(est.force_raw.x()),
                         static_cast<float>(est.force_raw.y()),
                         static_cast<float>(est.force_raw.z())};
  // Shared filtered-block + diagnostics + gates — same casts as the wire/SeqLock
  // mirror, so reuse the single source of truth (force_raw stays CSV-only).
  rtc::grasp::FillPullEstimateData(est, pod.estimate);
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_PULL_ESTIMATOR_LOG_POD_HPP_
