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

  // Pre-filter -P∥(Σ f_obj + m·g) [N] — CSV-only surface (the wire/SeqLock
  // mirror deliberately omits the raw series; it exists here for filter
  // transient analysis).
  std::array<float, 3> force_raw{};

  // Filtered estimate + slip diagnostics + validity gates — byte-identical to
  // the SeqLock/wire mirror, filled via rtc::grasp::FillPullEstimateData so the
  // double→float casts live in exactly one place.
  rtc::grasp::PullEstimateData estimate{};

  // Observed grasp shape (CSV-only, stamped by PushPullEstimatorLog from the
  // wiring — the estimator core is told the axis, not how it was chosen).
  // Bit k = contact k opposed the thumb this tick, so thumb+index vs
  // thumb+middle vs tripod is recoverable post-hoc; a normal that jumps
  // between grasp shapes is otherwise indistinguishable from a force step.
  std::uint8_t opposing_mask{0};
  // The opposing set was empty (no contact latched yet) and the provisional
  // all-tips bootstrap axis was used — samples here are not axis-trustworthy.
  bool opposing_fallback{false};
};

static_assert(std::is_trivially_copyable_v<PullEstimatorLogPod>,
              "PullEstimatorLogPod must be trivially copyable for SPSC ring");

/// Emit the CSV header (fixed shape — no per-instance context). The logger
/// appends '\n'.
inline void WritePullEstimatorLogHeader(std::ostream& os) {
  os << "t_relative_s";
  os << ",force_raw_x,force_raw_y,force_raw_z";
  os << ",force_x,force_y,force_z";
  os << ",inplane_x,inplane_y";
  os << ",magnitude,directional";
  os << ",friction_utilization,leakage_bound";
  os << ",valid_contact_count,valid,slip_risk,any_saturated,baseline_applied";
  os << ",opposing_mask,opposing_fallback";
}

/// Emit one row. The logger appends '\n' + flush.
inline void WritePullEstimatorLogRow(std::ostream& os, const PullEstimatorLogPod& p) {
  const auto& e = p.estimate;
  os << p.t_relative_s;
  os << ',' << p.force_raw[0] << ',' << p.force_raw[1] << ',' << p.force_raw[2];
  os << ',' << e.force[0] << ',' << e.force[1] << ',' << e.force[2];
  os << ',' << e.force_inplane[0] << ',' << e.force_inplane[1];
  os << ',' << e.magnitude << ',' << e.directional;
  os << ',' << e.friction_utilization << ',' << e.leakage_bound;
  os << ',' << e.valid_contact_count;
  os << ',' << (e.valid ? 1 : 0);
  os << ',' << (e.slip_risk ? 1 : 0);
  os << ',' << (e.any_saturated ? 1 : 0);
  os << ',' << (e.baseline_applied ? 1 : 0);
  os << ',' << static_cast<unsigned>(p.opposing_mask);
  os << ',' << (p.opposing_fallback ? 1 : 0);
}

/// Mirror one PullEstimate tick into the log POD (RT tick path — noexcept,
/// heap-free). `t_relative_s` is the session-relative CM timestamp
/// (ControllerState::t_relative_s), per the ThreadCsvProducer-family
/// first-column convention.
inline void FillPullEstimatorLogPod(const rtc::grasp::PullEstimate& est, double t_relative_s,
                                    PullEstimatorLogPod& pod) noexcept {
  pod.t_relative_s = t_relative_s;
  pod.force_raw = {static_cast<float>(est.force_raw.x()), static_cast<float>(est.force_raw.y()),
                   static_cast<float>(est.force_raw.z())};
  // Shared filtered-block + diagnostics + gates — same casts as the wire/SeqLock
  // mirror, so reuse the single source of truth (force_raw stays CSV-only).
  rtc::grasp::FillPullEstimateData(est, pod.estimate);
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_PULL_ESTIMATOR_LOG_POD_HPP_
