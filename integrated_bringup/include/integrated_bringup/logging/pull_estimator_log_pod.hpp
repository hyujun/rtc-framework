#ifndef INTEGRATED_BRINGUP_LOGGING_PULL_ESTIMATOR_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_PULL_ESTIMATOR_LOG_POD_HPP_

// Per-controller in-plane pull-force estimator log POD (#167 Phase-2). One
// per-tick row (`pull_estimator.csv`) shared by the three demo controllers
// (joint / task / wbc) — the estimator is a single shared component
// (PullEstimatorWiring), so the channel shape is controller-independent.
//
// Filled from rtc::grasp::PullEstimate (the Eigen-typed estimator output),
// NOT the SeqLock mirror PullEstimateData: the CSV additionally carries
// force_raw (pre-filter) for filter transient analysis, which the wire/POD
// surface deliberately omits. Path A: no rtc_msgs/.msg. YAML `msg_type` id
// is "integrated_bringup/PullEstimatorLog".

#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <array>
#include <cstdint>
#include <ostream>
#include <type_traits>

namespace integrated_bringup {

struct PullEstimatorLogPod {
  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // ── Estimate (reference frame, [N]) ───────────────────────────────────────
  std::array<float, 3> force_raw{};       // pre-filter -P∥(Σ f_obj + m·g)
  std::array<float, 3> force_filtered{};  // low-passed; matches msg pull_force
  std::array<float, 2> force_inplane{};   // Bᵀ·F̂ plane coordinates
  float magnitude{0.0f};                  // |F̂|
  float directional{0.0f};                // dᵀ·F̂ (0 unless direction set)

  // ── Diagnostics ───────────────────────────────────────────────────────────
  float friction_utilization{0.0f};  // max_i |f_t,i| / (μ_i f_n,i + ε)
  float leakage_bound{0.0f};         // grip→in-plane leakage bound [N]
  std::int32_t valid_contact_count{0};
  bool valid{false};
  bool slip_risk{false};
  bool any_saturated{false};
  bool baseline_applied{false};
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
}

/// Emit one row. The logger appends '\n' + flush.
inline void WritePullEstimatorLogRow(std::ostream& os, const PullEstimatorLogPod& p) {
  os << p.t_relative_s;
  os << ',' << p.force_raw[0] << ',' << p.force_raw[1] << ',' << p.force_raw[2];
  os << ',' << p.force_filtered[0] << ',' << p.force_filtered[1] << ',' << p.force_filtered[2];
  os << ',' << p.force_inplane[0] << ',' << p.force_inplane[1];
  os << ',' << p.magnitude << ',' << p.directional;
  os << ',' << p.friction_utilization << ',' << p.leakage_bound;
  os << ',' << p.valid_contact_count;
  os << ',' << (p.valid ? 1 : 0);
  os << ',' << (p.slip_risk ? 1 : 0);
  os << ',' << (p.any_saturated ? 1 : 0);
  os << ',' << (p.baseline_applied ? 1 : 0);
}

/// Mirror one PullEstimate tick into the log POD (RT tick path — noexcept,
/// heap-free). `t_relative_s` is the session-relative CM timestamp
/// (ControllerState::t_relative_s), per the ThreadCsvProducer-family
/// first-column convention.
inline void FillPullEstimatorLogPod(const rtc::grasp::PullEstimate& est, double t_relative_s,
                                    PullEstimatorLogPod& pod) noexcept {
  pod.t_relative_s = t_relative_s;
  for (int i = 0; i < 3; ++i) {
    pod.force_raw[static_cast<std::size_t>(i)] = static_cast<float>(est.force_raw[i]);
    pod.force_filtered[static_cast<std::size_t>(i)] = static_cast<float>(est.force_filtered[i]);
  }
  pod.force_inplane[0] = static_cast<float>(est.force_inplane[0]);
  pod.force_inplane[1] = static_cast<float>(est.force_inplane[1]);
  pod.magnitude = static_cast<float>(est.magnitude);
  pod.directional = static_cast<float>(est.directional);
  pod.friction_utilization = static_cast<float>(est.max_friction_utilization);
  pod.leakage_bound = static_cast<float>(est.leakage_bound);
  pod.valid_contact_count = est.valid_contact_count;
  pod.valid = est.valid;
  pod.slip_risk = est.slip_risk;
  pod.any_saturated = est.any_saturated;
  pod.baseline_applied = est.baseline_applied;
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_PULL_ESTIMATOR_LOG_POD_HPP_
