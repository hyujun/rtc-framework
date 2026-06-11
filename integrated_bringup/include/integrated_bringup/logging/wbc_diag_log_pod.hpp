#ifndef INTEGRATED_BRINGUP_LOGGING_WBC_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_WBC_DIAG_LOG_POD_HPP_

// Per-controller TSID/QP diagnostics log POD for the WBC controller. One
// solve services both arm + hand, so this is a single per-tick row (one
// `wbc_diag.csv`), NOT per-device. Carries solver health + the QP contact
// wrench solution that the per-device state logs cannot hold.
//
// Fields source (controller-private):
//   - phase / num_active_contacts / grasp_detected / max_force / qp_fail_count
//       ← WbcStateData staging buffer
//   - solve_time_us / solve_levels / qp_converged / lambda_opt
//       ← rtc::tsid::CommandOutput (tsid_output_)
//
// SPSC constraint: trivially copyable. lambda_opt is fixed-dim
// (Stage A-5a QP always emits max_contact_vars); num_contact_vars carries
// the runtime active dim for the writer. Path A: no rtc_msgs/.msg. YAML
// `msg_type` id is "integrated_bringup/WbcDiagLog".

#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string_view>
#include <type_traits>

namespace integrated_bringup {

struct WbcDiagLogPod {
  // contact vars cap: 8 fingertips × 6 (surface contact) headroom.
  static constexpr std::size_t kMaxContactVars = 48;

  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // ── Sizes ─────────────────────────────────────────────────────────────────
  std::uint8_t phase{0};            // WbcPhase enum (see DemoWbcController)
  std::uint8_t num_contact_vars{0}; // active λ dim (≤ kMaxContactVars)

  // ── Solver diagnostics ────────────────────────────────────────────────────
  double solve_time_us{0.0};
  std::int32_t solve_levels{0};
  std::int32_t qp_fail_count{0};
  std::int32_t num_active_contacts{0};
  bool qp_converged{false};
  bool grasp_detected{false};
  float max_force{0.0f};  // max fingertip |F| [N]

  // ── Stage C-2: low-level command source + CLIK A/B shadow ────────────────
  // command_source = the path that actually drove this tick (clik falls back
  // to integrator on CLIK Compute() failure). clik_* are the driving/shadow
  // CLIK diagnostics; shadow_* are the per-tick Δ between the two command
  // candidates (q_clik − q_integrator), valid only when shadow_valid.
  std::uint8_t command_source{0};   // 0=integrator, 1=clik (active this tick)
  bool clik_valid{false};           // clik_.Compute() succeeded this tick
  bool shadow_valid{false};         // both paths computed → Δ meaningful
  double clik_tcp_err{0.0};         // ‖e_x‖ [m+rad]
  double clik_manipulability{0.0};  // √det(JJᵀ+μ²I)
  double shadow_pos_delta{0.0};     // ‖q_clik − q_integrator‖
  double shadow_vel_delta{0.0};     // ‖v_clik − v_integrator‖

  // ── QP contact solution (wrench / λ per contact var) ─────────────────────
  std::array<double, kMaxContactVars> lambda_opt{};
};

static_assert(std::is_trivially_copyable_v<WbcDiagLogPod>,
              "WbcDiagLogPod must be trivially copyable for SPSC ring");

/// Translate the WbcPhase enum value to its name for the CSV. Values mirror
/// integrated_bringup::WbcPhase (kept standalone so this header has no
/// controller dependency; slot 5 is reserved/deprecated).
inline std::string_view WbcDiagLogPhaseStr(std::uint8_t v) noexcept {
  switch (v) {
    case 0: return "idle";
    case 1: return "approach";
    case 2: return "pre_grasp";
    case 3: return "closure";
    case 4: return "hold";
    case 6: return "release";
    case 7: return "fallback";
    default: return "unknown";
  }
}

/// Stage C-2: translate the CommandSource enum value to its name for the CSV
/// (values mirror integrated_bringup::CommandSource; kept standalone so this
/// header has no controller dependency).
inline std::string_view WbcDiagLogCommandSourceStr(std::uint8_t v) noexcept {
  switch (v) {
    case 0: return "integrator";
    case 1: return "clik";
    default: return "unknown";
  }
}

/// Emit the CSV header. `num_contact_vars` sets the λ column expansion
/// (captured once at registration = contact_mgr_config_.max_contact_vars,
/// the fixed QP dim). The logger appends '\n'.
inline void WriteWbcDiagLogHeader(std::ostream& os, std::size_t num_contact_vars) {
  os << "t_relative_s,phase";
  os << ",solve_time_us,qp_converged,solve_levels,qp_fail_count";
  os << ",num_active_contacts,grasp_detected,max_force";
  // Stage C-2: command source + CLIK A/B shadow columns.
  os << ",command_source,clik_valid,clik_tcp_err,clik_manipulability";
  os << ",shadow_valid,shadow_pos_delta,shadow_vel_delta";
  for (std::size_t i = 0; i < num_contact_vars; ++i) {
    os << ",lambda_" << i;
  }
}

/// Emit one row. λ loop is bounded by p.num_contact_vars (must equal the
/// count passed to the header writer). The logger appends '\n' + flush.
inline void WriteWbcDiagLogRow(std::ostream& os, const WbcDiagLogPod& p) {
  os << p.t_relative_s;
  os << ',' << WbcDiagLogPhaseStr(p.phase);
  os << ',' << p.solve_time_us;
  os << ',' << (p.qp_converged ? 1 : 0);
  os << ',' << p.solve_levels;
  os << ',' << p.qp_fail_count;
  os << ',' << p.num_active_contacts;
  os << ',' << (p.grasp_detected ? 1 : 0);
  os << ',' << p.max_force;
  // Stage C-2: command source + CLIK A/B shadow.
  os << ',' << WbcDiagLogCommandSourceStr(p.command_source);
  os << ',' << (p.clik_valid ? 1 : 0);
  os << ',' << p.clik_tcp_err;
  os << ',' << p.clik_manipulability;
  os << ',' << (p.shadow_valid ? 1 : 0);
  os << ',' << p.shadow_pos_delta;
  os << ',' << p.shadow_vel_delta;
  for (std::size_t i = 0; i < p.num_contact_vars; ++i) {
    os << ',' << p.lambda_opt[i];
  }
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_WBC_DIAG_LOG_POD_HPP_
