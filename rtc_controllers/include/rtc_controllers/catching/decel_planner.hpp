// ── Decel planner: the stop segment, pre-computed on the planner thread ──────
// (dynamic_catching MPC plan E1-F03, #629; decisions MD-24 – MD-33)
//
// What PlannerCycle runs in COMMITTED / CLOSING / DECEL once a decel MPC is
// configured: predict the RT's reference state at the effective instant, solve
// the stop problem (decel_mpc.hpp), and hand back a DecelPlanSnapshot for the
// cycle to publish. ROS-free, so the whole decision is testable against plain
// values; the cycle owns the SeqLock, the re-check and the counter.
//
// ── The grid and the stop end (MD-10, MD-31) ──────────────────────────────────
// Nodes sit on t_c + k·Δ_s, t_c the catch instant of the plan the RT FOLLOWS
// (PlannerRtState::plan_t_c_ns — not the planner's own last publish). The stop
// ends at t_c + N_s·Δ_s whatever k is: replan instance k solves N_s − k nodes
// with its own DecelMpc, built at configure time (the core's N is fixed at
// Init, and re-solving N_s nodes from a later t_eff would push the end out on
// every replan). Replans happen only for k ≤ k_max.
//
// ── One wake ──────────────────────────────────────────────────────────────────
//  1. No followed plan / unseeded command / wrong joint count → kNoState.
//  2. Before anything was published for this plan, wait until
//     t_c − now_lead ≤ t_pre (MD-26) → kNotDue.
//  3. t_eff = the first grid point ≥ now_lead + budget + 2·control_dt, k ≥ 0.
//     k > k_max → kPastReplanWindow; k ≤ the published k0 → kUpToDate (at
//     most one solve per grid point).
//  4. x₀ = the RT reference at t_eff (MD-28): (i) the planner's own latest
//     segment evaluated at t_eff when the RT reports following it; else (ii)
//     the RT's reported command (q, q̇) — reported at rt_state_ns + T_arm on
//     the lead axis — extrapolated to t_eff with q̈ from the planner's own
//     wake-to-wake difference of q̇ (capped by the acceleration box; 0 when it
//     cannot be trusted). q̇ is projected into the core's box (x0_clamped).
//  5. Solve instance k. The reference is the latest published segment
//     shifted by whole grid points (a column copy — the end is fixed, so no
//     padding) when one exists; a trust-region or rest refusal retries once
//     cold. No reference → cold (pre-solve + solve).
//  6. Publishable only if solved, within budget (measured from the start of
//     THIS decel step, not from the wake), before t_eff on the lead axis, and
//     slack_max / slack_terminal_max finite and within their thresholds
//     (MD-33) — written as positive comparisons, so a NaN fails.
//
// ── Contracts ─────────────────────────────────────────────────────────────────
//  • Configure() is non-RT: it Inits k_max + 1 cores (armature ZERO — MD-25:
//    armature is not used in control) and sizes every buffer.
//  • Plan() is noexcept, logs nothing, throws nothing, and allocates nothing
//    outside ProxQP (MD-22 / MD-23 — ProxQP's own mallocs are #654).
//  • Joint order: the RT speaks DEVICE order (PlannerRtState, the payload);
//    the cores speak the model's pinocchio velocity order. The mapping is
//    `device_of_model`, as for the search.
#pragma once

#include "rtc_controllers/catching/decel_mpc.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rtc::catching {

/// Largest age of the RT's report (steady now − rt_state_ns) the decel step
/// extrapolates from. The RT stores every tick, so anything older means the
/// tick stalled; 50 ms is one default planner wake timeout, 25 ticks at 2 ms.
inline constexpr std::int64_t kDecelMaxRtStateAgeNs = 50'000'000;

/// What one decel step did (the planner events CSV's decel columns).
enum class DecelOutcome : std::uint8_t {
  kOff = 0,  ///< not attempted: not configured, or not a decel mode
  kNoState,  ///< no followed plan / t_c, unseeded command, or a size mismatch
  /// The RT's report is older than kDecelMaxRtStateAgeNs (or from the
  /// future): extrapolating it to t_eff would be a guess (an RT stall).
  kStaleState,
  kNotDue,            ///< first solve waits for t_c − now_lead ≤ t_pre (MD-26)
  kUpToDate,          ///< the published segment already starts at this t_eff or later
  kPastReplanWindow,  ///< t_eff beyond t_c + k_max·Δ_s (MD-31)
  kInputNonFinite,    ///< the predicted x₀ is not finite
  kSolveFailed,       ///< the core refused or the QP failed (core_reason)
  kBudget,            ///< solved after budget_s
  kLate,              ///< t_eff passed while solving
  kSlack,             ///< slack non-finite or over its threshold (MD-33)
  kReady,             ///< publishable; the cycle's re-check decides
  kPublished,         ///< stored (set by the cycle)
  kSuperseded,        ///< the trial or the followed plan moved during the solve (cycle)
};

[[nodiscard]] const char* DecelOutcomeName(DecelOutcome o) noexcept;

struct DecelRecord {
  DecelOutcome outcome{DecelOutcome::kOff};
  DecelMpcReason core_reason{DecelMpcReason::kNone};
  std::int32_t k{-1};  ///< grid index of node 0 (t_eff = t_c + k·Δ_s)
  std::int32_t n_nodes{0};
  std::uint32_t decel_seq{0};  ///< the published segment's seq (cycle)
  /// Extrapolation span t_eff − (rt_state_ns + T_arm) [s] — the prediction's
  /// weak point (MD-28), recorded on every solve.
  double h_s{std::numeric_limits<double>::quiet_NaN()};
  bool qdd_trusted{false};   ///< the wake-to-wake q̈ estimate was used
  bool x0_clamped{false};    ///< q̇(t_eff) was projected into the box
  bool from_segment{false};  ///< x₀ came from the planner's own segment (path (i))
  bool presolved{false};     ///< cold: kinematic pre-solve + solve
  bool cold_retry{false};    ///< the shifted reference was refused, re-solved cold
  std::int32_t iterations{0};
  std::int32_t qp_status{-1};
  std::int64_t solve_ns{0};    ///< decel step start → solve end (the budget's measure)
  std::int64_t publish_ns{0};  ///< the stored segment's stamp (cycle); 0 when not stored
  double slack_max{std::numeric_limits<double>::quiet_NaN()};
  double slack_terminal_max{std::numeric_limits<double>::quiet_NaN()};
  double tau_ratio_max{std::numeric_limits<double>::quiet_NaN()};
};

/// The arm the decel planner plans for (configure time, from the binding).
/// Every array is MODEL order, `nv` entries.
struct DecelPlannerModel {
  std::shared_ptr<const pinocchio::Model> arm;  ///< hand-locked catch sub-model
  pinocchio::FrameIndex catch_frame{0};
  int nv{0};
  std::array<int, kMaxPlanNv> device_of_model{};
  std::array<double, kMaxPlanNv> q_min{};      ///< [rad]
  std::array<double, kMaxPlanNv> q_max{};      ///< [rad]
  std::array<double, kMaxPlanNv> qdot_max{};   ///< [rad/s], device ratings
  std::array<double, kMaxPlanNv> tau_max{};    ///< [N·m], device ratings
  std::array<double, kMaxPlanNv> qddot_cap{};  ///< [rad/s²], the q̈-estimate cap; 0 = no cap
  bool qddot_cap_valid{false};                 ///< false: q̈ is never extrapolated
};

struct DecelPlannerConstants {
  double eta_v{0.9};         ///< `planner.gamma.eta_v` (the core's velocity row)
  double t_arm_s{0.0};       ///< `joint_cmd.lag.T_arm` — real → lead axis
  double control_dt{0.002};  ///< the RT period [s]
  double budget_s{0.020};    ///< `planner.budget_s`
};

class DecelPlanner {
 public:
  using ClockFn = std::int64_t (*)() noexcept;

  DecelPlanner() = default;

  /// @brief Build the k_max + 1 cores (non-RT). On failure the planner is
  ///        unconfigured and `error` (if given) names the cause.
  [[nodiscard]] bool Configure(const DecelPlannerModel& model, const DecelPlannerConstants& consts,
                               const DecelPlannerParams& params, ClockFn clock,
                               std::string* error = nullptr);

  [[nodiscard]] bool Configured() const noexcept { return configured_; }

  /// Replace the clock (tests pin it; the cycle forwards its own).
  void SetClock(ClockFn clock) noexcept {
    if (clock != nullptr) {
      clock_ = clock;
    }
  }

  /// Drop the per-trial state (a trial reset, a new followed plan).
  void ResetTrial() noexcept;

  /// @brief One decel step (RT-safe). @return true when `out` holds a
  ///        publishable segment (rec.outcome == kReady); `out`'s publish_ns
  ///        and decel_seq are the caller's to fill.
  [[nodiscard]] bool Plan(const PlannerRtState& rt, DecelPlanSnapshot& out,
                          DecelRecord& rec) noexcept;

  /// The cycle stored `p` (its decel_seq filled): the next replan shifts it.
  void NotePublished(const DecelPlanSnapshot& p) noexcept;

  [[nodiscard]] const DecelPlannerParams& Params() const noexcept { return params_; }

  [[nodiscard]] int Nv() const noexcept { return nv_; }

  /// The core behind replan instance k (tests and diagnostics).
  [[nodiscard]] const DecelMpc& Core(int k) const noexcept {
    return *cores_[static_cast<std::size_t>(k)];
  }

 private:
  void UpdateAccelEstimate(const PlannerRtState& rt) noexcept;
  [[nodiscard]] bool PredictX0(const PlannerRtState& rt, std::int64_t t_eff_ns,
                               DecelRecord& rec) noexcept;
  void ShiftReference(int k, DecelMpcInput& in) noexcept;
  void Pack(const PlannerRtState& rt, int k, std::int64_t t_eff_ns, const DecelMpcResult& r,
            DecelPlanSnapshot& out) const noexcept;

  bool configured_{false};
  DecelPlannerParams params_{};
  ClockFn clock_{nullptr};
  int nv_{0};
  std::array<int, kMaxPlanNv> device_of_model_{};
  std::array<double, kMaxPlanNv> v_box_{};  // η_v · q̇_max, model order
  std::array<double, kMaxPlanNv> qdd_cap_{};
  bool qdd_cap_valid_{false};
  std::int64_t dt_ns_{0};
  std::int64_t t_pre_ns_{0};
  std::int64_t t_arm_ns_{0};
  std::int64_t lead_margin_ns_{0};  // budget + 2·control_dt
  std::int64_t budget_ns_{0};

  // k = 0..k_max. Behind pointers: a core owns a model copy and a solver and
  // is not meant to move.
  std::vector<std::unique_ptr<DecelMpc>> cores_;
  std::vector<DecelMpcInput> inputs_;  // sized per instance
  std::vector<DecelMpcResult> results_;

  // Per trial (ResetTrial / a new followed plan).
  std::uint32_t trial_plan_id_{0};
  std::int64_t trial_t_c_ns_{0};
  bool trial_open_{false};
  bool have_published_{false};
  DecelPlanSnapshot last_{};
  // q̈ estimate: the previous wake's reported q̇ (device order) and its instant.
  bool prev_valid_{false};
  std::int64_t prev_rt_ns_{0};
  std::array<double, kMaxPlanNv> prev_qd_{};
  std::array<double, kMaxPlanNv> qdd_est_{};  // device order
  bool qdd_est_valid_{false};
};

}  // namespace rtc::catching
