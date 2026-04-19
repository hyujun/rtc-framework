#pragma once

/// @file mock_phase_manager.hpp
/// @brief Test-only deterministic FSM implementing
/// `rtc::mpc::PhaseManagerBase`.
///
/// Purpose — exercise Phase 6's `HandlerMPCThread` pipeline end-to-end without
/// dragging any robot-specific FSM into `rtc_mpc/`. The FSM is a minimal
/// two-state machine (Phase A `"free_flight"` → Phase B `"near_object"`) that
/// can auto-advance after a configurable tick count **or** be force-flipped by
/// the test thread via @ref PhaseManagerBase::ForcePhase.
///
/// Scope — **test-only header**, not installed; do not include from production
/// code. Robot topology (contact frames, cost weights, EE target) is fully
/// externally supplied via @ref MockPhaseManager::Params so the manager stays
/// robot-agnostic and the test TU owns all URDF-bound state.
///
/// Thread-safety contract (per Phase 2 `PhaseManagerBase` header) — concrete
/// managers own their synchronisation. `ForcePhase` may run from the test
/// thread while `Update` runs on the MPC thread; only `current_id_` and
/// `forced_phase_id_` are `std::atomic<int>`. The two `PhaseCostConfig` slots
/// (Phase A / Phase B) are **pre-built in the constructor** and stored as
/// `const`, so `Update` reads them without locking. `tick_counter_` is written
/// only from `Update` (MPC thread) and is therefore plain `int`.
///
/// SSO invariants — `phase_name` in the returned `PhaseContext` and `ocp_type`
/// in the dispatch key both rely on libstdc++'s 15-char SSO bucket to stay
/// alloc-free on copy-construction (PhaseContext is value-returned each tick
/// and must not allocate in steady state). The literals used here
/// (`"free_flight"`=11, `"near_object"`=11, `"light_contact"`=13,
/// `"contact_rich"`=12) are all within the SSO limit. A `static_assert` below
/// enforces this so future renames cannot silently break the budget. Phase 7
/// implementers adding longer phase names MUST re-verify this invariant.
///
/// Cross-mode stretch (§Phase 6 Step 6) — when `Params::cross_mode == true`,
/// Phase B switches `ocp_type` to `"contact_rich"`; otherwise both phases stay
/// in `"light_contact"`. Baseline integration tests use `cross_mode == false`
/// so that a single `LightContactMPC` handler can serve both phases without a
/// factory swap.

#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/phase/phase_cost_config.hpp"
#include "rtc_mpc/phase/phase_manager_base.hpp"
#include "rtc_mpc/types/contact_plan_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <string>

namespace rtc::mpc::test_utils {

// SSO budget guard — libstdc++ inlines strings up to 15 chars. Violating this
// would make `PhaseContext` return-by-value allocate once per tick, breaking
// the RT-adjacent alloc audit that Phase 6 Step 5 relies on.
static_assert(sizeof("free_flight") <= 16,
              "phase_name 'free_flight' must fit libstdc++ SSO (<=15 chars)");
static_assert(sizeof("near_object") <= 16,
              "phase_name 'near_object' must fit libstdc++ SSO (<=15 chars)");
static_assert(sizeof("light_contact") <= 16,
              "ocp_type 'light_contact' must fit libstdc++ SSO");
static_assert(sizeof("contact_rich") <= 16,
              "ocp_type 'contact_rich' must fit libstdc++ SSO");

/// @brief Two-state deterministic FSM for Phase 6 integration tests.
class MockPhaseManager final : public PhaseManagerBase {
public:
  /// @brief Injectable configuration for the FSM. All URDF-bound state is
  ///        owned by the caller so the mock itself stays robot-agnostic.
  struct Params {
    /// Pre-built cost configs, one per phase. The mock copies these into
    /// internal `const` slots at construction; test code may free / re-use
    /// the source objects afterwards.
    PhaseCostConfig cost_config_A{};
    PhaseCostConfig cost_config_B{};

    /// Pre-built contact plans, one per phase. Phase A is typically empty
    /// ("free flight"), Phase B names the active contact frames.
    ContactPlan contact_plan_A{};
    ContactPlan contact_plan_B{};

    /// EE SE3 targets per phase; both default to identity → "track current
    /// pose". Test fixtures typically set `ee_target_B` to a 10-cm offset to
    /// drive convergence measurably.
    pinocchio::SE3 ee_target_A{pinocchio::SE3::Identity()};
    pinocchio::SE3 ee_target_B{pinocchio::SE3::Identity()};

    /// Auto-transition after this many `Update` calls (MPC ticks). Negative
    /// values disable auto-transition (Phase A forever until `ForcePhase`).
    int transition_tick{50};

    /// Stretch-test toggle — when true, Phase B switches `ocp_type` to
    /// `"contact_rich"`, forcing a factory handler swap. Baseline integration
    /// tests set this to false so a single `LightContactMPC` handles both.
    bool cross_mode{false};
  };

  /// @brief Construct with a fully-populated @ref Params. Copies the cost
  ///        configs into const slots — the caller may discard @p p after.
  explicit MockPhaseManager(const Params &p) noexcept
      : cost_config_A_(p.cost_config_A), cost_config_B_(p.cost_config_B),
        contact_plan_A_(p.contact_plan_A), contact_plan_B_(p.contact_plan_B),
        ee_target_A_(p.ee_target_A), ee_target_B_(p.ee_target_B),
        transition_tick_(p.transition_tick), cross_mode_(p.cross_mode) {}

  // ── PhaseManagerBase overrides ──────────────────────────────────────────

  /// @brief No-op — this mock takes all config via `Params` at construction.
  /// The YAML node is ignored so tests don't need to fabricate a YAML tree.
  void Init(const YAML::Node & /*cfg*/) override {}

  /// @brief Advance the FSM one tick, return the fresh @ref PhaseContext.
  ///
  /// Transition rules (evaluated in order):
  /// 1. If `ForcePhase(id)` was called since the last `Update`, jump to that
  ///    phase and report `phase_changed == true`.
  /// 2. Else, if the tick counter just reached `transition_tick` (non-negative
  ///    values only) while currently in Phase A, advance to Phase B.
  /// 3. Else, hold the current phase and report `phase_changed == false`.
  PhaseContext Update(const Eigen::VectorXd & /*q*/,
                      const Eigen::VectorXd & /*v*/,
                      const Eigen::VectorXd & /*sensor*/,
                      const pinocchio::SE3 & /*tcp*/, double /*t*/) override {
    const int prev_id = current_id_.load(std::memory_order_relaxed);
    int next_id = prev_id;

    const int forced =
        forced_phase_id_.exchange(kNoForcedPhase, std::memory_order_acq_rel);
    if (forced != kNoForcedPhase) {
      next_id = forced;
    } else {
      ++tick_counter_;
      if (transition_tick_ >= 0 && prev_id == 0 &&
          tick_counter_ >= transition_tick_) {
        next_id = 1;
      }
    }

    const bool changed = (next_id != prev_id);
    if (changed) {
      current_id_.store(next_id, std::memory_order_relaxed);
    }

    return BuildContext(next_id, changed);
  }

  /// @brief No-op — test fixtures poke the FSM via `Params` / `ForcePhase`.
  void SetTaskTarget(const YAML::Node & /*target*/) override {}

  [[nodiscard]] int CurrentPhaseId() const override {
    return current_id_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::string CurrentPhaseName() const override {
    return std::string{NameFor(current_id_.load(std::memory_order_relaxed))};
  }

  /// @brief Queue a phase override. The *next* `Update` picks it up and
  ///        reports `phase_changed == true`. Idempotent if already pending.
  void ForcePhase(int phase_id) override {
    forced_phase_id_.store(phase_id, std::memory_order_release);
  }

  // ── Test-only probes (not part of PhaseManagerBase) ─────────────────────

  [[nodiscard]] int TickCounter() const noexcept { return tick_counter_; }

private:
  // Sentinel matching any non-{0,1} id; `ForcePhase` is assumed to use valid
  // ids, so reserving the most-negative value for "no override pending" is
  // safe in practice.
  static constexpr int kNoForcedPhase = -1;

  [[nodiscard]] static const char *NameFor(int id) noexcept {
    return (id == 1) ? "near_object" : "free_flight";
  }

  [[nodiscard]] PhaseContext BuildContext(int id, bool changed) const {
    PhaseContext ctx{};
    ctx.phase_id = id;
    ctx.phase_name = NameFor(id); // SSO-safe (see static_assert above)
    ctx.phase_changed = changed;
    if (id == 0) {
      ctx.contact_plan = contact_plan_A_;
      ctx.cost_config = cost_config_A_;
      ctx.ee_target = ee_target_A_;
      ctx.ocp_type = "light_contact";
    } else {
      ctx.contact_plan = contact_plan_B_;
      ctx.cost_config = cost_config_B_;
      ctx.ee_target = ee_target_B_;
      ctx.ocp_type = cross_mode_ ? "contact_rich" : "light_contact";
    }
    return ctx;
  }

  // Pre-built, immutable-after-ctor cost / plan / target slots.
  const PhaseCostConfig cost_config_A_;
  const PhaseCostConfig cost_config_B_;
  const ContactPlan contact_plan_A_;
  const ContactPlan contact_plan_B_;
  const pinocchio::SE3 ee_target_A_;
  const pinocchio::SE3 ee_target_B_;
  const int transition_tick_;
  const bool cross_mode_;

  std::atomic<int> current_id_{0};
  std::atomic<int> forced_phase_id_{kNoForcedPhase};
  int tick_counter_{0}; // MPC thread only — no atomics needed.
};

} // namespace rtc::mpc::test_utils
