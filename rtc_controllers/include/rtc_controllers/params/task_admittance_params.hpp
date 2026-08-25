// ── Task-space admittance controller configuration (G2 schema layer) ────────
// The gains POD and the YAML schema that fills it, lifted out of
// TaskAdmittanceController so both outlive the adapter (#236 S7c-2, D-B/G2).
// See params/joint_pd_params.hpp for why this layer exists, and
// params/task_impedance_params.hpp for why `sensor_frame` leaves here as a
// STRING rather than a resolved pinocchio frame index.
#pragma once

#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/wrench_conditioning.hpp"
#include "rtc_controllers/compliance/wrench_pipeline.hpp"
#include <rtc_base/types/types.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <string>

namespace rtc::params {

// ── Gain / feature configuration (trivially copyable POD for SeqLock) ──────
struct TaskAdmittanceParams {
  /// §7.2 virtual dynamics + §7.4/§7.5 bounds.
  compliance::AdmittanceParams admittance{};

  /// §10.6 staleness / contact.
  compliance::WrenchPipelineParams wrench{};

  // ── §7.3 differential IK ─────────────────────────────────────────────────
  /// Closed-loop pose gain on e(X, X_c) [1/s]. §7.3's formula is pure
  /// feedforward (`ν = ν_c`); with a tracking error that never gets corrected
  /// the arm drifts away from the compliant frame it is supposed to realise,
  /// so a CLIK term is added here and reported as a spec gap (§12). Set to 0
  /// to reproduce §7.3 literally.
  std::array<double, 3> ik_kp_pos{{2.0, 2.0, 2.0}};
  std::array<double, 3> ik_kp_rot{{2.0, 2.0, 2.0}};
  /// Nullspace posture centering toward q_null [1/s]; bites only when nv > 6.
  double nullspace_kp{0.5};
  /// `true` (§7.3 default): q_cmd = q_meas + q̇Δt — no position windup, but a
  /// lagging lower controller drags the command back. `false`: integrate the
  /// command itself (smoother, can wind away from the arm), which is why it is
  /// the only mode guarded by `command_divergence_limit` — on a JOINT-axis
  /// binding. See that field for why a task-space one guards it elsewhere.
  bool integrate_from_measured{true};

  // ── §6.5 σ_min-adaptive DLS ──────────────────────────────────────────────
  double singularity_threshold{0.02};  ///< σ₀: DLS engages below this (also DEGRADED)
  double singularity_critical{0.005};  ///< σ_min below this → SAFE_STOP
  double max_damping{0.05};            ///< λ_max for the DLS ramp

  // ── Safety / activation ──────────────────────────────────────────────────
  double pose_error_limit{0.5};  ///< ‖e(X, X_c)‖ bound → SAFE_STOP
  /// ‖q_cmd − q_meas‖ [rad] → SAFE_STOP. JOINT-axis guard: a task-space binding
  /// reaches the same event through `pose_error_limit` and leaves this unread
  /// (#478). Kept in the schema for the joint-compliance bindings.
  double command_divergence_limit{0.5};
  double joint_limit_margin{0.0};       ///< δ [rad] shrinking the q_cmd clamp band
  double activation_ramp_time{0.5};     ///< [s] wrench 0→1 linear ramp (§10.7); ≤0 = none
  double saturation_persist_time{0.1};  ///< [s] velocity clamp held longer → DEGRADED
};

/// Everything the §7 schema yields that is NOT a gain — see
/// TaskImpedanceConfig for why these are bundled and why they are left
/// untouched when the YAML node is null.
struct TaskAdmittanceConfig {
  /// UNRESOLVED frame name from `external_wrench.sensor_frame`.
  std::string sensor_frame;
  /// §7.1: admittance takes force as its INPUT, so a source is REQUIRED and
  /// this starts true — `external_wrench.enabled: false` is rejected, not a
  /// fallback (unlike the impedance controller, where A=NONE is a real law).
  bool wrench_enabled{true};
  compliance::WrenchConditioningConfig wrench{};
  CommandType command_type{CommandType::kPosition};
};

/// Parse the task-admittance YAML schema into @p out and @p config.
///
/// A NULL @p cfg is not a no-op: the NUM-6 posture floor still runs on @p out
/// (see ParseTaskImpedanceParams for the reasoning). @p config is NOT written
/// on that path.
///
/// @throws std::runtime_error on a mis-shaped sequence, a non-positive
///         `desired_inertia` / `pose_error_limit`, a disabled wrench source
///         (§7.1) or a non-position `command_type`. Every throw happens BEFORE
///         @p config is written, so a rejected reconfigure leaves the caller's
///         live state untouched (issue #172).
///
/// Non-RT: called from LoadConfig / on_configure, never from a tick.
void ParseTaskAdmittanceParams(const YAML::Node& cfg, TaskAdmittanceParams& out,
                               TaskAdmittanceConfig& config);

}  // namespace rtc::params
