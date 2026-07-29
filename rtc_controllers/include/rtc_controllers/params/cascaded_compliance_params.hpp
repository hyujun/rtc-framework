// ── Cascaded compliance controller configuration (G2 schema layer) ──────────
// The gains POD and the YAML schema that fills it, lifted out of
// CascadedComplianceController so both outlive the adapter (#236 S7c-2,
// D-B/G2). See params/joint_pd_params.hpp for why this layer exists, and
// params/task_impedance_params.hpp for why `sensor_frame` leaves here as a
// STRING rather than a resolved pinocchio frame index.
#pragma once

#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/impedance_law.hpp"
#include "rtc_controllers/compliance/wrench_conditioning.hpp"
#include "rtc_controllers/compliance/wrench_pipeline.hpp"
#include <rtc_base/types/types.hpp>

#include <yaml-cpp/yaml.h>

#include <string>

namespace rtc::params {

// ── Gain / feature configuration (trivially copyable POD for SeqLock) ──────
struct CascadedComplianceParams {
  /// OUTER loop: §7.2 virtual dynamics + §7.4/§7.5 bounds. `stiffness` = K_p^a;
  /// setting it to zero is hand-guiding (§7.6 MUST-3: the frame then STAYS
  /// where the force left it instead of returning to X_d).
  compliance::AdmittanceParams admittance{};

  /// INNER loop: §6.2 Cartesian stiffness / damping. These are the gains that
  /// make the arm track X_c; they are NOT the compliance the operator tunes —
  /// that is `admittance` above.
  compliance::ImpedanceParams impedance{};

  /// §10.6 staleness / contact.
  compliance::WrenchPipelineParams wrench{};

  // Nullspace posture task (bites only when nv > 6), torque domain like the
  // impedance controller: Nᵀ-projected, so it cannot disturb the task.
  double nullspace_kp{0.0};  ///< posture centering stiffness [N·m/rad]
  double nullspace_kd{2.0};  ///< nullspace joint damping [N·m·s/rad]

  // §6.5 σ_min-adaptive DLS for Λ_S (nullspace projector + the §7.6 MUST-1
  // ratio). The inner law itself is Jacobian-transpose and needs no inverse.
  double singularity_threshold{0.02};  ///< σ₀: DLS engages below this (also DEGRADED)
  double singularity_critical{0.005};  ///< σ_min below this → SAFE_STOP
  double max_damping{0.05};            ///< λ_max for the DLS ramp

  // Safety layer (§5.3, §10.5) — torque domain.
  double joint_limit_margin{0.1};  ///< δ [rad]: repulsive band width
  double joint_limit_kp{0.0};      ///< k_lim [N·m/rad]; 0 disables the SPRING term only
  double joint_limit_kd{2.0};      ///< d_lim [N·m·s/rad]; independent of k_lim
  double max_torque_rate{2000.0};  ///< [N·m/s] slew limit (dt-scaled)
  double pose_error_limit{1.5};    ///< ‖e(X, X_c)‖ bound → SAFE_STOP

  // Activation and E-STOP.
  double activation_ramp_time{0.5};     ///< [s] 0→1 linear ramp (§10.7); ≤0 = no ramp
  double estop_damping{5.0};            ///< D for the torque E-STOP hold ĝ(q) − D·q̇ (E-8)
  double saturation_persist_time{0.1};  ///< [s] saturation held longer → DEGRADED

  /// §7.6 MUST-1 minimum ω_i/ω_a. Diagnostic only (see the class note); 0
  /// silences the flag rather than latching anything.
  double min_bandwidth_ratio{3.0};
};

/// Everything the §7.6 schema yields that is NOT a gain — see
/// TaskImpedanceConfig for why these are bundled and why they are left
/// untouched when the YAML node is null.
struct CascadedComplianceConfig {
  /// UNRESOLVED frame name from `external_wrench.sensor_frame`.
  std::string sensor_frame;
  /// §7.6: the outer admittance loop has no input without a wrench source, so
  /// this starts true and `external_wrench.enabled: false` is rejected.
  bool wrench_enabled{true};
  compliance::WrenchConditioningConfig wrench{};
  CommandType command_type{CommandType::kTorque};
};

/// Parse the cascaded-compliance YAML schema into @p out and @p config.
///
/// A NULL @p cfg is not a no-op: the NUM-6 posture floor still runs on @p out
/// (see ParseTaskImpedanceParams for the reasoning). @p config is NOT written
/// on that path.
///
/// Every scalar in this schema goes through a finiteness check: `std::max(0.0,
/// NaN)` returns 0.0 — silently a third value — and an infinite gain reaches the
/// admittance state on the first tick, so the controller latches SAFE_STOP with
/// `nan_inf` and nothing points at the config line that caused it.
///
/// @throws std::runtime_error on a non-finite scalar, a section that is present
///         but not a map, a mis-shaped sequence, a non-positive
///         `desired_inertia` / `max_torque_rate` / `pose_error_limit`, a
///         disabled wrench source (§7.6) or a non-torque `command_type`. Every
///         throw happens BEFORE @p config is written, so a rejected reconfigure
///         leaves the caller's live state untouched (issue #172).
///
/// Non-RT: called from LoadConfig / on_configure, never from a tick.
void ParseCascadedComplianceParams(const YAML::Node& cfg, CascadedComplianceParams& out,
                                   CascadedComplianceConfig& config);

}  // namespace rtc::params
