// ── Task-space impedance controller configuration (G2 schema layer) ─────────
// The gains POD, the two axis enums and the YAML schema that fills them, lifted
// out of TaskImpedanceController so all four outlive the adapter (#236 S7c-2,
// D-B/G2). See params/joint_pd_params.hpp for why this layer exists.
//
// Like the other parsers here this layer is rclcpp-free. It is also MODEL-free,
// which is the boundary the compliance schemas forced: `sensor_frame` comes out
// as a STRING and is never resolved into a pinocchio::FrameIndex here.
// Resolution needs the robot model — and specifically the SUBMODEL actually in
// use, which the owner may have swapped moments earlier — so a parse layer that
// did it could not validate a config file without first building a model. The
// owner does the lookup instead, and keeps it the LAST thing that can fail so a
// bad frame name cannot leave a controller half-reconfigured.
#pragma once

#include "rtc_controllers/compliance/impedance_law.hpp"
#include "rtc_controllers/compliance/inertia_shaping.hpp"
#include "rtc_controllers/compliance/wrench_conditioning.hpp"
#include <rtc_base/types/types.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <string>

namespace rtc::params {

/// Task selection (axis B): which Cartesian DoF the impedance law regulates.
enum class TaskSelection : std::uint8_t { kFullSe3, kTranslationOnly };

/// Control-law family (axis A). Explicit rather than inferred from "is a
/// wrench configured?": the two laws have different failure modes and
/// different singularity exposure, so which one runs must be a declared
/// choice, not a side effect of wiring a sensor.
enum class TaskImpedanceFormulation : std::uint8_t {
  kJacobianTranspose,  ///< §6.2 — Λ-free, task-singularity-free (default)
  kInertiaShaping,     ///< §6.3 — Λ_S Λ_d⁻¹ shaping; needs Λ_S ⇒ §6.5 applies
};

// ── Gain / feature configuration (trivially copyable POD for SeqLock) ──────
struct TaskImpedanceParams {
  /// Cartesian stiffness/damping, per axis — the §6.2 law's OWN parameter POD,
  /// nested rather than re-declared. K_p is a stiffness [N/m or N·m/rad], K_d a
  /// damping [N·s/m or N·m·s/rad] — NOT the OSC acceleration-form gains.
  ///
  /// It used to be four loose arrays here carrying the SAME defaults as
  /// compliance::ImpedanceParams (200 / 28 / 20 / 9), so the copy was a second
  /// definition of one set of numbers, and every RT tick rebuilt the POD
  /// POSITIONALLY — `ImpedanceParams{kp_pos, kd_pos, kp_rot, kd_rot}` — a
  /// silent dependency on the core's field ORDER that no test could see.
  /// Nesting removes both (#236 S6b), and the sibling
  /// CascadedComplianceParams was already written this way.
  ///
  /// Nesting is only a de-duplication when the defaults MATCH; where they do
  /// not, it just renames the second definition (#236 D-S5b, and see
  /// agent_docs/design-principles.md §코어의 형태). The YAML keys are unchanged
  /// (`kp_pos:` …) — schema changes are S8's business.
  compliance::ImpedanceParams impedance{};

  // Nullspace posture task (bites only when nv > task DoF m). Kd ≥ 2√Kp is
  // enforced by ParseTaskImpedanceParams (§6.4); Kp = 0 with Kd > 0 (pure
  // damping) is allowed EXCEPT under TRANSLATION_ONLY (§6.1).
  double nullspace_kp{0.0};  ///< posture centering stiffness toward q_null [N·m/rad]
  double nullspace_kd{2.0};  ///< nullspace joint damping [N·m·s/rad]

  // σ_min-adaptive DLS for the nullspace Λ_S (§6.5).
  double singularity_threshold{0.02};  ///< σ₀: DLS engages below this (also DEGRADED)
  double singularity_critical{0.005};  ///< σ_min below this → SAFE_STOP
  double max_damping{0.05};            ///< λ_max for the DLS ramp

  // Safety layer (§5.3, §10.5).
  double joint_limit_margin{0.1};  ///< δ [rad]: repulsive band width
  double joint_limit_kp{0.0};      ///< k_lim [N·m/rad]; 0 disables the SPRING term only
  double joint_limit_kd{
      2.0};  ///< d_lim [N·m·s/rad]; independent of k_lim (pure damping if k_lim=0)
  double max_torque_rate{2000.0};  ///< [N·m/s] slew limit (dt-scaled, never 500 Hz)
  double pose_error_limit{1.5};    ///< ‖e‖ bound → SAFE_STOP when exceeded

  // Activation and E-STOP.
  double activation_ramp_time{0.5};     ///< [s] gain 0→1 linear ramp (§10.7); ≤0 = no ramp
  double estop_damping{5.0};            ///< D for the torque E-STOP hold ĝ(q) − D·q̇ (E-8)
  double saturation_persist_time{0.1};  ///< [s] saturation held longer → DEGRADED

  // ── §6.3 inertia shaping (only read when formulation == kInertiaShaping) ─
  /// Λ_d and the §5.2 ratio bound, held as the CORE's own struct rather than
  /// as three loose fields. Held this way for the reason TaskAdmittanceParams
  /// holds compliance::AdmittanceParams: the defaults and the §5.2 bound then
  /// have ONE definition instead of a copy here that nothing compares against,
  /// and Compute() passes `gains.inertia` straight through instead of
  /// re-assembling the struct positionally on every RT tick — an aggregate init
  /// that silently depends on field order.
  compliance::InertiaShapingParams inertia{};

  // ── External wrench staleness / contact (§10.6) ──────────────────────────
  double wrench_timeout{0.05};        ///< [s] older than this → fade + DEGRADED
  double wrench_fadeout_time{0.1};    ///< [s] linear ramp to ZERO (never hold the last value)
  double contact_threshold{5.0};      ///< [N] ‖f_LWA‖ above this → RUNNING_CONTACT
  double contact_release_ratio{0.6};  ///< release at ratio·threshold (hysteresis, §10.6 MUST)
};

/// Everything the §6.2/§6.3 schema yields that is NOT a gain.
///
/// Bundled rather than returned through four separate out-references because
/// the owner has to commit them together: the frame lookup and the wrench
/// reconfigure both still have to run — and can still throw — AFTER the parse,
/// so nothing here may be written onto live state until those succeed.
///
/// Left untouched when the YAML node is null: an absent node means none of
/// these keys were read, which is not the same as reading them as their
/// defaults (that would silently retire a configured wrench source).
struct TaskImpedanceConfig {
  /// UNRESOLVED frame name from `external_wrench.sensor_frame` — see the file
  /// header for why this layer does not turn it into a pinocchio::FrameIndex.
  /// Empty means "not configured"; what that resolves to belongs to the owner.
  std::string sensor_frame;
  /// A=NONE is a first-class law here (unlike admittance/cascade), so a wrench
  /// source is OFF unless the file asks for one.
  bool wrench_enabled{false};
  compliance::WrenchConditioningConfig wrench{};
  TaskImpedanceFormulation formulation{TaskImpedanceFormulation::kJacobianTranspose};
  CommandType command_type{CommandType::kTorque};
};

/// Parse the task-impedance YAML schema into @p out and @p config.
///
/// A NULL @p cfg is not a no-op: the NUM-6 posture floor and the §6.1 guard
/// below still run on @p out. The loader half of NUM-6 is "regardless of
/// whether the key is present", and an absent node is the widest case of an
/// absent key — the values then come from the constructor or a previous
/// set_gains(), which are exactly the two paths the floor exists for. @p config
/// is NOT written on that path (see its docs).
///
/// @param selection in — the axis-B choice the owner is running. §6.1 makes
///        `nullspace_stiffness == 0` a configure ERROR under TRANSLATION_ONLY,
///        so the schema cannot be validated without it.
///
/// @throws std::runtime_error on a §6.1 violation, a non-torque `command_type`,
///         an unknown `formulation`, a mis-shaped or non-SPD `desired_inertia`,
///         a mis-shaped `external_wrench` entry, or an F8 threshold
///         (`max_torque_rate`, `pose_error_limit`) that is not strictly positive
///         and finite — 0 silently disables the §10.5 slew stage, a
///         non-positive pose bound latches SAFE_STOP on the first tick, and
///         NaN/inf make the CRITICAL bound unreachable. Every throw happens BEFORE
///         @p config is written, so a rejected reconfigure leaves the caller's
///         live state untouched (issue #172).
///
/// Non-RT: called from LoadConfig / on_configure, never from a tick.
void ParseTaskImpedanceParams(const YAML::Node& cfg, TaskImpedanceParams& out,
                              TaskSelection selection, TaskImpedanceConfig& config);

}  // namespace rtc::params
