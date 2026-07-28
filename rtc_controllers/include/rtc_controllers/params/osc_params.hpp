// ── Operational-space controller configuration (G2 schema layer) ────────────
// The gains POD and the YAML schema that fills it, lifted out of
// OperationalSpaceController so both outlive the adapter (#236 S7c-1, D-B/G2).
// See params/joint_pd_params.hpp for why this layer exists.
//
// This layer stays yaml-cpp-only — deliberately no rclcpp. A retired key is
// therefore REPORTED to the caller rather than logged here (see RetiredKeys):
// a future binding must be able to validate a config without pulling the ROS
// logging stack into the parse path, and the operator-facing wording belongs to
// whoever owns the node anyway.
#pragma once

#include <rtc_base/types/types.hpp>

#include <yaml-cpp/yaml.h>

#include <array>

namespace rtc::params {

/// Keys the schema no longer honours but that a deployed config may still
/// carry. Reported, never silently accepted: LoadConfig ignores unknown keys,
/// so without this an existing config keeps parsing clean while its behaviour
/// changes underneath it (#236 S2b retired `damping`).
struct OscRetiredKeys {
  bool damping{false};  ///< superseded by max_damping (λ_max) + singularity_threshold (σ₀)
};

/// Task-space PD (acceleration form) plus the §6.5 DLS and posture knobs.
struct OscParams {
  // a = kp·e_pose + kd·(ẋ_d − ẋ) + a_ff. kp is stiffness-like [1/s²], kd
  // derivative [1/s]. Defaults are sane torque-OSC starting points.
  std::array<double, 3> kp_pos{{100.0, 100.0, 100.0}};  ///< Cartesian position gain    [1/s²]
  std::array<double, 3> kd_pos{{20.0, 20.0, 20.0}};     ///< Cartesian position damping [1/s]
  std::array<double, 3> kp_rot{{50.0, 50.0, 50.0}};     ///< Cartesian orientation gain [1/s²]
  std::array<double, 3> kd_rot{{10.0, 10.0, 10.0}};     ///< Cartesian orientation damp [1/s]

  // §6.5 σ_min-adaptive DLS damping — same names/defaults as the
  // impedance/admittance/cascade controllers. Replaced a CONSTANT λ in #236 S2b.
  double max_damping{0.05};            ///< λ_max for the DLS ramp
  double singularity_threshold{0.02};  ///< σ₀: DLS engages below this

  // Dynamically-consistent null-space posture task (only meaningful when nv > 6).
  // τ₀ is summed directly into joint torque, so these are TORQUE units, not the
  // acceleration-form gains above.
  double null_kp{0.0};  ///< posture centering stiffness toward safe_position [N·m/rad]
  double null_kd{1.0};  ///< null-space joint damping                         [N·m·s/rad]

  double estop_damping{5.0};  ///< D for the torque E-STOP hold ĝ(q) − D·q̇ [N·m·s/rad]

  /// Retained for YAML back-compat; torque OSC always compensates g(q)+C·v.
  /// Parsed but ignored by the law.
  bool enable_gravity_compensation{true};

  double trajectory_speed{0.1};           ///< max translational trajectory speed [m/s]
  double trajectory_angular_speed{0.5};   ///< max angular trajectory speed       [rad/s]
  double max_traj_velocity{0.5};          ///< max TCP velocity during trajectory [m/s]
  double max_traj_angular_velocity{1.0};  ///< max TCP angular velocity           [rad/s]
};

/// Parse the OSC YAML schema into @p out.
///
/// A NULL @p cfg is not a no-op: the posture floor below still runs. NUM-6
/// requires the loader to floor the posture gains "regardless of whether the key
/// is present", and an absent node is the widest case of an absent key — the
/// values then come from the constructor or a previous set_gains(), which are
/// exactly the two paths the floor exists for. Returning early would leave the
/// caller reporting a gain the law refuses to run.
///
/// @param command_type in/out — OSC is a torque controller; a `command_type` key
///        that is not "torque" throws rather than silently mislabelling the
///        output (position/velocity task control belongs to CLIK). Validated
///        BEFORE the caller commits the parsed gains so a rejected reconfigure
///        does not mutate live RT gains (issue #172).
/// @param retired optional; receives which retired keys were present.
///
/// @throws std::runtime_error on a non-torque command_type.
///
/// Non-RT: called from LoadConfig / on_configure, never from a tick.
void ParseOscParams(const YAML::Node& cfg, OscParams& out, CommandType& command_type,
                    OscRetiredKeys* retired = nullptr);

}  // namespace rtc::params
