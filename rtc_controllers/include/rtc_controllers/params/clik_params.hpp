// ── Closed-loop IK (CLIK) controller configuration (G2 schema layer) ────────
// The gains POD and the YAML schema that fills it, lifted out of ClikController
// so both outlive the adapter (#236 S7c-1, D-B/G2). See
// params/joint_pd_params.hpp for why this layer exists, and params/osc_params.hpp
// for why a retired key is reported rather than logged here.
#pragma once

#include <rtc_base/types/types.hpp>

#include <yaml-cpp/yaml.h>

#include <array>

namespace rtc::params {

/// Keys the schema no longer honours but that a deployed config may still carry.
struct ClikRetiredKeys {
  bool damping{false};  ///< superseded by max_damping (λ_max) + singularity_threshold (σ₀)
};

/// Task-space P gains plus the §6.5 DLS, null-space and trajectory knobs.
struct ClikParams {
  std::array<double, 3> kp_translation{{1.0, 1.0, 1.0}};  ///< translation P gain (x,y,z)  [1/s]
  std::array<double, 3> kp_rotation{{1.0, 1.0, 1.0}};     ///< rotation P gain (rx,ry,rz)  [1/s]

  // §6.5 σ_min-adaptive DLS damping — same names/defaults as the
  // impedance/admittance/cascade controllers. Replaced a CONSTANT λ in #236 S3b
  // (issue #258); away from singularities λ² is now exactly 0, so J⁺ is the
  // undamped pseudoinverse there and tracking is not biased.
  double max_damping{0.05};            ///< λ_max for the DLS ramp
  double singularity_threshold{0.02};  ///< σ₀: DLS engages below this

  double null_kp{0.5};           ///< null-space joint-centering gain [1/s]
  bool enable_null_space{true};  ///< enable the null-space secondary task
  bool control_6dof{false};      ///< 6-DOF (translation + orientation) control

  double trajectory_speed{0.1};           ///< TCP translational speed for duration [m/s]
  double trajectory_angular_speed{0.5};   ///< TCP rotational speed for duration    [rad/s]
  double max_traj_velocity{0.5};          ///< max TCP velocity during trajectory   [m/s]
  double max_traj_angular_velocity{1.0};  ///< max TCP angular velocity             [rad/s]
};

/// Parse the CLIK YAML schema into @p out.
///
/// As with the OSC schema, a NULL @p cfg is not a no-op: NUM-6's loader half is
/// "regardless of whether the key is present", and an absent node is the widest
/// case of that — the gain then came from the constructor or a previous
/// set_gains(), the two paths the posture floor exists for.
///
/// @param command_type in/out — overwritten only when the `command_type` key is
///        present. Unlike OSC this law runs in either domain, so there is
///        nothing to reject here.
/// @param retired optional; receives which retired keys were present.
///
/// Non-RT: called from LoadConfig / on_configure, never from a tick.
void ParseClikParams(const YAML::Node& cfg, ClikParams& out, CommandType& command_type,
                     ClikRetiredKeys* retired = nullptr);

}  // namespace rtc::params
