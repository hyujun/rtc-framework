// ── `estimator.inekf` — the configuration of the IMU + leg-kinematics source ──
//
//   estimator:
//     source: inekf                  # truth | inekf (fixed at configure)
//     inekf:                         # required with source: inekf; checked when present
//       imu_frame: "<frame>"         # model frame of the primary IMU site's axes;
//                                    #   must hang on the floating-base root joint
//       imu:
//         sample_period_s: T         # = 1 / control_rate (configure refuses another)
//         gyro_sigma: σ              # per-sample white noise [rad/s]
//         accel_sigma: σ             # per-sample white noise [m/s²]
//         gyro_bias_walk: σ          # density [rad/s²/√Hz]
//         accel_bias_walk: σ         # density [m/s³/√Hz]
//       core:                        # rtc::estimation::ContactInekf::Config, by field
//         gravity, contact_noise_density, joint_position_sigma, kinematic_sigma,
//         contact_gate_chi2, prior_velocity_sigma, prior_position_sigma,
//         prior_yaw_sigma, prior_gyro_bias_sigma, prior_accel_bias_sigma,
//         stationary_accel_tolerance, stationary_gyro_tolerance
//       fsm:                         # rtc::estimation::ContactFsm::Config, by field
//         enter_speed, slip_speed, nis_low, nis_high, enter_dwell, confirm_dwell,
//         release_dwell, slip_dwell, nis_dwell, recover_dwell,
//         evidence_time_constant, unknown_noise_scale
//       ground:                      # the support-height pin (ContactInekfSequencer::GroundConfig)
//         enabled: bool              # only on flat ground: the plane is the stance at the start
//         sigma: σ                   # > 0, the plane's error at a lift-off [m]
//         gate_chi2: χ²              # > 0, the pin's NIS gate (1 dof)
//         max_gated: n               # ≥ 1 gated pins in a row before an inflated one
//       init:
//         window_ticks: N            # ≥ 1 consecutive still samples start the filter
//         timeout_s: s               # > 0: no start within it → the controller faults
//         yaw: ψ                     # the initial ZYX heading of the IMU, [−2π, 2π] rad
//         joint_speed_tolerance: v   # > 0, the largest body joint speed of a still sample [rad/s]
//       policy:
//         max_hold_ticks: n          # ≥ 0 ticks a repeated IMU sample keeps the state valid
//         max_joint_imu_skew_ticks: n  # ≥ 0; a larger joint / IMU receipt gap skips the update
//         max_skew_invalid_ticks: n  # ≥ max_joint_imu_skew_ticks; larger → the state is invalid
//         max_no_anchor_s: s         # > 0 without a confirmed contact → reset
//         max_gated_s: s             # > 0 of updates that gated every contact → reset
//         max_refused_ticks: n       # ≥ 1 consecutive refused steps → reset
//
// Every key is required and no other is accepted. The per-sample σ of the IMU
// becomes the core's continuous density σ·√T (T = sample_period_s) here, once;
// the FSM's period is T. The core and FSM fields keep their own contracts
// (contact_inekf.hpp, contact_fsm.hpp) and are checked by their Init() as well.
#pragma once

#include "rtc_controllers/estimation/contact_fsm.hpp"
#include "rtc_controllers/estimation/contact_inekf.hpp"

#include <yaml-cpp/yaml.h>

#include <string>

namespace integrated_bringup::locomotion {

/// @brief `estimator.inekf.imu`: the IMU's noise as the sensor states it.
struct InekfImuNoise {
  double sample_period_s{0.0};
  double gyro_sigma{0.0};   ///< per sample [rad/s]
  double accel_sigma{0.0};  ///< per sample [m/s²]
  double gyro_bias_walk{0.0};
  double accel_bias_walk{0.0};
};

/// @brief `estimator.inekf.init`.
struct InekfInitConfig {
  int window_ticks{1};
  double timeout_s{0.0};
  double yaw{0.0};
  double joint_speed_tolerance{0.0};
};

/// @brief `estimator.inekf.policy`.
struct InekfPolicyConfig {
  int max_hold_ticks{0};
  int max_joint_imu_skew_ticks{0};
  int max_skew_invalid_ticks{0};
  double max_no_anchor_s{0.0};
  double max_gated_s{0.0};
  int max_refused_ticks{1};
};

/// @brief The parsed `estimator.inekf` section.
struct InekfSourceConfig {
  std::string imu_frame;
  InekfImuNoise imu;
  rtc::estimation::ContactInekf::Config core;                   ///< densities already σ·√T
  rtc::estimation::ContactFsm::Config fsm;                      ///< period = imu.sample_period_s
  rtc::estimation::ContactInekfSequencer::GroundConfig ground;  ///< height 0: the start's stance
  InekfInitConfig init;
  InekfPolicyConfig policy;
};

/// @brief Parse and check `estimator.inekf` (YAML only — the frame and the
///        control rate are the controller's checks).
/// @throws std::invalid_argument naming the key on a missing or unknown key, a value
///         that is not a finite number in its range, an inverted pair, or a core / FSM
///         configuration their Init() refuses.
[[nodiscard]] InekfSourceConfig ParseInekfSourceConfig(const YAML::Node& inekf,
                                                       const std::string& context);

}  // namespace integrated_bringup::locomotion
