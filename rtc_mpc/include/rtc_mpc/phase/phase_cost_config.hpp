#ifndef RTC_MPC_PHASE_PHASE_COST_CONFIG_HPP_
#define RTC_MPC_PHASE_PHASE_COST_CONFIG_HPP_

/// @file phase_cost_config.hpp
/// @brief Generic, robot-agnostic cost container consumed by the OCP layer.
///
/// `PhaseCostConfig` is a passive record; it never contains FSM logic.
/// Concrete phase managers (e.g. `ur5e_bringup::GraspPhaseManager`) pick a
/// pre-loaded config per phase and hand it to the OCP builder via
/// `PhaseContext`. The container lives on the **OCP build / reconfigure
/// path**, not the 500Hz RT loop, so `Eigen::VectorXd` and `std::map` are
/// permitted here — RT-path interchange goes through
/// `rtc_mpc/types/mpc_solution_types.hpp`.
///
/// `custom_weights` is the extension point for robot-specific costs
/// (e.g. `"hand_posture"`, `"joint_range"`) without leaking robot identifiers
/// into rtc_mpc. OCP handlers look up keys by name; absent keys mean weight
/// 0.0 (see @ref PhaseCostConfig::CustomWeight).

#include "rtc_mpc/model/robot_model_handler.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <map>
#include <string>
#include <string_view>

namespace rtc::mpc {

/// @brief Failure modes for @ref PhaseCostConfig::LoadFromYaml.
enum class PhaseCostConfigError {
  kNoError = 0,
  kModelNotInitialised,        ///< RobotModelHandler passed in is unusable
  kInvalidYamlSchema,          ///< missing required key, wrong YAML node kind
  kInvalidWeightSign,          ///< any weight scalar/vector entry < 0
  kInvalidHorizon,             ///< horizon_length <= 0
  kInvalidDt,                  ///< dt <= 0
  kPostureRefDimMismatch,      ///< q_posture_ref.size() != nq
  kForceTargetDimMismatch,     ///< F_target.size() != sum(contact_dims)
  kPlacementWeightDimMismatch, ///< W_placement.size() != 6
};

/// @brief Generic cost weights + references for one OCP build.
///
/// YAML schema expected by @ref LoadFromYaml:
/// ```yaml
/// horizon_length: 20        # int, > 0
/// dt: 0.01                  # double, > 0
/// w_frame_placement: 100.0
/// w_state_reg:        1.0
/// w_control_reg:      0.01
/// w_contact_force:    0.001
/// w_centroidal_momentum: 0.0
/// W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]   # 6-vec
/// q_posture_ref: [0, 0, ...]                              # nq entries
/// F_target:      [0, 0, 0,  0, 0, 0]                      # sum(dim) entries
/// custom_weights:                                         # optional
///   hand_posture: 5.0
///   joint_range: 2.0
/// ```
///
/// Sizes must match the `RobotModelHandler` passed to `LoadFromYaml`:
/// - `q_posture_ref.size()` == `model.nq()`
/// - `F_target.size()`      == Σ `contact_frames[i].dim`
/// - `W_placement.size()`   == 6  (SE3: [tx ty tz rx ry rz])
struct PhaseCostConfig {
  // Horizon / timing ──────────────────────────────────────────────────────────
  int horizon_length{20};
  double dt{0.01};

  // Scalar weights (non-negative) ─────────────────────────────────────────────
  double w_frame_placement{0.0};
  double w_state_reg{0.0};
  double w_control_reg{0.0};
  double w_contact_force{0.0};
  double w_centroidal_momentum{0.0};

  // Vector weights / references ───────────────────────────────────────────────
  Eigen::Matrix<double, 6, 1> W_placement{
      Eigen::Matrix<double, 6, 1>::Zero()}; ///< per-axis SE3 placement weight
  Eigen::VectorXd q_posture_ref{}; ///< nq — state-reg reference posture
  Eigen::VectorXd F_target{}; ///< Σ(contact_dims) — desired contact force

  // Robot-agnostic extension point ────────────────────────────────────────────
  /// Arbitrary scalar weights keyed by string. Phase managers (robot-specific
  /// layer) populate this; OCP handlers (rtc_mpc) look up expected keys and
  /// fall back to 0.0 when absent. Keep keys free of robot identifiers in
  /// shared code — robot-specific keys live in downstream bringup configs.
  std::map<std::string, double> custom_weights{};

  /// @brief Lookup a custom weight; absent key returns 0.0 (no throw).
  [[nodiscard]] double CustomWeight(std::string_view key) const noexcept {
    // std::map::find with string_view requires a heterogeneous comparator
    // (C++14 transparent lookup), which std::map<std::string,...> lacks by
    // default. Construct a string for the lookup — non-RT path so allocation
    // is acceptable.
    const auto it = custom_weights.find(std::string{key});
    return it == custom_weights.end() ? 0.0 : it->second;
  }

  /// @brief Populate a @ref PhaseCostConfig from YAML, validating against the
  ///        caller-supplied robot model.
  ///
  /// @param cfg     YAML node matching the schema in this header.
  /// @param model   Initialised `RobotModelHandler` — provides `nq()` and
  ///                `contact_frames()` for dimension checks.
  /// @param out     Destination. Left untouched on any error.
  /// @return `kNoError` on success, otherwise the first validation failure.
  ///         Never throws.
  [[nodiscard]] static PhaseCostConfigError
  LoadFromYaml(const YAML::Node &cfg, const RobotModelHandler &model,
               PhaseCostConfig &out) noexcept;
};

} // namespace rtc::mpc

#endif // RTC_MPC_PHASE_PHASE_COST_CONFIG_HPP_
