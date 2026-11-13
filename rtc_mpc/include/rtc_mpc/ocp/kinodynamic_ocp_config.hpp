#ifndef RTC_MPC_OCP_KINODYNAMIC_OCP_CONFIG_HPP_
#define RTC_MPC_OCP_KINODYNAMIC_OCP_CONFIG_HPP_

/// @file kinodynamic_ocp_config.hpp
/// @brief The static configuration of @ref KinodynamicOCP — horizon, step,
///        integrator and every cost weight — and its YAML loader.
///
/// YAML schema (`mpc.kinodynamic:`; every key required, an unknown key
/// refused, every weight finite and `≥ 0`):
/// ```yaml
/// kinodynamic:
///   horizon_nodes: 32        # int in [1, kMaxHorizon]: stages; the horizon has
///                            # horizon_nodes + 1 state nodes
///   dt: 0.035                # [s] > 0: node spacing
///   integrator: semi_implicit_euler   # or rk2
///   terminal_scaling: 4.0    # ≥ 0: terminal weight = terminal_scaling · terminal_weights
///   state_weights:           # one stage's state weight, on the tangent [δq; δv]
///     base_position: [wx, wy, wz]          # free-flyer tangent (pinocchio difference,
///     base_orientation: [wr, wp, wy]       # LOCAL); roll, pitch, yaw (tangent order)
///     joints: {<joint>: w}                 # joint position
///     base_linear_velocity: [wx, wy, wz]   # base LOCAL
///     base_angular_velocity: [wx, wy, wz]  # base LOCAL
///     joint_velocities: {<joint>: w}
///   terminal_weights: {...}  # the state_weights schema
///   input_weights:
///     contact_force: [wx, wy, wz]          # every contact (world axes)
///     contact_moment: [wx, wy, wz]         # every 6-D contact (world axes)
///     joint_accelerations: {<joint>: w}
///   contact_tracking:        # every contact's 18 rows (ContactTrackingResidual)
///     position: [wx, wy, wz]
///     orientation: [wx, wy, wz]
///     linear_velocity: [wx, wy, wz]
///     angular_velocity: [wx, wy, wz]
///     linear_acceleration: [wx, wy, wz]
///     angular_acceleration: [wx, wy, wz]
/// ```
///
/// Joints are named (ARCH-1): every joint of the model after its free-flyer
/// root must have an entry in each of the three joint maps; a name the model
/// does not have is ignored (one file serves models that lock different
/// joints). The loader lays the weights onto the model's tangent and input
/// layout (`WbLayout`), so a config belongs to the model it was loaded for.
///
/// Configure time: the loader allocates and never throws.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

namespace rtc::mpc {

/// @brief Why @ref LoadKinodynamicOCPConfig refused a node.
enum class KinodynamicOCPConfigError {
  kNoError = 0,
  kModelNotInitialised,  ///< the handler has not been initialised
  kFixedBase,            ///< the model's root is not a free-flyer (6 base velocities)
  kNotAMap,              ///< the section, or one of its maps, is not a map
  kUnknownKey,           ///< a key the schema does not have
  kMissingKey,           ///< a required key is absent or null
  kWrongType,            ///< a value of the wrong kind or length
  kNonFinite,            ///< a NaN or infinite number
  kNegativeWeight,       ///< a weight (or `terminal_scaling`) below 0
  kInvalidHorizon,       ///< `horizon_nodes` outside `[1, kMaxHorizon]`
  kInvalidDt,            ///< `dt` not positive
  kUnknownIntegrator,    ///< an integrator name other than the two
  kMissingJoint,         ///< a model joint with no entry in a joint map
};

/// @brief Rows of one contact's tracking weight (`ContactTrackingResidual`).
inline constexpr int kContactTrackingRows = 18;

/// @brief What @ref KinodynamicOCP is built with, laid onto one model.
struct KinodynamicOCPConfig {
  int horizon_nodes{0};  ///< stages; the horizon has `horizon_nodes + 1` state nodes
  double dt{0.0};        ///< [s] node spacing
  KinodynamicIntegrator integrator{KinodynamicIntegrator::kSemiImplicitEuler};
  double terminal_scaling{0.0};
  Eigen::VectorXd state_weights{};     ///< `2·nv` — the tangent `[δq; δv]`
  Eigen::VectorXd terminal_weights{};  ///< `2·nv` — before `terminal_scaling`
  Eigen::VectorXd input_weights{};     ///< `nu` — the `WbLayout` input
  /// One contact's rows `[p; e_R; v; ω; a; α]`, the same for every contact.
  Eigen::Matrix<double, kContactTrackingRows, 1> contact_tracking_weights{
      Eigen::Matrix<double, kContactTrackingRows, 1>::Zero()};
};

/// @brief Read `mpc.kinodynamic:` (the file comment's schema) for @p model.
/// @param node   the `kinodynamic:` map itself
/// @param model  an initialised floating-base handler
/// @param out    written only on success
[[nodiscard]] KinodynamicOCPConfigError LoadKinodynamicOCPConfig(
    const YAML::Node& node, const RobotModelHandler& model, KinodynamicOCPConfig& out) noexcept;

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_KINODYNAMIC_OCP_CONFIG_HPP_
