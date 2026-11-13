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
///   constraints:             # the contact constraints and soft limits
///     foot_constraint:       # ContactConstraintResidual gains (stance and
///       position_z: 100.0    # swing share them; swing uses the three z ones)
///       orientation: 80.0
///       linear_velocity_xy: 20.0
///       linear_velocity_z: 10.0
///       angular_velocity: 20.0
///       linear_acceleration_xy: 1.0     # > 0
///       linear_acceleration_z: 1.0      # > 0
///       angular_acceleration: 1.0       # > 0
///     friction:              # every contact, world axes (ContactWrenchConeResidual)
///       mu: 0.4              # > 0
///       grip_force: 0.0      # [N] ≥ 0
///       regularization: 25.0 # ε [N²] > 0
///       barrier_mu: 0.2      # relaxed log barrier on the margin [N]; > 0
///       barrier_delta: 5.0   # [N] > 0
///     contact_rectangle:     # CoP bounds on each 6-D contact frame's x-y plane [m];
///       x_min: -0.09         # must contain the frame origin:
///       x_max: 0.09          # x_min < 0 < x_max, y_min < 0 < y_max
///       y_min: -0.03
///       y_max: 0.03
///       barrier_mu: 0.6      # relaxed log barrier on the margins [N·m]; > 0
///       barrier_delta: 0.03  # [N·m] > 0
///     joint_limits: {mu: 1200.0, delta: 0.1}   # barrier on the model's limits; δ > 0
///     collision:             # sphere-pair barrier, active outside full stance
///       enabled: false
///       mu: 1500.0
///       delta: 0.04          # > 0
///       pairs:               # frames of the model; offsets in the frame [m]
///         - {frame_a: <frame>, offset_a: [x, y, z], radius_a: r,
///            frame_b: <frame>, offset_b: [x, y, z], radius_b: r}
/// ```
///
/// Joints are named (ARCH-1): every joint of the model after its free-flyer
/// root must have an entry in each of the three joint maps; a name the model
/// does not have is ignored (one file serves models that lock different
/// joints). The loader lays the weights onto the model's tangent and input
/// layout (`WbLayout`), so a config belongs to the model it was loaded for.
///
/// Every value of `constraints:` is finite; the ones marked `> 0` refuse 0
/// (a zero regularisation has no cone derivative at the apex, a zero `δ`
/// divides, a zero acceleration gain turns the equality into one the
/// initial node cannot meet); the rest refuse negatives. The model's joint
/// limits are checked too: a limited one-entry joint with `lower ≥ upper` is
/// refused. Collision frames are model frame names; one the model lacks is
/// refused, whether or not `enabled`.
///
/// Configure time: the loader allocates and never throws.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/barrier_costs.hpp"
#include "rtc_mpc/ocp/contact_constraints.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <vector>

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
  kNegativeWeight,       ///< a weight, gain, barrier μ, radius or grip force (or
                         ///< `terminal_scaling`) below 0
  kInvalidHorizon,       ///< `horizon_nodes` outside `[1, kMaxHorizon]`
  kInvalidDt,            ///< `dt` not positive
  kUnknownIntegrator,    ///< an integrator name other than the two
  kMissingJoint,         ///< a model joint with no entry in a joint map
  kNonPositive,          ///< a `constraints:` value that must be > 0 is not
  kInvalidRectangle,     ///< a contact rectangle that does not contain its origin
  kUnknownFrame,         ///< a collision frame the model does not have
  kInvalidJointLimit,    ///< a limited one-entry model joint with lower ≥ upper
};

/// @brief Rows of one contact's tracking weight (`ContactTrackingResidual`).
inline constexpr int kContactTrackingRows = 18;

/// @brief The `constraints:` section, laid onto one model.
struct KinodynamicConstraintsConfig {
  FootConstraintGains foot_constraint{};
  FrictionCone friction{};
  RelaxedLogBarrier friction_barrier{};
  ContactRectangle contact_rectangle{};
  RelaxedLogBarrier cop_barrier{};
  PolynomialBarrier joint_limits{};
  bool collision_enabled{false};
  PolynomialBarrier collision{};
  std::vector<SpherePair> collision_pairs{};  ///< frames resolved in the model
};

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
  KinodynamicConstraintsConfig constraints{};
};

/// @brief Read `mpc.kinodynamic:` (the file comment's schema) for @p model.
/// @param node   the `kinodynamic:` map itself
/// @param model  an initialised floating-base handler
/// @param out    written only on success
[[nodiscard]] KinodynamicOCPConfigError LoadKinodynamicOCPConfig(
    const YAML::Node& node, const RobotModelHandler& model, KinodynamicOCPConfig& out) noexcept;

}  // namespace rtc::mpc

#endif  // RTC_MPC_OCP_KINODYNAMIC_OCP_CONFIG_HPP_
