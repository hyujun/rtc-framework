// ── Includes: project header first, then third-party, then C++ stdlib ──────────
#pragma once

#include "ur5e_rt_controller/rt_controller_interface.hpp"

// Suppress warnings emitted by Pinocchio / Eigen headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>   // computeJointJacobian
#include <pinocchio/algorithm/kinematics.hpp> // forwardKinematics
#include <pinocchio/algorithm/rnea.hpp>       // computeGeneralizedGravity, computeCoriolisMatrix
#include <pinocchio/multibody/data.hpp>       // pinocchio::Data
#include <pinocchio/multibody/model.hpp>      // pinocchio::Model
#include <pinocchio/parsers/urdf.hpp>         // urdf::buildModel
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <span>
#include <string>
#include <string_view>

namespace ur5e_rt_controller
{

/// Model-based PD controller with Pinocchio gravity / Coriolis compensation.
///
/// Control law:
/// @code
///   command[i] = Kp * e[i]  +  Kd * ė[i]  +  g(q)[i]  [+  C(q,v)·v [i]]
/// @endcode
///
///   - e[i]          : position error  (target − current)
///   - ė[i]          : error derivative  (Δe / dt)
///   - g(q)          : gravity torque vector computed by Pinocchio RNEA
///   - C(q,v)·v      : Coriolis / centrifugal forces (optional, disabled by default)
///
/// All Eigen work buffers are pre-allocated in the constructor; no heap
/// allocation occurs on the 500 Hz RT path.  Pinocchio algorithms use
/// only internally-pre-allocated Data members, also heap-free at runtime.
///
/// ### Usage — replace PDController in custom_controller.cpp
/// @code
///   // 1. Add at the top of custom_controller.cpp:
///   //    #include "ur5e_rt_controller/controllers/pinocchio_controller.hpp"
///
///   // 2. Change the controller_ member type (in class CustomController):
///   //    std::unique_ptr<urtc::RTControllerInterface> controller_;
///
///   // 3. Update the constructor initialiser:
///   //    controller_(std::make_unique<urtc::PinocchioController>(
///   //        "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
///   //        urtc::PinocchioController::Gains{
///   //            .kp = 5.0,
///   //            .kd = 0.5,
///   //            .enable_gravity_compensation  = true,
///   //            .enable_coriolis_compensation = false}))
///
///   // 4. Remove the controller_->set_gains() call in DeclareAndLoadParameters()
///   //    because PinocchioController receives gains through its constructor.
/// @endcode
class PinocchioController final : public RTControllerInterface
{
public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains
  {
    std::array<double, 6> kp{{5.0, 5.0, 5.0, 5.0, 5.0, 5.0}};
    std::array<double, 6> kd{{0.5, 0.5, 0.5, 0.5, 0.5, 0.5}};
    bool enable_gravity_compensation{true};     ///< Add g(q) to commands
    bool enable_coriolis_compensation{false};   ///< Add C(q,v)·v to commands
  };

  /// Construct and load the robot model from a URDF file.
  ///
  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      PD gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  ///
  /// @note  Model loading happens once here; it is NOT on the RT path.
  explicit PinocchioController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState & state) noexcept override;

  void SetRobotTarget(std::span<const double, kNumRobotJoints> target) noexcept override;
  void SetHandTarget(std::span<const double, kNumHandJoints> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Accessors (non-RT reads only — do NOT call from the 500 Hz path) ─────
  void set_gains(const Gains & g) noexcept {gains_ = g;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

  /// Gravity torque vector g(q) cached after the most recent Compute() call.
  [[nodiscard]] std::array<double, kNumRobotJoints> gravity_torques() const noexcept;

  /// End-effector (last joint) position in the world frame cached after Compute().
  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept;

  /// 6×nv Jacobian of the last joint in the local frame, cached after Compute().
  /// Row order: [vx, vy, vz, wx, wy, wz].
  [[nodiscard]] Eigen::MatrixXd jacobian() const noexcept {return jacobian_;}

private:
  // ── Pinocchio model + work data (allocated once in constructor) ───────────
  pinocchio::Model model_;
  pinocchio::Data data_;

  // Pre-allocated Eigen vectors — reused every cycle, no heap alloc on RT path
  Eigen::VectorXd q_;                 ///< Joint-position vector   (size nq)
  Eigen::VectorXd v_;                 ///< Joint-velocity vector   (size nv)
  Eigen::VectorXd coriolis_forces_;   ///< C(q,v)·v result buffer  (size nv)
  Eigen::MatrixXd jacobian_;          ///< 6×nv Jacobian buffer

  // ── Controller state ─────────────────────────────────────────────────────
  Gains gains_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<double, kNumHandJoints> hand_target_{};
  std::array<double, kNumRobotJoints> prev_error_{};

  // Cached diagnostic outputs
  std::array<double, kNumRobotJoints> gravity_torques_{};
  std::array<double, 3> tcp_position_{};

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity{2.0};

  // ── Private helpers ───────────────────────────────────────────────────────

  /// Drive toward kSafePosition (called when estopped_).
  [[nodiscard]] ControllerOutput ComputeEstop(
    const ControllerState & state) noexcept;

  /// Clamp every command to ±kMaxJointVelocity.
  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampCommands(
    std::array<double, kNumRobotJoints> cmds) noexcept;

  /// Copy robot state into Eigen buffers and run Pinocchio algorithms.
  /// Updates gravity_torques_, tcp_position_, jacobian_, and (optionally)
  /// coriolis_forces_.  All operations use pre-allocated members; no heap
  /// allocation occurs here.
  void UpdateDynamics(const RobotState & robot) noexcept;
};

}  // namespace ur5e_rt_controller
