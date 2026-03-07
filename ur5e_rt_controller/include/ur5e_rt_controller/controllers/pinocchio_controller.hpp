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

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
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
      double kp{5.0};
      double kd{0.5};
      bool enable_gravity_compensation{true};   ///< Add g(q) to commands
      bool enable_coriolis_compensation{false}; ///< Add C(q,v)·v to commands
    };

    /// Construct and load the robot model from a URDF file.
    ///
    /// @param urdf_path  Absolute path to the UR5e URDF file.
    /// @param gains      PD gains and feature flags.
    /// @throws std::runtime_error  if the URDF cannot be parsed.
    ///
    /// @note  Model loading happens once here; it is NOT on the RT path.
    explicit PinocchioController(std::string_view urdf_path, Gains gains = {});

    // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
    [[nodiscard]] ControllerOutput Compute(const ControllerState &state) noexcept override;

    void SetRobotTarget(std::span<const double> target) noexcept override;
    void SetHandTarget(std::span<const double> target) noexcept override;

    [[nodiscard]] std::string_view Name() const noexcept override;

    void TriggerEstop() noexcept override;
    void ClearEstop() noexcept override;
    [[nodiscard]] bool IsEstopped() const noexcept override;
    void SetHandEstop(bool active) noexcept override;

    // ── Accessors (non-RT reads only — do NOT call from the 500 Hz path) ─────
    void set_gains(const Gains &g) noexcept { gains_ = g; }
    [[nodiscard]] Gains gains() const noexcept { return gains_; }

    /// Gravity torque vector g(q) cached after the most recent Compute() call.
    [[nodiscard]] std::array<double, kNumRobotJoints> gravity_torques() const noexcept;

    /// End-effector (last joint) position in the world frame cached after Compute().
    [[nodiscard]] std::array<double, 3> tcp_position() const noexcept;

    /// 6×nv Jacobian of the last joint in the local frame, cached after Compute().
    /// Row order: [vx, vy, vz, wx, wy, wz].
    [[nodiscard]] Eigen::MatrixXd jacobian() const noexcept { return jacobian_; }

  private:
    // ── Pinocchio model + work data (allocated once in constructor) ───────────
    pinocchio::Model model_;
    pinocchio::Data data_;

    // Pre-allocated Eigen vectors — reused every cycle, no heap alloc on RT path
    Eigen::VectorXd q_;               ///< Joint-position vector   (size nq)
    Eigen::VectorXd v_;               ///< Joint-velocity vector   (size nv)
    Eigen::VectorXd coriolis_forces_; ///< C(q,v)·v result buffer  (size nv)
    Eigen::MatrixXd jacobian_;        ///< 6×nv Jacobian buffer

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
        const ControllerState &state) noexcept;

    /// Clamp every command to ±kMaxJointVelocity.
    [[nodiscard]] static std::array<double, kNumRobotJoints> ClampCommands(
        std::array<double, kNumRobotJoints> cmds) noexcept;

    /// Copy robot state into Eigen buffers and run Pinocchio algorithms.
    /// Updates gravity_torques_, tcp_position_, jacobian_, and (optionally)
    /// coriolis_forces_.  All operations use pre-allocated members; no heap
    /// allocation occurs here.
    void UpdateDynamics(const RobotState &robot) noexcept;
  };

  // ── Constructor ─────────────────────────────────────────────────────────────

  inline PinocchioController::PinocchioController(std::string_view urdf_path,
                                                  Gains gains)
      : data_(pinocchio::Model{}), gains_(gains)
  {
    // Build model from URDF — may throw if the file is missing or malformed.
    // This runs only once at startup, not on the RT path.
    pinocchio::urdf::buildModel(std::string(urdf_path), model_);

    // Re-create Data with the actual model dimensions.
    data_ = pinocchio::Data(model_);

    // Pre-allocate all Eigen work buffers to their final sizes.
    q_ = pinocchio::neutral(model_); // neutral config
    v_ = Eigen::VectorXd::Zero(model_.nv);
    coriolis_forces_ = Eigen::VectorXd::Zero(model_.nv);
    jacobian_ = Eigen::MatrixXd::Zero(6, model_.nv); // 6×nv
  }

  // ── RTControllerInterface implementation ────────────────────────────────────

  inline ControllerOutput PinocchioController::Compute(
      const ControllerState &state) noexcept
  {
    if (estopped_)
    {
      return ComputeEstop(state);
    }

    // Step 1 — Update Pinocchio state and compute dynamics (no allocation).
    UpdateDynamics(state.robot);

    // Step 2 — PD control + gravity (+ optional Coriolis) compensation.
    ControllerOutput output;
    const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);

    for (std::size_t i = 0; i < kNumRobotJoints; ++i)
    {
      const double e = robot_target_[i] - state.robot.positions[i];
      const double de = (e - prev_error_[i]) / dt;

      output.robot_commands[i] = gains_.kp * e + gains_.kd * de + gravity_torques_[i];

      if (gains_.enable_coriolis_compensation)
      {
        output.robot_commands[i] +=
            coriolis_forces_[static_cast<Eigen::Index>(i)];
      }

      prev_error_[i] = e;
    }

    output.robot_commands = ClampCommands(output.robot_commands);
    return output;
  }

  inline void PinocchioController::SetRobotTarget(
      std::span<const double> target) noexcept
  {
    const std::size_t n = std::min(target.size(), kNumRobotJoints);
    for (std::size_t i = 0; i < n; ++i)
    {
      robot_target_[i] = target[i];
    }
  }

  inline void PinocchioController::SetHandTarget(
      std::span<const double> target) noexcept
  {
    const std::size_t n = std::min(target.size(), kNumHandJoints);
    for (std::size_t i = 0; i < n; ++i)
    {
      hand_target_[i] = target[i];
    }
  }

  inline std::string_view PinocchioController::Name() const noexcept
  {
    return "PinocchioController";
  }

  inline void PinocchioController::TriggerEstop() noexcept
  {
    estopped_.store(true, std::memory_order_relaxed);
  }

  inline void PinocchioController::ClearEstop() noexcept
  {
    estopped_.store(false, std::memory_order_relaxed);
    prev_error_ = {}; // reset derivative term on recovery
  }

  inline bool PinocchioController::IsEstopped() const noexcept
  {
    return estopped_.load(std::memory_order_relaxed);
  }

  inline void PinocchioController::SetHandEstop(bool active) noexcept
  {
    hand_estopped_.store(active, std::memory_order_relaxed);
  }

  inline std::array<double, kNumRobotJoints>
  PinocchioController::gravity_torques() const noexcept
  {
    return gravity_torques_;
  }

  inline std::array<double, 3> PinocchioController::tcp_position() const noexcept
  {
    return tcp_position_;
  }

  // ── Private helpers ──────────────────────────────────────────────────────────

  inline ControllerOutput PinocchioController::ComputeEstop(
      const ControllerState &state) noexcept
  {
    ControllerOutput output;
    for (std::size_t i = 0; i < kNumRobotJoints; ++i)
    {
      output.robot_commands[i] =
          gains_.kp * (kSafePosition[i] - state.robot.positions[i]);
    }
    output.robot_commands = ClampCommands(output.robot_commands);
    return output;
  }

  inline std::array<double, kNumRobotJoints> PinocchioController::ClampCommands(
      std::array<double, kNumRobotJoints> cmds) noexcept
  {
    for (auto &c : cmds)
    {
      c = std::clamp(c, -kMaxJointVelocity, kMaxJointVelocity);
    }
    return cmds;
  }

  inline void PinocchioController::UpdateDynamics(
      const RobotState &robot) noexcept
  {
    // Copy std::array → Eigen vectors (no allocation — writes into pre-allocated
    // q_ and v_ which are already the correct size from the constructor).
    const std::size_t nv =
        static_cast<std::size_t>(model_.nv);
    const std::size_t n = std::min(kNumRobotJoints, nv);

    for (std::size_t i = 0; i < n; ++i)
    {
      q_[static_cast<Eigen::Index>(i)] = robot.positions[i];
      v_[static_cast<Eigen::Index>(i)] = robot.velocities[i];
    }

    // ── Gravity torque vector g(q) ─────────────────────────────────────────
    // computeGeneralizedGravity runs RNEA with v=0, a=0.
    // It stores the result in data_.g and returns a const ref to it.
    if (gains_.enable_gravity_compensation)
    {
      const Eigen::VectorXd &g =
          pinocchio::computeGeneralizedGravity(model_, data_, q_);
      for (std::size_t i = 0; i < n; ++i)
      {
        gravity_torques_[i] = g[static_cast<Eigen::Index>(i)];
      }
    }

    // ── Forward kinematics ────────────────────────────────────────────────
    // forwardKinematics updates data_.oMi (joint placements in world frame).
    pinocchio::forwardKinematics(model_, data_, q_, v_);

    // ── TCP position — last movable joint's world-frame translation ───────
    // For UR5e the last joint index is model_.njoints - 1.
    // Use pinocchio::updateFramePlacements if a named "tool0" frame exists.
    const auto last_joint =
        static_cast<pinocchio::JointIndex>(model_.njoints - 1);
    const pinocchio::SE3 &tcp = data_.oMi[last_joint];
    tcp_position_[0] = tcp.translation()[0];
    tcp_position_[1] = tcp.translation()[1];
    tcp_position_[2] = tcp.translation()[2];

    // ── End-effector Jacobian ─────────────────────────────────────────────
    // computeJointJacobian requires an updated kinematics call (done above).
    // The result is a 6×nv matrix stored in data_.J via jacobian_.
    pinocchio::computeJointJacobian(model_, data_, q_, last_joint, jacobian_);

    // ── Coriolis / centrifugal forces  C(q,v)·v ──────────────────────────
    if (gains_.enable_coriolis_compensation)
    {
      const Eigen::MatrixXd &C =
          pinocchio::computeCoriolisMatrix(model_, data_, q_, v_);
      // noalias() avoids a temporary allocation — result written directly
      // into the pre-allocated coriolis_forces_ buffer.
      coriolis_forces_.noalias() = C * v_;
    }
  }

} // namespace ur5e_rt_controller
