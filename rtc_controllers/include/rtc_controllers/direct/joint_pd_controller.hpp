#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <mutex>
#include <span>
#include <string_view>
#include <vector>

namespace rtc
{

/// Joint-space PD controller with optional gravity/Coriolis compensation.
///
/// Control law:
/// @code
///   τ[i] = ff_vel[i] + Kp[i]*e[i] + Kd[i]*ė[i]  [+ g(q)[i]]  [+ C(q,v)·v [i]]
/// @endcode
///
/// UpdateGainsFromMsg layout: [kp×6, kd×6, gravity(0/1), coriolis(0/1), trajectory_speed]
class JointPDController final : public RTControllerInterface
{
public:
  struct Gains
  {
    std::array<double, 6> kp{{200.0, 200.0, 150.0, 120.0, 120.0, 120.0}};
    std::array<double, 6> kd{{30.0, 30.0, 25.0, 20.0, 20.0, 20.0}};
    bool   enable_gravity_compensation{false};
    bool   enable_coriolis_compensation{false};
    double trajectory_speed{1.0};
  };

  explicit JointPDController(std::string_view urdf_path);
  JointPDController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface ──────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetRobotTarget(
    std::span<const double, kNumRobotJoints> target) noexcept override;

  void SetHandTarget(
    std::span<const float, kNumHandMotors> target) noexcept override;

  void InitializeHoldPosition(
    const ControllerState & state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "JointPDController";
  }

  // ── E-STOP ─────────────────────────────────────────────────────────────────
  void TriggerEstop() noexcept override;
  void ClearEstop()   noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Controller registry hooks ──────────────────────────────────────────────
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override
  {
    return command_type_;
  }

  // ── Gain accessors ─────────────────────────────────────────────────────────
  void  set_gains(const Gains & g) noexcept {gains_ = g;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

  // ── Diagnostic accessors (non-RT only) ─────────────────────────────────────
  [[nodiscard]] std::array<double, kNumRobotJoints> gravity_torques() const noexcept;
  [[nodiscard]] std::array<double, 3>               tcp_position()    const noexcept;
  [[nodiscard]] Eigen::MatrixXd                     jacobian()        const noexcept
  {
    return jacobian_;
  }

private:
  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity{2.0};
  static constexpr double kMaxJointTorque{150.0};

  // ── Pinocchio model and work buffers ───────────────────────────────────────
  pinocchio::Model model_;
  pinocchio::Data  data_;

  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd coriolis_forces_;
  Eigen::MatrixXd jacobian_;

  // ── Controller state ───────────────────────────────────────────────────────
  Gains  gains_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<float, kNumHandMotors>   hand_target_{};
  std::array<double, kNumRobotJoints> prev_error_{};

  std::mutex target_mutex_;
  std::atomic<bool> new_target_{false};
  trajectory::JointSpaceTrajectory<kNumRobotJoints> trajectory_;
  double trajectory_time_{0.0};

  // ── Diagnostic caches ──────────────────────────────────────────────────────
  std::array<double, kNumRobotJoints> gravity_torques_{};
  std::array<double, 3>               tcp_position_{};

  // ── E-STOP ─────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  CommandType command_type_{CommandType::kTorque};

  // ── Internal helpers ───────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(
    const ControllerState & state) noexcept;

  void UpdateDynamics(const RobotState & robot) noexcept;

  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampCommands(
    std::array<double, kNumRobotJoints> cmds, CommandType type) noexcept;
};

}  // namespace rtc
