// ── Includes: project header first, then third-party, then C++ stdlib ──────────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"

// Suppress warnings emitted by Pinocchio / Eigen headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>     // computeJointJacobians, getJointJacobian
#include <pinocchio/algorithm/kinematics.hpp>   // forwardKinematics (via computeJointJacobians)
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

#include <Eigen/Cholesky>   // LDLT
#include <Eigen/Core>

#include <array>
#include <atomic>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ur5e_bringup
{

using rtc::kNumRobotJoints;
using rtc::kNumHandMotors;
using rtc::RTControllerInterface;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::CommandType;
namespace trajectory = rtc::trajectory;

/// Demo Task-Space Controller: CLIK (arm) + P control (hand).
///
/// Controls the end-effector in Cartesian space via damped Jacobian
/// pseudoinverse, while simultaneously running proportional position
/// control on the 10-DOF hand motors.
///
/// ### Arm control law (same as ClikController)
/// @code
///   pos_error    = x_des − FK(q)
///   J^#          = J^T (J J^T + λ²I)^{−1}         [damped pseudoinverse]
///   N            = I − J^# J                        [null-space projector]
///   dq           = kp · J^# · pos_error + null_kp · N · (q_null − q)
///   q_cmd        = q + clamp(dq, ±v_max) * dt
/// @endcode
///
/// ### Hand control law (same as DemoJointController)
/// @code
///   hand_cmd[i]  = hand_pos[i] + hand_kp[i] * (hand_target[i] - hand_pos[i]) * dt
/// @endcode
///
/// ### Target convention (`SetRobotTarget` / `/target_joint_positions`)
///   - 3-DOF mode: `target[0..2]` = TCP position [x,y,z], `target[3..5]` = null-space ref
///   - 6-DOF mode: `target[0..2]` = TCP position [x,y,z], `target[3..5]` = [roll,pitch,yaw]
///
/// ### Gains layout for UpdateGainsFromMsg
///   `[kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1), hand_kp×10]` = 20 values
class DemoTaskController final : public RTControllerInterface {
public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains
  {
    // Arm (CLIK) gains
    std::array<double, 6> kp{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}; ///< Cartesian position/orientation gain [1/s]
    double damping{0.01};            ///< Damping factor λ for J^#
    double null_kp{0.5};             ///< Null-space joint-centering gain [1/s]
    bool   enable_null_space{true};  ///< Enable null-space secondary task
    double trajectory_speed{0.1};    ///< Max translational speed for trajectory [m/s]
    bool   control_6dof{false};      ///< Enable 6-DOF (translation + orientation) control

    // Hand gains
    std::array<float, kNumHandMotors> hand_kp{{
      50.0f, 50.0f, 50.0f, 50.0f, 50.0f,
      50.0f, 50.0f, 50.0f, 50.0f, 50.0f}};
  };

  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      Gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit DemoTaskController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState & state) noexcept override;

  void SetRobotTarget(std::span<const double, kNumRobotJoints> target) noexcept override;
  void SetHandTarget(std::span<const float, kNumHandMotors> target)  noexcept override;

  void InitializeHoldPosition(const ControllerState & state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1), hand_kp×10] = 20 values
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override {return command_type_;}

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains & g) noexcept {gains_ = g;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept
  {
    return tcp_position_;
  }

  [[nodiscard]] std::array<double, 3> position_error() const noexcept
  {
    return {pos_error_[0], pos_error_[1], pos_error_[2]};
  }

private:
  // ── Pinocchio model + pre-allocated Data ─────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::JointIndex end_id_{0};

  // ── Pre-allocated Eigen work buffers — zero heap alloc on the RT path ────
  Eigen::VectorXd q_;
  Eigen::MatrixXd J_full_;
  Eigen::MatrixXd J_pos_;
  Eigen::Matrix3d JJt_;
  Eigen::Matrix3d JJt_inv_;
  Eigen::MatrixXd Jpinv_;
  Eigen::MatrixXd N_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd null_err_;
  Eigen::VectorXd null_dq_;
  Eigen::Vector3d pos_error_;
  Eigen::Matrix<double, 6, 6> JJt_6d_;
  Eigen::Matrix<double, 6, 6> JJt_inv_6d_;
  Eigen::MatrixXd Jpinv_6d_;
  Eigen::Matrix<double, 6, 1> pos_error_6d_;

  Eigen::LDLT<Eigen::Matrix3d> ldlt_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_6d_;

  // ── Controller state ──────────────────────────────────────────────────────
  Gains gains_;
  pinocchio::SE3 tcp_target_pose_{pinocchio::SE3::Identity()};
  std::array<double, 3> tcp_target_{};
  std::array<double, kNumRobotJoints> null_target_{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  std::array<float, kNumHandMotors> hand_target_{};
  std::array<double, 3> tcp_position_{};

  bool target_initialized_{false};
  std::atomic<bool> new_target_{false};
  std::mutex target_mutex_;
  trajectory::TaskSpaceTrajectory trajectory_;
  double trajectory_time_{0.0};

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity{2.0};
  static constexpr float  kMaxHandVelocity{1.0f};

  CommandType command_type_{CommandType::kPosition};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState & state) noexcept;

  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampVelocity(
    std::array<double, kNumRobotJoints> dq) noexcept;

  [[nodiscard]] static std::array<float, kNumHandMotors> ClampHandCommands(
    std::span<const float, kNumHandMotors> commands) noexcept;
};

}  // namespace ur5e_bringup
