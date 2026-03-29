// ── Includes: project header first, then third-party, then C++ stdlib ──────────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"

// Suppress warnings emitted by Pinocchio / Eigen headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>       // updateFramePlacement, getFrameJacobian
#include <pinocchio/algorithm/jacobian.hpp>     // computeJointJacobians, getJointJacobian
#include <pinocchio/algorithm/kinematics.hpp>   // forwardKinematics (via computeJointJacobians)
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
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
using rtc::kMaxDeviceChannels;
using rtc::RTControllerInterface;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::CommandType;
using rtc::GoalType;
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
///   `[kp_translation×3, kp_rotation×3, damping, null_kp, enable_null_space(0/1),
///    control_6dof(0/1), trajectory_speed, trajectory_angular_speed,
///    hand_trajectory_speed, max_traj_velocity, max_traj_angular_velocity,
///    hand_max_traj_velocity]` = 16 values
class DemoTaskController final : public RTControllerInterface {
public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains
  {
    // Arm (CLIK) gains — translation / rotation separated
    std::array<double, 3> kp_translation{{1.0, 1.0, 1.0}}; ///< Translation proportional gain (x,y,z) [1/s]
    std::array<double, 3> kp_rotation{{1.0, 1.0, 1.0}};    ///< Rotation proportional gain (rx,ry,rz) [1/s]
    double damping{0.01};            ///< Damping factor λ for J^#
    double null_kp{0.5};             ///< Null-space joint-centering gain [1/s]
    bool   enable_null_space{true};  ///< Enable null-space secondary task
    bool   control_6dof{false};      ///< Enable 6-DOF (translation + orientation) control

    // Trajectory speed
    double trajectory_speed{0.1};           ///< TCP translational speed for trajectory duration [m/s]
    double trajectory_angular_speed{0.5};   ///< TCP rotational speed for trajectory duration [rad/s]
    double hand_trajectory_speed{1.0};      ///< Hand motor speed for trajectory duration [rad/s]

    // Trajectory velocity limits
    double max_traj_velocity{0.5};             ///< Max TCP velocity during task-space trajectory [m/s]
    double max_traj_angular_velocity{1.0};     ///< Max TCP angular velocity during trajectory [rad/s]
    double hand_max_traj_velocity{2.0};        ///< Max hand motor velocity during trajectory [rad/s]
  };

  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      Gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit DemoTaskController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState & state) noexcept override;

  void SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept override;

  void InitializeHoldPosition(const ControllerState & state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //                enable_null_space(0/1), control_6dof(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                hand_trajectory_speed, max_traj_velocity,
  //                max_traj_angular_velocity, hand_max_traj_velocity] = 16 values
  void LoadConfig(const YAML::Node & cfg) override;
  void OnDeviceConfigsSet() override;
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
  // ── Phase 1→2 intermediate: parsed sensor data ──────────────────────────
  struct FingertipSensorData {
    std::array<int32_t, rtc::kBarometerCount> baro{};
    std::array<int32_t, 3> tof{};
    std::array<float, 3> force{};
    std::array<float, 3> displacement{};
    float contact_flag{0.0f};
    bool valid{false};
  };
  std::array<FingertipSensorData, rtc::kMaxFingertips> fingertip_data_{};
  int num_active_fingertips_{0};
  rtc::GraspStateData grasp_state_{};

  // ── Phase 2→3 intermediate: computed trajectory results ─────────────────
  struct ComputedTrajectory {
    std::array<double, kMaxDeviceChannels> positions{};
    std::array<double, kMaxDeviceChannels> velocities{};
  };
  ComputedTrajectory hand_computed_{};
  bool estop_active_{false};

  // ── 3-phase pipeline ────────────────────────────────────────────────────
  void ReadState(const ControllerState & state) noexcept;
  void ComputeControl(const ControllerState & state, double dt) noexcept;
  [[nodiscard]] ControllerOutput WriteOutput(const ControllerState & state, double dt) noexcept;

  // ── Pinocchio model + pre-allocated Data ─────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::JointIndex end_id_{0};
  pinocchio::FrameIndex tip_frame_id_{0};
  bool                  use_frame_fk_{false};  // true when tip_link resolves to an operational frame

  // ── Pre-allocated Eigen work buffers — zero heap alloc on the RT path ────
  Eigen::VectorXd q_;
  Eigen::MatrixXd J_full_;
  Eigen::MatrixXd J_pos_;
  Eigen::Matrix3d JJt_;
  Eigen::Matrix3d JJt_inv_;
  Eigen::MatrixXd Jpinv_;
  Eigen::MatrixXd N_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd traj_dq_;       // feedforward-only trajectory velocity (for logging)
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
  std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> device_targets_{};
  std::array<double, 3> tcp_position_{};

  bool target_initialized_{false};
  std::atomic<bool> new_target_{false};
  std::mutex target_mutex_;
  trajectory::TaskSpaceTrajectory trajectory_;
  trajectory::TaskSpaceTrajectory::State traj_state_{};
  double trajectory_time_{0.0};
  trajectory::JointSpaceTrajectory<kNumHandMotors> hand_trajectory_;
  double hand_trajectory_time_{0.0};
  std::atomic<bool> hand_new_target_{false};

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_max_velocity_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_lower_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_upper_;

  CommandType command_type_{CommandType::kPosition};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState & state) noexcept;

  static void ClampCommands(
    std::array<double, kMaxDeviceChannels>& commands, int n,
    const std::vector<double>& lower,
    const std::vector<double>& upper) noexcept;
};

}  // namespace ur5e_bringup
