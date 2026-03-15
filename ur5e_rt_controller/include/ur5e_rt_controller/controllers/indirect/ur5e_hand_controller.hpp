#ifndef UR5E_RT_CONTROLLER_CONTROLLERS_UR5E_HAND_CONTROLLER_H_
#define UR5E_RT_CONTROLLER_CONTROLLERS_UR5E_HAND_CONTROLLER_H_

#include <array>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include "ur5e_rt_controller/rt_controller_interface.hpp"

namespace ur5e_rt_controller
{

// Unified Proportional (P) position controller for UR5e arm + hand.
//
// Robot arm control law (identical to PController):
//   robot_command[i] = current_pos[i] + robot_kp[i] * (robot_target[i] - current_pos[i]) * dt
//   Output clamped to [-kMaxJointVelocity, +kMaxJointVelocity] rad/s
//
// Hand motor control law (same formula applied to hand motors):
//   hand_command[i] = hand_pos[i] + hand_kp[i] * (hand_target[i] - hand_pos[i]) * dt
//   Output clamped to [-kMaxHandVelocity, +kMaxHandVelocity] rad/s
//
// Target message layout for /target_joint_positions (Float64MultiArray):
//   data[0..5]  : robot arm joint targets (rad)
//   data[6..15] : hand motor targets (rad), optional — ignored if size < 16
//
// Gains layout for UpdateGainsFromMsg: [robot_kp×6, hand_kp×10] = 16 values
class UrFiveEHandController final : public RTControllerInterface {
public:
  struct Gains
  {
    std::array<double, kNumRobotJoints> robot_kp{{120.0, 120.0, 100.0, 80.0, 80.0, 80.0}};
    std::array<float, kNumHandMotors>   hand_kp{{
      50.0f, 50.0f, 50.0f, 50.0f, 50.0f,
      50.0f, 50.0f, 50.0f, 50.0f, 50.0f}};
  };

  explicit UrFiveEHandController(std::string_view urdf_path);
  UrFiveEHandController(std::string_view urdf_path, Gains gains);

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
    return "UrFiveEHandController";
  }

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [robot_kp×6, hand_kp×10] = 16 values
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override {return command_type_;}

  void set_gains(Gains gains) noexcept {gains_ = gains;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

private:
  Gains gains_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<float, kNumHandMotors>   hand_target_{};

  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::JointIndex end_id_{0};
  Eigen::VectorXd       q_;

  CommandType command_type_{CommandType::kPosition};

  // Clamps each robot command to [-kMaxJointVelocity, +kMaxJointVelocity].
  static constexpr double kMaxJointVelocity = 2.0;   // rad/s
  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampRobotCommands(
    std::span<const double, kNumRobotJoints> commands) noexcept;

  // Clamps each hand command to [-kMaxHandVelocity, +kMaxHandVelocity].
  static constexpr float kMaxHandVelocity = 1.0f;    // rad/s
  [[nodiscard]] static std::array<float, kNumHandMotors> ClampHandCommands(
    std::span<const float, kNumHandMotors> commands) noexcept;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_CONTROLLERS_UR5E_HAND_CONTROLLER_H_
