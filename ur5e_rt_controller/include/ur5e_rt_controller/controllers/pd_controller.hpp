#ifndef UR5E_RT_CONTROLLER_CONTROLLERS_PD_CONTROLLER_H_
#define UR5E_RT_CONTROLLER_CONTROLLERS_PD_CONTROLLER_H_

#include <array>
#include <atomic>
#include <span>
#include <string_view>
#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include "ur5e_rt_controller/trajectory/joint_space_trajectory.hpp"

namespace ur5e_rt_controller
{

// Proportional-Derivative (PD) position controller with E-STOP support.
//
// Computes: command[i] = kp * e[i] + kd * de[i]/dt
//
// On E-STOP, commands drive the robot toward kSafePosition instead of the
// user-specified target, allowing a controlled stop.
class PDController final : public RTControllerInterface {
public:
  // Aggregated gains — supports C++20 designated initialiser at call site:
  //   PDController ctrl{{.kp = 5.0, .kd = 0.5}};
  struct Gains
  {
    std::array<double, 6> kp{{5.0, 5.0, 5.0, 5.0, 5.0, 5.0}};
    std::array<double, 6> kd{{0.5, 0.5, 0.5, 0.5, 0.5, 0.5}};
    double trajectory_speed{1.0};
  };

  PDController() noexcept;
  explicit PDController(Gains gains) noexcept;

  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetRobotTarget(
    std::span<const double, kNumRobotJoints> target) noexcept override;

  void SetHandTarget(
    std::span<const double, kNumHandJoints> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "PDController";
  }

  // ── E-STOP interface ────────────────────────────────────────────────────────
  void TriggerEstop() noexcept override;
  void ClearEstop()   noexcept override;
  [[nodiscard]] bool IsEstopped()    const noexcept override;
  void SetHandEstop(bool enabled)    noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp×6, kd×6]
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;

  // ── Gain accessors ──────────────────────────────────────────────────────────
  void set_gains(Gains gains) noexcept {gains_ = gains;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}
  [[nodiscard]] Gains gains()  const noexcept {return gains_;}

private:
  // Safe joint configuration used when E-STOP is active [rad].
  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity = 2.0;  // rad/s

  Gains  gains_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<double, kNumHandJoints> hand_target_{};
  std::array<double, kNumRobotJoints> previous_errors_{};

  bool new_target_{false};
  trajectory::JointSpaceTrajectory<kNumRobotJoints> trajectory_;
  double trajectory_time_{0.0};

  // Atomic flags allow safe access from both the RT control thread and the
  // 50 Hz timeout-monitor thread.
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampCommands(
    std::span<const double, kNumRobotJoints> commands) noexcept;

  [[nodiscard]] static double ComputeDerivative(
    double current_error, double previous_error, double dt) noexcept;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_CONTROLLERS_PD_CONTROLLER_H_
