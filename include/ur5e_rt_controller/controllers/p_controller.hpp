#ifndef UR5E_RT_CONTROLLER_CONTROLLERS_P_CONTROLLER_H_
#define UR5E_RT_CONTROLLER_CONTROLLERS_P_CONTROLLER_H_

#include <array>
#include <span>
#include <string_view>

#include "ur5e_rt_controller/rt_controller_interface.hpp"

namespace ur5e_rt_controller {

// Proportional (P) position controller.
//
// Computes: command[i] = kp * (target[i] - current[i])
// Output is clamped to joint velocity limits before publishing.
class PController final : public RTControllerInterface {
 public:
  // kp must be >= 0; validated with a C++20 requires-clause at the call site.
  template <NonNegativeFloat T>
  explicit PController(T kp) noexcept : kp_(static_cast<double>(kp)) {}

  [[nodiscard]] ControllerOutput Compute(
      const ControllerState& state) noexcept override;

  void SetRobotTarget(
      std::span<const double, kNumRobotJoints> target) noexcept override;

  void SetHandTarget(
      std::span<const double, kNumHandJoints> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override {
    return "PController";
  }

  // Accessors (Google C++ Style: getter matches member name w/o trailing _).
  void   set_kp(double kp) noexcept { kp_ = kp; }
  [[nodiscard]] double kp() const noexcept { return kp_; }

 private:
  double kp_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<double, kNumHandJoints>  hand_target_{};

  // Clamps each command to [-kMaxJointVelocity, +kMaxJointVelocity].
  static constexpr double kMaxJointVelocity = 2.0;  // rad/s
  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampCommands(
      std::span<const double, kNumRobotJoints> commands) noexcept;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_CONTROLLERS_P_CONTROLLER_H_
