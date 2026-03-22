#ifndef RTC_CONTROLLERS_P_CONTROLLER_H_
#define RTC_CONTROLLERS_P_CONTROLLER_H_

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

#include "rtc_controller_interface/rt_controller_interface.hpp"

namespace rtc
{

// Proportional (P) position controller.
//
// Computes: command[i] = kp * (target[i] - current[i])
// Output is clamped to joint velocity limits before publishing.
class PController final : public RTControllerInterface {
public:
  struct Gains
  {
    std::array<double, 6> kp{{120.0, 120.0, 100.0, 80.0, 80.0, 80.0}};
  };

  explicit PController(std::string_view urdf_path);
  PController(std::string_view urdf_path, Gains gains);

  template<typename T>
  explicit PController(std::string_view urdf_path, T kp)
  : PController(urdf_path)
  {
    gains_.kp.fill(static_cast<double>(kp));
  }

  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept override;

  void InitializeHoldPosition(
    const ControllerState & state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "PController";
  }

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp×6]
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override {return command_type_;}

  // Accessors (Google C++ Style: getter matches member name w/o trailing _).
  void set_gains(Gains gains) noexcept {gains_ = gains;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}
  void set_kp(double kp) noexcept {gains_.kp.fill(kp);}

private:
  Gains gains_;
  std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> device_targets_{};

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::JointIndex end_id_{0};
  Eigen::VectorXd q_;

  CommandType command_type_{CommandType::kPosition};

  // Clamps each command to [-kMaxJointVelocity, +kMaxJointVelocity].
  static constexpr double kMaxJointVelocity = 2.0;  // rad/s
  static void ClampCommands(
    std::array<double, kMaxDeviceChannels>& commands, int n) noexcept;
};

}  // namespace rtc

#endif  // RTC_CONTROLLERS_P_CONTROLLER_H_
