#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"

#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <memory>
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
/// UpdateGainsFromMsg layout: [kp×nv, kd×nv, gravity(0/1), coriolis(0/1), trajectory_speed]
/// where nv = model DOF (e.g. 6 for UR5e, determined at construction from URDF)
class JointPDController final : public RTControllerInterface
{
public:
  struct Gains
  {
    std::array<double, kMaxRobotDOF> kp{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
                                          100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
    std::array<double, kMaxRobotDOF> kd{{20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
                                          20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
    bool   enable_gravity_compensation{false};
    bool   enable_coriolis_compensation{false};
    double trajectory_speed{1.0};
  };

  explicit JointPDController(std::string_view urdf_path);
  JointPDController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface ──────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept override;

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
  void OnDeviceConfigsSet() override;
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
  [[nodiscard]] std::array<double, kMaxRobotDOF> gravity_torques() const noexcept;
  [[nodiscard]] std::array<double, 3>               tcp_position()    const noexcept;
  [[nodiscard]] Eigen::MatrixXd                     jacobian()        const noexcept
  {
    return jacobian_;
  }

private:
  std::vector<double> safe_position_;
  std::vector<double> max_joint_velocity_;
  std::vector<double> max_joint_torque_;

  // ── Pinocchio via rtc_urdf_bridge ────────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  Eigen::VectorXd coriolis_forces_;
  Eigen::MatrixXd jacobian_;

  // ── Controller state ───────────────────────────────────────────────────────
  Gains  gains_;
  std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> device_targets_{};
  std::array<double, kMaxRobotDOF> prev_error_{};

  std::mutex target_mutex_;
  std::atomic<bool> new_target_{false};
  trajectory::JointSpaceTrajectory<kMaxRobotDOF> trajectory_;
  double trajectory_time_{0.0};

  // ── Diagnostic caches ──────────────────────────────────────────────────────
  std::array<double, kMaxRobotDOF> gravity_torques_{};
  std::array<double, 3>               tcp_position_{};

  // ── E-STOP ─────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  CommandType command_type_{CommandType::kTorque};

  // ── Internal helpers ───────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(
    const ControllerState & state) noexcept;

  void UpdateDynamics(const DeviceState & dev) noexcept;

  void ClampCommands(
    std::array<double, kMaxDeviceChannels>& cmds, int n, CommandType type) const noexcept;
};

}  // namespace rtc
