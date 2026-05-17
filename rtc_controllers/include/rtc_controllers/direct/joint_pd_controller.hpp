#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include <rtc_base/threading/seqlock.hpp>
#include <rtc_base/concurrency/spsc_queue.hpp>
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <memory>
#include <span>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rtc {

/// Joint-space PD controller with optional gravity/Coriolis compensation.
///
/// Control law:
/// @code
///   τ[i] = ff_vel[i] + Kp[i]*e[i] + Kd[i]*ė[i]  [+ g(q)[i]]  [+ C(q,v)·v [i]]
/// @endcode
///
/// Gains struct (loaded once via LoadConfig, fixed at runtime — JointPD does
/// not expose runtime parameter channel; demo controllers do): kp[nv], kd[nv],
/// enable_gravity, enable_coriolis, trajectory_speed where nv = model DOF
/// (e.g. 6 for UR5e, determined at construction from URDF).
class JointPDController final : public RTControllerInterface {
 public:
  struct Gains {
    std::array<double, kMaxRobotDOF> kp{
        {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
    std::array<double, kMaxRobotDOF> kd{
        {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
    bool enable_gravity_compensation{false};
    bool enable_coriolis_compensation{false};
    double trajectory_speed{1.0};
  };

  explicit JointPDController(std::string_view urdf_path);
  JointPDController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface ──────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override { return "JointPDController"; }

  // ── E-STOP ─────────────────────────────────────────────────────────────────
  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Controller registry hooks ──────────────────────────────────────────────
  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  // ── Gain accessors ─────────────────────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_lock_.Store(g); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  // ── Diagnostic accessors (non-RT only) ─────────────────────────────────────
  [[nodiscard]] std::array<double, kMaxRobotDOF> gravity_torques() const noexcept;
  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept;

  [[nodiscard]] Eigen::MatrixXd jacobian() const noexcept { return jacobian_; }

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
  SeqLock<Gains> gains_lock_;

  // TargetSlot holds the per-device joint target arrays. The RT thread is
  // the SOLE writer of target_seqlock_ (writes happen inside Compute via the
  // SPSC drain + self-init paths). Off-RT writers (ROS sub callbacks reaching
  // SetDeviceTarget) push onto pending_targets_, never touching target_seqlock_
  // directly. This single-writer invariant is what makes the SeqLock safe.
  struct TargetSlot {
    std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> targets{};
  };

  static_assert(std::is_trivially_copyable_v<TargetSlot>,
                "TargetSlot must be trivially copyable for SeqLock<TargetSlot>");

  struct PendingTarget {
    int device_idx{0};
    int num_values{0};
    std::array<double, kMaxDeviceChannels> values{};
  };

  static_assert(std::is_trivially_copyable_v<PendingTarget>,
                "PendingTarget must be trivially copyable for SpscQueue");

  static constexpr std::size_t kPendingTargetDepth = 4;

  SeqLock<TargetSlot> target_seqlock_;
  SpscQueue<PendingTarget, kPendingTargetDepth> pending_targets_;
  // Flipping target_initialized_ back to false from off-RT (e.g. ClearEstop)
  // makes the next Compute() re-run the self-init path, seeding the slot from
  // the current state and rebuilding the trajectory. RT thread is still the
  // sole writer of target_seqlock_ itself; this atomic is just the signal.
  std::atomic<bool> target_initialized_{false};
  bool new_target_pending_{false};  // RT-thread-only; gates trajectory re-init

  std::array<double, kMaxRobotDOF> prev_error_{};

  trajectory::JointSpaceTrajectory<kMaxRobotDOF> trajectory_;
  double trajectory_time_{0.0};

  // ── Diagnostic caches ──────────────────────────────────────────────────────
  std::array<double, kMaxRobotDOF> gravity_torques_{};
  std::array<double, 3> tcp_position_{};

  // ── E-STOP ─────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  CommandType command_type_{CommandType::kTorque};

  // ── Internal helpers ───────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  void UpdateDynamics(const DeviceState& dev, const Gains& gains) noexcept;

  void ClampCommands(std::array<double, kMaxDeviceChannels>& cmds, int n,
                     CommandType type) const noexcept;
};

}  // namespace rtc
