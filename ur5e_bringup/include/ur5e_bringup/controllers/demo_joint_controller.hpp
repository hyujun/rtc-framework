#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_

#include <array>
#include <atomic>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"

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

// Unified trajectory-based position controller for UR5e arm + hand.
//
// Generates quintic rest-to-rest trajectories to smoothly move from the
// current position to each new target. No proportional gain is applied;
// the trajectory output is sent directly as the position command.
//
// Target message layout for /target_joint_positions (Float64MultiArray):
//   data[0..5]  : robot arm joint targets (rad)
//   data[6..15] : hand motor targets (rad), optional — ignored if size < 16
//
// Gains layout for UpdateGainsFromMsg:
//   [robot_trajectory_speed, hand_trajectory_speed,
//    robot_max_traj_velocity, hand_max_traj_velocity] = 4 values
class DemoJointController final : public RTControllerInterface {
public:
  struct Gains
  {
    double robot_trajectory_speed{1.0};       ///< Desired joint speed for trajectory duration [rad/s]
    double hand_trajectory_speed{1.0};        ///< Desired hand speed for trajectory duration [rad/s]
    double robot_max_traj_velocity{3.14};     ///< Max joint velocity during trajectory [rad/s]
    double hand_max_traj_velocity{2.0};       ///< Max hand motor velocity during trajectory [rad/s]
  };

  explicit DemoJointController(std::string_view urdf_path);
  DemoJointController(std::string_view urdf_path, Gains gains);

  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept override;

  void InitializeHoldPosition(
    const ControllerState & state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "DemoJointController";
  }

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [robot_trajectory_speed, hand_trajectory_speed,
  //                robot_max_traj_velocity, hand_max_traj_velocity] = 4 values
  void LoadConfig(const YAML::Node & cfg) override;
  void OnDeviceConfigsSet() override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override {return command_type_;}

  void set_gains(Gains gains) noexcept {gains_ = gains;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

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
  ComputedTrajectory robot_computed_{};
  ComputedTrajectory hand_computed_{};

  // ── 3-phase pipeline ────────────────────────────────────────────────────
  void ReadState(const ControllerState & state) noexcept;
  void ComputeControl(const ControllerState & state, double dt) noexcept;
  [[nodiscard]] ControllerOutput WriteOutput(const ControllerState & state, double dt) noexcept;

  // ── Internal state ──────────────────────────────────────────────────────
  Gains gains_;
  std::array<std::array<double, rtc::kMaxDeviceChannels>, ControllerState::kMaxDevices> device_targets_{};

  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::JointIndex end_id_{0};
  pinocchio::FrameIndex tip_frame_id_{0};
  bool                  use_frame_fk_{false};  // true when tip_link resolves to an operational frame
  Eigen::VectorXd       q_;

  CommandType command_type_{CommandType::kPosition};

  // ── Trajectory ───────────────────────────────────────────────────────────
  std::mutex target_mutex_;
  std::atomic<bool> robot_new_target_{false};
  std::atomic<bool> hand_new_target_{false};
  trajectory::JointSpaceTrajectory<kNumRobotJoints> robot_trajectory_;
  trajectory::JointSpaceTrajectory<kNumHandMotors>  hand_trajectory_;
  double robot_trajectory_time_{0.0};
  double hand_trajectory_time_{0.0};

  std::array<std::vector<double>, ControllerState::kMaxDevices> device_max_velocity_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_lower_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_upper_;
  static void ClampCommands(
    std::array<double, kMaxDeviceChannels>& commands, int n,
    const std::vector<double>& lower,
    const std::vector<double>& upper) noexcept;

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};
  bool estop_active_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};

  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState & state) noexcept;
};

}  // namespace ur5e_bringup

#endif  // UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_
