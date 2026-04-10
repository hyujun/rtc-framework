#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "ur5e_bringup/controllers/virtual_tcp.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"
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
//    robot_max_traj_velocity, hand_max_traj_velocity,
//    grasp_contact_threshold, grasp_force_threshold,
//    grasp_min_fingertips,
//    grasp_command, grasp_target_force] = 9 values
class DemoJointController final : public RTControllerInterface {
public:
  struct Gains
  {
    double robot_trajectory_speed{1.0};       ///< Desired joint speed for trajectory duration [rad/s]
    double hand_trajectory_speed{1.0};        ///< Desired hand speed for trajectory duration [rad/s]
    double robot_max_traj_velocity{3.14};     ///< Max joint velocity during trajectory [rad/s]
    double hand_max_traj_velocity{2.0};       ///< Max hand motor velocity during trajectory [rad/s]

    // Virtual TCP (fingertip-based control point)
    VirtualTcpConfig vtcp;

    // Grasp detection parameters
    float grasp_contact_threshold{0.5f};      ///< Contact probability threshold (0.0~1.0)
    float grasp_force_threshold{1.0f};        ///< Force magnitude threshold [N]
    int   grasp_min_fingertips{2};            ///< Min fingertips for grasp detection
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
  //                robot_max_traj_velocity, hand_max_traj_velocity,
  //                grasp_contact_threshold, grasp_force_threshold,
  //                grasp_min_fingertips,
  //                grasp_command, grasp_target_force] = 9 values
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
  rtc::ToFSnapshotData tof_snapshot_{};

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

  // ── rtc_urdf_bridge ────────────────────────────────────────────
  std::string urdf_path_;  // stored from constructor, used in LoadConfig
  std::unique_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> arm_handle_;
  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::FrameIndex root_frame_id_{0};
  bool                  use_root_frame_{false};
  // ── Hand tree-model for fingertip FK ──────────────────────────────────
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> hand_handle_;
  static constexpr std::size_t kNumFingertips = 4;
  std::array<pinocchio::FrameIndex, kNumFingertips> fingertip_frame_ids_{};
  pinocchio::FrameIndex hand_root_frame_id_{0};
  bool use_hand_root_frame_{false};
  std::array<Eigen::Vector3d, kNumFingertips> fingertip_positions_{};
  std::array<Eigen::Matrix3d, kNumFingertips> fingertip_rotations_{};
  Eigen::VectorXd hand_q_;  // pre-allocated for hand FK

  // ── Virtual TCP (fingertip-based control point) ───────────────────────
  pinocchio::SE3 vtcp_pose_{pinocchio::SE3::Identity()};    ///< World-frame virtual TCP pose (cached)
  bool vtcp_valid_{false};                                    ///< Virtual TCP computed successfully
  std::array<FingertipVtcpInput, kNumFingertips> vtcp_inputs_{};  ///< Pre-allocated

  void UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp) noexcept;

  void InitArmModel(const rtc_urdf_bridge::ModelConfig & config);
  void InitHandModel(const rtc_urdf_bridge::ModelConfig & config);

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

  // ── Grasp controller (force_pi mode) ──────────────────────────────────────
  std::string grasp_controller_type_{"contact_stop"};
  std::unique_ptr<rtc::grasp::GraspController> grasp_controller_;
  /// Finger index → hand motor indices mapping (thumb, index, middle)
  static constexpr std::array<std::array<int, 3>, 3> kFingerJointMap{{
    {0, 1, 2}, {3, 4, 5}, {6, 7, 8}
  }};

  /// Hand joint indices (matches ur5e hand joint order in YAML).
  /// Used by the contact_stop release-phase gate below.
  static constexpr std::size_t kHandIdxThumbCmcFe  = 1;
  static constexpr std::size_t kHandIdxIndexMcpFe  = 4;
  static constexpr std::size_t kHandIdxMiddleMcpFe = 7;
  /// Hysteresis on target↔actual delta to reject sensor noise (rad).
  static constexpr double kContactStopReleaseEps = 0.005;

  /// Previous grasp phase (for state-transition logging; non-RT critical).
  uint8_t prev_grasp_phase_{0};

  // ── Logging (throttled, debug only — RT-safe by throttle interval) ───────
  rclcpp::Logger logger_{rclcpp::get_logger("DemoJointController")};
  rclcpp::Clock  log_clock_{RCL_STEADY_TIME};

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
