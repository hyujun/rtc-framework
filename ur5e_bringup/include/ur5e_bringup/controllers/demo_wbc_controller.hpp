#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_

// Project headers (order: RTC base → interface → controllers → bridge → tsid)
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include "ur5e_description/ur5e_constants.hpp"
#include "ur5e_bringup/bringup_logging.hpp"

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

// Third-party
#include <Eigen/Core>

#include <rclcpp/clock.hpp>

// C++ stdlib
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ur5e_bringup
{

using rtc::kMaxDeviceChannels;
using rtc::kNumHandMotors;
using rtc::kNumRobotJoints;
using rtc::RTControllerInterface;
using rtc::CommandType;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::GoalType;
namespace trajectory = rtc::trajectory;

// ── WBC Phase (8-state FSM) ─────────────────────────────────────────────────
//
// Phase 4 MVP implements: kIdle, kApproach, kPreGrasp, kFallback.
// Contact phases (kClosure, kHold, kRetreat, kRelease) are skeleton-only.
enum class WbcPhase : uint8_t {
  kIdle,       ///< Home pose hold (position hold)
  kApproach,   ///< Joint-space quintic trajectory to pre-grasp
  kPreGrasp,   ///< TSID QP → position (no contact, fine positioning)
  kClosure,    ///< TSID QP → position (contact forming, Phase 4B)
  kHold,       ///< TSID QP → position (grasp holding, Phase 4B)
  kRetreat,    ///< Quintic trajectory retreat (Phase 4B)
  kRelease,    ///< Finger open ramp (Phase 4B)
  kFallback    ///< Safety: position hold at last valid q
};

// ── DemoWbcController ────────────────────────────────────────────────────────
//
// Whole-body controller demo for UR5e + 10-DoF hand using TSID QP.
// TSID produces optimal acceleration a; position is obtained by integration:
//   v_next = v_curr + a · dt,  q_next = q_curr + v_next · dt
//
// Gains layout for UpdateGainsFromMsg:
//   [grasp_cmd(0/1/2), grasp_target_force,
//    arm_traj_speed, hand_traj_speed,
//    se3_weight, force_weight, posture_weight] = 7 values
class DemoWbcController final : public RTControllerInterface {
public:
  static constexpr int kArmDof  = static_cast<int>(kNumRobotJoints);  // 6
  static constexpr int kHandDof = static_cast<int>(kNumHandMotors);   // 10
  static constexpr int kFullDof = kArmDof + kHandDof;                 // 16
  static constexpr int kNumPhases = 8;

  struct Gains {
    double arm_trajectory_speed{0.5};      ///< Joint-space traj speed [rad/s]
    double hand_trajectory_speed{1.0};     ///< Hand traj speed [rad/s]
    double arm_max_traj_velocity{2.0};     ///< Max arm joint velocity [rad/s]
    double hand_max_traj_velocity{4.0};    ///< Max hand motor velocity [rad/s]
    double grasp_target_force{2.0};        ///< Target grasp force [N]
    double se3_weight{100.0};              ///< SE3Task weight (runtime tuning)
    double force_weight{10.0};             ///< ForceTask weight
    double posture_weight{1.0};            ///< PostureTask weight
  };

  explicit DemoWbcController(std::string_view urdf_path);

  // ── RTControllerInterface overrides ──────────────────────────────────────
  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept override;

  void InitializeHoldPosition(
    const ControllerState & state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "DemoWbcController";
  }

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Registry hooks ──────────────────────────────────────────────────────
  void LoadConfig(const YAML::Node & cfg) override;
  void OnDeviceConfigsSet() override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override
  {
    return command_type_;
  }

private:
  // ── Model initialization ────────────────────────────────────────────────
  void InitModels(const rtc_urdf_bridge::ModelConfig & config);
  void BuildJointReorderMap();

  // ── 3-phase pipeline (RT path) ──────────────────────────────────────────
  void ReadState(const ControllerState & state) noexcept;
  void ComputeControl(const ControllerState & state, double dt) noexcept;
  [[nodiscard]] ControllerOutput WriteOutput(
    const ControllerState & state) noexcept;

  // ── FSM ─────────────────────────────────────────────────────────────────
  WbcPhase phase_{WbcPhase::kIdle};
  WbcPhase prev_phase_{WbcPhase::kIdle};

  void UpdatePhase(const ControllerState & state) noexcept;
  void OnPhaseEnter(WbcPhase new_phase,
                    const ControllerState & state) noexcept;

  // ── Control modes ───────────────────────────────────────────────────────
  void ComputePositionMode(double dt) noexcept;
  void ComputeTSIDPosition(const ControllerState & state, double dt) noexcept;
  void ComputeFallback() noexcept;

  // ── E-STOP ──────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};
  bool estop_active_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};

  [[nodiscard]] ControllerOutput ComputeEstop(
    const ControllerState & state) noexcept;

  // ── Configuration ───────────────────────────────────────────────────────
  Gains gains_;
  std::string urdf_path_;
  CommandType command_type_{CommandType::kPosition};

  // Grasp command from UpdateGainsFromMsg (0=idle/abort, 1=approach, 2=release)
  std::atomic<int> grasp_cmd_{0};

  // ── rtc_urdf_bridge ─────────────────────────────────────────────────────
  std::unique_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> arm_handle_;
  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::FrameIndex root_frame_id_{0};
  bool use_root_frame_{false};

  // Full model (16-DoF) — shared_ptr lifetime for PinocchioCache
  std::shared_ptr<const pinocchio::Model> full_model_ptr_;

  // Joint reorder: external [arm0..5, hand0..9] → Pinocchio q/v index
  std::array<int, kFullDof> ext_to_pin_q_{};
  std::array<int, kFullDof> ext_to_pin_v_{};
  bool joint_reorder_valid_{false};

  // ── TSID ────────────────────────────────────────────────────────────────
  rtc::tsid::TSIDController tsid_controller_;
  rtc::tsid::PinocchioCache pinocchio_cache_;
  rtc::tsid::ContactState contact_state_;
  rtc::tsid::ControlReference control_ref_;
  rtc::tsid::RobotModelInfo robot_info_;
  rtc::tsid::ContactManagerConfig contact_mgr_config_;
  rtc::tsid::CommandOutput tsid_output_;

  bool tsid_initialized_{false};
  int qp_fail_count_{0};
  int max_qp_fail_before_fallback_{5};

  // Phase presets — pre-resolved from YAML at init for RT-safe access.
  // Indexed by static_cast<int>(WbcPhase).
  std::array<rtc::tsid::PhasePreset, kNumPhases> phase_presets_{};
  std::array<bool, kNumPhases> phase_preset_valid_{};

  // ── TSID → Position integration ─────────────────────────────────────────
  //
  // All vectors are in Pinocchio joint order (full model, 16-DoF).
  Eigen::VectorXd q_curr_full_;       ///< [nv] current q (sensor, per tick)
  Eigen::VectorXd v_curr_full_;       ///< [nv] current v (sensor, per tick)
  Eigen::VectorXd q_next_full_;       ///< [nv] integrated position (output)
  Eigen::VectorXd v_next_full_;       ///< [nv] integrated velocity
  Eigen::VectorXd q_min_clamped_;     ///< [nv] q_lower + margin
  Eigen::VectorXd q_max_clamped_;     ///< [nv] q_upper - margin
  Eigen::VectorXd v_limit_;           ///< [nv] velocity limit

  // ControlState for TSID compute (pre-allocated)
  rtc::tsid::ControlState ctrl_state_;

  // ── Target management ───────────────────────────────────────────────────
  std::mutex target_mutex_;
  std::atomic<bool> robot_new_target_{false};
  std::atomic<bool> hand_new_target_{false};

  std::array<std::array<double, kMaxDeviceChannels>,
    ControllerState::kMaxDevices> device_targets_{};

  // Task-space goal (computed from FK of arm target in SetDeviceTarget)
  pinocchio::SE3 tcp_goal_{pinocchio::SE3::Identity()};
  bool tcp_goal_valid_{false};

  // ── Trajectory (position mode phases) ───────────────────────────────────
  trajectory::JointSpaceTrajectory<kNumRobotJoints> robot_trajectory_;
  trajectory::JointSpaceTrajectory<kNumHandMotors>  hand_trajectory_;
  double robot_trajectory_time_{0.0};
  double hand_trajectory_time_{0.0};

  // ── Device limits ───────────────────────────────────────────────────────
  std::array<std::vector<double>,
    ControllerState::kMaxDevices> device_max_velocity_;
  std::array<std::vector<double>,
    ControllerState::kMaxDevices> device_position_lower_;
  std::array<std::vector<double>,
    ControllerState::kMaxDevices> device_position_upper_;

  static void ClampCommands(
    std::array<double, kMaxDeviceChannels>& commands, int n,
    const std::vector<double>& lower,
    const std::vector<double>& upper) noexcept;

  // ── Computed output (intermediate) ──────────────────────────────────────
  struct ComputedTrajectory {
    std::array<double, kMaxDeviceChannels> positions{};
    std::array<double, kMaxDeviceChannels> velocities{};
  };
  ComputedTrajectory robot_computed_{};
  ComputedTrajectory hand_computed_{};

  // ── FSM thresholds ──────────────────────────────────────────────────────
  double epsilon_approach_{0.01};       ///< m, approach → pre-grasp
  double epsilon_pregrasp_{0.005};      ///< m, pre-grasp → closure
  double force_contact_threshold_{0.2}; ///< N, contact detection
  double force_hold_threshold_{1.0};    ///< N, hold → retreat

  // Integration safety margins
  double position_margin_{0.02};        ///< rad, from joint limits
  double velocity_scale_{0.95};         ///< fraction of max velocity

  // ── Utility ─────────────────────────────────────────────────────────────
  void ExtractFullState(const ControllerState & state) noexcept;
  [[nodiscard]] double ComputeTcpError(
    const pinocchio::SE3 & target) noexcept;

  // ── Logging ─────────────────────────────────────────────────────────────
  rclcpp::Logger logger_{ur5e_bringup::logging::DemoWbcLogger()};
  rclcpp::Clock  log_clock_{RCL_STEADY_TIME};
};

}  // namespace ur5e_bringup

#endif  // UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_
