#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_

// Project headers (order: RTC base → interface → controllers → bridge → tsid)
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include "ur5e_bringup/bringup_logging.hpp"
#include "ur5e_description/ur5e_constants.hpp"

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "rtc_mpc/thread/mock_mpc_thread.hpp"
#include "rtc_mpc/thread/mpc_thread.hpp"
#include "ur5e_bringup/phase/grasp_phase_manager.hpp"

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

namespace ur5e_bringup {

using rtc::CommandType;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::GoalType;
using rtc::kMaxDeviceChannels;
using rtc::kNumHandMotors;
using rtc::kNumRobotJoints;
using rtc::RTControllerInterface;
namespace trajectory = rtc::trajectory;

// ── WBC Phase (8-state FSM) ─────────────────────────────────────────────────
//
// Phase 4 MVP implements: kIdle, kApproach, kPreGrasp, kFallback.
// Contact phases (kClosure, kHold, kRetreat, kRelease) are skeleton-only.
enum class WbcPhase : uint8_t {
  kIdle,     ///< Home pose hold (position hold)
  kApproach, ///< Joint-space quintic trajectory to pre-grasp
  kPreGrasp, ///< TSID QP → position (no contact, fine positioning)
  kClosure,  ///< TSID QP → position (contact forming, Phase 4B)
  kHold,     ///< TSID QP → position (grasp holding, Phase 4B)
  kRetreat,  ///< Quintic trajectory retreat (Phase 4B)
  kRelease,  ///< Finger open ramp (Phase 4B)
  kFallback  ///< Safety: position hold at last valid q
};

// ── DemoWbcController ────────────────────────────────────────────────────────
//
// Whole-body controller demo for UR5e + 10-DoF hand using TSID QP.
// TSID produces optimal acceleration a; position is obtained by integration:
//   v_next = v_curr + a · dt,  q_next = q_curr + v_next · dt
//
// Gains layout for UpdateGainsFromMsg (Phase 5):
//   [grasp_cmd(0/1/2), grasp_target_force,
//    arm_traj_speed, hand_traj_speed,
//    se3_weight, force_weight, posture_weight,
//    mpc_enable(0/1), riccati_gain_scale(0..1)] = 9 values
// Phase 4 compatibility: first 7 indices are unchanged; trailing 2 are
// optional (a 7-entry message keeps Phase 4 semantics exactly).
class DemoWbcController final : public RTControllerInterface {
public:
  static constexpr int kArmDof = static_cast<int>(kNumRobotJoints); // 6
  static constexpr int kHandDof = static_cast<int>(kNumHandMotors); // 10
  static constexpr int kFullDof = kArmDof + kHandDof;               // 16
  static constexpr int kNumPhases = 8;

  struct Gains {
    double arm_trajectory_speed{0.5};   ///< Joint-space traj speed [rad/s]
    double hand_trajectory_speed{1.0};  ///< Hand traj speed [rad/s]
    double arm_max_traj_velocity{2.0};  ///< Max arm joint velocity [rad/s]
    double hand_max_traj_velocity{4.0}; ///< Max hand motor velocity [rad/s]
    double grasp_target_force{2.0};     ///< Target grasp force [N]
    double se3_weight{100.0};           ///< SE3Task weight (runtime tuning)
    double force_weight{10.0};          ///< ForceTask weight
    double posture_weight{1.0};         ///< PostureTask weight
  };

  explicit DemoWbcController(std::string_view urdf_path);

  // ── RTControllerInterface overrides ──────────────────────────────────────
  [[nodiscard]] ControllerOutput
  Compute(const ControllerState &state) noexcept override;

  void SetDeviceTarget(int device_idx,
                       std::span<const double> target) noexcept override;

  void InitializeHoldPosition(const ControllerState &state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override {
    return "DemoWbcController";
  }

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Test accessors (const snapshots, not RT-safe) ───────────────────────
  struct FingertipReport {
    float force_magnitude{0.0f};
    float force_rate{0.0f};
    float contact_flag{0.0f};
    bool valid{false};
  };
  [[nodiscard]] FingertipReport
  GetFingertipReportForTesting(int fingertip_idx) const noexcept;
  [[nodiscard]] int GetNumActiveFingertipsForTesting() const noexcept {
    return num_active_fingertips_;
  }
  [[nodiscard]] WbcPhase GetPhaseForTesting() const noexcept { return phase_; }
  void ForcePhaseForTesting(WbcPhase p) noexcept { phase_ = p; }
  void SetGraspCmdForTesting(int v) noexcept {
    grasp_cmd_.store(v, std::memory_order_release);
  }

  // ── Registry hooks ──────────────────────────────────────────────────────
  void LoadConfig(const YAML::Node &cfg) override;
  void OnDeviceConfigsSet() override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override {
    return command_type_;
  }

private:
  // ── Model initialization ────────────────────────────────────────────────
  void InitModels(const rtc_urdf_bridge::ModelConfig &config);
  void BuildJointReorderMap();

  // ── TSID task/constraint YAML factory ───────────────────────────────────
  //
  // Dispatches on the `type:` field of each entry under `tsid.tasks` /
  // `tsid.constraints`. Supported tasks: posture, se3, force. Supported
  // constraints: eom, joint_limit, friction_cone. Unknown types log ERROR
  // and skip. Called once in LoadConfig after TSIDController::init().
  void BuildTsidTasks(const YAML::Node &tsid_node);
  void BuildTsidConstraints(const YAML::Node &tsid_node);

  // ── 3-phase pipeline (RT path) ──────────────────────────────────────────
  void ReadState(const ControllerState &state) noexcept;
  void ComputeControl(const ControllerState &state, double dt) noexcept;
  [[nodiscard]] ControllerOutput
  WriteOutput(const ControllerState &state) noexcept;

  // ── FSM ─────────────────────────────────────────────────────────────────
  WbcPhase phase_{WbcPhase::kIdle};
  WbcPhase prev_phase_{WbcPhase::kIdle};

  void UpdatePhase(const ControllerState &state) noexcept;
  void OnPhaseEnter(WbcPhase new_phase, const ControllerState &state) noexcept;

  // ── Control modes ───────────────────────────────────────────────────────
  void ComputePositionMode(double dt) noexcept;
  void ComputeTSIDPosition(const ControllerState &state, double dt) noexcept;
  void ComputeFallback() noexcept;

  // ── E-STOP ──────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};
  bool estop_active_{false};

  /// Arm joint position the E-STOP path drives to. Authoritative source is
  /// LoadConfig(cfg["estop"]["arm_safe_position"]); this initializer only
  /// provides a safe default for unit tests that construct the controller
  /// without calling LoadConfig.
  std::array<double, kNumRobotJoints> safe_position_{0.0,   -1.57, 1.57,
                                                     -1.57, -1.57, 0.0};

  [[nodiscard]] ControllerOutput
  ComputeEstop(const ControllerState &state) noexcept;

  // ── Configuration ───────────────────────────────────────────────────────
  rtc::SeqLock<Gains> gains_lock_;
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

  // ── Fingertip sensor data (parsed in ReadState) ─────────────────────────
  //
  // Populated each tick from state.devices[1].sensor_data / inference_data.
  // Consumed by contact detection (kClosure -> kHold) and anomaly monitoring
  // (kHold slip/deformation -> kFallback).
  struct FingertipSensorData {
    std::array<int32_t, rtc::kBarometerCount> baro{};
    std::array<int32_t, 3> tof{};
    std::array<float, 3> force{};
    std::array<float, 3> displacement{};
    float force_magnitude{0.0f};      ///< ||force||  (cached, N)
    float prev_force_magnitude{0.0f}; ///< previous tick, for df/dt
    float force_rate{0.0f};           ///< df/dt [N/s] (smoothed)
    float contact_flag{0.0f};
    bool valid{false};
  };
  std::array<FingertipSensorData, rtc::kMaxFingertips> fingertip_data_{};
  int num_active_fingertips_{0};
  bool force_rate_initialized_{false};

  // ── TSID → Position integration ─────────────────────────────────────────
  //
  // All vectors are in Pinocchio joint order (full model, 16-DoF).
  Eigen::VectorXd q_curr_full_;   ///< [nv] current q (sensor, per tick)
  Eigen::VectorXd v_curr_full_;   ///< [nv] current v (sensor, per tick)
  Eigen::VectorXd q_next_full_;   ///< [nv] integrated position (output)
  Eigen::VectorXd v_next_full_;   ///< [nv] integrated velocity
  Eigen::VectorXd q_min_clamped_; ///< [nv] q_lower + margin
  Eigen::VectorXd q_max_clamped_; ///< [nv] q_upper - margin
  Eigen::VectorXd v_limit_;       ///< [nv] velocity limit

  // ControlState for TSID compute (pre-allocated)
  rtc::tsid::ControlState ctrl_state_;

  // ── Target management ───────────────────────────────────────────────────
  std::mutex target_mutex_;
  std::atomic<bool> robot_new_target_{false};
  std::atomic<bool> hand_new_target_{false};

  std::array<std::array<double, kMaxDeviceChannels>,
             ControllerState::kMaxDevices>
      device_targets_{};

  // Task-space goal (computed from FK of arm target in SetDeviceTarget)
  pinocchio::SE3 tcp_goal_{pinocchio::SE3::Identity()};
  bool tcp_goal_valid_{false};

  // ── Trajectory (position mode phases) ───────────────────────────────────
  trajectory::JointSpaceTrajectory<kNumRobotJoints> robot_trajectory_;
  trajectory::JointSpaceTrajectory<kNumHandMotors> hand_trajectory_;
  double robot_trajectory_time_{0.0};
  double hand_trajectory_time_{0.0};

  // ── Device limits ───────────────────────────────────────────────────────
  std::array<std::vector<double>, ControllerState::kMaxDevices>
      device_max_velocity_;
  std::array<std::vector<double>, ControllerState::kMaxDevices>
      device_position_lower_;
  std::array<std::vector<double>, ControllerState::kMaxDevices>
      device_position_upper_;

  static void ClampCommands(std::array<double, kMaxDeviceChannels> &commands,
                            int n, const std::vector<double> &lower,
                            const std::vector<double> &upper) noexcept;

  // ── Computed output (intermediate) ──────────────────────────────────────
  struct ComputedTrajectory {
    std::array<double, kMaxDeviceChannels> positions{};
    std::array<double, kMaxDeviceChannels> velocities{};
  };
  ComputedTrajectory robot_computed_{};
  ComputedTrajectory hand_computed_{};

  // ── MPC integration (Phase 5 + 7b) ──────────────────────────────────────
  //
  // When `mpc_enabled_` is true, `InitializeHoldPosition` spawns one of two
  // MPC thread implementations:
  //
  //   `mpc.engine: "mock"`     (Phase 5, default) — MockMPCThread publishes
  //                            a self-regularising hold target. Keeps the
  //                            Phase 4 fixed-reference + Riccati-scaled
  //                            path alive without pulling in Aligator.
  //
  //   `mpc.engine: "handler"`  (Phase 7b, opt-in) — HandlerMPCThread drives
  //                            a real LightContact / ContactRich solve via
  //                            MPCFactory, with GraspPhaseManager supplying
  //                            phase context. Cross-mode swap between
  //                            light_contact and contact_rich is handled
  //                            inside HandlerMPCThread.
  //
  // If `mpc_enabled_` is false, no MPC thread is spawned and the Phase 4
  // self-regularising hold path runs exclusively.
  enum class MpcEngine { kMock, kHandler };

  bool mpc_enabled_{false};
  MpcEngine mpc_engine_{MpcEngine::kMock};
  rtc::mpc::MPCSolutionManager mpc_manager_;
  std::unique_ptr<rtc::mpc::MPCThread> mpc_thread_;

  // Handler-mode dependencies (null when engine != kHandler).
  //
  // `mpc_model_handler_` stays owned by the controller because HandlerMPCThread
  // only holds a non-owning reference to it; its lifetime must bracket the
  // MPC thread. `phase_manager_owned_` holds the manager between LoadConfig
  // (build + validate YAML) and InitializeHoldPosition (ownership transferred
  // into HandlerMPCThread::Configure). `phase_manager_ptr_` is a borrowed
  // raw pointer that stays valid for the thread's lifetime; the controller
  // uses it to bridge WBC FSM edges onto the grasp FSM command bus
  // (`SetCommand` / `ForcePhase`).
  std::unique_ptr<rtc::mpc::RobotModelHandler> mpc_model_handler_;
  std::unique_ptr<ur5e_bringup::phase::GraspPhaseManager> phase_manager_owned_;
  ur5e_bringup::phase::GraspPhaseManager *phase_manager_ptr_{nullptr};

  // Pre-parsed YAML nodes (kept alive for handler-mode startup and for
  // cross-mode swap inside HandlerMPCThread).
  YAML::Node mpc_light_cfg_;
  YAML::Node mpc_rich_cfg_;
  YAML::Node phase_cfg_;

  // Pre-allocated MPC reference buffers (sized in LoadConfig).
  Eigen::VectorXd mpc_q_ref_;
  Eigen::VectorXd mpc_v_ref_;
  Eigen::VectorXd mpc_a_ff_;
  Eigen::VectorXd mpc_lambda_ref_;
  Eigen::VectorXd mpc_u_fb_;

  // ── FSM thresholds ──────────────────────────────────────────────────────
  double epsilon_approach_{0.01};       ///< m, approach → pre-grasp
  double epsilon_pregrasp_{0.005};      ///< m, pre-grasp → closure
  double force_contact_threshold_{0.2}; ///< N, contact detection
  double force_hold_threshold_{1.0};    ///< N, hold → retreat
  int min_contacts_for_hold_{2};        ///< # fingertips required -> kHold
  double slip_rate_threshold_{5.0};     ///< N/s, |df/dt| slip guard (kHold)
  double deformation_threshold_{0.015}; ///< m, ||disp|| guard (kHold)

  // Approach start pose (saved on kApproach entry, reused on kRetreat)
  std::array<double, kNumRobotJoints> q_approach_start_{};

  // Integration safety margins
  double position_margin_{0.02}; ///< rad, from joint limits
  double velocity_scale_{0.95};  ///< fraction of max velocity
  float force_rate_alpha_{0.1f}; ///< EMA smoothing for df/dt (500Hz→~20Hz BW)

  // ── Utility ─────────────────────────────────────────────────────────────
  void ExtractFullState(const ControllerState &state) noexcept;
  [[nodiscard]] double ComputeTcpError(const pinocchio::SE3 &target) noexcept;

  // ── Logging ─────────────────────────────────────────────────────────────
  rclcpp::Logger logger_{ur5e_bringup::logging::DemoWbcLogger()};
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
};

} // namespace ur5e_bringup

#endif // UR5E_BRINGUP_CONTROLLERS_DEMO_WBC_CONTROLLER_H_
