// ── Includes: project header first, then third-party, then C++ stdlib
// ──────────
#pragma once

#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "integrated_bringup/controllers/tof_snapshot.hpp"
#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"
#include "integrated_bringup/support/combined_model_cache.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "integrated_bringup/support/pull_estimator_wiring.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"
#include "rtc_urdf_bridge/pinocchio_cache.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include <rtc_msgs/srv/grasp_command.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>

#include <Eigen/Cholesky>  // LDLT
#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

using rtc::CommandType;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::GoalType;
using rtc::kMaxDeviceChannels;
using rtc::RTControllerInterface;
namespace trajectory = rtc::trajectory;

// Fixed-size POD capacity (conservative for humanoid-class robots).
// Actual DoF held in arm_dof_ / hand_dof_ runtime members, resolved in
// LoadConfig (arm) and OnDeviceConfigsSet (hand).
inline constexpr int kDemoTaskMaxArmDof = 32;
inline constexpr int kDemoTaskMaxHandDof = 32;
// Combined arm+hand actuated model capacity — sizes the ext↔Pinocchio reorder
// maps (unified kin&dyn Phase 4). Actual full_dof_ resolved at runtime.
inline constexpr int kDemoTaskMaxFullDof = kDemoTaskMaxArmDof + kDemoTaskMaxHandDof;

/// Demo Task-Space Controller: CLIK (arm) + P control (hand).
///
/// Controls the end-effector in Cartesian space via damped Jacobian
/// pseudoinverse, while simultaneously running proportional position
/// control on the 10-DOF hand motors.
///
/// ### Arm control law (same as ClikController)
/// @code
///   pos_error    = x_des − FK(q)
///   J^#          = J^T (J J^T + λ²I)^{−1}         [damped pseudoinverse]
///   N            = I − J^# J                        [null-space projector]
///   dq           = kp · J^# · pos_error + null_kp · N · (q_null − q)
///   q_des       += clamp(dq, ±v_max) * dt        [q_des = q_actual at
///   trajectory init] q_cmd        = q_des
/// @endcode
///
/// ### Hand control law (same as DemoJointController)
/// @code
///   hand_cmd[i]  = hand_pos[i] + hand_kp[i] * (hand_target[i] - hand_pos[i]) *
///   dt
/// @endcode
///
/// ### Target convention (`SetRobotTarget` / `/target_joint_positions`)
///   - 3-DOF mode: `target[0..2]` = TCP position [x,y,z], `target[3..5]` =
///   null-space ref
///   - 6-DOF mode: `target[0..2]` = TCP position [x,y,z], `target[3..5]` =
///   [roll,pitch,yaw]
///
/// ### Runtime tunable parameters (per-controller LifecycleNode)
///   ROS 2 parameters declared on `/demo_task_controller/<name>`: see
///   `DeclareGainParameters()` in demo_task_controller.cpp. Read-only caps
///   `max_traj_velocity` / `max_traj_angular_velocity` /
///   `hand_max_traj_velocity`. Force-PI grasp transitions: `~/grasp_command`
///   srv (rtc_msgs/GraspCommand).
class DemoTaskController final : public RTControllerInterface {
 public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains {
    // Arm (CLIK) gains — translation / rotation separated
    std::array<double, 3> kp_translation{
        {5.0, 5.0, 5.0}};  ///< Translation proportional gain (x,y,z) [1/s]
    std::array<double, 3> kp_rotation{
        {2.0, 2.0, 2.0}};          ///< Rotation proportional gain (rx,ry,rz) [1/s]
    double damping{0.01};          ///< Damping factor λ for J^#
    double null_kp{0.5};           ///< Null-space joint-centering gain [1/s]
    bool enable_null_space{true};  ///< Enable null-space secondary task
    bool control_6dof{false};      ///< Enable 6-DOF (translation + orientation) control

    // Trajectory speed
    double trajectory_speed{0.1};  ///< TCP translational speed for trajectory duration [m/s]
    double trajectory_angular_speed{0.5};  ///< TCP rotational speed for trajectory duration [rad/s]
    double hand_trajectory_speed{1.0};     ///< Hand motor speed for trajectory duration [rad/s]

    // Trajectory velocity limits
    double max_traj_velocity{0.5};          ///< Max TCP velocity during task-space trajectory [m/s]
    double max_traj_angular_velocity{1.0};  ///< Max TCP angular velocity during trajectory [rad/s]
    double hand_max_traj_velocity{2.0};     ///< Max hand motor velocity during trajectory [rad/s]

    // Virtual TCP (fingertip-based control point)
    VirtualTcpConfig vtcp;

    // Grasp detection parameters
    // Grasp detection parameters. Capability-aware (see demo_shared.yaml /
    // sensor_layout.has_native_contact): sensor A requires both contact_thresh
    // AND force_thresh; sensor B requires force_thresh only.
    float grasp_contact_threshold{0.5f};  ///< Native prob threshold [0,1] — sensor A only
    float grasp_force_threshold{1.0f};    ///< |F| threshold [N] — common
    int grasp_min_fingertips{2};          ///< grasp_detected = active_count ≥ N

    // Trajectory / grasp FSM tuning
    double pi_rotation_margin{0.15};  ///< Split quintic trajectory when |angle|>π-margin [rad]
    double contact_stop_release_eps{0.005};   ///< Hand contact-stop release hysteresis [rad]
    double contact_stop_lpf_cutoff_hz{20.0};  ///< Bessel LPF cutoff for latched hold [Hz]
  };

  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      Gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit DemoTaskController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Phase 4: controller-owned sub/pub lifecycle ─────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) noexcept override;
  void PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //                enable_null_space(0/1), control_6dof(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                hand_trajectory_speed, max_traj_velocity,
  //                max_traj_angular_velocity, hand_max_traj_velocity,
  //                grasp_contact_threshold, grasp_force_threshold,
  //                grasp_min_fingertips,
  //                grasp_command, grasp_target_force] = 21 values
  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_lock_.Store(g); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept { return tcp_position_; }

  [[nodiscard]] std::array<double, 3> position_error() const noexcept {
    return {pos_error_[0], pos_error_[1], pos_error_[2]};
  }

  // Phase 4c: ControllerOutput::grasp_state / tof_snapshot fields were
  // removed — tests read the post-Compute() staging buffers directly via
  // these accessors (RT-7 safe; no behavior change).
  [[nodiscard]] const ::rtc::grasp::GraspStateData& GetGraspStateForTesting() const noexcept {
    return grasp_state_;
  }

  /// Test-only: the body the publish thread would Load right now (#234 P-1).
  /// Distinct from GetGraspStateForTesting(), which returns the RT staging
  /// buffer: a tick that fills the staging buffer but never stores it gets
  /// republished as the *previous* body under the current stamp, and only this
  /// accessor can tell those two apart.
  [[nodiscard]] ::rtc::grasp::GraspStateData GetPublishedGraspStateForTesting() const noexcept {
    return grasp_state_lock_.Load();
  }

  [[nodiscard]] const ::integrated_bringup::ToFSnapshotData& GetToFSnapshotForTesting()
      const noexcept {
    return tof_snapshot_;
  }

 private:
  // ── Phase 1→2 intermediate: parsed sensor data ──────────────────────────
  struct FingertipSensorData {
    std::array<int32_t, kHandBaroChannelsCapacity> baro{};
    std::array<int32_t, 3> tof{};
    std::array<float, 3> force{};
    std::array<float, 3> displacement{};
    float force_mag{0.0f};  ///< ‖force‖ cached in ReadState (reused by grasp + vtcp)
    float contact_flag{0.0f};
    bool valid{false};
  };

  std::array<FingertipSensorData, rtc::kMaxSensorGroups> fingertip_data_{};
  int num_active_fingertips_{0};
  /// Fingertips carrying a raw sensor lane (baro/ToF), derived from
  /// num_sensor_channels / stride.  Separate from num_active_fingertips_
  /// (inference-group count) so a force-only stream (0 sensor channels) does
  /// not publish an all-zero junk ToF snapshot.  Gates the ToF snapshot only.
  int num_sensor_fingertips_{0};
  ::rtc::grasp::GraspStateData grasp_state_{};
  ::integrated_bringup::ToFSnapshotData tof_snapshot_{};

  // ── Phase 2→3 intermediate: computed trajectory results ─────────────────
  struct ComputedTrajectory {
    std::array<double, kMaxDeviceChannels> positions{};
    std::array<double, kMaxDeviceChannels> velocities{};
  };

  ComputedTrajectory hand_computed_{};
  bool estop_active_{false};
  // RT-thread-only cache of gains.control_6dof, set in ComputeControl so the
  // Fill* methods avoid a full Gains SeqLock copy just to read one bool.
  // Set after ComputeControl's E-STOP early-return, so it is valid only on
  // non-E-STOP ticks — every reader must sit behind an `estop_active_` guard.
  bool control_6dof_cached_{false};

  // ── Hand fingertip FK dispatch (serial hand_handle_ ↔ closed_hand_fk_) ────
  // #121: single branch point for closed-chain vs serial hand FK so every call
  // site stays byte-for-byte when closure is absent/inactive.
  //   ComputeHandForwardKinematics: run the per-tick hand FK (closed Update or
  //     serial ComputeForwardKinematics); returns false if no valid hand device.
  //   HandFingertipPose: hand-root-relative fingertip pose for finger f (closed
  //     or serial); false if that fingertip is inactive.
  [[nodiscard]] bool ComputeHandForwardKinematics(const ControllerState& state) noexcept;
  [[nodiscard]] bool HandFingertipPose(std::size_t f, pinocchio::SE3& out) const noexcept;
  void ConfigureClosedChainHandFk();

  // ── 3-phase pipeline ────────────────────────────────────────────────────
  void ReadState(const ControllerState& state) noexcept;
  void ComputeControl(const ControllerState& state, double dt) noexcept;
  // WriteOutput was split into 3 explicit-intent methods (see
  // demo_joint_controller.hpp for the bucket contract). Compute() calls
  // them in order WriteJointCommand → FillLogOutput → FillPublishOutput.
  [[nodiscard]] ControllerOutput WriteJointCommand(const ControllerState& state,
                                                   double dt) noexcept;
  void FillLogOutput(const ControllerState& state, ControllerOutput& output, double dt) noexcept;
  void FillPublishOutput(const ControllerState& state, ControllerOutput& output,
                         double dt) noexcept;

  // Sensor-derived grasp aggregates (per-fingertip |F| / contact flags /
  // grasp detection). Sourced from fingertip_data_, which ReadState refreshes
  // every tick including E-STOP — hence shared by ComputeControl and
  // FillEstopPublishState. Contains no control-law output.
  void FillGraspSensorAggregates(const Gains& gains) noexcept;

  // E-STOP counterpart of FillLogOutput's SeqLock store (#234 P-1) — see
  // demo_joint_controller.hpp for the fresh-stamp/stale-body rationale. Called
  // from FillLogOutput's E-STOP branch, i.e. before the tail
  // PushPullEstimatorLog, so the CSV row and the wire carry the same tick.
  // RT tick path — noexcept, heap-free.
  void FillEstopPublishState(double dt) noexcept;

  // ── Controller state (gains before urdf_path to match constructor init
  // order) ─
  rtc::SeqLock<Gains> gains_lock_;

  // ── rtc_urdf_bridge ────────────────────────────────────────────
  std::string urdf_path_;  // stored from constructor, used in LoadConfig
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> arm_handle_;
  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::FrameIndex root_frame_id_{0};
  bool use_root_frame_{false};

  // ── Unified kin&dyn cache (Phase 4) ──────────────────────────────────────
  // Arm TCP FK / Jacobian now come from a WBC-style combined (arm+hand)
  // PinocchioCache updated once per non-E-STOP tick, replacing the arm-only
  // arm_handle_ direct calls. arm_handle_ is retained solely for the E-STOP
  // TF-alive path (ComputeEstop), which stays on the arm sub-model by design
  // (§주의/제약). The combined control model is selected exactly like
  // DemoWbcController::InitModels (GetActuatedModel → tree 'wbc' → full).
  // Shared combined arm+hand model cache wiring (model select, ext→Pinocchio
  // reorder map, per-tick state scatter, arm-TCP FK / Jacobian source) — see
  // CombinedModelCache (#174). Task consumes both the arm TCP pose and the tip
  // Jacobian columns (via ext_to_pin_v) from it.
  CombinedModelCache combined_cache_;
  int full_dof_{0};
  // registered_frames indices for the arm TCP tip / base (< 0 = universe/base
  // uses world). Registered on the combined cache with the ARM sub-model frame
  // ids (frame-id consistency proven by test_wbc_arm_tcp_cache_equivalence).
  int task_tcp_frame_idx_{-1};
  int task_base_frame_idx_{-1};
  // ── Hand tree-model for fingertip FK ──────────────────────────────────
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> hand_handle_;
  static constexpr std::size_t kNumFingertips = 4;
  std::array<pinocchio::FrameIndex, kNumFingertips> fingertip_frame_ids_{};
  pinocchio::FrameIndex hand_root_frame_id_{0};
  bool use_hand_root_frame_{false};
  std::array<pinocchio::SE3, kNumFingertips> T_tcp_fingertip_{};
  std::array<Eigen::Vector3d, kNumFingertips> fingertip_positions_{};
  std::array<Eigen::Matrix3d, kNumFingertips> fingertip_rotations_{};
  // Per-tick: did HandFingertipPose produce a real pose for fingertip f this
  // tick? False for a downstream (closed-chain loop) tip until the loop first
  // converges, so FillPublishOutput must not publish the zero-init cache as a
  // valid TF (#125 F1).
  std::array<bool, kNumFingertips> fingertip_pose_valid_{};
  Eigen::VectorXd hand_q_;  // pre-allocated for hand FK
  // #121: closed-chain-consistent hand fingertip FK. Active only for extended-URDF
  // (loop-closure) hands whose fingertips are downstream of a loop-passive joint;
  // otherwise inactive and the serial hand_handle_ path runs byte-for-byte.
  ClosedChainHandFk closed_hand_fk_;

  // Arm TCP pose (base-relative when a root frame is registered), cached once
  // per tick at the top of ComputeControl from the unified combined-model cache
  // (ArmTcpPoseFromCache). Reused by the CLIK law, the fingertip composition,
  // and FillLog/FillPublishOutput so the publish/log path never re-reads the
  // cache (mirrors DemoJointController::arm_tcp_pose_).
  pinocchio::SE3 arm_tcp_pose_{pinocchio::SE3::Identity()};

  // ── Virtual TCP (fingertip-based control point) ───────────────────────
  pinocchio::SE3 T_tcp_vtcp_{pinocchio::SE3::Identity()};  ///< TCP → virtual TCP transform
  pinocchio::SE3 vtcp_pose_{pinocchio::SE3::Identity()};  ///< World-frame virtual TCP pose (cached)
  bool vtcp_valid_{false};                                ///< Virtual TCP computed successfully
  Eigen::Matrix3d skew_buf_{Eigen::Matrix3d::Zero()};     ///< Jacobian modification buffer
  std::array<FingertipVtcpInput, kNumFingertips> vtcp_inputs_{};  ///< Pre-allocated

  void UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp, const Gains& gains) noexcept;

  void InitArmModel(const rtc_urdf_bridge::ModelConfig& config);
  void InitHandModel(const rtc_urdf_bridge::ModelConfig& config);

  // Model-dimension bound for arm-device channel loops (issue #172 pattern).
  // The device's reported channel count is wire-derived and independent of the
  // model DOF, so every loop that indexes an nv-wide Eigen buffer by a device
  // channel index has to intersect the two. Returns min(nc0, nv).
  [[nodiscard]] std::size_t ArmCommandBound(int nc0) const noexcept {
    return std::min(static_cast<std::size_t>(std::max(nc0, 0)),
                    static_cast<std::size_t>(desired_q_.size()));
  }

  // ── Pre-allocated Eigen work buffers — zero heap alloc on the RT path ────
  Eigen::VectorXd q_;
  Eigen::MatrixXd J_full_;
  Eigen::MatrixXd J_pos_;
  Eigen::Matrix3d JJt_;
  Eigen::Matrix3d JJt_inv_;
  Eigen::MatrixXd Jpinv_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd desired_q_;  ///< nv: integrated desired joint position
  Eigen::VectorXd traj_dq_;    // feedforward-only trajectory velocity (for logging)
  Eigen::VectorXd null_err_;
  Eigen::VectorXd null_dq_;
  Eigen::Vector3d pos_error_;
  Eigen::Matrix<double, 6, 6> JJt_6d_;
  Eigen::Matrix<double, 6, 6> JJt_inv_6d_;
  Eigen::MatrixXd Jpinv_6d_;
  Eigen::Matrix<double, 6, 1> pos_error_6d_;

  Eigen::LDLT<Eigen::Matrix3d> ldlt_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_6d_;

  // ── Controller state ──────────────────────────────────────────────────────
  // RT-thread-only working copy of the SE3 target (materialised from the
  // SeqLock POD each tick).
  pinocchio::SE3 tcp_target_pose_{pinocchio::SE3::Identity()};

  /// Null-space posture seed parsed from YAML. RT thread copies it into
  /// TargetSlot::null_target on first-tick self-init.
  std::array<double, kDemoTaskMaxArmDof> null_target_init_{};

  // TargetSlot — Eigen SE3 mirrored as flat doubles. RT thread is the SOLE
  // writer of target_seqlock_; off-RT writers hand their goal to the base
  // mailbox via SetDeviceTarget → PushPendingTarget (#206).
  static constexpr std::size_t kSE3RotDoubles = 9;
  static constexpr std::size_t kSE3TransDoubles = 3;

  struct TargetSlot {
    std::array<double, 3> tcp_target{};
    std::array<double, kSE3RotDoubles> tcp_target_rot{};
    std::array<double, kSE3TransDoubles> tcp_target_t{};
    std::array<double, kDemoTaskMaxArmDof> null_target{};
    std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> targets{};
  };

  static_assert(std::is_trivially_copyable_v<TargetSlot>,
                "TargetSlot must be trivially copyable for SeqLock<TargetSlot>");

  // One drained target → this controller's slot. Called on the RT tick from
  // DrainTargetSlot(), with device_idx and values already bounded by the base.
  // Device 0 is the task device: its goal is an SE3 pose or a
  // position+nullspace pair depending on `control_6dof`. `is_task` is unused —
  // this controller's task ingress is the base default, which forwards to
  // SetDeviceTarget because device 0's buffer already IS the pose.
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override;

  rtc::SeqLock<TargetSlot> target_seqlock_;
  // Per-device self-init flags — see demo_joint_controller.hpp. The arm/TCP
  // hold is seeded on the first tick; the hand (device 1) hold-seed is deferred
  // until device 1 first reports a valid measured state so it is never locked to
  // the zero-initialized targets[1] (ur5e fine / p1b finger collapse).
  std::atomic<bool> arm_target_initialized_{false};
  std::atomic<bool> hand_target_initialized_{false};

  // Base hook — see demo_joint_controller.hpp. Previously duplicated inside
  // on_activate (#196 §3).
  void ResetTargetInitialization() noexcept override {
    arm_target_initialized_.store(false, std::memory_order_release);
    hand_target_initialized_.store(false, std::memory_order_release);
  }

  TargetSlot current_target_slot_{};

  // RT-thread-only: refresh current_target_slot_ from the SeqLock, drain any
  // off-RT SetDeviceTarget marshal entries, run first-tick self-init.
  void DrainTargetSlot(const ControllerState& state) noexcept;

  std::array<double, 3> tcp_position_{};

  bool new_target_pending_{false};  // RT-thread-only; gates trajectory re-init
  trajectory::TaskSpaceTrajectory trajectory_;
  trajectory::TaskSpaceTrajectory::State traj_state_{};
  double trajectory_time_{0.0};

  // ── Multi-segment trajectory (π-rotation defense) ──────────────
  pinocchio::SE3 pending_goal_pose_{pinocchio::SE3::Identity()};
  double pending_duration_{0.0};
  bool has_pending_segment_{false};
  // Hand trajectory fixed at compile-time capacity; only the first
  // hand_dof_ slots are initialised + read at runtime (caller-trim).
  trajectory::JointSpaceTrajectory<kDemoTaskMaxHandDof> hand_trajectory_;
  double hand_trajectory_time_{0.0};
  bool hand_new_target_pending_{false};  // RT-thread-only

  // ── Grasp controller (force_pi mode) ──────────────────────────────────────
  // Hand grasp-intervention mode, resolved once from the whitelisted
  // `grasp_controller_type` string in LoadConfig so the RT hot path branches on
  // an enum instead of comparing a std::string every tick.
  GraspHandMode grasp_hand_mode_{GraspHandMode::kContactStop};
  std::unique_ptr<rtc::grasp::GraspController> grasp_controller_;

  // ── In-plane pull-force estimator (#167) ──────────────────────────────────
  // Configured in LoadConfig from the demo_shared `pull_estimator` block; tip
  // links resolve against the tree-model tip_links order (== fingertip_data_ /
  // fingertip_rotations_ slot order). Disabled (null estimator) without a hand
  // tree-model or when the block is absent. Output rides grasp_state_.pull.
  PullEstimatorWiring pull_wiring_;
  /// Finger index → hand joint indices mapping (ragged; fingers may have
  /// different DoF). finger_joint_map_[f][0 .. finger_dof_[f]) valid for
  /// f < num_grasp_fingers_. Cached from `DemoSharedConfig` in LoadConfig.
  int num_grasp_fingers_{3};
  std::array<int, rtc::grasp::kMaxGraspFingers> finger_dof_{{3, 3, 3}};
  std::array<std::array<int, rtc::grasp::kMaxDoFPerFinger>, rtc::grasp::kMaxGraspFingers>
      finger_joint_map_{{{{0, 1, 2}}, {{3, 4, 5}}, {{6, 7, 8}}}};

  /// contact_stop release-phase gate, per finger: hand-joint index + loosening
  /// sign (+1: angle-increase loosens; -1: angle-decrease loosens).
  /// Cached from `DemoSharedConfig::hand_release_gate` in LoadConfig.
  int num_release_gates_{3};
  std::array<std::size_t, rtc::grasp::kMaxGraspFingers> release_gate_idx_{{1, 4, 7}};
  std::array<int, rtc::grasp::kMaxGraspFingers> release_gate_sign_{{+1, -1, -1}};

  /// contact_stop hold latch (RT-thread-only). Set when contact first engages,
  /// cleared only by the release-phase gate or E-STOP — so the hand keeps its
  /// position after contact drops, rather than snapping back to the trajectory.
  bool contact_latched_{false};
  /// Hold target while latched. Refreshed from the LPF output on every tick
  /// that contact is present; frozen once contact drops (no random-walk drift).
  std::array<double, kDemoTaskMaxHandDof> hand_hold_position_{};
  /// Bessel LPF over measured hand position; warmed every valid tick, its
  /// output is used as the desired hold position while latched. Init in
  /// LoadConfig (throws); Apply/Reset are noexcept + heap-free (RT-safe).
  rtc::BesselFilterN<kDemoTaskMaxHandDof> hand_pos_filter_;

  /// Previous grasp phase (for state-transition logging; non-RT critical).
  uint8_t prev_grasp_phase_{0};

  // ── Logging (throttled — RT-safe by throttle interval) ───────────────────
  // Sub-logger handle cached at construction; bringup_logging.hpp owns the
  // canonical name ("integrated_bringup.demo_task_controller"). All hot-path log
  // calls must use *_THROTTLE variants with constants from
  // integrated_bringup::logging.
  rclcpp::Logger logger_{integrated_bringup::logging::DemoTaskLogger()};
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  /// Arm joint position the E-STOP path drives to. Authoritative source is
  /// LoadConfig(cfg["estop"]["arm_safe_position"]); only the first
  /// arm_dof_ slots are read by ComputeEstop. Zero-initialized by default.
  std::array<double, kDemoTaskMaxArmDof> safe_position_{};

  // Runtime DoF (resolved by LoadConfig/OnDeviceConfigsSet from YAML +
  // device configs). arm_dof_ from `estop.arm_safe_position` length;
  // hand_dof_ from secondary device joint_state_names size (0 when absent).
  int arm_dof_{0};
  int hand_dof_{0};

  // Sensor capability cache — populated in OnDeviceConfigsSet from
  // GetSensorLayout(secondary). See demo_joint_controller.hpp for rationale.
  bool has_native_contact_{false};
  bool has_native_displacement_{false};

  std::array<std::vector<double>, ControllerState::kMaxDevices> device_max_velocity_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_lower_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_upper_;

  CommandType command_type_{CommandType::kPosition};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  // ── Phase 4: controller-owned topic sub/pub handles ───────────────────
  ControllerTopicHandles owned_topics_;

  // RT compute → publish-thread handoff for controller-owned grasp/tof
  // topics. WriteOutput stores the freshly-computed values here; the publish
  // thread Loads them inside PublishNonRtSnapshot. Replaces the
  // PublishSnapshot::group_commands[gi].{grasp_state,tof_snapshot} slots.
  rtc::SeqLock<::rtc::grasp::GraspStateData> grasp_state_lock_;
  rtc::SeqLock<::integrated_bringup::ToFSnapshotData> tof_snapshot_lock_;

  // ── Phase B (gain→parameter migration): per-controller ROS 2 parameters ──
  //
  // Tunable gains (kp_translation, damping, trajectory_speed, ...) are
  // declared as parameters on the controller's own LifecycleNode in
  // on_configure. The set-parameters callback rebuilds a Gains snapshot
  // and stores it via gains_lock_.Store(); the SeqLock provides RT-safe
  // publication to the 500 Hz Compute() reader.
  //
  // max_traj_velocity / max_traj_angular_velocity / hand_max_traj_velocity
  // are declared with read_only=true (D-2): YAML/launch overrides at boot
  // are honoured, but `ros2 param set` is rejected post-startup.
  //
  // Force-PI grasp command rides on a separate srv (grasp_command_srv_)
  // because it is a one-shot event, not state.
  void DeclareGainParameters() noexcept;

  // Close open log channels and unbind every typed LogHandle (#238). Called
  // from on_cleanup and from on_configure's failure-rollback paths so a leaked
  // channel never makes the next RegisterLog() return an unbound handle.
  void ResetLogState() noexcept;
  rcl_interfaces::msg::SetParametersResult OnGainParametersSet(
      const std::vector<rclcpp::Parameter>& params) noexcept;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Service<rtc_msgs::srv::GraspCommand>::SharedPtr grasp_command_srv_;

  // ── Phase C (controller-owned CSV logging) ────────────────────────────
  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  std::vector<ParsedLogEntry> parsed_log_entries_;

  rtc::ControllerLogSet log_set_{"demo_task_controller"};

  // Lifetime CSV drop count already reported by the drain timer (#234 P-20) —
  // DrainControllerLogs warns once per new burst rather than once per drain.
  std::uint64_t log_drops_reported_{0};
  rtc::LogHandle<integrated_bringup::DeviceStateLogPod> primary_state_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceStateLogPod> secondary_state_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceSensorLogPod> secondary_sensor_log_handle_;
  rtc::LogHandle<integrated_bringup::PullEstimatorLogPod> pull_estimator_log_handle_;

  std::vector<std::string> primary_joint_names_;
  std::vector<std::string> secondary_joint_names_;
  std::vector<std::string> secondary_motor_names_;
  std::vector<std::string> secondary_sensor_names_;

  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
};

}  // namespace integrated_bringup
