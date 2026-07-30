#ifndef UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_
#define UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_

#include "integrated_bringup/controllers/fingertip_force_guard.hpp"
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
#include "rtc_urdf_bridge/pinocchio_cache.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include <rtc_msgs/srv/grasp_command.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <cstdint>
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
inline constexpr int kDemoJointMaxArmDof = 32;
inline constexpr int kDemoJointMaxHandDof = 32;
// Combined arm+hand actuated model capacity — sizes the ext↔Pinocchio reorder
// maps (unified kin&dyn Phase 5). Actual full_dof_ resolved at runtime.
inline constexpr int kDemoJointMaxFullDof = kDemoJointMaxArmDof + kDemoJointMaxHandDof;

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
// Runtime tunable parameters (per-controller LifecycleNode):
//   ROS 2 parameters declared on /demo_joint_controller/<name>: see
//   DeclareGainParameters() in demo_joint_controller.cpp. Read-only caps
//   robot_max_traj_velocity, hand_max_traj_velocity. Force-PI grasp
//   transitions: ~/grasp_command srv (rtc_msgs/GraspCommand).
class DemoJointController final : public RTControllerInterface {
 public:
  struct Gains {
    double robot_trajectory_speed{1.0};    ///< Desired joint speed for trajectory duration [rad/s]
    double hand_trajectory_speed{1.0};     ///< Desired hand speed for trajectory duration [rad/s]
    double robot_max_traj_velocity{3.14};  ///< Max joint velocity during trajectory [rad/s]
    double hand_max_traj_velocity{2.0};    ///< Max hand motor velocity during trajectory [rad/s]

    // Virtual TCP (fingertip-based control point)
    VirtualTcpConfig vtcp;

    // Grasp detection parameters. Capability-aware (see
    // demo_shared.yaml / sensor_layout.has_native_contact):
    //   sensor A: contact_thresh AND force_thresh (both required)
    //   sensor B: force_thresh only (contact_thresh unconsumed)
    float grasp_contact_threshold{0.5f};  ///< Native prob threshold [0,1] — sensor A only
    float grasp_force_threshold{1.0f};    ///< |F| threshold [N] — common
    int grasp_min_fingertips{2};          ///< grasp_detected = active_count ≥ N

    // Trajectory / grasp FSM tuning
    double contact_stop_release_eps{0.005};   ///< Hand contact-stop release hysteresis [rad]
    double contact_stop_lpf_cutoff_hz{20.0};  ///< Bessel LPF cutoff for latched hold [Hz]
    /// Bessel LPF cutoff for the per-axis fingertip force that contact_stop
    /// thresholds [Hz]. Higher than the hold-position cutoff on purpose: a 4th
    /// order Bessel's DC group delay is ~2.11/(2*pi*fc), so the 20 Hz used for
    /// position would delay the freeze by ~17 ms — a third of the 50 ms BT tick
    /// this latch exists to cover. 50 Hz costs ~6.7 ms instead.
    double contact_stop_force_lpf_cutoff_hz{50.0};
    /// Delta-spike guard on the raw force entering that LPF (see
    /// fingertip_force_guard.hpp). Per-axis |Δ| rejection threshold [N] and the
    /// consecutive-hold cap [ticks] that keeps a genuine sustained step from
    /// being rejected forever.
    double contact_stop_force_guard_delta_n{4.0};
    int contact_stop_force_guard_max_hold_ticks{2};
  };

  explicit DemoJointController(std::string_view urdf_path);
  DemoJointController(std::string_view urdf_path, Gains gains);

  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override { return "DemoJointController"; }

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
  // gains layout: [robot_trajectory_speed, hand_trajectory_speed,
  //                robot_max_traj_velocity, hand_max_traj_velocity,
  //                grasp_contact_threshold, grasp_force_threshold,
  //                grasp_min_fingertips,
  //                grasp_command, grasp_target_force] = 9 values
  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  void set_gains(Gains gains) noexcept { gains_lock_.Store(gains); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  // Phase 4c: ControllerOutput::grasp_state / tof_snapshot fields were
  // removed — tests read the post-Compute() staging buffers directly via
  // these accessors (RT-7 safe; no behavior change).
  [[nodiscard]] const ::rtc::grasp::GraspStateData& GetGraspStateForTesting() const noexcept {
    return grasp_state_;
  }

  /// Test-only: the FILTERED contact_stop aggregates. Distinct from
  /// GetGraspStateForTesting(), whose max_force / num_active_contacts stay raw
  /// — only these two drive the freeze decision, so a test that reads the
  /// GraspState cannot tell whether the LPF is wired in at all.
  struct ContactStopAggregates {
    float max_force{0.0F};
    int active_count{0};
  };

  [[nodiscard]] ContactStopAggregates GetContactStopAggregatesForTesting() const noexcept {
    return {contact_stop_max_force_, contact_stop_active_count_};
  }

  /// Test-only: the delta-spike guard's per-tick output — what the force LPF was
  /// fed ([fingertip * 3 + axis]) and whether the raw triplet was held out.
  /// Distinct from both the raw force and the filter output: on a held tick all
  /// three differ, and only this pair says the guard is what changed the input.
  [[nodiscard]] std::span<const float> GetGuardedFingertipForceForTesting() const noexcept {
    return fingertip_force_guarded_;
  }

  [[nodiscard]] bool WasForceGuardRejectedForTesting(std::size_t f) const noexcept {
    return f < force_guard_rejected_.size() && force_guard_rejected_[f] != 0;
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

  /// Test-only: override the grasp_hand_mode_ that LoadConfig would set from
  /// demo_shared.yaml, so the "none" no-op / contact_stop branches can be
  /// exercised without a full YAML load. Accepts the same whitelisted strings.
  void SetGraspControllerTypeForTesting(std::string_view type) {
    grasp_hand_mode_ = ParseGraspHandMode(std::string(type));
  }

  /// Test-only: fingertips carrying a raw sensor lane (gates the ToF snapshot),
  /// separate from the inference-group count reported in GraspState.
  [[nodiscard]] int GetNumSensorFingertipsForTesting() const noexcept {
    return num_sensor_fingertips_;
  }

 private:
  // ── Phase 1→2 intermediate: parsed sensor data ──────────────────────────
  // Backend = hardware raw, controller = behavior: force/in_contact for
  // grasp detection; tof retained for the ToF publish snapshot. Slot 0
  // (contact_flag) and slots 4..6 (displacement) are no longer consumed
  // by this controller — see Layer D for full backend/controller split.
  struct FingertipSensorData {
    std::array<int32_t, 3> tof{};  ///< ToF distances (publish-only)
    std::array<float, 3> force{};  ///< fx, fy, fz (link frame, N)
    float force_mag{0.0f};         ///< ‖force‖ cached in ReadState (reused by grasp + vtcp)
    float contact_flag{0.0f};      ///< native sigmoid prob (sensor A only)
    bool valid{false};             ///< inference_enable[f] (backend fresh)
    bool in_contact{false};        ///< capability-aware: |F|>force_thresh AND
                                   ///< (sensor A path also requires native
                                   ///< prob > contact_thresh)
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

  ComputedTrajectory robot_computed_{};
  ComputedTrajectory hand_computed_{};

  // ── Hand fingertip FK dispatch (serial hand_handle_ ↔ closed_hand_fk_) ────
  // #121: single branch point for closed-chain vs serial hand FK so every call
  // site stays byte-for-byte when closure is absent/inactive. (See
  // demo_task_controller.hpp for the contract.)
  [[nodiscard]] bool ComputeHandForwardKinematics(const ControllerState& state) noexcept;
  [[nodiscard]] bool HandFingertipPose(std::size_t f, pinocchio::SE3& out) const noexcept;
  void ConfigureClosedChainHandFk();

  // ── 3-phase pipeline ────────────────────────────────────────────────────
  void ReadState(const ControllerState& state) noexcept;
  /// Advance fingertip `f`'s per-axis force LPF from the raw force ReadState
  /// just parsed, writing fingertip_force_filt_ / fingertip_force_mag_filt_.
  /// The raw triplet passes through the delta-spike guard first, so
  /// fingertip_force_guarded_ / force_guard_rejected_ are refreshed here too.
  void UpdateFingertipForceFilter(std::size_t f,
                                  const FingertipForceGuardConfig& guard_cfg) noexcept;
  /// Config-time (throws): Init every per-fingertip force filter and clear the
  /// derived state. Called from the constructor and from LoadConfig.
  void InitFingertipForceFilters(double cutoff_hz);
  void ComputeControl(const ControllerState& state, double dt) noexcept;
  // WriteOutput was split into 3 explicit-intent methods. The Compute()
  // dispatcher calls them in this order:
  //   1) WriteJointCommand — wire-bound fields only (devices[i].commands,
  //      num_channels, goal_type, num_devices, command_type, valid). The
  //      DeviceBackend reads ONLY these via WriteCommand().
  //   2) FillLogOutput    — fields the DeviceStateLogPod reads
  //      (goal_positions, trajectory_positions, trajectory_velocities,
  //      actual_task_positions, task_goal_positions) + SeqLock store of
  //      grasp/tof staging buffers.
  //   3) FillPublishOutput — fields the publish snapshot / owned_topics
  //      consumes (target_positions, target_velocities, trajectory_task_*,
  //      arm_tip_pose*, virtual_tcp_pose*, task_link_poses*). Fields shared
  //      with log are filled here too: each consumer's method is self-
  //      contained so the intent of each block is obvious.
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

  // E-STOP counterpart of FillLogOutput's SeqLock store (#234 P-1). The CM
  // stamps every publish snapshot with the current wall clock and the publish
  // thread re-loads whatever the owned SeqLock holds, so an E-STOP tick that
  // stores nothing is republished as fresh-stamp/stale-body — a `valid=1` pull
  // estimate that outlives the loop that produced it. Sensor-derived fields
  // (ReadState refreshed them this tick) stay; every field the control law
  // would have produced is neutralized: the pull estimate runs its E-STOP tick
  // (valid=false + decay) and the Force-PI per-finger diagnostics are cleared.
  // grasp_phase / grasp_target_force are FSM latches, not per-tick derivations,
  // so they are left alone. RT tick path — noexcept, heap-free.
  void FillEstopPublishState(double dt) noexcept;

  // ── Internal state ──────────────────────────────────────────────────────
  rtc::SeqLock<Gains> gains_lock_;

  // SeqLock for the per-device joint target slots. RT thread (Compute) is the
  // SOLE writer of target_seqlock_; off-RT writers hand their goal to the base
  // mailbox via SetDeviceTarget → PushPendingTarget, and the RT drain calls
  // ApplyPendingTarget below. The slot layout and its meaning stay here — the
  // marshal itself is base-owned (#206).
  struct TargetSlot {
    std::array<std::array<double, rtc::kMaxDeviceChannels>, ControllerState::kMaxDevices> targets{};
  };

  static_assert(std::is_trivially_copyable_v<TargetSlot>,
                "TargetSlot must be trivially copyable for SeqLock<TargetSlot>");

  // One drained target → this controller's slot. Called on the RT tick from
  // DrainTargetSlot(), with device_idx and values already bounded by the base.
  // `is_task` is unused: this controller keeps one joint slot per device, and
  // its task ingress is the base default (which forwards to SetDeviceTarget).
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override;

  rtc::SeqLock<TargetSlot> target_seqlock_;
  // Per-device self-init flags. The arm (device 0) is seeded immediately on the
  // first tick; the hand (device 1) hold-seed is DEFERRED until it first reports
  // a valid measured state. A single shared flag used to flip true after the
  // arm seed even when the hand was still coming up, leaving targets[1] at its
  // zero init and driving every finger to 0 (ur5e fine / p1b collapse). Split so
  // the hand seed retries each tick until device 1 is valid.
  std::atomic<bool> arm_target_initialized_{false};
  std::atomic<bool> hand_target_initialized_{false};

  // Base hook — every activation re-arms both self-init paths so a re-activated
  // controller holds where the robot actually is, not where it was commanded
  // before the switch. Previously duplicated inside on_activate; centralising it
  // here keeps every controller's reset on one contract (#196 §3).
  void ResetTargetInitialization() noexcept override {
    arm_target_initialized_.store(false, std::memory_order_release);
    hand_target_initialized_.store(false, std::memory_order_release);
  }

  // RT-thread-only working copy of the current TargetSlot; ComputeControl /
  // WriteOutput read from this instead of touching target_seqlock_ multiple
  // times per tick. Refreshed by DrainTargetSlot() at the top of Compute().
  TargetSlot current_target_slot_{};

  // RT-thread-only: refresh current_target_slot_ from the SeqLock, drain any
  // off-RT SetDeviceTarget marshal entries, run first-tick self-init.
  void DrainTargetSlot(const ControllerState& state) noexcept;

  // ── rtc_urdf_bridge ────────────────────────────────────────────
  std::string urdf_path_;  // stored from constructor, used in LoadConfig
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> arm_handle_;
  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::FrameIndex root_frame_id_{0};
  bool use_root_frame_{false};

  // ── Unified kin&dyn combined-model cache (#174) ──────────────────────────
  // Arm TCP FK comes from the shared combined (arm+hand) model cache, updated
  // once per non-E-STOP tick (replacing the arm-only arm_handle_ FK; arm_handle_
  // retained only for the E-STOP TF path). Joint is FK-only (no Jacobian
  // consumer). The cache wiring — model select, ext→Pinocchio reorder map, per-
  // tick state scatter, arm-TCP FK — is shared with task/wbc via CombinedModelCache.
  CombinedModelCache combined_cache_;
  int full_dof_{0};
  // registered_frames indices for the arm TCP tip / base (< 0 = universe/world).
  // Registered on the combined cache with the ARM sub-model frame ids (frame-id
  // consistency proven by test_wbc_arm_tcp_cache_equivalence).
  int arm_tcp_frame_idx_{-1};
  int arm_base_frame_idx_{-1};

  // Arm base→tip FK computed once per tick in ComputeControl; FillLogOutput /
  // FillPublishOutput reuse this instead of recomputing the full FK pass.
  // Valid only on non-E-STOP ticks — ComputeEstop bypasses ComputeControl and
  // recomputes its own FK, so estop readers must not touch this member.
  pinocchio::SE3 arm_tcp_pose_{pinocchio::SE3::Identity()};
  // ── Hand tree-model for fingertip FK ──────────────────────────────────
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> hand_handle_;
  static constexpr std::size_t kNumFingertips = 4;
  std::array<pinocchio::FrameIndex, kNumFingertips> fingertip_frame_ids_{};
  pinocchio::FrameIndex hand_root_frame_id_{0};
  bool use_hand_root_frame_{false};
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

  // ── Virtual TCP (fingertip-based control point) ───────────────────────
  pinocchio::SE3 vtcp_pose_{pinocchio::SE3::Identity()};  ///< World-frame virtual TCP pose (cached)
  bool vtcp_valid_{false};                                ///< Virtual TCP computed successfully
  std::array<FingertipVtcpInput, kNumFingertips> vtcp_inputs_{};  ///< Pre-allocated

  void UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp, const Gains& gains) noexcept;

  void InitArmModel(const rtc_urdf_bridge::ModelConfig& config);
  void InitHandModel(const rtc_urdf_bridge::ModelConfig& config);

  CommandType command_type_{CommandType::kPosition};

  // ── Trajectory ───────────────────────────────────────────────────────────
  // RT-thread-only flags; SetDeviceTarget marshals through the base mailbox and
  // ApplyPendingTarget flips these when a device-0 / device-1 entry is drained.
  bool robot_new_target_pending_{false};
  bool hand_new_target_pending_{false};
  // Templates fixed at compile-time capacity; only the first arm_dof_ /
  // hand_dof_ slots are initialised + read at runtime (caller-trim pattern).
  trajectory::JointSpaceTrajectory<kDemoJointMaxArmDof> robot_trajectory_;
  trajectory::JointSpaceTrajectory<kDemoJointMaxHandDof> hand_trajectory_;
  double robot_trajectory_time_{0.0};
  double hand_trajectory_time_{0.0};

  std::array<std::vector<double>, ControllerState::kMaxDevices> device_max_velocity_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_lower_;
  std::array<std::vector<double>, ControllerState::kMaxDevices> device_position_upper_;

  // ── Grasp controller (force_pi mode) ──────────────────────────────────────
  // Hand grasp-intervention mode, resolved once from the whitelisted
  // `grasp_controller_type` string in LoadConfig so the RT hot path branches on
  // an enum instead of comparing a std::string every tick.
  GraspHandMode grasp_hand_mode_{GraspHandMode::kContactStop};
  std::unique_ptr<rtc::grasp::GraspController> grasp_controller_;
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
  std::array<double, kDemoJointMaxHandDof> hand_hold_position_{};
  /// Bessel LPF over measured hand position; warmed every valid tick, its
  /// output is used as the desired hold position while latched. Init in
  /// LoadConfig (throws); Apply/Reset are noexcept + heap-free (RT-safe).
  rtc::BesselFilterN<kDemoJointMaxHandDof> hand_pos_filter_;

  // ── Fingertip force LPF (contact_stop decision variable) ─────────────────
  //
  // One filter PER FINGERTIP rather than a single BesselFilterN<3*N>: an
  // invalid fingertip must not advance its own filter, and Apply() is
  // all-channels-or-nothing. Runs on every tick regardless of
  // grasp_controller_type so the logged column means the same thing in every
  // run; force_pi keeps its own separate scalar filter over |F|.
  //
  // Per-axis, not on |F|: filtering the magnitude lets zero-mean axis noise
  // rectify into a positive bias that walks the aggregate toward the contact
  // threshold with no contact present.
  std::array<rtc::BesselFilterN<3>, rtc::kMaxSensorGroups> force_lpf_{};
  /// False until the next valid sample seeds the corresponding filter. Cleared
  /// whenever a fingertip goes invalid so re-entry seeds to the live force
  /// instead of ramping up from a delay line the world has left behind.
  std::array<bool, rtc::kMaxSensorGroups> force_lpf_primed_{};
  /// Delta-spike guard in front of each filter — the LPF is fed this guard's
  /// output, never the raw triplet. Invalidated together with
  /// force_lpf_primed_ so a dropout does not leave a stale comparison base
  /// that would reject the live force forever.
  std::array<FingertipForceGuard, rtc::kMaxSensorGroups> force_guard_{};
  /// LPF output packed [fingertip * 3 + axis] — contiguous because the CSV POD
  /// fill takes a span. Zero on invalid fingertips, matching raw `ft.force`.
  std::array<float, 3U * static_cast<std::size_t>(rtc::kMaxSensorGroups)> fingertip_force_filt_{};
  /// What the LPF was actually fed, same packing. Equal to the raw triplet on
  /// accepted ticks; the held triplet on rejected ones. CSV-only — no consumer
  /// of this controller's outputs reads it.
  std::array<float, 3U * static_cast<std::size_t>(rtc::kMaxSensorGroups)>
      fingertip_force_guarded_{};
  /// Per-fingertip guard verdict for the current tick (1 = raw was held out).
  std::array<std::uint8_t, rtc::kMaxSensorGroups> force_guard_rejected_{};
  /// ‖filtered force‖ per fingertip [N].
  std::array<float, rtc::kMaxSensorGroups> fingertip_force_mag_filt_{};
  /// contact_stop aggregates derived from the FILTERED magnitudes. Kept apart
  /// from grasp_state_.max_force / num_active_contacts, which stay raw so the
  /// published GraspState and its BT consumers see an unchanged contract.
  float contact_stop_max_force_{0.0F};
  int contact_stop_active_count_{0};

  /// Previous grasp phase (for state-transition logging; non-RT critical).
  uint8_t prev_grasp_phase_{0};

  // ── In-plane pull-force estimator (#167) ──────────────────────────────────
  // Configured in LoadConfig from the demo_shared `pull_estimator` block; tip
  // links resolve against the tree-model tip_links order (== fingertip_data_ /
  // fingertip_rotations_ slot order). Disabled (null estimator) without a hand
  // tree-model or when the block is absent. Output rides grasp_state_.pull.
  PullEstimatorWiring pull_wiring_;

  // ── Logging (throttled — RT-safe by throttle interval) ───────────────────
  // Sub-logger handle cached at construction; bringup_logging.hpp owns the
  // canonical name ("integrated_bringup.demo_joint_controller"). All hot-path log
  // calls must use *_THROTTLE variants with constants from
  // integrated_bringup::logging.
  rclcpp::Logger logger_{integrated_bringup::logging::DemoJointLogger()};
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};
  bool estop_active_{false};

  /// F5 device-readability gate for the arm (device 0), evaluated once at the
  /// top of Compute() so every phase of this tick — ReadState, DrainTargetSlot,
  /// ComputeControl, the three output fills, ComputeEstop — sees one consistent
  /// answer. Same idiom and same reason as estop_active_ above.
  ///
  /// False means device 0 did not report the arm_dof_ channels this controller
  /// reads, so its unreported joints would come back as a finite 0.0 and the
  /// trajectory, the FK and the emitted command would all run at a partially
  /// ZERO configuration. The answer is silence on device 0 (zero-length), NOT
  /// nc0 zeros — see rtc_controller_interface/device_readability.hpp and §3.7 of
  /// rtc_controllers/docs/compliance-conventions.md. The hand (device 1) is
  /// untouched by this flag: it keeps its own state, target and trajectory.
  /// The hand's own width is gated by hand_readable_ below, not by this flag.
  ///
  /// RT-thread-only, like estop_active_ — no synchronisation needed.
  bool arm_readable_{false};

  /// The same F5 gate on the SECONDARY axis (issue #291). arm_readable_ is
  /// device 0's policy and deliberately does not silence the hand (§3.7,
  /// "secondary passthrough 유지"); this flag is the hand's answer about
  /// ITSELF. False means device 1 did not report the hand_dof_ channels the
  /// hand loops read — hand_dof_ comes from YAML joint_state_names.size()
  /// while num_channels is the wire length, so a start-up mismatch would read
  /// the unreported finger joints as a finite 0 and ramp them origin-ward.
  /// The answer is the same as the arm's: silence device 1 (zero-length) with
  /// the reference lanes held at the measurement. RT-thread-only.
  bool hand_readable_{false};

  /// Arm joint position the E-STOP path drives to. Authoritative source is
  /// LoadConfig(cfg["estop"]["arm_safe_position"]); only the first
  /// arm_dof_ slots are read by ComputeEstop. Zero-initialised by default
  /// so unit/integration paths that skip LoadConfig still see a
  /// deterministic value.
  std::array<double, kDemoJointMaxArmDof> safe_position_{};

  // Runtime DoF (resolved by LoadConfig/OnDeviceConfigsSet from YAML +
  // device configs). arm_dof_ from `estop.arm_safe_position` length;
  // hand_dof_ from secondary device joint_state_names size (0 when absent).
  // RT-path loops iterate over these.
  int arm_dof_{0};
  int hand_dof_{0};

  // Sensor capability cache — populated in OnDeviceConfigsSet from
  // GetSensorLayout(secondary). RT path (ReadState) reads only these bools,
  // not the optional layout object. Default false ⇒ controllers fall back to
  // |F| > grasp_force_threshold derive when secondary device omits
  // sensor_layout or declares both capabilities false.
  bool has_native_contact_{false};
  bool has_native_displacement_{false};

  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  // ── Phase 4: controller-owned topic sub/pub handles ───────────────────
  ControllerTopicHandles owned_topics_;

  // RT compute → publish-thread handoff for controller-owned grasp/tof
  // topics. WriteOutput stores the freshly-computed values here; the publish
  // thread Loads them inside PublishNonRtSnapshot. Replaces the
  // PublishSnapshot::group_commands[gi].{grasp_state,tof_snapshot} slots.
  rtc::SeqLock<::rtc::grasp::GraspStateData> grasp_state_lock_;
  rtc::SeqLock<::integrated_bringup::ToFSnapshotData> tof_snapshot_lock_;

  // ── Phase D (gain→parameter migration): per-controller ROS 2 parameters ──
  //
  // Tunable gains (robot_trajectory_speed, hand_trajectory_speed, grasp_*) are
  // declared as parameters on the controller's own LifecycleNode in
  // on_configure. The set-parameters callback rebuilds a Gains snapshot and
  // stores it via gains_lock_; SeqLock provides RT-safe handoff to the 500 Hz
  // Compute() reader.
  //
  // robot_max_traj_velocity / hand_max_traj_velocity are declared with
  // read_only=true (D-2): startup overrides honoured, runtime `param set`
  // rejected.
  //
  // Force-PI grasp_command rides on a dedicated srv (grasp_command_srv_)
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

  // ── Phase C (controller-owned CSV logging) ──────────────────────────────
  //
  // Declared via `logs:` section in YAML; each entry maps an rtc_msgs/<*Log>
  // type to a CSV under
  // `<session>/controllers/demo_joint_controller/<instance>.csv`. Push site is
  // *only* inside Compute() (Q-ACTIVITY-GATING) — drain timer runs on a non-RT
  // callback group at 100 ms and writes pending rows.
  //
  // Coexists with the legacy CM-side DataLogger path during Phase C transition
  // (parallel write for byte-comparison); Track R removes the legacy side.
  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  std::vector<ParsedLogEntry> parsed_log_entries_;

  rtc::ControllerLogSet log_set_{"demo_joint_controller"};

  // Lifetime CSV drop count already reported by the drain timer (#234 P-20) —
  // DrainControllerLogs warns once per new burst rather than once per drain.
  std::uint64_t log_drops_reported_{0};
  rtc::LogHandle<integrated_bringup::DeviceStateLogPod> primary_state_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceStateLogPod> secondary_state_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceSensorLogPod> secondary_sensor_log_handle_;
  rtc::LogHandle<integrated_bringup::PullEstimatorLogPod> pull_estimator_log_handle_;

  // Captured at on_configure for header expansion (not RT).
  std::vector<std::string> primary_joint_names_;
  std::vector<std::string> secondary_joint_names_;
  std::vector<std::string> secondary_motor_names_;
  std::vector<std::string> secondary_sensor_names_;
  /// Hand DeviceSensorLayout stride, cached for the CSV header/row writers. 0
  /// on a force-only hand → no barometer/ToF columns at all.
  std::size_t secondary_sensor_values_per_group_{
      integrated_bringup::DeviceSensorLogPod::kSensorValuesPerFingertip};

  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;

  // Pod fill helpers live in integrated_bringup/logging/pod_fill.hpp (shared by
  // all 3 demo controllers — Phase C).
};

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_CONTROLLERS_DEMO_JOINT_CONTROLLER_H_
