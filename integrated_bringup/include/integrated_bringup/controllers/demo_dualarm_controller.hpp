#ifndef INTEGRATED_BRINGUP_CONTROLLERS_DEMO_DUALARM_CONTROLLER_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_DEMO_DUALARM_CONTROLLER_HPP_

// ── DemoDualArmController: several task frames of one device group, one QP ──
//
// The binding for a robot whose primary device group is a kinematic TREE (a
// trunk with two arms) and whose task is stated on more than one frame of it.
// Every tick it hands the multi-frame call of rtc::tsid::ClikReferenceGenerator
// a list of SE3 frame tasks — each with its own base frame, so a task can be
// "this hand with respect to the torso" — plus posture groups, and integrates
// the solution into the group's position command. The QP itself (rows,
// relative Jacobian, boxes, torque rows, braking bound) is the core's; what is
// decided HERE is what the core is given, and what happens on the ticks it is
// not called.
//
// Layout (docs/dynamic_catching/ref/mpc_multiframe_clik_formulation.md §2 is
// the formulation; the differences are tabled in the package README):
//   - model: the combined-model cache every demo controller shares. The body
//     group's joints are the "arm" set of the solve; the hand group's joints
//     are decision variables locked through their velocity box, because this
//     controller commands the hand in joint space, not through the QP;
//   - evaluation: at the COMMAND state. The cache the solve reads carries this
//     controller's own previous command on the body joints, so error, Jacobian
//     and boxes follow the commanded path and servo lag stays out of the loop;
//   - tasks: a YAML list. Each names its frame and its BASE frame, and the two
//     are both registered frames of the cache — also when the base is a fixed
//     frame, so that the target, the error and the rows are all taken in the
//     base frame's axes by one code path;
//   - goals: one `<task>/task_goal` subscription per task. A goal carries the
//     frame it is EXPRESSED in (`header.frame_id`), which need not be the
//     task's base frame: the tick that applies the goal converts it into the
//     base frame once, at that tick's command state, and from then on the goal
//     is a pose fixed in the base frame;
//   - reference: a quintic task-space trajectory per task from the current
//     reference to the goal, giving the pose and the feed-forward twist;
//   - posture: `<body group>/joint_goal` sets the posture target;
//   - hand: `<hand group>/joint_goal` drives a joint-space quintic.
//
// Ticks that do not solve:
//   - E-STOP: neither the solve nor a trajectory runs. Both groups command the
//     measured positions (the controller manager substitutes its own hold on
//     the wire). On release the command is re-seeded from the measurement and
//     every goal becomes "where the frame is now" — nothing resumes;
//   - a failed solve keeps the previous command; `fault.max_qp_fail_ticks` of
//     them in a row raise a controller-local fault latch, which holds the last
//     command, refuses goals and leaves only through /rtc_cm/reset_fault;
//   - an unreadable body device freezes the solve and the references and
//     silences the group; an unreadable hand device silences the hand only.

#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/dualarm_diag_log_pod.hpp"
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"
#include "integrated_bringup/support/combined_model_cache.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

// Pinocchio headers (warnings suppressed)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/timer.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace integrated_bringup {

inline constexpr int kDualArmBodyDeviceIdx = 0;
inline constexpr int kDualArmHandDeviceIdx = 1;
/// Capacity of the fixed per-joint buffers; a device group wider than this is
/// refused at configure. Not a joint count of any robot.
inline constexpr int kDualArmMaxBodyDof = static_cast<int>(DualArmDiagLogPod::kMaxJoints);
inline constexpr int kDualArmMaxHandDof = 32;
inline constexpr std::size_t kDualArmMaxTasks = DualArmDiagLogPod::kMaxTasks;
inline constexpr std::size_t kDualArmMaxPostureGroups = 8;
inline constexpr std::size_t kDualArmMaxTargetFrames = 8;
inline constexpr std::size_t kDualArmMaxFingertips = 4;

/// The controller's YAML, parsed and range-checked. Pure data: nothing here
/// has met a model or a device yet.
struct DualArmConfig {
  struct Task {
    std::string name;          ///< also the topic token: `<name>/task_goal`
    std::string frame;         ///< the frame the task moves
    std::string base_frame;    ///< the frame its target, error and rows are taken in
    double gain_linear{0.0};   ///< [1/s]
    double gain_angular{0.0};  ///< [1/s]
    double weight{1.0};
    double fb_lin_max{0.0};  ///< [m/s], 0 = no cap
    double fb_ang_max{0.0};  ///< [rad/s], 0 = no cap
  };

  struct PostureGroup {
    std::string name;
    std::vector<std::string> joints;  ///< joints of the body device group
    double weight{0.0};
    double gain{0.0};  ///< [1/s]
  };

  std::vector<Task> tasks;
  /// Frames a task goal may be expressed in (`header.frame_id`). A goal with
  /// an empty frame id is in its task's base frame and needs no entry here.
  std::vector<std::string> target_frames;
  std::vector<PostureGroup> posture_groups;

  double damping_sq{0.0};
  double w_smooth{0.0};
  int max_iter{0};
  /// Fraction of the device torque limit the torque rows may use, (0, 1.2].
  double eta_tau{0.0};
  double limit_margin{0.0};        ///< [rad], body joints only
  double joint_velocity_max{0.0};  ///< [rad/s], caps the device rating
  bool brake_enabled{false};
  double brake_margin{1.0};

  double linear_speed{0.0};       ///< [m/s]
  double angular_speed{0.0};      ///< [rad/s]
  double linear_speed_max{0.0};   ///< [m/s], bounds the trajectory's peak
  double angular_speed_max{0.0};  ///< [rad/s]
  double hand_speed{0.0};         ///< [rad/s]
  double hand_speed_max{0.0};     ///< [rad/s]

  int max_qp_fail_ticks{0};
  double track_err_max{0.0};  ///< [rad], 0 = off
};

/// Largest `eta_tau` the YAML may carry. The core takes (0, 1]; this binding
/// passes `tau_max = eta_tau · device limit` with a core margin of 1, which is
/// the same product in every row that reads it.
inline constexpr double kDualArmEtaTauMax = 1.2;

/// Parse and range-check the controller's YAML node. Throws std::runtime_error
/// naming the key. Every key is required: there is no default that is right
/// for a robot this file has not seen.
[[nodiscard]] DualArmConfig ParseDualArmConfig(const YAML::Node& cfg);

class DemoDualArmController final : public rtc::RTControllerInterface {
 public:
  /// Runtime-tunable values, as the tick reads them.
  struct Gains {
    std::array<double, kDualArmMaxTasks> task_gain_linear{};
    std::array<double, kDualArmMaxTasks> task_gain_angular{};
    std::array<double, kDualArmMaxPostureGroups> posture_gain{};
    double linear_speed{0.1};
    double angular_speed{0.5};
    double hand_speed{1.0};
  };

  explicit DemoDualArmController(std::string_view urdf_path);
  ~DemoDualArmController() override;

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState& state) noexcept override;

  /// Joint goals of the two device groups: the body group's is the posture
  /// target, the hand group's the hand's. A goal that does not carry exactly
  /// the group's joints is refused and counted — the base lets a short
  /// positional goal through, and here it would set some joints' posture and
  /// leave the rest.
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  /// Refused and counted. The base forwards a task goal on a group topic into
  /// SetDeviceTarget, where its six values would be read as joint positions;
  /// task goals arrive on `<task>/task_goal`.
  void SetDeviceTaskTarget(int device_idx, std::span<const double> task6) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override { return "DemoDualArmController"; }

  [[nodiscard]] rtc::CommandType GetCommandType() const noexcept override {
    return rtc::CommandType::kPosition;
  }

  // E-STOP and fault hooks: every body is an atomic store; the tick acts.
  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void ResetFault() noexcept override;
  [[nodiscard]] bool HasLatchedFault() const noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml_cfg) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) noexcept override;

  void PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept override;

  // ── Read-only views (tests, diagnostics) ─────────────────────────────────

  /// Why the model / solve could not be set up, or empty. on_configure refuses
  /// on a non-empty string; the hook that builds the runtime cannot refuse by
  /// itself.
  [[nodiscard]] const std::string& ConfigError() const noexcept { return config_error_; }

  /// The record of the last Compute(). RT-owned: read it from the thread that
  /// calls Compute().
  [[nodiscard]] const DualArmDiagLogPod& LastTick() const noexcept { return tick_; }

  [[nodiscard]] std::size_t NumTasks() const noexcept { return num_tasks_; }

  /// The frame names a task goal's header.frame_id may carry.
  [[nodiscard]] const std::vector<std::string>& TargetFrameNames() const noexcept {
    return target_frame_names_;
  }

  /// What the `<task>/task_goal` subscription does with a message — the same
  /// body, callable without a node. Returns the ingress verdict.
  TaskGoalReject DeliverTaskGoal(std::size_t task, const rtc_msgs::msg::RobotTarget& msg) noexcept;

  [[nodiscard]] std::uint32_t GroupGoalRejectCount() const noexcept {
    return group_goal_rejects_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] Gains GetGains() const noexcept { return gains_lock_.Load(); }

  /// Replace the runtime gains (what the parameter callback does after its
  /// checks). Values are bounded again where the tick uses them.
  void SetGains(const Gains& gains) noexcept { gains_lock_.Store(gains); }

 protected:
  void OnDeviceConfigsSet() override;
  void ApplyPendingTarget(int device_idx, std::span<const double> values,
                          bool is_task) noexcept override;

 private:
  /// Test seam for the allocation gate: the goal-application step of the tick
  /// has to be measurable apart from the solve it shares a tick with.
  friend struct DualArmTestAccess;

  struct TaskRt {
    // Configuration (fixed after the runtime is built).
    int frame_idx{-1};                   ///< cache registered-frame index of the task frame
    int base_idx{-1};                    ///< ... of its base frame
    pinocchio::FrameIndex frame_fid{0};  ///< model frame ids, for the measured FK
    pinocchio::FrameIndex base_fid{0};
    double weight{1.0};
    double fb_lin_max{0.0};
    double fb_ang_max{0.0};
    // Reference state (RT-owned).
    rtc::trajectory::TaskSpaceTrajectory traj;
    double traj_time{0.0};
    bool traj_active{false};
    pinocchio::SE3 goal{pinocchio::SE3::Identity()};      ///< in the base frame
    pinocchio::SE3 ref_pose{pinocchio::SE3::Identity()};  ///< T^d(t), in the base frame
    Eigen::Matrix<double, 6, 1> twist_ff{Eigen::Matrix<double, 6, 1>::Zero()};
    std::uint32_t seen_sequence{0};  ///< last ingress sequence the tick consumed
    std::uint32_t goal_sequence{0};  ///< sequence of the goal in force (0 = hold pose)
    double err_norm{0.0};            ///< ‖6D pose error‖ of the last solved tick
    std::array<std::uint32_t, static_cast<std::size_t>(DualArmDiagLogPod::GoalDrop::kCount)>
        drops{};
  };

  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  // ── Non-RT setup ─────────────────────────────────────────────────────────
  /// Build everything that needs the devices and the model: the cache, the
  /// frames, the solve. Returns the reason it could not, or empty.
  [[nodiscard]] std::string BuildRuntime();
  void ResetRuntime() noexcept;
  [[nodiscard]] std::string SetupModel();
  [[nodiscard]] std::string SetupFrames();
  [[nodiscard]] std::string SetupClik();
  [[nodiscard]] std::string SetupMeasuredKinematics();
  void DeclareParameters();
  [[nodiscard]] rcl_interfaces::msg::SetParametersResult OnParametersSet(
      const std::vector<rclcpp::Parameter>& params) noexcept;
  void ResetLogState() noexcept;
  /// Undo what on_configure created (also after it failed half-way).
  void TearDownConfigured() noexcept;
  /// Largest task / posture gain the discrete loop takes without overshoot:
  /// K·h ≤ 1.
  [[nodiscard]] double GainUpperBound() const noexcept;

  // ── RT tick steps ────────────────────────────────────────────────────────
  void ServiceRequests() noexcept;
  void Reseed(const rtc::ControllerState& state) noexcept;
  void UpdateEvalCache() noexcept;
  void ConsumeTaskGoals(bool apply) noexcept;
  [[nodiscard]] bool ApplyTaskGoal(std::size_t k, const TaskGoal& goal) noexcept;
  void AdvanceReference(std::size_t k, double dt) noexcept;
  void RunClikTick(const rtc::ControllerState& state, double dt) noexcept;
  void RunHandLane(const rtc::ControllerState& state, double dt) noexcept;
  void UpdateMeasuredKinematics(const rtc::ControllerState& state,
                                rtc::ControllerOutput& output) noexcept;
  void WriteBodyCommand(const rtc::ControllerState& state, rtc::ControllerOutput& output) noexcept;
  void WriteHandCommand(const rtc::ControllerState& state, rtc::ControllerOutput& output) noexcept;
  void LatchFault(DualArmDiagLogPod::FaultCause cause) noexcept;
  void FillTickRecord() noexcept;
  void CountGroupGoalReject(const char* reason) noexcept;

  // ── Configuration ────────────────────────────────────────────────────────
  rclcpp::Logger logger_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  DualArmConfig cfg_;
  bool cfg_loaded_{false};
  std::string config_error_{"the controller configuration has not been loaded"};
  std::vector<ParsedLogEntry> parsed_log_entries_;

  // ── Model and solve (built by BuildRuntime) ──────────────────────────────
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<CombinedModelCache> combined_cache_;
  rtc::tsid::ClikReferenceGenerator clik_;
  bool clik_ready_{false};
  int body_dof_{0};
  int hand_dof_{0};
  int full_dof_{0};
  std::size_t num_tasks_{0};
  std::size_t num_posture_groups_{0};
  std::array<TaskRt, kDualArmMaxTasks> tasks_{};
  std::vector<std::string> target_frame_names_;
  std::array<int, kDualArmMaxTargetFrames> target_frame_idx_{};
  Eigen::VectorXd q_eval_;         ///< [nq] the state the solve is evaluated at
  Eigen::VectorXd v_eval_;         ///< [nv]
  Eigen::VectorXd q_posture_des_;  ///< [nq] posture target
  std::array<double, kDualArmMaxBodyDof> body_q_min_{};  ///< margined box, device order
  std::array<double, kDualArmMaxBodyDof> body_q_max_{};
  std::array<double, kDualArmMaxHandDof> hand_q_min_{};  ///< device limits
  std::array<double, kDualArmMaxHandDof> hand_q_max_{};
  std::vector<std::string> body_joint_names_;
  std::vector<std::string> hand_joint_names_;
  std::vector<std::string> hand_motor_names_;
  std::vector<std::string> task_names_;

  // ── Measured kinematics (TF and the log's measured poses) ────────────────
  std::unique_ptr<pinocchio::Data> meas_data_;
  bool meas_ready_{false};
  std::string root_link_name_;
  std::string tip_link_name_;
  pinocchio::FrameIndex root_fid_{0};
  pinocchio::FrameIndex tip_fid_{0};
  bool has_tip_{false};
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> hand_handle_;
  ClosedChainHandFk closed_hand_fk_;
  Eigen::VectorXd hand_q_;
  std::array<pinocchio::FrameIndex, kDualArmMaxFingertips> fingertip_frame_ids_{};
  std::vector<std::string> fingertip_names_;
  bool use_hand_root_frame_{false};
  pinocchio::FrameIndex hand_root_frame_id_{0};
  std::size_t num_fingertips_{0};

  // ── RT state ─────────────────────────────────────────────────────────────
  std::array<double, kDualArmMaxBodyDof> q_cmd_{};
  std::array<double, kDualArmMaxBodyDof> qd_cmd_{};
  bool seeded_{false};
  bool need_reseed_{true};
  rtc::trajectory::JointSpaceTrajectory<kDualArmMaxHandDof> hand_traj_;
  double hand_traj_time_{0.0};
  bool hand_traj_active_{false};
  bool hand_seeded_{false};
  std::array<double, kDualArmMaxHandDof> hand_cmd_{};
  std::array<double, kDualArmMaxHandDof> hand_goal_{};
  int qp_fail_streak_{0};
  DualArmDiagLogPod::FaultCause fault_cause_{DualArmDiagLogPod::FaultCause::kNone};
  std::uint32_t serviced_generation_{0};
  bool activation_seen_{false};
  std::uint32_t serviced_estop_epoch_{0};
  std::uint32_t serviced_fault_reset_epoch_{0};
  // Tick-local, read once at the top of Compute().
  bool estop_active_{false};
  bool body_readable_{false};
  bool hand_readable_{false};
  bool reseeded_this_tick_{false};
  bool clik_ran_this_tick_{false};
  Gains gains_tick_{};
  std::array<rtc::tsid::ClikReferenceGenerator::FrameTask, kDualArmMaxTasks> frame_tasks_{};
  DualArmDiagLogPod tick_{};

  // ── Cross-thread ─────────────────────────────────────────────────────────
  rtc::SeqLock<Gains> gains_lock_;
  std::array<TaskGoalIngress, kDualArmMaxTasks> ingress_{};
  std::atomic<bool> estop_requested_{false};
  std::atomic<std::uint32_t> estop_epoch_{0};
  std::atomic<std::uint32_t> fault_reset_epoch_{0};
  std::atomic<bool> fault_latched_{false};
  std::atomic<std::uint32_t> group_goal_rejects_{0};

  // ── Topics, logs, parameters ─────────────────────────────────────────────
  ControllerTopicHandles owned_topics_;
  rtc::ControllerLogSet log_set_{"demo_dualarm_controller"};
  rtc::LogHandle<DeviceStateLogPod> body_state_log_handle_;
  rtc::LogHandle<DeviceStateLogPod> hand_state_log_handle_;
  rtc::LogHandle<DualArmDiagLogPod> diag_log_handle_;
  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
  std::uint64_t log_drops_reported_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_DEMO_DUALARM_CONTROLLER_HPP_
