// ── DemoComplianceController: the binding that will carry task-space admittance
//
// AT THIS COMMIT IT IS A COPY OF DemoTaskController AND NOTHING ELSE. The
// admittance law (compliance-conventions.md §7) lands in #469 S3; S2 exists so
// that landing is a diff against a controller that already registers,
// configures, activates and drives the arm, instead of a diff that has to do
// all of that at once.
//
// WHY A COPY. Three cheaper-looking shapes were considered and none of them is
// available here:
//   - Register the same class under a second config_key. The CM puts Name() and
//     config_key in ONE lookup namespace and refuses the whole bring-up on a
//     collision, so a class cannot answer to two keys
//     (modification-guide.md §Adding a New Controller, item 3).
//   - Subclass DemoTaskController and override Compute(). Every member it would
//     need is private, so this means editing a shipped controller's header to
//     widen access — a PROC-6 regression surface on three deployed profiles for
//     the benefit of the one controller that does not ship yet.
//   - Extract the shared hand/arm lane into support/ first. That is the right
//     end state and it is a sprint of its own (#469 D-A6): joint, task and wbc
//     already hold three copies of that lane, so the extraction has to satisfy
//     all four callers, not one.
// The copy's cost is 4.6k duplicated lines. The cost is paid, deliberately, and
// what keeps it from silently rotting is test_compliance_task_equivalence.cpp:
// for as long as this controller has no law of its own, "identical output to
// DemoTaskController on identical input" is the whole specification, and that
// test is the only sensor that reads it. Adding a controller breaks no existing
// test in this package — every per-controller suite here enumerates the three
// shipped names by hand — so a divergence introduced by a bad rename would
// otherwise ship green.
//
// Everything below this banner is DemoTaskController with its identifiers
// renamed. Read the reasoning there as authoritative for the arm and hand
// lanes; read it here for how they will be driven.
// ─────────────────────────────────────────────────────────────────────────────
// ── Includes: project header first, then third-party, then C++ stdlib
// ──────────
#pragma once

#include "integrated_bringup/controllers/fingertip_force_guard.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "integrated_bringup/controllers/tof_snapshot.hpp"
#include "integrated_bringup/logging/compliance_diag_log_pod.hpp"
#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/grasp_diag_log_pod.hpp"
#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/logging/task_diag_log_pod.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"
#include "integrated_bringup/support/combined_model_cache.hpp"
#include "integrated_bringup/support/compliance_wrench_source.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "integrated_bringup/support/joint_tail_stats.hpp"
#include "integrated_bringup/support/momentum_observer_wiring.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "integrated_bringup/support/pull_estimator_wiring.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/compliance_state_machine.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/compliance/wrench_pipeline.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/params/task_admittance_params.hpp"
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
inline constexpr int kDemoComplianceMaxArmDof = 32;
inline constexpr int kDemoComplianceMaxHandDof = 32;
// Combined arm+hand actuated model capacity — sizes the ext↔Pinocchio reorder
// maps (unified kin&dyn Phase 4). Actual full_dof_ resolved at runtime.
inline constexpr int kDemoComplianceMaxFullDof =
    kDemoComplianceMaxArmDof + kDemoComplianceMaxHandDof;

/// Ticks an external task goal may be held while the control frame it was
/// authored in is unavailable, before the goal expires and the arm falls back to
/// holding the frame that IS available (#292 R1).
///
/// Deliberately a tick count, not a duration: the frame comes back when the
/// closed-chain hand FK walk-in converges, and that walk-in is
/// ⌈Δ/max_seed_increment⌉ *iterations* long — it scales with tick count, not
/// with wall time, so converting through control_rate would make the budget
/// rate-dependent for no reason. A measured p1b walk-in is ~93 ticks.
inline constexpr std::uint32_t kComplianceVtcpFrameWaitTicks = 1000;

/// Demo Compliance Controller: CLIK (arm) + P control (hand). Identical to
/// DemoTaskController until the §7 admittance law lands (#469 S3) — see the banner
/// at the top of this file for why it is a copy and what pins it.
///
/// Controls the end-effector in Cartesian space via damped Jacobian
/// pseudoinverse, while simultaneously running proportional position
/// control on the 10-DOF hand motors.
///
/// ### Arm control law — CLIK (task-space velocity), partially converged
///
/// The retired `ClikController` adapter ran this same law (#236 S7c deleted the class).
/// This binding is converged onto `rtc_controllers`' cores for BOTH of its
/// halves, and the split still matters when reading the code below:
///   - **DLS + null-space projector → converged** (#282). `compliance::DifferentialIk`
///     owns J^# and N, and λ is the §6.5 σ_min-adaptive ramp — NOT the constant it
///     used to be. The posture gain multiplies q̇_null BEFORE the projection,
///     because that is the argument the shared law takes.
///   - **Task-velocity assembly → converged** (#314). `kp ⊙ e + ν_ff` is
///     `task/task_vel_law.hpp` (`ComputeTaskVelocity` for the 6-DOF branch,
///     `ComputeTranslationVelocity` for the translation-only one). What stays on
///     THIS side is the frame transport: ν_ff is rotated into the Jacobian frame
///     here, because which frame that is (vtcp or world-aligned) is a binding
///     concern the core law does not know about.
/// @code
///   pos_error    = x_des − FK(q)
///   λ²           = 0                       if σ_min(J) ≥ σ₀       [§6.5 ramp]
///                = λ_max²(1 − (σ_min/σ₀)²) if σ_min(J) < σ₀
///   J^#          = J^T (J J^T + λ²I)^{−1}         [damped least squares]
///   N            = I − J^# J                        [null-space projector]
///   dq           = J^# · (kp ⊙ pos_error + ν_ff) + N · (nullspace_kp · (q_null − q))
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
///   ROS 2 parameters declared on `/demo_compliance_controller/<name>`: see
///   `DeclareGainParameters()` in compliance/parameters.cpp. Read-only caps
///   `max_traj_velocity` / `max_traj_angular_velocity` /
///   `hand_max_traj_velocity`. Force-PI grasp transitions: `~/grasp_command`
///   srv (rtc_msgs/GraspCommand).
class DemoComplianceController final : public RTControllerInterface {
 public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  //
  // THE TASK-SPACE GAINS ARE SPELLED THE WAY THE §7 SCHEMA SPELLS THEM (#469
  // D-A13), which is why they read `ik_kp_pos` / `ik_kp_rot` / `nullspace_kp`
  // here and `kp_translation` / `kp_rotation` / `null_kp` in the demo_task
  // sibling this class was copied from. `ParseTaskAdmittanceParams` reads this
  // controller's YAML for the admittance law, and it parses those three keys
  // into `TaskAdmittanceParams` — where NOTHING in rtc_controllers consumes
  // them, because the IK step they gain is the binding's. Keeping both
  // spellings in one file would therefore ship a key that is parsed and never
  // read: an operator who set `ik_kp_pos` would see the arm not change, which
  // is a worse failure than a rejected key. One name, two readers, one
  // consumer — and the core's copy can never disagree with this one because
  // they come from the same scalar.
  struct Gains {
    // Arm (CLIK) gains — translation / rotation separated
    std::array<double, 3> ik_kp_pos{
        {5.0, 5.0, 5.0}};  ///< Translation proportional gain (x,y,z) [1/s]
    std::array<double, 3> ik_kp_rot{
        {2.0, 2.0, 2.0}};  ///< Rotation proportional gain (rx,ry,rz) [1/s]
    // §6.5 σ_min-adaptive DLS. These two REPLACED a single constant λ
    // (`damping`, retired in #282) and are named/defaulted to match the five
    // task-space schemas in rtc_controllers/src/params/ — see LoadConfig for
    // why the old key is reported rather than mapped onto max_damping.
    //
    // ONE σ₀ parameterises BOTH branches, and σ_min does not mean the same thing
    // in them: `control_6dof` feeds the mixed-unit 6×n Jacobian (m/rad rows plus
    // dimensionless axis rows, so σ_min is dominated by the O(1) rotational
    // part), while the 3-DOF branch feeds the translation-only 3×n Jacobian,
    // whose σ_min scales with link length. The shipped 0.02 is tuned for the
    // 6-DOF lane all three robot configs run; a tree that flips `control_6dof`
    // at runtime is reinterpreting this gain, not carrying it over. The retired
    // constant λ was unit-agnostic and had no such coupling — it is the price of
    // a law that is parameterised on the Jacobian's conditioning.
    double singularity_threshold{0.02};  ///< σ₀: DLS engages below this
    double max_damping{0.05};            ///< λ_max: ceiling of the §6.5 ramp
    double nullspace_kp{0.5};            ///< Null-space joint-centering gain [1/s]
    bool enable_null_space{true};        ///< Enable null-space secondary task
    bool control_6dof{false};            ///< Enable 6-DOF (translation + orientation) control

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

    /// Which hand grasp-intervention law runs: contact_stop (binary freeze),
    /// force_pi (adaptive PI) or none (trajectory passthrough). Resolved from
    /// the whitelisted `grasp_controller_type` string once in LoadConfig so the
    /// RT hot path branches on an enum instead of comparing a std::string.
    ///
    /// It rides the Gains snapshot rather than a plain member because the mode
    /// and the thresholds it is interpreted with have to reach the RT tick in
    /// ONE SeqLock body — the tick loads Gains once and branches on this field,
    /// so a write can never be observed half-applied against a stale threshold.
    GraspHandMode grasp_hand_mode{GraspHandMode::kContactStop};

    // Trajectory / grasp FSM tuning
    double pi_rotation_margin{0.15};  ///< Split quintic trajectory when |angle|>π-margin [rad]
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

  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      Gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit DemoComplianceController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Controller-local fault latch (#260, §10.6) ───────────────────────────
  //
  // This is the ONE controller family that latches a fault of its own, so the
  // base no-ops are not an answer here: with them, `/rtc_cm/reset_fault` would
  // report "nothing latched" at a SAFE_STOP that has frozen the arm, and the
  // latch would have no exit at all. DELIBERATELY SEPARATE from the E-STOP pair
  // above (E-8) — `ClearEstop()` must not release this, and this must not
  // release the global latch.
  //
  // Off-RT (the CM service callback). Wait-free: the request is an atomic flag
  // the next tick consumes at the HEAD of `Compute()`, ahead of the E-STOP
  // branch, so a controller-local fault can be cleared while the global latch
  // is still up. `ResetTargetInitialization()` drops an unconsumed request, for
  // the same reason `BeginBiasCalibration()` cannot beat the latch: nobody
  // re-authorises a fault at an activation boundary.
  void ResetFault() noexcept override;
  /// Observable half of the above: the tick-end snapshot of the FSM's latch.
  /// Read off-RT, never blocks.
  [[nodiscard]] bool HasLatchedFault() const noexcept override;

  // ── Phase 4: controller-owned sub/pub lifecycle ─────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) noexcept override;
  void PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [ik_kp_pos×3, ik_kp_rot×3, singularity_threshold,
  //                max_damping, nullspace_kp,
  //                enable_null_space(0/1), control_6dof(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                hand_trajectory_speed, max_traj_velocity,
  //                max_traj_angular_velocity, hand_max_traj_velocity,
  //                grasp_contact_threshold, grasp_force_threshold,
  //                grasp_min_fingertips,
  //                grasp_command, grasp_target_force] = 22 values
  // (21 before #282 retired the single `damping` λ for the §6.5 pair.)
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

  /// Test-only: the configure-time `external_wrench.source` selection. Until S3
  /// wires it into the tick this accessor is the ONLY reader, which is exactly
  /// why the parse is asserted against the shipped profiles rather than trusted.
  [[nodiscard]] ComplianceWrenchSource GetWrenchSourceForTesting() const noexcept {
    return wrench_source_;
  }

  /// One tick's view of the §7 lane, at the seam between its two halves
  /// (#469 S3). Exposed as one POD rather than six accessors because the
  /// interesting assertions are relations BETWEEN these values — that
  /// `deviation` is exactly what an integrator fed `alpha * wrench_lwa` would
  /// hold, and that `wrench_lwa` is exactly the wrench that was published.
  /// Every field is staged by the tick for the #469 S4 diagnostic lane anyway.
  struct ComplianceProbe {
    Eigen::Matrix<double, 6, 1> wrench_lwa{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> deviation{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> velocity{Eigen::Matrix<double, 6, 1>::Zero()};
    double alpha{0.0};
    rtc::compliance::ComplianceState state{rtc::compliance::ComplianceState::kHolding};
    rtc::compliance::WrenchPipelineStatus status{};
    bool engaged{false};
    /// The control frame's origin, world — the pipeline's lever-arm base. A
    /// wrench published anywhere else is transported here, so this is the datum
    /// a source needs in order to hand over an application point at all.
    Eigen::Vector3d task_origin{Eigen::Vector3d::Zero()};
    /// Where the arm's control frame MEASURABLY is, world. The deviation says
    /// what the law asked for; this says whether the arm was actually asked to
    /// go there — the two are separated by the whole composition + CLIK path,
    /// which is otherwise observable only in joint space.
    Eigen::Vector3d tcp_position{Eigen::Vector3d::Zero()};
    /// The source verdict as of the LAST ComputeSecondary — the one that will
    /// gate the NEXT tick's wrench, NOT the one this tick's law consumed
    /// (D-A14). The `compliance_diag` row carries the latter, staged in
    /// ComputeControl; these two are one tick apart by construction, and a
    /// fixture that reads both is what pins that gap in place.
    bool quality_low{false};
    std::uint8_t invalid_reason{0};
  };

  [[nodiscard]] ComplianceProbe GetComplianceProbeForTesting() const noexcept {
    ComplianceProbe p;
    p.wrench_lwa = wrench_lwa_;
    p.deviation = admittance_.deviation();
    p.velocity = admittance_.velocity();
    p.alpha = compliance_alpha_;
    p.state = compliance_state_;
    p.status = wrench_status_;
    p.engaged = compliance_engaged_;
    p.task_origin = compliance_task_origin_;
    p.tcp_position = Eigen::Vector3d(tcp_position_[0], tcp_position_[1], tcp_position_[2]);
    p.quality_low = wrench_quality_low_;
    p.invalid_reason = wrench_invalid_reason_;
    return p;
  }

  [[nodiscard]] const rtc::params::TaskAdmittanceParams& GetAdmittanceParamsForTesting()
      const noexcept {
    return admittance_params_;
  }

  /// The §7.3 tail's activation totals (#484). By reference — the members are
  /// atomics, so this type does not copy.
  ///
  /// This is the seam the tests assert on rather than the WARN line itself: the
  /// log macro's throttle state is a process-global `static` at the expansion
  /// point (it is shared by every instance in the binary), so a test written
  /// against the output would be order-dependent on every other test that
  /// reaches the same line. The counters are what the line reports; pinning them
  /// pins it.
  [[nodiscard]] const JointTailStats& GetJointTailStatsForTesting() const noexcept {
    return joint_tail_stats_;
  }

  /// Inject a wrench straight into the pipeline, bypassing the source adapter.
  /// That seam is the point: `FromPullEstimate` has its own exhaustive suite
  /// (#469 S1) and re-deriving a known pull through fingertip FK would test it
  /// a second time while testing the LAW not at all. The publish site that
  /// joins the two is asserted separately, on the pipeline's own freshness.
  void PublishExternalWrenchForTesting(const rtc::compliance::Wrench6& wrench,
                                       const Eigen::Vector3d& p_apply) noexcept {
    wrench_apply_point_ = p_apply;
    wrench_pipeline_.Publish(
        std::span<const double, rtc::compliance::kWrenchDim>(wrench.data(), wrench.size()));
  }

  /// Test-only: the body the publish thread would Load right now (#234 P-1).
  /// Distinct from GetGraspStateForTesting(), which returns the RT staging
  /// buffer: a tick that fills the staging buffer but never stores it gets
  /// republished as the *previous* body under the current stamp, and only this
  /// accessor can tell those two apart.
  [[nodiscard]] ::rtc::grasp::GraspStateData GetPublishedGraspStateForTesting() const noexcept {
    return grasp_state_lock_.Load();
  }

  /// Test-only: per-finger states as the grasp controller itself holds them.
  /// The point of comparing this against
  /// GetPublishedGraspStateForTesting().finger_stiffness_est is that neither
  /// side alone pins the wiring — under the shipped tuning the estimate sits
  /// at its 1.0 seed, so a hardcoded constant satisfies any assertion made
  /// about the published value on its own (#424).
  [[nodiscard]] std::span<const ::rtc::grasp::FingerState> GetGraspFingerStatesForTesting()
      const noexcept {
    if (!grasp_controller_) {
      return {};
    }
    return grasp_controller_->finger_states();
  }

  /// Test-only: bind the grasp_diag CSV channel from a test-owned
  /// ControllerLogSet. Production binds it in on_configure; the unit fixtures
  /// build the controller through LoadConfig alone, so every log handle stays
  /// unbound and PushGraspDiagLog early-returns. A row-count assertion written
  /// against that state would pass with the push deleted outright — the exact
  /// shape of the #424 fill() mutation that survived its own test.
  void SetGraspDiagLogHandleForTesting(rtc::LogHandle<integrated_bringup::GraspDiagLogPod> h) {
    grasp_diag_log_handle_ = std::move(h);
  }

  /// Test-only: bind the momentum_observer CSV channel from a test-owned
  /// ControllerLogSet. The URDF fixtures stop at SetDeviceNameConfigs, so every
  /// production-bound log handle is unbound and UpdateMomentumObserverChannels
  /// skips the CSV lane; a row assertion written against that state would pass
  /// with the push deleted outright. (The topic lane is guarded separately and
  /// is unaffected by this handle.)
  void SetMomentumObserverLogHandleForTesting(
      rtc::LogHandle<integrated_bringup::MomentumObserverLogPod> h) {
    momentum_observer_log_handle_ = std::move(h);
  }

  /// Test-only: bind the compliance_diag CSV channel from a test-owned
  /// ControllerLogSet (#469 S4). Same reason as the two above, and here it is
  /// the only way to assert the channel at all: the unit fixtures build through
  /// LoadConfig, so the production handle is unbound and the tail push
  /// early-returns — a row-count assertion written against that state passes
  /// with the push deleted outright (#424's surviving fill() mutation).
  void SetComplianceDiagLogHandleForTesting(
      rtc::LogHandle<integrated_bringup::ComplianceDiagLogPod> h) {
    compliance_diag_log_handle_ = std::move(h);
  }

  /// Test-only: whether on_configure actually built the PayloadEstimate
  /// publisher. That call sits behind `if (momentum_wiring_.enabled())`, and the
  /// disabled side of that branch had no observable at all — the CSV lane is
  /// bound through a different path, so a gate that leaked (publisher created
  /// with the observer off) or over-gated (no publisher with it on) looked
  /// identical from every other fixture. Distinct from
  /// MomentumObserverConfigErrorForTesting(), which reports whether the WIRING
  /// resolved, not whether the lifecycle acted on it (#454, from #135).
  [[nodiscard]] bool HasPayloadEstimatePublisherForTesting() const noexcept {
    return static_cast<bool>(owned_topics_.payload_pub);
  }

  /// Test-only: the configure-time error BuildMomentumObserverWiring raised, or
  /// empty when the wiring resolved (enabled, or deliberately disabled).
  /// on_configure consumes the same string — see OnDeviceConfigsSet.
  [[nodiscard]] const std::string& MomentumObserverConfigErrorForTesting() const {
    return momentum_config_error_;
  }

  /// Test-only: lift the shipped K_est_max pin so the stiffness estimate can
  /// leave its seed. The deployed value equals the seed (#425), which makes
  /// the published mirror a constant and hides a fill that ignores its source.
  /// Read-modify-write so the rest of the configured block survives.
  bool SetGraspKEstMaxForTesting(double k_est_max) {
    if (!grasp_controller_) {
      return false;
    }
    auto p = grasp_controller_->params();
    p.K_est_max = k_est_max;
    grasp_controller_->set_params(p);
    return true;
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

  /// Test-only: the phase value the non-RT quiet gate would read right now.
  /// Distinct from GetGraspStateForTesting().grasp_phase, which the output fill
  /// writes: a tick that steps the FSM but never mirrors leaves the gate
  /// believing the hand is idle and admits a mode change mid-grasp.
  [[nodiscard]] uint8_t GetGraspPhaseMirrorForTesting() const noexcept {
    return grasp_phase_pub_.load(std::memory_order_acquire);
  }

  /// Test-only: drive the Force-PI FSM the way the grasp_command service would,
  /// without standing up a LifecycleNode. Returns false when no force_pi_grasp
  /// block built a controller. Same shape as SetGraspControllerTypeForTesting —
  /// a test-only door onto existing behaviour, no new production surface.
  bool CommandGraspForTesting(double target_force) {
    if (!grasp_controller_) {
      return false;
    }
    grasp_controller_->CommandGrasp(target_force);
    return true;
  }

  /// Test-only: the activity flag the grasp_command service gates on. The service
  /// itself needs a node and is verified at runtime; this pins the lifecycle
  /// writes that feed it.
  [[nodiscard]] bool IsActiveForTesting() const noexcept {
    return active_.load(std::memory_order_acquire);
  }

  /// Test-only: the latch value the non-RT quiet gate would read right now.
  /// Distinct from observing the freeze in the command: a tick that freezes the
  /// hand but never mirrors the latch leaves the gate believing the hand is
  /// quiet, and the freeze assertions cannot tell those two apart.
  [[nodiscard]] bool GetContactLatchedMirrorForTesting() const noexcept {
    return contact_latched_pub_.load(std::memory_order_acquire);
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

  /// Test-only: the per-fingertip force magnitude the Force-PI law is fed. Its
  /// own bank, so it is not the contact_stop aggregate and not the raw
  /// GraspState magnitude — the two independence assertions need a handle that
  /// is neither.
  [[nodiscard]] std::span<const float> GetForcePiFeedForTesting() const noexcept {
    return fingertip_force_mag_filt_grasp_;
  }

  [[nodiscard]] const ::integrated_bringup::ToFSnapshotData& GetToFSnapshotForTesting()
      const noexcept {
    return tof_snapshot_;
  }

  /// Test-only: the control frame the last Compute() tick ran in (#292).
  /// Exposes the member-mask derivation, which is otherwise invisible — it only
  /// reaches behaviour through ClassifyFrameTransition, and a mask that is wrong
  /// in a way the classifier ignores (kConstant, where it is pinned to 0xFF)
  /// would leave no trace anywhere else.
  [[nodiscard]] ControlFrameId GetControlFrameIdForTesting() const noexcept {
    return ControlFrameId{vtcp_valid_, vtcp_member_mask_, true};
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

  /// F5 device-readability gate for the arm (device 0), evaluated once at the
  /// top of Compute() next to estop_active_ so every phase of this tick sees one
  /// consistent answer. False means device 0 did not report the arm_dof_
  /// channels this controller reads, so CLIK, the null-space task and the
  /// emitted command would all run at a partially ZERO configuration. The answer
  /// is silence on device 0 (zero-length), NOT nc0 zeros — see
  /// rtc_controller_interface/device_readability.hpp and §3.7 of
  /// rtc_controllers/docs/compliance-conventions.md. RT-thread-only.
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
  // RT-thread-only cache of gains.control_6dof, set in ComputeControl so the
  // Fill* methods avoid a full Gains SeqLock copy just to read one bool.
  // Set after ComputeControl's E-STOP early-return, so it is valid only on
  // non-E-STOP ticks — every reader must sit behind an `estop_active_` guard.
  bool control_6dof_cached_{false};

  /// §6.5 singularity diagnostics staged by ComputeControl for the Phase C push
  /// at the tail of Compute() (#310). DifferentialIk::Result is a local there
  /// and both fields are recomputed every tick, so the staging buffer is what
  /// carries them across to the log push — the WBC controller stages its diag
  /// row the same way. RT-thread-only; written on every non-E-STOP tick
  /// INCLUDING the `!ok` hold, so a held tick still produces a row rather than
  /// a gap the reader has to interpret.
  integrated_bringup::TaskDiagLogPod task_diag_staging_{};

  /// §7 admittance diagnostics staged by ComputeControl for the Phase C push at
  /// the tail of Compute() (#469 S4). Staging is not a convenience here, it is
  /// the correctness argument: the tail runs after ComputeSecondary, where the
  /// NEXT tick's wrench verdict is computed, so a row assembled there would pair
  /// this tick's `invalid_reason` with the previous tick's wrench (D-A14). Every
  /// field is captured where the law used it. RT-thread-only; written on both
  /// sides of the F5 gate, so a held tick is a valid=0 row rather than a gap.
  integrated_bringup::ComplianceDiagLogPod compliance_diag_staging_{};

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
  /// Advance fingertip `f`'s per-axis force LPF from the raw force ReadState
  /// just parsed, writing fingertip_force_filt_ / fingertip_force_mag_filt_.
  /// The raw triplet passes through the delta-spike guard first, so
  /// fingertip_force_guarded_ / force_guard_rejected_ are refreshed here too.
  void UpdateFingertipForceFilter(std::size_t f,
                                  const FingertipForceGuardConfig& guard_cfg) noexcept;
  /// Config-time (throws): Init every per-fingertip force filter and clear the
  /// derived state. Called from the constructor and from LoadConfig.
  void InitFingertipForceFilters(double cutoff_hz, double grasp_cutoff_hz);
  /// Arm (device 0) stage: Jacobian extraction, hand FK for the arm-relative
  /// fingertip poses, CLIK and the null-space task. Held whole whenever the arm
  /// cannot be read this tick (F5) or the cache is not fresh.
  void ComputeControl(const ControllerState& state, double dt, const Gains& gains) noexcept;
  /// Secondary (device 1) stage: hand trajectory, grasp FSM, ToF snapshot.
  /// Deliberately NOT behind ComputeControl's gate — §3.7 "secondary
  /// passthrough 유지"; see the definition's header comment (#236 S7b).
  void ComputeSecondary(const ControllerState& state, double dt, const Gains& gains) noexcept;
  /// Leave the §7 law: drop the deviation, disown the wrench, re-arm the ramp
  /// and clear every latch whose value describes an engagement that has ended.
  /// Called on the FALLING EDGE of `compliance_engaged_` from every path that
  /// holds the arm instead of running the law — the F5 gate and the two
  /// mid-function holds (control frame unavailable, non-finite Jacobian). The
  /// edge guard is the point: `ResetForActivation()` re-arms the bias average,
  /// and re-arming it every tick of a 1000-tick frame wait means the average
  /// never completes once control comes back.
  void DisengageCompliance() noexcept;
  /// Stage the §7 diagnostic row a HELD tick leaves behind: a zeroed `valid=0`
  /// POD carrying only what is true without the law (source, param snapshot,
  /// FSM state). Runs at the head of every `Compute()` so a tick that returns
  /// early — through any gate, including one added later — cannot re-publish
  /// the previous tick's row under this tick's index (#429: a gap in `tick` has
  /// to mean a dropped row and nothing else). The task_diag lane defaults the
  /// same way, one member instead of a POD.
  void StageHeldComplianceDiagRow() noexcept;
  // WriteOutput was split into 3 explicit-intent methods (see
  // demo_joint_controller.hpp for the bucket contract). Compute() calls
  // them in order WriteJointCommand → FillLogOutput → FillPublishOutput.
  [[nodiscard]] ControllerOutput WriteJointCommand(const ControllerState& state,
                                                   double dt) noexcept;
  /// Arm half of WriteJointCommand's wire fill. Split out so the F5-silenced
  /// branch and the readable branch share ONE hand block instead of two copies
  /// of a device command lane that must never drift apart (#236 S7b).
  void WriteArmJointCommand(const ControllerState& state, rtc::DeviceOutput& out0,
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
  // The same arm sub-model, kept as a shared_ptr so the momentum observer can
  // build its OWN handle over it (RtModelHandle::GetModel() hands back a
  // reference, not ownership).
  std::shared_ptr<const pinocchio::Model> arm_model_;
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
  /// Fingertips that fed this tick's virtual TCP (bit f = fingertip f), pinned
  /// to 0xFF in kConstant mode where the control point does not depend on them.
  /// Written only by UpdateVirtualTcp: this function's early returns leave
  /// vtcp_inputs_ stale, so deriving the mask from that array anywhere else
  /// would read a previous tick's participating set. RT-thread-only.
  std::uint8_t vtcp_member_mask_{0};

  void UpdateVirtualTcp(const pinocchio::SE3& T_base_tcp, const Gains& gains) noexcept;

  void InitArmModel(const rtc_urdf_bridge::ModelConfig& config);
  void InitHandModel(const rtc_urdf_bridge::ModelConfig& config);

  // Arm-device channel loops bound themselves with rtc::ModelChannelBound(nc0,
  // desired_q_.size()) — the base predicate, called directly (#296). This class
  // carried a private wrapper with identical semantics until then; S7b lifted
  // the predicate to base but did not see that copy.
  //
  // The model dimension is desired_q_.size() and NOT arm_dof_, which is what the
  // sibling wbc binding passes: arm_dof_ comes from the YAML estop.arm_safe_
  // position length (or the first tick's channel count), so the two are not
  // interchangeable here and substituting one would change behaviour rather than
  // converge a spelling.
  //
  // Call sites cast the int result to std::size_t rather than base gaining a
  // size_t overload: the loops below index both Eigen buffers (Eigen::Index) and
  // device arrays (size_t), so no single return type removes every cast, and a
  // type-convenience overload would widen the base surface for nothing (ARCH-3).

  // ── Pre-allocated Eigen work buffers — zero heap alloc on the RT path ────
  Eigen::VectorXd q_;
  Eigen::MatrixXd J_full_;
  Eigen::MatrixXd J_pos_;
  // §6.5 damped least squares, one instance per task dimension (#282). Both are
  // Resize()d in InitArmModel and only Compute()/Solve()d on the tick. Two
  // objects rather than one re-Resize()d per branch: Resize() allocates, and
  // `control_6dof` is a runtime ros2 parameter, so a single instance would heap
  // -allocate on the RT tick the operator flips it (RT-1).
  rtc::compliance::DifferentialIk ik_6d_;
  rtc::compliance::DifferentialIk ik_3d_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd desired_q_;  ///< nv: integrated desired joint position
  Eigen::VectorXd traj_dq_;    // feedforward-only trajectory velocity (for logging)
  /// q̇_null = K_p·(q_null − q), the posture velocity handed to Solve() as the
  /// nullspace argument. One buffer where there used to be two (`null_err_` and
  /// the projected `null_dq_`): DifferentialIk applies N itself, so the
  /// projected result never needs to be named.
  Eigen::VectorXd qdot_null_;
  Eigen::Vector3d pos_error_;
  Eigen::Matrix<double, 6, 1> pos_error_6d_;

  // ── Controller state ──────────────────────────────────────────────────────
  // RT-thread-only working copy of the SE3 target (materialised from the
  // SeqLock POD each tick).
  pinocchio::SE3 tcp_target_pose_{pinocchio::SE3::Identity()};

  /// Null-space posture seed parsed from YAML. RT thread copies it into
  /// TargetSlot::null_target on first-tick self-init.
  std::array<double, kDemoComplianceMaxArmDof> null_target_init_{};

  // TargetSlot — Eigen SE3 mirrored as flat doubles. RT thread is the SOLE
  // writer of target_seqlock_; off-RT writers hand their goal to the base
  // mailbox via SetDeviceTarget → PushPendingTarget (#206).
  static constexpr std::size_t kSE3RotDoubles = 9;
  static constexpr std::size_t kSE3TransDoubles = 3;

  struct TargetSlot {
    std::array<double, 3> tcp_target{};
    std::array<double, kSE3RotDoubles> tcp_target_rot{};
    std::array<double, kSE3TransDoubles> tcp_target_t{};
    std::array<double, kDemoComplianceMaxArmDof> null_target{};
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
    // #292: a new activation has no external goal yet, and no frame wait in
    // progress. Both are atomic for the same reason the two flags above are —
    // this hook runs off-RT from on_activate while the RT tick reads them.
    target_is_hold_seed_.store(true, std::memory_order_release);
    frame_wait_ticks_.store(0, std::memory_order_release);
    // #260: an unconsumed fault reset dies at the activation boundary. Carrying
    // it across would let the first tick of a new activation release a latch
    // nobody re-authorised there — the same reason `BeginBiasCalibration()`
    // cannot beat the latch, and the reason this activation does not clear it
    // outright (compliance-conventions.md §fault 복구 진입점).
    reset_fault_requested_.store(false, std::memory_order_release);
  }

  // ── Control-frame contract (#292) ─────────────────────────────────────────
  // The invariant: run CLIK only when the frame the goal was authored in is the
  // frame being controlled this tick. See ClassifyFrameTransition in
  // support/virtual_tcp.hpp for the law and the reasoning.

  /// Frame the goal currently in current_target_slot_ was authored in.
  /// RT-thread-only, and deliberately NOT reset by ResetTargetInitialization:
  /// SeedHoldTarget rewrites it in full, and the arm self-init that calls it is
  /// gated on exactly the conditions ComputeControl needs to read it, so no
  /// reader can observe a previous activation's value. Left non-atomic because
  /// std::atomic over a 3-byte struct is not guaranteed lock-free (RT-4).
  ControlFrameId target_frame_id_{};
  /// Is the goal still the self-init hold seed (as opposed to an external one)?
  /// A hold seed may be moved to the current frame freely — that is what makes
  /// a frame transition free of arm motion; an external goal may not.
  std::atomic<bool> target_is_hold_seed_{true};
  /// Consecutive ticks an external goal has been held for a frame mismatch.
  std::atomic<std::uint32_t> frame_wait_ticks_{0};

  /// Seed the arm hold target at `pose` and tag it with frame `id`, so the goal
  /// and the control frame agree and CLIK sees zero task error. Shared by the
  /// first-tick self-init and ComputeControl's frame-transition branch.
  /// RT-thread-only. The caller owns the target_seqlock_ Store (the self-init
  /// batches it into its existing slot_dirty publish).
  void SeedHoldTarget(const pinocchio::SE3& pose, const rtc::DeviceState& dev0,
                      ControlFrameId id) noexcept;

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
  trajectory::JointSpaceTrajectory<kDemoComplianceMaxHandDof> hand_trajectory_;
  double hand_trajectory_time_{0.0};
  bool hand_new_target_pending_{false};  // RT-thread-only

  // ── Grasp controller (force_pi mode) ──────────────────────────────────────
  // The mode itself lives in Gains (see Gains::grasp_hand_mode) — this is the
  // RT-thread-only cache of the value THIS tick runs under, so FillLogOutput can
  // gate the Force-PI diagnostics on it without a second Gains SeqLock copy.
  // Sibling of control_6dof_cached_ above. Written at the top of
  // ComputeSecondary, i.e. INSIDE Compute()'s `!estop_active_` gate, which is
  // also where the mode edge is compared and consumed (the closure is the edge's
  // only consumer, so the two must not straddle that gate — hoisting the write
  // into Compute() is what let an E-STOP tick swallow a mode change). Both
  // consumers of the cache — the closure here and FillLogOutput's Force-PI
  // diagnostics — run only on the same non-E-STOP ticks: FillLogOutput
  // E-STOP-early-returns before reading it, so it can never see an earlier
  // tick's value. The reason to cache rather than re-Load is not the copy:
  // re-Loading would let a mode write that landed mid-tick make the log describe
  // a law that did not run.
  GraspHandMode grasp_hand_mode_cached_{GraspHandMode::kContactStop};
  std::unique_ptr<rtc::grasp::GraspController> grasp_controller_;

  // ── In-plane pull-force estimator (#167) ──────────────────────────────────
  // Configured in LoadConfig from the demo_shared `pull_estimator` block; tip
  // links resolve against the tree-model tip_links order (== fingertip_data_ /
  // fingertip_rotations_ slot order). Disabled (null estimator) without a hand
  // tree-model or when the block is absent. Output rides grasp_state_.pull.
  PullEstimatorWiring pull_wiring_;

  // #469 S2: which estimate will drive the admittance law. Resolved at
  // configure from `external_wrench.source` and NOT read by anything yet — S3 is
  // the first consumer. Parsed now rather than then so a profile whose source is
  // misspelled is refused by the bring-up that ships it, instead of by the
  // sprint that finally reads the field.
  ComplianceWrenchSource wrench_source_{ComplianceWrenchSource::kPullEstimator};

  // ── §7 admittance, parsed but not yet ticked (#469 S3) ────────────────────
  //
  // `ParseTaskAdmittanceParams` is the SSoT for this law's shape: M/D/K, the
  // §7.4 displacement box, the §7.5 velocity limits, the §10.6 staleness
  // window, and the §6.5 DLS pair this binding already carried under the same
  // names. It reads THIS controller's config node, so the schema is shared with
  // the CLIK keys above rather than nested — which is also why the gain names
  // were unified (see the note on Gains).
  //
  // Filled at LoadConfig and read by NOTHING in this commit. Parsed a sprint
  // early on purpose: every throw in that parser (non-positive desired_inertia,
  // a mis-shaped sequence, `external_wrench.enabled: false`, a non-position
  // command_type) becomes a configure FAILURE, and D1 turns that into a refused
  // bring-up — so a profile S3 would have rejected is rejected by the sprint
  // that shipped it, on a tick that does not yet depend on the values.
  rtc::params::TaskAdmittanceParams admittance_params_{};
  rtc::params::TaskAdmittanceConfig admittance_config_{};

  // Owns the wrench's frame transport, bias, deadband, saturation and §10.6
  // ageing. Configured (sample rate + conditioning) at LoadConfig; nothing
  // publishes into it or reads out of it until S3 wires tick rows 8-9.
  rtc::compliance::WrenchPipeline wrench_pipeline_{};

  // x̃_c = X_c ⊖ X_d — the DEVIATION, not the frame. `Reset()` collapses X_c
  // onto X_d, which is what E-STOP / SAFE_STOP / device-invalid owe it (S3
  // AC2); no caller does that yet because no caller steps it yet.
  rtc::compliance::AdmittanceIntegrator admittance_{};

  // §7 fault classification (SAFE_STOP latched vs DEGRADED recoverable).
  rtc::compliance::ComplianceStateMachine compliance_fsm_{};

  // ── Per-tick compliance state (#469 S3) ──────────────────────────────────
  /// Where the published wrench acts, world. Staged by the source in
  /// ComputeSecondary and consumed by `WrenchPipeline::Update` on the NEXT
  /// tick — see the note at the publish site for why the two are a tick apart.
  Eigen::Vector3d wrench_apply_point_{Eigen::Vector3d::Zero()};
  rtc::compliance::WrenchPipelineStatus wrench_status_{};
  /// §3.2.4 quality, as judged by the SOURCE (not the pipeline): the pull
  /// estimate can be fresh, finite and still untrustworthy — slipping, a
  /// saturated tip, or leakage as large as the signal.
  bool wrench_quality_low_{false};
  /// `rtc::grasp::PullInvalidReason` as a wire constant; diagnostics only.
  std::uint8_t wrench_invalid_reason_{0};

  /// True while the arm lane is actually running the law. The EDGE matters:
  /// falling ⇒ reset the deviation and the pipeline, rising ⇒ restart the
  /// §10.7 ramp. Held ticks (E-STOP, unreadable device, cold cache) must not
  /// accrue a deviation the arm would then be asked to realise on resume.
  bool compliance_engaged_{false};
  /// Seconds since the last rising edge, clamped at `activation_ramp_time`.
  /// Suspended while a bias average is owed and re-armed when one is re-entered:
  /// §10.6's lattice runs the ramp AFTER the bias commits, so a ramp that ran to
  /// completion during (or before) the average would hand the first conditioned
  /// wrench to the law at α = 1 — the step §10.7 exists to forbid.
  double compliance_ramp_elapsed_{0.0};
  /// The conditioned wrench the law actually integrated, LWA at the task frame,
  /// and the §10.7 ramp scale applied to it. Staged for the #469 S4 diagnostic
  /// lane — "the arm moved and the CSV cannot say what pushed it" is the
  /// failure this lane exists to prevent.
  Eigen::Matrix<double, 6, 1> wrench_lwa_{Eigen::Matrix<double, 6, 1>::Zero()};
  double compliance_alpha_{0.0};
  /// The control-frame origin the wrench was transported to this tick.
  Eigen::Vector3d compliance_task_origin_{Eigen::Vector3d::Zero()};
  /// Last `ComplianceStateMachine::Step` verdict. Read by the SAFE_STOP hold
  /// and staged for diagnostics (#469 S4).
  rtc::compliance::ComplianceState compliance_state_{rtc::compliance::ComplianceState::kHolding};

  /// [s] DEGRADED → RUNNING dwell. NOT part of the §7 schema — the parser has
  /// no key for it — so it is read from this controller's own YAML and lives
  /// outside `admittance_params_` rather than being quietly defaulted inside
  /// the state machine call.
  double degraded_recovery_time_{0.5};

  // ── §7.3 joint tail observation (#484) ───────────────────────────────────
  /// Per-ACTIVATION totals, not per-tick: the tail's report is consumed the tick
  /// it is produced (throttled WARN) and accumulated here for the deactivation
  /// summary. Reset in `on_activate` — see joint_tail_stats.hpp for why this is
  /// a counter and a log line rather than a `compliance_diag` column.
  JointTailStats joint_tail_stats_{};

  // ── Generalized-momentum observer (#135 Layer 1b) ─────────────────────────
  // Parsed in LoadConfig (the YAML is there) and BUILT in OnDeviceConfigsSet,
  // which is the first point the arm device's joint_state_names exist — the
  // wiring pins that order on its own model handle, so it cannot be built any
  // earlier. Disabled without a `momentum_observer` block. Output is the
  // joint-space residual r; it goes to momentum_observer.csv and nowhere else
  // this layer (#135 D12 — the PayloadEstimate wire surface arrives with
  // Layer 2A, when there is a payload estimate to put in it).
  MomentumObserverParams momentum_params_;
  MomentumObserverWiring momentum_wiring_;
  /// RT → publish-thread egress for the PayloadEstimate topic (#135 D12).
  /// Written on every enabled tick by UpdateMomentumObserverChannels, read by
  /// PublishNonRtSnapshot. The SAME row the CSV lane pushes, so the file and
  /// the topic cannot report different numbers for one tick.
  rtc::SeqLock<integrated_bringup::MomentumObserverLogPod> payload_estimate_lock_;
  /// Non-empty when BuildMomentumObserverWiring threw in OnDeviceConfigsSet.
  /// That hook cannot propagate — CM calls SetDeviceNameConfigs outside any
  /// try/catch — so the failure is latched here and on_configure turns it into
  /// CallbackReturn::FAILURE. A config error must not degrade to "the observer
  /// quietly did not run".
  std::string momentum_config_error_;
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
  /// cleared only by the release-phase gate, E-STOP, or the mode-switch race
  /// closure — so the hand keeps its position after contact drops, rather than
  /// snapping back to the trajectory.
  bool contact_latched_{false};
  /// Non-RT-readable mirror of contact_latched_, stored once per tick by the
  /// grasp block. The parameter callback's quiet gate has to know whether the
  /// hold is engaged before it lets the grasp mode move, and contact_latched_ is
  /// RT-thread-only: a callback that read it directly would be a data race, and
  /// one that assumed instead would open the switch on a held object. Reading a
  /// tick-old value is fine and unavoidable — the callback cannot stop the RT
  /// thread, which is exactly why ComputeSecondary carries the race closure.
  std::atomic<bool> contact_latched_pub_{false};
  /// Non-RT-readable mirror of grasp_controller_->phase(), stored on EVERY tick.
  /// Same reason as the latch mirror, one step stronger: the quiet gate must not
  /// admit a mode change mid-grasp, and the FSM is mutated by
  /// GraspController::Update() on the RT tick — so reading phase() from the
  /// callback would be a genuine data race on a safety decision. kIdle is 0, so
  /// the zero init is the right answer before the first force_pi tick.
  ///
  /// It used to be stored only inside the force_pi branch, justified by "the value
  /// cannot go stale under another mode: nothing steps the FSM there". The premise
  /// held; the conclusion did not. Nothing steps the FSM under another mode — and
  /// that is exactly the problem, because the switch INTO another mode can land
  /// after a tick has stepped the FSM off Idle: the mirror then froze non-Idle
  /// with nothing left to move it, the gate refused every subsequent switch, and
  /// grasp_command refused RELEASE because the mode was no longer force_pi. The
  /// fix is on both ends — the non-force_pi ticks now Reset() the FSM, and this
  /// mirror is stored unconditionally so it reports that.
  std::atomic<uint8_t> grasp_phase_pub_{0};
  /// Did the Force-PI law actually drive the hand on this tick? RT-thread-only
  /// (written in ComputeControl, read by the grasp_diag push at the tail of
  /// Compute) — no atomic, both ends are the same tick on the same thread.
  /// Cleared at the top of ComputeControl so a tick that early-returns before
  /// the grasp block reports false rather than the previous tick's answer.
  bool grasp_force_pi_ran_{false};
  /// Is this controller Active? Written by on_activate / on_deactivate, read by
  /// the grasp_command service — which outlives deactivation, so without this it
  /// answered "grasp started" for a controller whose ticks had stopped, arming a
  /// request that fired on the next activation.
  std::atomic<bool> active_{false};
  /// Hold target while latched. Refreshed from the LPF output on every tick
  /// that contact is present; frozen once contact drops (no random-walk drift).
  std::array<double, kDemoComplianceMaxHandDof> hand_hold_position_{};
  /// Bessel LPF over measured hand position; warmed every valid tick, its
  /// output is used as the desired hold position while latched. Init in
  /// LoadConfig (throws); Apply/Reset are noexcept + heap-free (RT-safe).
  rtc::BesselFilterN<kDemoComplianceMaxHandDof> hand_pos_filter_;

  // ── Fingertip force LPF (contact_stop + force_pi decision variables) ─────
  //
  // One filter PER FINGERTIP rather than a single BesselFilterN<3*N>: an
  // invalid fingertip must not advance its own filter, and Apply() is
  // all-channels-or-nothing. Runs on every tick regardless of
  // grasp_controller_type so the logged column means the same thing in every
  // run.
  //
  // Per-axis, not on |F|: filtering the magnitude lets zero-mean axis noise
  // rectify into a positive bias that walks the aggregate toward the contact
  // threshold with no contact present.
  //
  // TWO BANKS, ONE GUARD. The guard removes wire artefacts (delta spikes,
  // NaN/Inf) — that is a property of the signal source, so both consumers want
  // exactly the same answer and share one instance. The cutoff is a per-consumer
  // tuning axis and is NOT shared: contact_stop is a latch that trades noise for
  // low latency (its freeze delay is hand travel), while force_pi closes a servo
  // loop around a derivative-based slip detector (`df/dt < -df_slip_threshold`
  // is 0.01 N per tick at 500 Hz) and trades latency for noise. Merging the two
  // would force one of them onto the other's operating point.
  std::array<rtc::BesselFilterN<3>, rtc::kMaxSensorGroups> force_lpf_{};
  /// Second bank at the Force-PI cutoff, fed the same guarded triplet, primed
  /// and invalidated on exactly the same ticks as force_lpf_ (one shared
  /// force_lpf_primed_ flag — they only ever differ in cutoff).
  std::array<rtc::BesselFilterN<3>, rtc::kMaxSensorGroups> force_lpf_grasp_{};
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
  /// ‖filtered force‖ per fingertip [N], contact_stop bank.
  std::array<float, rtc::kMaxSensorGroups> fingertip_force_mag_filt_{};
  /// ‖filtered force‖ per fingertip [N], Force-PI bank. This is what the PI law
  /// servos on — the norm is taken AFTER the per-axis filter, so unlike a filter
  /// applied to |F| it carries no rectification bias.
  std::array<float, rtc::kMaxSensorGroups> fingertip_force_mag_filt_grasp_{};
  /// contact_stop aggregates derived from the FILTERED magnitudes. Kept apart
  /// from grasp_state_.max_force / num_active_contacts, which stay raw so the
  /// published GraspState and its BT consumers see an unchanged contract.
  float contact_stop_max_force_{0.0F};
  int contact_stop_active_count_{0};

  /// Previous grasp phase (for state-transition logging; non-RT critical).
  uint8_t prev_grasp_phase_{0};

  // ── Logging (throttled — RT-safe by throttle interval) ───────────────────
  // Sub-logger handle cached at construction; bringup_logging.hpp owns the
  // canonical name ("integrated_bringup.demo_compliance_controller"). All hot-path log
  // calls must use *_THROTTLE variants with constants from
  // integrated_bringup::logging.
  rclcpp::Logger logger_{integrated_bringup::logging::DemoComplianceLogger()};
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  // ── Controller-local fault latch (#260) — NOT the E-STOP pair above (E-8) ──
  /// Set off-RT by `ResetFault()`, consumed (and cleared) at the head of the
  /// next `Compute()`. A request that finds no latch is a no-op there, not an
  /// error: `ComplianceStateMachine::ResetFault()` holds that line itself.
  std::atomic<bool> reset_fault_requested_{false};
  /// Tick-END snapshot of `ComplianceStateMachine::in_safe_stop()`, which is
  /// what `HasLatchedFault()` answers with. Stored at the end of the tick and
  /// not where the FSM steps, because the reset service reads it to decide
  /// whether the latch survived the tick that consumed the request.
  std::atomic<bool> latched_fault_{false};

  /// Arm joint position the E-STOP path drives to. Authoritative source is
  /// LoadConfig(cfg["estop"]["arm_safe_position"]); only the first
  /// arm_dof_ slots are read by ComputeEstop. Zero-initialized by default.
  std::array<double, kDemoComplianceMaxArmDof> safe_position_{};

  // Runtime DoF (resolved by LoadConfig/OnDeviceConfigsSet from YAML +
  // device configs). arm_dof_ from the required YAML `arm_dof` key (one
  // source, #340); hand_dof_ from the secondary group's joint_state_names size
  // when declared, else the device's own first reported num_channels.
  int arm_dof_{0};
  int hand_dof_{0};
  // Which of hand_dof_'s two sources won (#307) — see demo_joint_controller.hpp
  // for why the diagnostic cannot name one key unconditionally.
  bool hand_dof_from_config_{false};

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
  // Tunable gains (ik_kp_pos, max_damping, trajectory_speed, ...) are
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

  rtc::ControllerLogSet log_set_{"demo_compliance_controller"};

  // Lifetime CSV drop count already reported by the drain timer (#234 P-20) —
  // DrainControllerLogs warns once per new burst rather than once per drain.
  std::uint64_t log_drops_reported_{0};
  rtc::LogHandle<integrated_bringup::DeviceStateLogPod> primary_state_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceStateLogPod> secondary_state_log_handle_;
  rtc::LogHandle<integrated_bringup::DeviceSensorLogPod> secondary_sensor_log_handle_;
  rtc::LogHandle<integrated_bringup::PullEstimatorLogPod> pull_estimator_log_handle_;
  rtc::LogHandle<integrated_bringup::MomentumObserverLogPod> momentum_observer_log_handle_;
  rtc::LogHandle<integrated_bringup::TaskDiagLogPod> task_diag_log_handle_;
  rtc::LogHandle<integrated_bringup::ComplianceDiagLogPod> compliance_diag_log_handle_;
  /// Per-tick Force-PI servo + stiffness-estimator diagnostics (#428).
  /// Bound only when a `force_pi_grasp` block is configured; see
  /// LogRegistrationContext::grasp_diag_enabled for why the gate is the
  /// block and not the current grasp_hand_mode.
  rtc::LogHandle<integrated_bringup::GraspDiagLogPod> grasp_diag_log_handle_;

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
};

}  // namespace integrated_bringup
