// LocomotionDiagLog — one row per locomotion_controller tick: the mode and its
// causes, the state source's verdict, the whole-body QP's outcome and margins,
// the measured base, the contact forces, the servo / feedforward split per body
// joint, and what walking is judged against (the command, the gait, the plan's
// base velocity, the measured contact normal forces), the plan sampled this
// tick before the plausibility check (a refused one too), and the state
// estimator's own record (estimator_telemetry.hpp; zero under the truth source). The joint lanes
// the generic joint log already carries (measured q, the position target, gravity, feedforward) are
// in JointFeedforwardLog, written by the same tick with the same `tick` column for the join.
#ifndef INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_DIAG_LOG_POD_HPP_

#include "integrated_bringup/controllers/locomotion/estimator_telemetry.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

inline constexpr std::string_view kLocomotionDiagLogMsgType =
    "integrated_bringup/LocomotionDiagLog";
inline constexpr std::string_view kLocomotionDiagLogInstance = "locomotion_diag";

/// The plan the tick sampled, as it was before the plausibility check — a
/// refused plan is here although `plan_id` (the followed one) is not.
struct LocomotionPlanSampleDiag {
  std::uint64_t plan_id{0};  ///< 0 = nothing sampled
  bool converged{false};
  std::int32_t iterations{0};
  double prim_inf{0.0};        ///< the solve's primal residual
  double prim_head{0.0};       ///< its largest node residual over the leading nodes
  std::int32_t prim_node{-1};  ///< the node of its largest residual; −1 = none
  double dual_inf{0.0};
  std::uint32_t mask{0};         ///< the sample's contact mask
  double progress{0.0};          ///< where in the horizon it sampled, [0, 1]
  double a_joint_max{0.0};       ///< max |a_ref| over the body joints [rad/s²]
  std::int32_t a_joint_idx{-1};  ///< its body joint (body order); −1 = none
  std::uint8_t check{
      0};  ///< the check it failed; 0 = passed or not checked (not the earlier plan's)
};

struct LocomotionDiagLogPod {
  static constexpr std::size_t kMaxJoints = 32;
  static constexpr std::size_t kMaxContacts = 4;

  /// What the controller commanded this tick.
  enum class Mode : std::uint8_t {
    kWaiting =
        0,  ///< no latched reference (no valid state yet, the E-STOP, the pose guard): damping
    kNominal = 1,   ///< the QP's torque
    kDegraded = 2,  ///< the QP missed: the last valid torque, held
    kFault = 3,     ///< latched: damping only (feedforward −Kd·v, target = measured q)
  };
  /// Why the fault latched (the first cause; kNone while not latched).
  enum class FaultCause : std::uint8_t {
    kNone = 0,
    kSolveHealth = 1,  ///< the QP's health reached Fault
    kSource = 2,       ///< the state source was not ok for too many ticks in a row
    kNonFinite = 3,    ///< the output was not finite
    kModelEpoch = 4,   ///< the state is in another model layout than the controller's
    kPlanLoss = 5,     ///< the MPC plan was lost with the robot off a still double support
    kPoseGuard = 6,    ///< the pose guard refused the measured pose for the whole settle time
  };
  /// Where this tick's reference came from.
  enum class ReferenceSource : std::uint8_t {
    kIdle = 0,   ///< the latched measured pose (no MPC, or no usable plan)
    kStand = 1,  ///< the MPC plan, every contact in support
    kWalk = 2,   ///< the MPC plan, a contact in swing
    kFault = 3,  ///< the fault latch: no reference is followed
  };

  double t_relative_s{0.0};
  std::uint64_t tick{0};        ///< ControllerState::iteration
  std::uint64_t tick_count{0};  ///< ticks since this activation (1 = the first)
  std::uint8_t mode{0};
  bool estop{false};  ///< E-STOP active: the body got damping whatever `mode` says
  std::uint8_t fault_cause{0};
  bool fault_latched{false};
  std::uint32_t fault_count{0};  ///< latches since configure
  /// The pose guard's last verdict of this reference episode (locomotion/pose_guard.hpp
  /// bits; 0 = passed or not asked yet). Kept while the guard's fault is latched.
  std::uint32_t pose_guard_mask{0};

  std::uint8_t source_status{0};  ///< StateSourceStatus
  std::uint32_t source_fresh_mask{0};
  std::uint32_t source_valid_mask{0};
  std::uint32_t source_bad_ticks{0};  ///< consecutive ticks with the source not ok
  std::uint64_t source_sequence{0};   ///< the body joint message the state was read from
  double source_age_us{0.0};          ///< now − the state's stamp [µs]

  std::uint8_t health{0};  ///< rtc::tsid::SolveHealthState
  std::uint32_t health_bad_run{0};
  bool tau_valid{false};
  bool qp_converged{false};
  std::int32_t qp_status{-1};
  std::int32_t qp_iterations{0};
  std::int32_t rows_dropped{0};
  double solve_time_us{0.0};
  double wbc_tick_us{0.0};  ///< this Compute() from entry to return
  double qp_pri_res{0.0};
  double qp_dua_res{0.0};
  double eom_residual_inf{0.0};  ///< [N, N·m]; NaN when the tick did not solve
  double margin_friction_n{0.0};
  double margin_cop_nm{0.0};
  double margin_yaw_nm{0.0};
  double margin_torque_nm{0.0};
  double margin_joint_accel{0.0};
  bool margins_stale{true};

  std::array<double, 3> base_p{};       ///< measured base position, world [m]
  std::array<double, 4> base_q_xyzw{};  ///< measured base orientation
  std::array<double, 6> base_v{};       ///< measured base twist, base frame [v; ω]
  std::array<double, 3> ref_base_p{};   ///< the reference's base position [m]
  std::array<double, 3> com{};          ///< full-model CoM at the measured state, world [m]

  // The MPC plan (all 0 / false when the controller runs without one).
  std::uint8_t ref_source{0};       ///< ReferenceSource
  std::uint64_t plan_id{0};         ///< the sampled plan; 0 = none this tick
  double policy_age_us{-1.0};       ///< this tick − the plan's observation [µs]; −1 = none
  bool mpc_converged{false};        ///< the sampled plan's solve converged
  bool mpc_initial_solve{false};    ///< it was a standing (initial) solve
  double mpc_solve_us{0.0};         ///< its solve time [µs]
  std::uint64_t schedule_epoch{0};  ///< its gait commit count
  std::uint32_t contact_mask{0};    ///< the contacts the QP held in support this tick
  std::uint64_t plausibility_rejects{0};  ///< plans refused by the plausibility check, total
  double u_fb_norm{0.0};                  ///< ‖the Riccati feedback added to a_des‖
  std::uint32_t mpc_state_epoch{0};       ///< the state stream's epoch
  std::array<double, kMaxContacts * 3> foot_p{};  ///< contact frames, full-model FK, world [m]
  std::array<double, 6> a_opt_base{};             ///< the QP's base acceleration [m/s², rad/s²]

  double servo_norm{0.0};  ///< ‖kp⊙(q_cmd − q) − kd⊙v‖ over the body joints [N·m]
  double ff_norm{0.0};     ///< ‖τ_ff‖ over the body joints [N·m]

  std::uint8_t num_contacts{0};
  std::array<double, kMaxContacts * 6> lambda{};  ///< per contact [f; τ] (dim 3: f, then 0)

  std::uint8_t num_joints{0};
  std::array<double, kMaxJoints> v{};          ///< measured joint velocity [rad/s]
  std::array<double, kMaxJoints> tau_servo{};  ///< kp(q_cmd − q) − kd v [N·m]

  // What walking is judged against. The command is the filter's output as the
  // command timer last handed it over — what the planner was given, not what
  // the sampled plan was solved for (that one is up to a filter period plus
  // the MPC's latency older).
  double cmd_vx{0.0};  ///< [m/s], heading frame
  double cmd_vy{0.0};  ///< [m/s]
  double cmd_wz{0.0};  ///< [rad/s]
  double cmd_h{0.0};   ///< base height [m]
  /// The filter had a fresh input at that step (false: no command path, or the
  /// input was stale and the output is decaying to rest).
  bool cmd_valid{false};
  /// The phase manager's gait (an index into its gait table) when the timer
  /// read it; −1 = no MPC. It is the latest gait the scheduler advanced to, so
  /// it leads the sampled plan's contact_mask.
  std::int32_t gait_latest{-1};
  std::array<double, 3> ref_base_v{};  ///< the plan's base linear velocity, base frame; NaN = none
  /// Bit c: contact c's lane group is fresh by its backend's own flag (inference_enable).
  std::uint32_t meas_contact_valid{0};
  /// The measured contact force's z per contact, in the sensor's frame (a
  /// supporting sole reads negative); NaN when its bit above is clear.
  std::array<double, kMaxContacts> meas_fz{};
  /// Refused plans in stand / walk that gave way to the plan followed
  /// before them, total.
  std::uint64_t plan_holds{0};
  LocomotionPlanSampleDiag sample{};
  locomotion::EstimatorTelemetry estimator{};  ///< the source's record of this tick
};

static_assert(std::is_trivially_copyable_v<LocomotionDiagLogPod>,
              "LocomotionDiagLogPod must be trivially copyable for SPSC ring");

/// Column geometry, derived once from the configured names and captured by both
/// writers (the header is written before any pod exists).
struct LocomotionDiagLogColumns {
  std::size_t contacts{0};
  std::size_t joints{0};
};

[[nodiscard]] inline LocomotionDiagLogColumns LocomotionDiagLogColumnsFor(
    const std::vector<std::string>& contact_names, const std::vector<std::string>& joint_names) {
  return {std::min(contact_names.size(), LocomotionDiagLogPod::kMaxContacts),
          std::min(joint_names.size(), LocomotionDiagLogPod::kMaxJoints)};
}

/// Emit the CSV header. The logger appends '\n'.
inline void WriteLocomotionDiagLogHeader(std::ostream& os,
                                         const std::vector<std::string>& contact_names,
                                         const std::vector<std::string>& joint_names,
                                         const LocomotionDiagLogColumns& cols) {
  os << "t_relative_s,tick,tick_count,mode,estop,fault_cause,fault_latched,fault_count,"
        "pose_guard_mask,source_status,source_fresh_mask,source_valid_mask,source_bad_ticks,source_"
        "sequence,"
        "source_age_us,"
        "health,health_bad_run,tau_valid,qp_converged,qp_status,qp_iterations,rows_dropped,"
        "solve_time_us,wbc_tick_us,qp_pri_res,qp_dua_res,eom_residual_inf,"
        "margin_friction_n,margin_cop_nm,margin_yaw_nm,margin_torque_nm,margin_joint_accel,"
        "margins_stale,base_px,base_py,base_pz,base_qx,base_qy,base_qz,base_qw,"
        "base_vx,base_vy,base_vz,base_wx,base_wy,base_wz,ref_base_px,ref_base_py,ref_base_pz,"
        "com_x,com_y,com_z,ref_source,plan_id,policy_age_us,mpc_converged,mpc_initial_solve,"
        "mpc_solve_us,schedule_epoch,contact_mask,plausibility_rejects,u_fb_norm,mpc_state_epoch";
  for (std::size_t c = 0; c < cols.contacts; ++c) {
    for (const char* axis : {"x", "y", "z"}) {
      os << ",foot_" << contact_names[c] << '_' << axis;
    }
  }
  os << ",a_base_vx,a_base_vy,a_base_vz,a_base_wx,a_base_wy,a_base_wz,servo_norm,ff_norm";
  for (std::size_t c = 0; c < cols.contacts; ++c) {
    for (const char* axis : {"fx", "fy", "fz", "tx", "ty", "tz"}) {
      os << ",lambda_" << contact_names[c] << '_' << axis;
    }
  }
  for (const char* prefix : {"v_", "tau_servo_"}) {
    for (std::size_t i = 0; i < cols.joints; ++i) {
      os << ',' << prefix << joint_names[i];
    }
  }
  os << ",cmd_vx,cmd_vy,cmd_wz,cmd_h,cmd_valid,gait_latest,ref_base_vx,ref_base_vy,ref_base_vz,"
        "meas_contact_valid";
  for (std::size_t c = 0; c < cols.contacts; ++c) {
    os << ",meas_fz_" << contact_names[c];
  }
  os << ",plan_holds,sample_plan_id,sample_converged,sample_iterations,sample_prim_inf,"
        "sample_prim_head,sample_prim_node,sample_dual_inf,sample_mask,sample_progress,"
        "sample_a_joint_max,sample_a_joint_idx,sample_check";
  os << ",est_initialized,est_reset_reason,est_reset_count,est_imu_sequence,est_periods,"
        "est_skew_us,est_hold_ticks,est_step_status,est_core_status,est_report_valid,"
        "est_planned_mask,est_mask,est_no_anchor_ticks,est_gated_ticks,est_refused_ticks,"
        "est_unobservable,imu_gx,imu_gy,imu_gz,imu_ax,imu_ay,imu_az";
  for (const char* field : {"mode", "reason", "update", "evidence", "speed", "m2"}) {
    for (std::size_t c = 0; c < cols.contacts; ++c) {
      os << ",est_" << field << '_' << contact_names[c];
    }
  }
  for (std::size_t i = 0; i < 15 + 3 * cols.contacts; ++i) {
    os << ",est_p_" << i;
  }
}

inline void WriteLocomotionDiagLogRow(std::ostream& os, const LocomotionDiagLogPod& p,
                                      const LocomotionDiagLogColumns& cols) {
  os << p.t_relative_s << ',' << p.tick << ',' << p.tick_count << ','
     << static_cast<unsigned>(p.mode) << ',' << (p.estop ? 1 : 0) << ','
     << static_cast<unsigned>(p.fault_cause) << ',' << (p.fault_latched ? 1 : 0) << ','
     << p.fault_count << ',' << p.pose_guard_mask << ',' << static_cast<unsigned>(p.source_status)
     << ',' << p.source_fresh_mask << ',' << p.source_valid_mask << ',' << p.source_bad_ticks << ','
     << p.source_sequence << ',' << p.source_age_us << ',' << static_cast<unsigned>(p.health) << ','
     << p.health_bad_run << ',' << (p.tau_valid ? 1 : 0) << ',' << (p.qp_converged ? 1 : 0) << ','
     << p.qp_status << ',' << p.qp_iterations << ',' << p.rows_dropped << ',' << p.solve_time_us
     << ',' << p.wbc_tick_us << ',' << p.qp_pri_res << ',' << p.qp_dua_res << ','
     << p.eom_residual_inf << ',' << p.margin_friction_n << ',' << p.margin_cop_nm << ','
     << p.margin_yaw_nm << ',' << p.margin_torque_nm << ',' << p.margin_joint_accel << ','
     << (p.margins_stale ? 1 : 0);
  for (double x : p.base_p) {
    os << ',' << x;
  }
  for (double x : p.base_q_xyzw) {
    os << ',' << x;
  }
  for (double x : p.base_v) {
    os << ',' << x;
  }
  for (double x : p.ref_base_p) {
    os << ',' << x;
  }
  for (double x : p.com) {
    os << ',' << x;
  }
  os << ',' << static_cast<unsigned>(p.ref_source) << ',' << p.plan_id << ',' << p.policy_age_us
     << ',' << (p.mpc_converged ? 1 : 0) << ',' << (p.mpc_initial_solve ? 1 : 0) << ','
     << p.mpc_solve_us << ',' << p.schedule_epoch << ',' << p.contact_mask << ','
     << p.plausibility_rejects << ',' << p.u_fb_norm << ',' << p.mpc_state_epoch;
  for (std::size_t i = 0; i < cols.contacts * 3; ++i) {
    os << ',' << p.foot_p[i];
  }
  for (double x : p.a_opt_base) {
    os << ',' << x;
  }
  os << ',' << p.servo_norm << ',' << p.ff_norm;
  for (std::size_t i = 0; i < cols.contacts * 6; ++i) {
    os << ',' << p.lambda[i];
  }
  for (std::size_t i = 0; i < cols.joints; ++i) {
    os << ',' << p.v[i];
  }
  for (std::size_t i = 0; i < cols.joints; ++i) {
    os << ',' << p.tau_servo[i];
  }
  os << ',' << p.cmd_vx << ',' << p.cmd_vy << ',' << p.cmd_wz << ',' << p.cmd_h << ','
     << (p.cmd_valid ? 1 : 0) << ',' << p.gait_latest;
  for (double x : p.ref_base_v) {
    os << ',' << x;
  }
  os << ',' << p.meas_contact_valid;
  for (std::size_t c = 0; c < cols.contacts; ++c) {
    os << ',' << p.meas_fz[c];
  }
  const auto& s = p.sample;
  os << ',' << p.plan_holds << ',' << s.plan_id << ',' << (s.converged ? 1 : 0) << ','
     << s.iterations << ',' << s.prim_inf << ',' << s.prim_head << ',' << s.prim_node << ','
     << s.dual_inf << ',' << s.mask << ',' << s.progress << ',' << s.a_joint_max << ','
     << s.a_joint_idx << ',' << static_cast<unsigned>(s.check);
  const auto& e = p.estimator;
  os << ',' << (e.initialized ? 1 : 0) << ',' << static_cast<unsigned>(e.reset_reason) << ','
     << e.reset_count << ',' << e.imu_sequence << ',' << e.periods << ','
     << static_cast<double>(e.skew_ns) * 1e-3 << ',' << e.hold_ticks << ','
     << static_cast<unsigned>(e.step_status) << ',' << static_cast<unsigned>(e.core_status) << ','
     << (e.report_valid ? 1 : 0) << ',' << e.planned_mask << ',' << e.estimated_mask << ','
     << e.no_anchor_ticks << ',' << e.gated_ticks << ',' << e.refused_ticks << ','
     << static_cast<unsigned>(e.unobservable);
  for (double x : e.imu) {
    os << ',' << x;
  }
  const std::size_t nc = std::min(cols.contacts, locomotion::kMaxEstimatorSlots);
  for (std::size_t c = 0; c < nc; ++c) {
    os << ',' << static_cast<unsigned>(e.mode[c]);
  }
  for (std::size_t c = 0; c < nc; ++c) {
    os << ',' << static_cast<unsigned>(e.reason[c]);
  }
  for (std::size_t c = 0; c < nc; ++c) {
    os << ',' << static_cast<unsigned>(e.update[c]);
  }
  for (const auto* field : {&e.evidence, &e.speed, &e.m2}) {
    for (std::size_t c = 0; c < nc; ++c) {
      os << ',' << (*field)[c];
    }
  }
  for (std::size_t i = 0; i < 15 + 3 * nc; ++i) {
    os << ',' << e.p_diag[i];
  }
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_DIAG_LOG_POD_HPP_
