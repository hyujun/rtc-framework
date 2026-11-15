// LocomotionDiagLog — one row per locomotion_controller tick: the mode and its
// causes, the state source's verdict, the whole-body QP's outcome and margins,
// the measured base, the contact forces, and the servo / feedforward split per
// body joint. The joint lanes the generic joint log already carries (measured q,
// the position target, gravity, feedforward) are in JointFeedforwardLog,
// written by the same tick with the same `tick` column for the join.
#ifndef INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_DIAG_LOG_POD_HPP_

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

struct LocomotionDiagLogPod {
  static constexpr std::size_t kMaxJoints = 32;
  static constexpr std::size_t kMaxContacts = 4;

  /// What the controller commanded this tick.
  enum class Mode : std::uint8_t {
    kWaiting = 0,   ///< no valid state yet: the reference is not latched, nothing commanded
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
  };

  double t_relative_s{0.0};
  std::uint64_t tick{0};        ///< ControllerState::iteration
  std::uint64_t tick_count{0};  ///< ticks since this activation (1 = the first)
  std::uint8_t mode{0};
  std::uint8_t fault_cause{0};
  bool fault_latched{false};
  std::uint32_t fault_count{0};  ///< latches since configure

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
  std::array<double, 3> ref_base_p{};   ///< the latched reference's base position [m]
  std::array<double, 3> com{};          ///< full-model CoM at the measured state, world [m]
  std::array<double, kMaxContacts * 3> foot_p{};  ///< contact frames, full-model FK, world [m]
  std::array<double, 6> a_opt_base{};             ///< the QP's base acceleration [m/s², rad/s²]

  double servo_norm{0.0};  ///< ‖kp⊙(q_cmd − q) − kd⊙v‖ over the body joints [N·m]
  double ff_norm{0.0};     ///< ‖τ_ff‖ over the body joints [N·m]

  std::uint8_t num_contacts{0};
  std::array<double, kMaxContacts * 6> lambda{};  ///< per contact [f; τ] (dim 3: f, then 0)

  std::uint8_t num_joints{0};
  std::array<double, kMaxJoints> v{};          ///< measured joint velocity [rad/s]
  std::array<double, kMaxJoints> tau_servo{};  ///< kp(q_cmd − q) − kd v [N·m]
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
  os << "t_relative_s,tick,tick_count,mode,fault_cause,fault_latched,fault_count,"
        "source_status,source_fresh_mask,source_valid_mask,source_bad_ticks,source_sequence,"
        "source_age_us,"
        "health,health_bad_run,tau_valid,qp_converged,qp_status,qp_iterations,rows_dropped,"
        "solve_time_us,wbc_tick_us,qp_pri_res,qp_dua_res,eom_residual_inf,"
        "margin_friction_n,margin_cop_nm,margin_yaw_nm,margin_torque_nm,margin_joint_accel,"
        "margins_stale,base_px,base_py,base_pz,base_qx,base_qy,base_qz,base_qw,"
        "base_vx,base_vy,base_vz,base_wx,base_wy,base_wz,ref_base_px,ref_base_py,ref_base_pz,"
        "com_x,com_y,com_z";
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
}

inline void WriteLocomotionDiagLogRow(std::ostream& os, const LocomotionDiagLogPod& p,
                                      const LocomotionDiagLogColumns& cols) {
  os << p.t_relative_s << ',' << p.tick << ',' << p.tick_count << ','
     << static_cast<unsigned>(p.mode) << ',' << static_cast<unsigned>(p.fault_cause) << ','
     << (p.fault_latched ? 1 : 0) << ',' << p.fault_count << ','
     << static_cast<unsigned>(p.source_status) << ',' << p.source_fresh_mask << ','
     << p.source_valid_mask << ',' << p.source_bad_ticks << ',' << p.source_sequence << ','
     << p.source_age_us << ',' << static_cast<unsigned>(p.health) << ',' << p.health_bad_run << ','
     << (p.tau_valid ? 1 : 0) << ',' << (p.qp_converged ? 1 : 0) << ',' << p.qp_status << ','
     << p.qp_iterations << ',' << p.rows_dropped << ',' << p.solve_time_us << ',' << p.wbc_tick_us
     << ',' << p.qp_pri_res << ',' << p.qp_dua_res << ',' << p.eom_residual_inf << ','
     << p.margin_friction_n << ',' << p.margin_cop_nm << ',' << p.margin_yaw_nm << ','
     << p.margin_torque_nm << ',' << p.margin_joint_accel << ',' << (p.margins_stale ? 1 : 0);
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
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_DIAG_LOG_POD_HPP_
