// locomotion_diag.csv as a contract for the tools that read it: a fixed run on
// the installed g1_loco profile (nominal, degraded, a source fault and its
// damping ticks, the body contact lane live) read back from the file. Held:
//   - the column set and order — the columns below keep their names and
//     places (downstream tools read them by name); columns appended after them
//     are not listed and are ignored;
//   - one row per tick run, the tick counters running 1..N;
//   - no infinities, and NaN only in the columns that mean "no value this
//     tick" (a residual before any solve, a reference velocity with no plan);
//   - ranges that hold on any tuning: flags are 0 / 1, the base is above the
//     ground with a unit quaternion, the latched fault shows at the end.
// The values themselves are tuning and are not compared.
#include "csv_log_fixture.hpp"
#include "integrated_bringup/logging/locomotion_diag_log_pod.hpp"
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using integrated_bringup::LocomotionControllerTestAccess;
namespace fs = std::filesystem;

// The columns of the first release of the file, in order. The writer appends
// new columns after these; a rename, a move or a removal here breaks the tools.
constexpr const char* kContractColumns =
    "t_relative_s,tick,tick_count,mode,estop,fault_cause,fault_latched,fault_count,"
    "pose_guard_mask,source_status,source_fresh_mask,source_valid_mask,source_bad_ticks,"
    "source_sequence,source_age_us,health,health_bad_run,tau_valid,qp_converged,qp_status,"
    "qp_iterations,rows_dropped,solve_time_us,wbc_tick_us,qp_pri_res,qp_dua_res,"
    "eom_residual_inf,margin_friction_n,margin_cop_nm,margin_yaw_nm,margin_torque_nm,"
    "margin_joint_accel,margins_stale,base_px,base_py,base_pz,base_qx,base_qy,base_qz,base_qw,"
    "base_vx,base_vy,base_vz,base_wx,base_wy,base_wz,ref_base_px,ref_base_py,ref_base_pz,com_x,"
    "com_y,com_z,ref_source,plan_id,policy_age_us,mpc_converged,mpc_solve_us,schedule_epoch,"
    "contact_mask,plausibility_rejects,u_fb_norm,mpc_state_epoch,foot_left_foot_x,"
    "foot_left_foot_y,foot_left_foot_z,foot_right_foot_x,foot_right_foot_y,foot_right_foot_z,"
    "a_base_vx,a_base_vy,a_base_vz,a_base_wx,a_base_wy,a_base_wz,servo_norm,ff_norm,"
    "lambda_left_foot_fx,lambda_left_foot_fy,lambda_left_foot_fz,lambda_left_foot_tx,"
    "lambda_left_foot_ty,lambda_left_foot_tz,lambda_right_foot_fx,lambda_right_foot_fy,"
    "lambda_right_foot_fz,lambda_right_foot_tx,lambda_right_foot_ty,lambda_right_foot_tz,"
    "v_left_hip_pitch_joint,v_left_hip_roll_joint,v_left_hip_yaw_joint,v_left_knee_joint,"
    "v_left_ankle_pitch_joint,v_left_ankle_roll_joint,v_right_hip_pitch_joint,"
    "v_right_hip_roll_joint,v_right_hip_yaw_joint,v_right_knee_joint,v_right_ankle_pitch_joint,"
    "v_right_ankle_roll_joint,v_waist_yaw_joint,v_waist_roll_joint,v_waist_pitch_joint,"
    "v_left_shoulder_pitch_joint,v_left_shoulder_roll_joint,v_left_shoulder_yaw_joint,"
    "v_left_elbow_joint,v_right_shoulder_pitch_joint,v_right_shoulder_roll_joint,"
    "v_right_shoulder_yaw_joint,v_right_elbow_joint,tau_servo_left_hip_pitch_joint,"
    "tau_servo_left_hip_roll_joint,tau_servo_left_hip_yaw_joint,tau_servo_left_knee_joint,"
    "tau_servo_left_ankle_pitch_joint,tau_servo_left_ankle_roll_joint,"
    "tau_servo_right_hip_pitch_joint,tau_servo_right_hip_roll_joint,"
    "tau_servo_right_hip_yaw_joint,tau_servo_right_knee_joint,tau_servo_right_ankle_pitch_joint,"
    "tau_servo_right_ankle_roll_joint,tau_servo_waist_yaw_joint,tau_servo_waist_roll_joint,"
    "tau_servo_waist_pitch_joint,tau_servo_left_shoulder_pitch_joint,"
    "tau_servo_left_shoulder_roll_joint,tau_servo_left_shoulder_yaw_joint,"
    "tau_servo_left_elbow_joint,tau_servo_right_shoulder_pitch_joint,"
    "tau_servo_right_shoulder_roll_joint,tau_servo_right_shoulder_yaw_joint,"
    "tau_servo_right_elbow_joint,cmd_vx,cmd_vy,cmd_wz,cmd_h,cmd_valid,gait_latest,ref_base_vx,"
    "ref_base_vy,ref_base_vz,meas_contact_valid,plan_holds,sample_plan_id,sample_converged,"
    "sample_iterations,sample_prim_inf,sample_prim_head,sample_prim_node,sample_dual_inf,"
    "sample_mask,sample_progress,sample_a_joint_max,sample_a_joint_idx,sample_check,"
    "est_initialized,est_reset_reason,est_reset_count,est_imu_sequence,est_periods,est_skew_us,"
    "est_hold_ticks,est_step_status,est_core_status,est_report_valid,est_planned_mask,est_mask,"
    "est_no_anchor_ticks,est_gated_ticks,est_refused_ticks,est_unobservable,imu_gx,imu_gy,"
    "imu_gz,imu_ax,imu_ay,imu_az,est_mode_left_foot,est_mode_right_foot,est_reason_left_foot,"
    "est_reason_right_foot,est_update_left_foot,est_update_right_foot,est_evidence_left_foot,"
    "est_evidence_right_foot,est_speed_left_foot,est_speed_right_foot,est_m2_left_foot,"
    "est_m2_right_foot,est_p_0,est_p_1,est_p_2,est_p_3,est_p_4,est_p_5,est_p_6,est_p_7,est_p_8,"
    "est_p_9,est_p_10,est_p_11,est_p_12,est_p_13,est_p_14,est_p_15,est_p_16,est_p_17,est_p_18,"
    "est_p_19,est_p_20";

// Columns that carry NaN when the tick has no value for them.
const std::set<std::string> kNanMeansNoValue = {"qp_pri_res",  "qp_dua_res",  "eom_residual_inf",
                                                "ref_base_vx", "ref_base_vy", "ref_base_vz"};

class LocomotionDiagContract : public LocomotionControllerTest {
 protected:
  void StampContacts(int64_t now) {
    auto& body = state_.devices[kBody];
    body.num_inference_groups = 2;
    for (std::size_t c = 0; c < 2; ++c) {
      body.inference_enable[c] = true;
      body.inference_recv_steady_ns[c] = now;
      body.inference_data[c * 7 + 3] = -170.0F - 3.0F * static_cast<float>(c);
    }
  }

  rtc::testing::ScopedSessionDir session_;
};

TEST_F(LocomotionDiagContract, ExistingColumnsKeepTheirNamesAndPlacesAndHoldSaneValues) {
  const YAML::Node cfg = ShippedConfig();
  ConfigureWithDevices(cfg, ShippedLaneDevices(cfg));
  Activate();
  MakeStanding();
  // The joints move off the latched pose and have a velocity (a fixed
  // function of the tick), so the servo / feedforward columns are not 0.
  const std::vector<double> stand = StandPose();
  int tick = 0;
  const auto step = [this, &stand, &tick](bool stale) {
    auto& body = state_.devices[kBody];
    for (std::size_t c = 0; c < stand.size(); ++c) {
      const double phase = 0.1 * static_cast<double>(tick) + 0.3 * static_cast<double>(c);
      body.positions[c] = stand[c] + 0.002 * std::sin(phase);
      body.velocities[c] = 0.02 * std::cos(phase);
    }
    ++tick;
    StampContacts(NowNs());
    last_out_ = stale ? StaleTick() : Tick();
  };
  for (int k = 0; k < 30; ++k) {
    step(false);  // nominal
  }
  for (int k = 0; k < 2; ++k) {
    step(true);  // degraded: a short source drop
  }
  for (int k = 0; k < 10; ++k) {
    step(false);
  }
  for (int k = 0; k < 8; ++k) {
    step(true);  // past max_source_bad_ticks: the source fault, then damping
  }
  for (int k = 0; k < 10; ++k) {
    step(false);  // the fault stays latched
  }
  ASSERT_EQ(LastMode(), Mode::kFault);
  LocomotionControllerTestAccess::DrainLogs(*ctrl_);
  ASSERT_EQ(LocomotionControllerTestAccess::LogDrops(*ctrl_), 0U);
  const fs::path diag = LocomotionControllerTestAccess::LogPath(
      *ctrl_, integrated_bringup::kLocomotionDiagLogInstance);
  const auto got = integrated_bringup::testfx::ReadCsv(diag);
  constexpr std::size_t kTicksRun = 60U;
  ASSERT_EQ(got.rows.size(), kTicksRun);

  // The column set and order: every contract column is where it was.
  std::vector<std::string> want;
  {
    std::string rest = kContractColumns;
    for (std::size_t pos = 0;;) {
      const std::size_t comma = rest.find(',', pos);
      want.push_back(rest.substr(pos, comma == std::string::npos ? comma : comma - pos));
      if (comma == std::string::npos) {
        break;
      }
      pos = comma + 1;
    }
  }
  ASSERT_GE(got.header.size(), want.size());
  for (std::size_t c = 0; c < want.size(); ++c) {
    EXPECT_EQ(got.header[c], want[c]) << "column " << c << " is not where the tools expect it";
  }

  // Every row is as wide as the header; in the contract columns no infinities,
  // and NaN only where it means "no value".
  for (std::size_t r = 0; r < got.rows.size(); ++r) {
    ASSERT_EQ(got.rows[r].size(), got.header.size()) << "row " << r;
    for (std::size_t c = 0; c < want.size(); ++c) {
      const double v = std::strtod(got.rows[r][c].c_str(), nullptr);
      EXPECT_FALSE(std::isinf(v)) << got.header[c] << " row " << r;
      if (std::isnan(v)) {
        EXPECT_EQ(kNanMeansNoValue.count(got.header[c]), 1U) << got.header[c] << " row " << r;
      }
    }
  }

  // Ranges that hold on any tuning.
  for (std::size_t r = 0; r < got.rows.size(); ++r) {
    EXPECT_EQ(got.At(r, "tick"), static_cast<double>(r + 1));
    EXPECT_EQ(got.At(r, "tick_count"), static_cast<double>(r + 1));
    for (const char* flag : {"estop", "fault_latched", "qp_converged", "tau_valid"}) {
      const double v = got.At(r, flag);
      EXPECT_TRUE(v == 0.0 || v == 1.0) << flag << " row " << r;
    }
    EXPECT_GT(got.At(r, "base_pz"), 0.0) << "row " << r;
    const double qn = std::hypot(std::hypot(got.At(r, "base_qx"), got.At(r, "base_qy")),
                                 std::hypot(got.At(r, "base_qz"), got.At(r, "base_qw")));
    EXPECT_NEAR(qn, 1.0, 1e-6) << "row " << r;
    for (const char* us : {"solve_time_us", "wbc_tick_us", "source_age_us"}) {
      EXPECT_GE(got.At(r, us), 0.0) << us << " row " << r;
    }
  }
  EXPECT_EQ(got.At(got.rows.size() - 1, "fault_latched"), 1.0) << "the source fault shows";
}

}  // namespace
