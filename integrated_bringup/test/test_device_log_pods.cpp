// Unit tests for integrated_bringup POD mirrors of rtc_msgs/DeviceStateLog
// and rtc_msgs/DeviceSensorLog. Verifies trivial-copy guarantee, header
// emission against runtime joint/sensor names, row column count agreement
// with the header, and num_*-bounded array iteration.

#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/device_wbc_log_pod.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/logging/wbc_diag_log_pod.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <array>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace {

int CountCommas(const std::string& s) {
  int n = 0;
  for (char c : s)
    if (c == ',')
      ++n;
  return n;
}

}  // namespace

TEST(DeviceStateLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::DeviceStateLogPod>);
}

TEST(DeviceStateLogPod, HeaderColumnsMatchRowColumnsForArmHand) {
  // ur5e arm 6 + hand 10 motors.
  const std::vector<std::string> joint_names{"shoulder_pan", "shoulder_lift", "elbow",   "wrist_1",
                                             "wrist_2",      "wrist_3",       "thumb1",  "thumb2",
                                             "index1",       "index2",        "middle1", "middle2",
                                             "ring1",        "ring2",         "pinky1",  "pinky2"};
  const std::vector<std::string> motor_names{"m0", "m1", "m2",  "m3",  "m4",  "m5",  "m6",  "m7",
                                             "m8", "m9", "m10", "m11", "m12", "m13", "m14", "m15"};

  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceStateLogHeader(hdr_os, joint_names, motor_names);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceStateLogPod pod{};
  pod.t_relative_s = 1.5;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  pod.num_motors = static_cast<std::uint8_t>(motor_names.size());
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceStateLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row)) << "header: " << hdr << "\nrow: " << row;
}

TEST(DeviceStateLogPod, RowRespectsRuntimeNumJoints) {
  // Header sized for 16 joints, but pod populates only 6 — row must
  // emit exactly 6 values per joint-array column.
  const std::vector<std::string> joint_names_16(16, "j");
  const std::vector<std::string> motor_names_0;
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceStateLogHeader(hdr_os, joint_names_16, motor_names_0);
  const int hdr_commas = CountCommas(hdr_os.str());

  integrated_bringup::DeviceStateLogPod pod{};
  pod.num_joints = 6;
  pod.num_motors = 0;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceStateLogRow(row_os, pod);
  const int row_commas = CountCommas(row_os.str());

  // Row will have FEWER columns than header by design — runtime num_joints
  // bounded — but task-space and trailing categorical columns are still
  // emitted. We assert row_commas < hdr_commas to confirm the joint loop
  // is bounded by num_joints rather than kMaxJoints.
  EXPECT_LT(row_commas, hdr_commas);
}

TEST(DeviceSensorLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::DeviceSensorLogPod>);
}

TEST(DeviceSensorLogPod, HeaderColumnsMatchRowColumnsFor4Fingertips) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.t_relative_s = 0.25;
  pod.num_fingertips = static_cast<std::uint8_t>(sensor_names.size());
  pod.inference_valid = true;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row)) << "header: " << hdr << "\nrow: " << row;
}

// A force-only hand declares values_per_group=0. The barometer/ToF block must
// then be absent entirely rather than emitted as a permanently-zero column
// group that reads downstream as a flatlining sensor.
TEST(DeviceSensorLogPod, ForceOnlyLayoutEmitsNoBaroTofColumns) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, /*values_per_group=*/0);
  const std::string hdr = hdr_os.str();

  EXPECT_EQ(hdr.find("_raw_"), std::string::npos) << hdr;
  EXPECT_EQ(hdr.find("_filt_0"), std::string::npos) << hdr;
  // The inference lane survives; so do the LPF'd force columns.
  EXPECT_NE(hdr.find("ft_thumb_fx"), std::string::npos) << hdr;
  EXPECT_NE(hdr.find("ft_thumb_fx_filt"), std::string::npos) << hdr;
  EXPECT_NE(hdr.find("force_filtered_valid"), std::string::npos) << hdr;

  // 1 timestamp + inference_valid + force_filtered_valid + 4*(7 + 3) = 43 cols.
  EXPECT_EQ(CountCommas(hdr), 42) << hdr;
}

TEST(DeviceSensorLogPod, ForceOnlyLayoutHeaderMatchesRow) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, /*values_per_group=*/0);

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.num_fingertips = static_cast<std::uint8_t>(sensor_names.size());
  pod.force_filtered_valid = true;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, /*values_per_group=*/0);

  EXPECT_EQ(CountCommas(hdr_os.str()), CountCommas(row_os.str()))
      << "header: " << hdr_os.str() << "\nrow: " << row_os.str();
}

// The valid flag is what separates "this controller has no force LPF" (wbc,
// empty span) from "the filter output was genuinely 0" — a zero column alone
// cannot say which.
TEST(DeviceSensorLogPod, ForceFilteredValidDistinguishesAbsentFilterFromZeroOutput) {
  rtc::ControllerState state{};
  state.num_devices = 2;
  state.devices[1].num_channels = 1;
  state.devices[1].inference_enable[0] = true;

  const std::array<float, 3> filtered{{1.5F, -2.0F, 0.5F}};
  integrated_bringup::DeviceSensorLogPod with_filter{};
  integrated_bringup::FillDeviceSensorLogPod(state, 1, /*num_active_fingertips=*/1, filtered,
                                             with_filter);
  EXPECT_TRUE(with_filter.force_filtered_valid);
  EXPECT_FLOAT_EQ(with_filter.force_filtered[0], 1.5F);
  EXPECT_FLOAT_EQ(with_filter.force_filtered[1], -2.0F);
  EXPECT_FLOAT_EQ(with_filter.force_filtered[2], 0.5F);

  integrated_bringup::DeviceSensorLogPod without_filter{};
  integrated_bringup::FillDeviceSensorLogPod(state, 1, /*num_active_fingertips=*/1, {},
                                             without_filter);
  EXPECT_FALSE(without_filter.force_filtered_valid);
  EXPECT_FLOAT_EQ(without_filter.force_filtered[0], 0.0F);
}

TEST(DeviceSensorLogPod, RowRespectsRuntimeNumFingertips) {
  // Pod populated with 2 fingertips; header (sized for 4) is wider.
  const std::vector<std::string> sensor_names_4{"a", "b", "c", "d"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names_4);
  const int hdr_commas = CountCommas(hdr_os.str());

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.num_fingertips = 2;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod);
  const int row_commas = CountCommas(row_os.str());

  EXPECT_LT(row_commas, hdr_commas);
}

TEST(DeviceStateLogPod, FirstColumnIsTRelativeS) {
  integrated_bringup::DeviceStateLogPod pod{};
  pod.t_relative_s = 0.123;
  std::ostringstream os;
  integrated_bringup::WriteDeviceStateLogRow(os, pod);
  EXPECT_EQ(os.str().find("0.123"), 0u);
}

TEST(DeviceSensorLogPod, FirstColumnIsTRelativeS) {
  integrated_bringup::DeviceSensorLogPod pod{};
  pod.t_relative_s = 7.5;
  std::ostringstream os;
  integrated_bringup::WriteDeviceSensorLogRow(os, pod);
  EXPECT_EQ(os.str().find("7.5"), 0u);
}

// ── DeviceWbcLogPod ────────────────────────────────────────────────────────

TEST(DeviceWbcLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::DeviceWbcLogPod>);
}

TEST(DeviceWbcLogPod, ArmRoleHeaderColumnsMatchRow) {
  // role 0 (arm): joint cols + SE3 task block, no motor/fingertip cols.
  const std::vector<std::string> joint_names{"shoulder_pan", "shoulder_lift", "elbow",
                                             "wrist_1",      "wrist_2",       "wrist_3"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/0, joint_names, {}, {});
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 0;
  pod.t_relative_s = 1.5;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row)) << "header: " << hdr << "\nrow: " << row;
  // SE3 task block present, motor/fingertip absent.
  EXPECT_NE(hdr.find("traj_task_vel_yaw"), std::string::npos);
  EXPECT_NE(hdr.find("accel_elbow"), std::string::npos);
  EXPECT_EQ(hdr.find("motor_pos_"), std::string::npos);
  EXPECT_EQ(hdr.find("fingertip_force_"), std::string::npos);
}

TEST(DeviceWbcLogPod, HandRoleHeaderColumnsMatchRow) {
  // role 1 (hand): joint cols + motor + fingertip-force, no SE3 task block.
  const std::vector<std::string> joint_names{"j0", "j1", "j2", "j3"};
  const std::vector<std::string> motor_names{"m0", "m1", "m2", "m3"};
  const std::vector<std::string> fingertip_names{"thumb", "index", "middle", "ring"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/1, joint_names, motor_names,
                                              fingertip_names);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 1;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  pod.num_motors = static_cast<std::uint8_t>(motor_names.size());
  pod.num_fingertips = static_cast<std::uint8_t>(fingertip_names.size());
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row)) << "header: " << hdr << "\nrow: " << row;
  EXPECT_NE(hdr.find("fingertip_force_thumb"), std::string::npos);
  // Per-axis force-vector blocks accompany the |F| magnitude block.
  EXPECT_NE(hdr.find("fingertip_fx_thumb"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fy_thumb"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fz_thumb"), std::string::npos);
  EXPECT_NE(hdr.find("motor_eff_m3"), std::string::npos);
  EXPECT_EQ(hdr.find("task_goal_x"), std::string::npos);
}

TEST(DeviceWbcLogPod, HandFingertipBlockFixedWidthWhenNamesEmpty) {
  // Regression: LEAP has no tactile sensors → empty fingertip_names at header
  // write, but the runtime fingertip count is nonzero. The fixed-width block
  // must keep header and row column counts equal regardless.
  const std::vector<std::string> joint_names{"j0", "j1"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/1, joint_names, {}, {});
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 1;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  pod.num_motors = 0;
  pod.num_fingertips = 2;  // runtime count < kMaxFingertips, no names registered
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod);

  EXPECT_EQ(CountCommas(hdr), CountCommas(row_os.str())) << "header: " << hdr;
  // Fixed-width blocks → numeric-labelled columns present for |F| and each axis.
  EXPECT_NE(hdr.find("fingertip_force_0"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_force_7"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fx_0"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fy_7"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fz_0"), std::string::npos);
}

TEST(DeviceWbcLogPod, RowRespectsRuntimeNumJoints) {
  const std::vector<std::string> joint_names_16(16, "j");
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/0, joint_names_16, {}, {});
  const int hdr_commas = CountCommas(hdr_os.str());

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 0;
  pod.num_joints = 6;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod);
  EXPECT_LT(CountCommas(row_os.str()), hdr_commas);
}

TEST(DeviceWbcLogPod, AccelColumnRoundTrips) {
  const std::vector<std::string> joint_names{"a", "b"};
  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 0;
  pod.num_joints = 2;
  pod.accelerations[0] = 3.25;
  pod.accelerations[1] = -1.5;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod);
  const std::string row = row_os.str();
  EXPECT_NE(row.find("3.25"), std::string::npos);
  EXPECT_NE(row.find("-1.5"), std::string::npos);
}

// ── WbcDiagLogPod ──────────────────────────────────────────────────────────

TEST(WbcDiagLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::WbcDiagLogPod>);
}

TEST(WbcDiagLogPod, HeaderColumnsMatchRow) {
  constexpr std::size_t kNVars = 12;  // 4 point contacts × 3
  std::ostringstream hdr_os;
  integrated_bringup::WriteWbcDiagLogHeader(hdr_os, kNVars);
  const std::string hdr = hdr_os.str();

  integrated_bringup::WbcDiagLogPod pod{};
  pod.t_relative_s = 2.0;
  pod.phase = 3;
  pod.num_contact_vars = static_cast<std::uint8_t>(kNVars);
  pod.qp_converged = true;
  std::ostringstream row_os;
  integrated_bringup::WriteWbcDiagLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row)) << "header: " << hdr << "\nrow: " << row;
  EXPECT_NE(hdr.find("lambda_11"), std::string::npos);
  // Fallback split (decision 6): Dynamic (qp_fail_count) and Kinematic
  // (kin_qp_fail_count) QP failure counters are distinct CSV columns.
  EXPECT_NE(hdr.find("qp_fail_count"), std::string::npos);
  EXPECT_NE(hdr.find("kin_qp_fail_count"), std::string::npos);
}

TEST(WbcDiagLogPod, PhaseStringTranslation) {
  integrated_bringup::WbcDiagLogPod pod{};
  pod.phase = 3;  // kClosure
  pod.num_contact_vars = 0;
  std::ostringstream os;
  integrated_bringup::WriteWbcDiagLogRow(os, pod);
  EXPECT_NE(os.str().find("closure"), std::string::npos);
}

TEST(WbcDiagLogPod, RowRespectsRuntimeNumContactVars) {
  std::ostringstream hdr_os;
  integrated_bringup::WriteWbcDiagLogHeader(hdr_os, 12);
  const int hdr_commas = CountCommas(hdr_os.str());

  integrated_bringup::WbcDiagLogPod pod{};
  pod.num_contact_vars = 6;
  std::ostringstream row_os;
  integrated_bringup::WriteWbcDiagLogRow(row_os, pod);
  EXPECT_LT(CountCommas(row_os.str()), hdr_commas);
}

TEST(WbcDiagLogPod, FirstColumnIsTRelativeS) {
  integrated_bringup::WbcDiagLogPod pod{};
  pod.t_relative_s = 0.456;
  pod.num_contact_vars = 0;
  std::ostringstream os;
  integrated_bringup::WriteWbcDiagLogRow(os, pod);
  EXPECT_EQ(os.str().find("0.456"), 0u);
}

// ── PullEstimatorLogPod (#167) ─────────────────────────────────────────────

TEST(PullEstimatorLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::PullEstimatorLogPod>);
}

TEST(PullEstimatorLogPod, HeaderColumnsMatchRow) {
  std::ostringstream hdr_os;
  integrated_bringup::WritePullEstimatorLogHeader(hdr_os);
  const std::string hdr = hdr_os.str();

  integrated_bringup::PullEstimatorLogPod pod{};
  pod.estimate.valid = true;
  pod.estimate.valid_contact_count = 3;
  std::ostringstream row_os;
  integrated_bringup::WritePullEstimatorLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row)) << "header: " << hdr << "\nrow: " << row;
  // The CSV is the only surface carrying the pre-filter force (#167 decision:
  // wire msg / SeqLock POD are filtered-only). Renamed from force_raw_* in
  // #234 P-13 — it is post-projection/gravity/baseline, so the old name read
  // as a raw sensor sum.
  EXPECT_NE(hdr.find("force_prefilter_x"), std::string::npos);
  EXPECT_EQ(hdr.find("force_raw_x"), std::string::npos);
  EXPECT_NE(hdr.find("friction_utilization"), std::string::npos);
  // Self-description columns (#234 P-5/P-12): without these the in-plane pair
  // and the contact set cannot be recovered from a stored run.
  for (const char* col : {"tick", "plane_normal_x", "basis_x_x", "basis_source", "invalid_reason",
                          "contact_mask", "touch_mask"}) {
    EXPECT_NE(hdr.find(col), std::string::npos) << "missing column: " << col;
  }
}

TEST(PullEstimatorLogPod, MaskColumnsCarryTheRoleBitOrder) {
  // The masks are indexed by contact index; the contact→role mapping used to
  // exist only in a one-shot startup INFO line, so a stored CSV could not be
  // decoded without that run's ROS log (#234 P-14).
  const std::vector<std::string> roles = {"thumb", "index", "middle"};
  std::ostringstream os;
  integrated_bringup::WritePullEstimatorLogHeader(os, roles);
  const std::string hdr = os.str();

  EXPECT_NE(hdr.find("contact_mask[thumb|index|middle]"), std::string::npos) << hdr;
  EXPECT_NE(hdr.find("touch_mask[thumb|index|middle]"), std::string::npos) << hdr;
  EXPECT_NE(hdr.find("opposing_mask[thumb|index|middle]"), std::string::npos) << hdr;

  // Reordering tip_names must change the header, not silently re-mean the bits.
  const std::vector<std::string> reordered = {"index", "thumb", "middle"};
  std::ostringstream os2;
  integrated_bringup::WritePullEstimatorLogHeader(os2, reordered);
  EXPECT_NE(os2.str(), hdr);
  EXPECT_NE(os2.str().find("contact_mask[index|thumb|middle]"), std::string::npos);

  // Eight contacts is the mask capacity (kMaxPullContacts <= 8) — the stamp
  // must not truncate at the four bits the plotter used to assume.
  const std::vector<std::string> eight = {"c0", "c1", "c2", "c3", "c4", "c5", "c6", "c7"};
  std::ostringstream os8;
  integrated_bringup::WritePullEstimatorLogHeader(os8, eight);
  EXPECT_NE(os8.str().find("contact_mask[c0|c1|c2|c3|c4|c5|c6|c7]"), std::string::npos);

  // No roles (unit-test / unconfigured path) → plain column names, still one
  // header token per row token.
  std::ostringstream os0;
  integrated_bringup::WritePullEstimatorLogHeader(os0);
  EXPECT_NE(os0.str().find(",contact_mask,"), std::string::npos) << os0.str();
}

TEST(PullEstimatorLogPod, RowCarriesTheContactAndTouchMasks) {
  integrated_bringup::PullEstimatorLogPod pod{};
  pod.tick = 4242;
  pod.estimate.contact_mask = 0b0000'0101;
  pod.estimate.touch_mask = 0b0000'0111;
  pod.opposing_mask = 0b0000'0110;
  std::ostringstream os;
  integrated_bringup::WritePullEstimatorLogRow(os, pod);
  const std::string row = os.str();

  // Emitted as unsigned decimals, not as raw chars.
  EXPECT_NE(row.find("4242"), std::string::npos) << row;
  EXPECT_NE(row.find(",5,7,6"), std::string::npos) << row;
}

TEST(PullEstimatorLogPod, FirstColumnIsTRelativeS) {
  integrated_bringup::PullEstimatorLogPod pod{};
  pod.t_relative_s = 0.789;
  std::ostringstream os;
  integrated_bringup::WritePullEstimatorLogRow(os, pod);
  EXPECT_EQ(os.str().find("0.789"), 0u);
}

TEST(PullEstimatorLogPod, FillMirrorsPullEstimate) {
  rtc::grasp::PullEstimate est{};
  est.force_raw = Eigen::Vector3d(1.0, 2.0, 3.0);
  est.force_filtered = Eigen::Vector3d(0.5, 1.5, 2.5);
  est.force_inplane = Eigen::Vector2d(0.25, 0.75);
  est.magnitude = 2.9;
  est.directional = -1.25;
  est.max_friction_utilization = 0.6;
  est.leakage_bound = 0.1;
  est.valid_contact_count = 2;
  est.valid = true;
  est.slip_risk = true;
  est.any_saturated = false;
  est.baseline_applied = true;

  est.plane_normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  est.basis_x = Eigen::Vector3d(1.0, 0.0, 0.0);
  est.basis_source = rtc::grasp::PullBasisSource::kCarry;
  est.invalid_reason = rtc::grasp::PullInvalidReason::kNone;
  est.contact_mask = 0b0000'0011;
  est.touch_mask = 0b0000'0111;

  integrated_bringup::PullEstimatorLogPod pod{};
  integrated_bringup::FillPullEstimatorLogPod(est, 1.5, /*tick=*/77, pod);

  EXPECT_DOUBLE_EQ(pod.t_relative_s, 1.5);
  EXPECT_EQ(pod.tick, 77U);
  EXPECT_FLOAT_EQ(pod.force_prefilter[0], 1.0F);
  EXPECT_FLOAT_EQ(pod.force_prefilter[2], 3.0F);
  EXPECT_FLOAT_EQ(pod.estimate.plane_normal[2], 1.0F);
  EXPECT_FLOAT_EQ(pod.estimate.basis_x[0], 1.0F);
  EXPECT_EQ(pod.estimate.basis_source,
            static_cast<std::uint8_t>(rtc::grasp::PullBasisSource::kCarry));
  EXPECT_EQ(pod.estimate.invalid_reason, 0U);
  EXPECT_EQ(pod.estimate.contact_mask, 0b0000'0011U);
  EXPECT_EQ(pod.estimate.touch_mask, 0b0000'0111U);
  EXPECT_FLOAT_EQ(pod.estimate.force[1], 1.5F);
  EXPECT_FLOAT_EQ(pod.estimate.force_inplane[1], 0.75F);
  EXPECT_FLOAT_EQ(pod.estimate.magnitude, 2.9F);
  EXPECT_FLOAT_EQ(pod.estimate.directional, -1.25F);
  EXPECT_FLOAT_EQ(pod.estimate.friction_utilization, 0.6F);
  EXPECT_FLOAT_EQ(pod.estimate.leakage_bound, 0.1F);
  EXPECT_EQ(pod.estimate.valid_contact_count, 2);
  EXPECT_TRUE(pod.estimate.valid);
  EXPECT_TRUE(pod.estimate.slip_risk);
  EXPECT_FALSE(pod.estimate.any_saturated);
  EXPECT_TRUE(pod.estimate.baseline_applied);
}
