// Unit tests for integrated_bringup POD mirrors of rtc_msgs/DeviceStateLog
// and rtc_msgs/DeviceSensorLog. Verifies trivial-copy guarantee, header
// emission against runtime joint/sensor names, row column count agreement
// with the header, and num_*-bounded array iteration.

#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/device_wbc_log_pod.hpp"
#include "integrated_bringup/logging/wbc_diag_log_pod.hpp"

#include <gtest/gtest.h>

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
  // Fixed-width block → numeric-labelled columns present.
  EXPECT_NE(hdr.find("fingertip_force_0"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_force_7"), std::string::npos);
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
