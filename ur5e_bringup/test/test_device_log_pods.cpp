// Unit tests for ur5e_bringup POD mirrors of rtc_msgs/DeviceStateLog
// and rtc_msgs/DeviceSensorLog. Verifies trivial-copy guarantee, header
// emission against runtime joint/sensor names, row column count agreement
// with the header, and num_*-bounded array iteration.

#include "ur5e_bringup/logging/device_sensor_log_pod.hpp"
#include "ur5e_bringup/logging/device_state_log_pod.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace {

int CountCommas(const std::string &s) {
  int n = 0;
  for (char c : s)
    if (c == ',')
      ++n;
  return n;
}

} // namespace

TEST(DeviceStateLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<ur5e::DeviceStateLogPod>);
}

TEST(DeviceStateLogPod, HeaderColumnsMatchRowColumnsForArmHand) {
  // ur5e arm 6 + hand 10 motors.
  const std::vector<std::string> joint_names{
      "shoulder_pan", "shoulder_lift", "elbow",   "wrist_1",
      "wrist_2",      "wrist_3",       "thumb1",  "thumb2",
      "index1",       "index2",        "middle1", "middle2",
      "ring1",        "ring2",         "pinky1",  "pinky2"};
  const std::vector<std::string> motor_names{
      "m0", "m1", "m2",  "m3",  "m4",  "m5",  "m6",  "m7",
      "m8", "m9", "m10", "m11", "m12", "m13", "m14", "m15"};

  std::ostringstream hdr_os;
  ur5e::WriteDeviceStateLogHeader(hdr_os, joint_names, motor_names);
  const std::string hdr = hdr_os.str();

  ur5e::DeviceStateLogPod pod{};
  pod.t_relative_s = 1.5;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  pod.num_motors = static_cast<std::uint8_t>(motor_names.size());
  std::ostringstream row_os;
  ur5e::WriteDeviceStateLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row))
      << "header: " << hdr << "\nrow: " << row;
}

TEST(DeviceStateLogPod, RowRespectsRuntimeNumJoints) {
  // Header sized for 16 joints, but pod populates only 6 — row must
  // emit exactly 6 values per joint-array column.
  const std::vector<std::string> joint_names_16(16, "j");
  const std::vector<std::string> motor_names_0;
  std::ostringstream hdr_os;
  ur5e::WriteDeviceStateLogHeader(hdr_os, joint_names_16, motor_names_0);
  const int hdr_commas = CountCommas(hdr_os.str());

  ur5e::DeviceStateLogPod pod{};
  pod.num_joints = 6;
  pod.num_motors = 0;
  std::ostringstream row_os;
  ur5e::WriteDeviceStateLogRow(row_os, pod);
  const int row_commas = CountCommas(row_os.str());

  // Row will have FEWER columns than header by design — runtime num_joints
  // bounded — but task-space and trailing categorical columns are still
  // emitted. We assert row_commas < hdr_commas to confirm the joint loop
  // is bounded by num_joints rather than kMaxJoints.
  EXPECT_LT(row_commas, hdr_commas);
}

TEST(DeviceSensorLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<ur5e::DeviceSensorLogPod>);
}

TEST(DeviceSensorLogPod, HeaderColumnsMatchRowColumnsFor4Fingertips) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle",
                                              "ring"};
  std::ostringstream hdr_os;
  ur5e::WriteDeviceSensorLogHeader(hdr_os, sensor_names);
  const std::string hdr = hdr_os.str();

  ur5e::DeviceSensorLogPod pod{};
  pod.t_relative_s = 0.25;
  pod.num_fingertips = static_cast<std::uint8_t>(sensor_names.size());
  pod.inference_valid = true;
  std::ostringstream row_os;
  ur5e::WriteDeviceSensorLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(CountCommas(hdr), CountCommas(row))
      << "header: " << hdr << "\nrow: " << row;
}

TEST(DeviceSensorLogPod, RowRespectsRuntimeNumFingertips) {
  // Pod populated with 2 fingertips; header (sized for 4) is wider.
  const std::vector<std::string> sensor_names_4{"a", "b", "c", "d"};
  std::ostringstream hdr_os;
  ur5e::WriteDeviceSensorLogHeader(hdr_os, sensor_names_4);
  const int hdr_commas = CountCommas(hdr_os.str());

  ur5e::DeviceSensorLogPod pod{};
  pod.num_fingertips = 2;
  std::ostringstream row_os;
  ur5e::WriteDeviceSensorLogRow(row_os, pod);
  const int row_commas = CountCommas(row_os.str());

  EXPECT_LT(row_commas, hdr_commas);
}

TEST(DeviceStateLogPod, FirstColumnIsTRelativeS) {
  ur5e::DeviceStateLogPod pod{};
  pod.t_relative_s = 0.123;
  std::ostringstream os;
  ur5e::WriteDeviceStateLogRow(os, pod);
  EXPECT_EQ(os.str().find("0.123"), 0u);
}

TEST(DeviceSensorLogPod, FirstColumnIsTRelativeS) {
  ur5e::DeviceSensorLogPod pod{};
  pod.t_relative_s = 7.5;
  std::ostringstream os;
  ur5e::WriteDeviceSensorLogRow(os, pod);
  EXPECT_EQ(os.str().find("7.5"), 0u);
}
