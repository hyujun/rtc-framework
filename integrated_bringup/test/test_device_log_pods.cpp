// Unit tests for integrated_bringup POD mirrors of rtc_msgs/DeviceStateLog
// and rtc_msgs/DeviceSensorLog. Verifies trivial-copy guarantee, header
// emission against runtime joint/sensor names, row column count agreement
// with the header, and num_*-bounded array iteration.

#include "integrated_bringup/logging/catching_diag_log_pod.hpp"
#include "integrated_bringup/logging/compliance_diag_log_pod.hpp"
#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/device_wbc_log_pod.hpp"
#include "integrated_bringup/logging/dualarm_diag_log_pod.hpp"
#include "integrated_bringup/logging/grasp_diag_log_pod.hpp"
#include "integrated_bringup/logging/inference_diag_log_pod.hpp"
#include "integrated_bringup/logging/momentum_observer_log_pod.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/logging/pull_estimator_log_pod.hpp"
#include "integrated_bringup/logging/task_diag_log_pod.hpp"
#include "integrated_bringup/logging/wbc_diag_log_pod.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

int CountCommas(const std::string& s) {
  int n = 0;
  for (char c : s)
    if (c == ',')
      ++n;
  return n;
}

std::vector<std::string> NumberedJoints(std::size_t n) {
  std::vector<std::string> names;
  for (std::size_t i = 0; i < n; ++i) {
    names.push_back("q" + std::to_string(i));
  }
  return names;
}

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string tok;
  std::istringstream is(line);
  while (std::getline(is, tok, ','))
    out.push_back(tok);
  if (!line.empty() && line.back() == ',')
    out.emplace_back();
  return out;
}

// Header ↔ row structural agreement: same token count, no empty header name,
// no duplicate header name. Comma counting alone (the pre-#551 check) passed
// a header whose columns were in a different order from the row's — every CSV
// consumer is name-based, so that reads silently wrong values.
::testing::AssertionResult HeaderRowAligned(const std::string& hdr, const std::string& row) {
  const auto h = SplitCsv(hdr);
  const auto r = SplitCsv(row);
  if (h.size() != r.size())
    return ::testing::AssertionFailure() << "header has " << h.size() << " columns, row has "
                                         << r.size() << "\nheader: " << hdr << "\nrow: " << row;
  std::vector<std::string> sorted = h;
  std::sort(sorted.begin(), sorted.end());
  for (std::size_t i = 0; i < sorted.size(); ++i) {
    if (sorted[i].empty())
      return ::testing::AssertionFailure() << "empty header name\nheader: " << hdr;
    if (i > 0 && sorted[i] == sorted[i - 1])
      return ::testing::AssertionFailure()
             << "duplicate header name '" << sorted[i] << "'\nheader: " << hdr;
  }
  return ::testing::AssertionSuccess();
}

// Row value under the header column named `name` ("" when the name is absent).
std::string ColumnValue(const std::string& hdr, const std::string& row, const std::string& name) {
  const auto h = SplitCsv(hdr);
  const auto r = SplitCsv(row);
  for (std::size_t i = 0; i < h.size() && i < r.size(); ++i)
    if (h[i] == name)
      return r[i];
  return {};
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

  const auto cols = integrated_bringup::DeviceStateLogColumnsFor(joint_names, motor_names);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceStateLogHeader(hdr_os, joint_names, motor_names, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceStateLogPod pod{};
  pod.t_relative_s = 1.5;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  pod.num_motors = static_cast<std::uint8_t>(motor_names.size());
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceStateLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
}

// SPEC CHANGE (#440): this case used to assert `row_commas < hdr_commas` —
// the row was deliberately narrower than the header when the device reported
// fewer channels than the config named. That contract is what let the sibling
// sensor channel write a 59-column header over 3-column rows for 138,248 rows
// while a header-driven reader read the wrong column's values as real signal.
// The row is now padded to the header's width and carries the runtime count in
// its own column instead.
TEST(DeviceStateLogPod, ShortRuntimeChannelCountPadsRatherThanNarrowsTheRow) {
  // Header sized for 16 joints, but the device reports only 6.
  const std::vector<std::string> joint_names_16 = NumberedJoints(16);
  const std::vector<std::string> motor_names_0;
  const auto cols = integrated_bringup::DeviceStateLogColumnsFor(joint_names_16, motor_names_0);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceStateLogHeader(hdr_os, joint_names_16, motor_names_0, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceStateLogPod pod{};
  pod.num_joints = 6;
  pod.num_motors = 0;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceStateLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
  // The runtime truth travels in the row, not in the column count.
  EXPECT_NE(hdr.find("num_joints"), std::string::npos) << hdr;
  EXPECT_EQ(row.substr(row.rfind(',') + 1), "0");  // num_motors
  const auto tail = row.substr(0, row.rfind(','));
  EXPECT_EQ(tail.substr(tail.rfind(',') + 1), "6");  // num_joints
}

// A device whose backend never came up reports 0 channels while the config
// still names N — the exact shape of the #440 session. Every joint column must
// still be present (and zero) so a reader resolving by header name cannot land
// on `command_type`.
TEST(DeviceStateLogPod, ZeroRuntimeChannelsStillEmitsEveryNamedColumn) {
  const std::vector<std::string> joint_names{"a", "b", "c"};
  const std::vector<std::string> motor_names{"m0", "m1"};
  const auto cols = integrated_bringup::DeviceStateLogColumnsFor(joint_names, motor_names);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceStateLogHeader(hdr_os, joint_names, motor_names, cols);

  integrated_bringup::DeviceStateLogPod pod{};
  pod.num_joints = 0;
  pod.num_motors = 0;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceStateLogRow(row_os, pod, cols);

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()))
      << "header: " << hdr_os.str() << "\nrow: " << row_os.str();
}

// ── A device group wider than 16 joints ──────────────────────────────────────
//
// The capacity was 16, sized for a hand. A humanoid upper body driven as ONE
// group (waist 3 + two 7-DoF arms) is 17, and at 16 its last joint — the right
// wrist yaw — simply did not appear: no column, no row value, and
// FillDeviceStateLogPod clamped num_joints to 16 as if the device had reported
// that. Nothing failed. These two cases put the boundary where it is visible.

namespace {

/// A state whose device 0 reports `n` channels with position i+1 on channel i,
/// so every channel's value says which channel it is.
rtc::ControllerState StateWithChannels(int n) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.devices[0].num_channels = n;
  for (int i = 0; i < n; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = static_cast<double>(i + 1);
  }
  return state;
}

}  // namespace

TEST(DeviceStateLogPod, ASeventeenJointGroupKeepsItsLastJoint) {
  const auto names = NumberedJoints(17);
  const auto cols = integrated_bringup::DeviceStateLogColumnsFor(names, {});
  EXPECT_EQ(cols.joints, 17u);

  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceStateLogHeader(hdr_os, names, {}, cols);
  const auto header = SplitCsv(hdr_os.str());
  const auto last_col = std::find(header.begin(), header.end(), "actual_pos_q16");
  ASSERT_NE(last_col, header.end()) << "no column for the 17th joint: " << hdr_os.str();

  integrated_bringup::DeviceStateLogPod pod{};
  integrated_bringup::FillDeviceStateLogPod(StateWithChannels(17), rtc::ControllerOutput{}, 0, pod);
  EXPECT_EQ(pod.num_joints, 17u);

  std::ostringstream row_os;
  integrated_bringup::WriteDeviceStateLogRow(row_os, pod, cols);
  const auto row = SplitCsv(row_os.str());
  ASSERT_EQ(row.size(), header.size());
  // Channel 16 carries 17 (see StateWithChannels) — read by column NAME, so a
  // row that kept its width but lost the value cannot pass.
  EXPECT_EQ(row[static_cast<std::size_t>(last_col - header.begin())], "17");
  EXPECT_EQ(row[row.size() - 2], "17") << "num_joints column";
}

// Past the capacity the channel still truncates — to 32 now — and it has to do
// so on BOTH writers and in the fill, or the row and the header part ways.
TEST(DeviceStateLogPod, AGroupWiderThanTheCapacityTruncatesToIt) {
  ASSERT_EQ(integrated_bringup::DeviceStateLogPod::kMaxJoints, 32u);
  const auto names = NumberedJoints(33);
  const auto cols = integrated_bringup::DeviceStateLogColumnsFor(names, {});
  EXPECT_EQ(cols.joints, 32u);

  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceStateLogHeader(hdr_os, names, {}, cols);
  const auto header = SplitCsv(hdr_os.str());
  EXPECT_NE(std::find(header.begin(), header.end(), "actual_pos_q31"), header.end());
  EXPECT_EQ(std::find(header.begin(), header.end(), "actual_pos_q32"), header.end());

  integrated_bringup::DeviceStateLogPod pod{};
  integrated_bringup::FillDeviceStateLogPod(StateWithChannels(33), rtc::ControllerOutput{}, 0, pod);
  EXPECT_EQ(pod.num_joints, 32u);

  std::ostringstream row_os;
  integrated_bringup::WriteDeviceStateLogRow(row_os, pod, cols);
  const auto row = SplitCsv(row_os.str());
  ASSERT_EQ(row.size(), header.size());
  EXPECT_EQ(row[row.size() - 2], "32") << "num_joints column";
}

TEST(DeviceSensorLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::DeviceSensorLogPod>);
}

TEST(DeviceSensorLogPod, HeaderColumnsMatchRowColumnsFor4Fingertips) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  const auto cols = integrated_bringup::DeviceSensorLogColumnsFor(sensor_names);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.t_relative_s = 0.25;
  pod.num_fingertips = static_cast<std::uint8_t>(sensor_names.size());
  pod.inference_valid = true;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
}

// A force-only hand declares values_per_group=0. The barometer/ToF block must
// then be absent entirely rather than emitted as a permanently-zero column
// group that reads downstream as a flatlining sensor.
TEST(DeviceSensorLogPod, ForceOnlyLayoutEmitsNoBaroTofColumns) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(
      hdr_os, sensor_names,
      integrated_bringup::DeviceSensorLogColumnsFor(sensor_names, /*values_per_group=*/0));
  const std::string hdr = hdr_os.str();

  EXPECT_EQ(hdr.find("_raw_"), std::string::npos) << hdr;
  EXPECT_EQ(hdr.find("_filt_0"), std::string::npos) << hdr;
  // The inference lane survives; so do the LPF'd force columns.
  EXPECT_NE(hdr.find("ft_thumb_fx"), std::string::npos) << hdr;
  EXPECT_NE(hdr.find("ft_thumb_fx_filt"), std::string::npos) << hdr;
  EXPECT_NE(hdr.find("force_filtered_valid"), std::string::npos) << hdr;

  // 1 timestamp + inference_valid + force_filtered_valid + 4*(7 + 3 + 3 + 1)
  // + num_fingertips = 60 cols. The 3 guarded + 1 guard-verdict columns per
  // fingertip were appended when the delta-spike guard landed, num_fingertips
  // when the width contract was fixed (#440); this count is the schema, not a
  // bound.
  EXPECT_EQ(CountCommas(hdr), 59) << hdr;
}

TEST(DeviceSensorLogPod, ForceOnlyLayoutHeaderMatchesRow) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  const auto cols =
      integrated_bringup::DeviceSensorLogColumnsFor(sensor_names, /*values_per_group=*/0);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, cols);

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.num_fingertips = static_cast<std::uint8_t>(sensor_names.size());
  pod.force_filtered_valid = true;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, cols);

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()))
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
  integrated_bringup::FillDeviceSensorLogPod(state, 1, /*num_active_fingertips=*/1,
                                             integrated_bringup::ForceFilterLogView{filtered},
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

// The guard columns are what make a session's guard behaviour reconstructible:
// raw and filtered alone cannot separate "the guard held this tick" from "the
// filter merely lagged". Column names must also stay clear of the trailing
// `_fx` / `_contact` tokens the rtc_tools raw-force and fingertip-label
// detectors match on, or the guarded block would be read as a second set of raw
// force columns.
TEST(DeviceSensorLogPod, GuardColumnsAppendedAfterFilteredBlockWithNonRawNames) {
  const std::vector<std::string> sensor_names{"thumb", "index"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(
      hdr_os, sensor_names,
      integrated_bringup::DeviceSensorLogColumnsFor(sensor_names, /*values_per_group=*/0));
  const std::string hdr = hdr_os.str();

  const auto raw_pos = hdr.find("ft_index_fx,");
  const auto filt_pos = hdr.find("ft_index_fx_filt");
  const auto guarded_pos = hdr.find("ft_index_fx_guarded");
  const auto rejected_pos = hdr.find("ft_index_force_guard_rejected");
  ASSERT_NE(raw_pos, std::string::npos) << hdr;
  ASSERT_NE(filt_pos, std::string::npos) << hdr;
  ASSERT_NE(guarded_pos, std::string::npos) << hdr;
  ASSERT_NE(rejected_pos, std::string::npos) << hdr;
  // Append-only: everything a pre-guard reader consumed positionally still
  // precedes the new blocks.
  EXPECT_LT(raw_pos, filt_pos) << hdr;
  EXPECT_LT(filt_pos, guarded_pos) << hdr;
  EXPECT_LT(guarded_pos, rejected_pos) << hdr;

  // No new column ends in the tokens the raw detectors key on.
  std::istringstream cols(hdr);
  std::string col;
  while (std::getline(cols, col, ',')) {
    if (col.find("_guarded") != std::string::npos ||
        col.find("_force_guard_rejected") != std::string::npos) {
      EXPECT_FALSE(col.size() >= 3 && col.compare(col.size() - 3, 3, "_fx") == 0) << col;
      EXPECT_FALSE(col.size() >= 8 && col.compare(col.size() - 8, 8, "_contact") == 0) << col;
    }
  }
}

// Header and row must still agree once the pod carries guard data (a row that
// emits the guarded block while the header does not — or vice versa — shifts
// every column after it).
TEST(DeviceSensorLogPod, GuardColumnsHeaderMatchesRow) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  const auto cols =
      integrated_bringup::DeviceSensorLogColumnsFor(sensor_names, /*values_per_group=*/0);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, cols);

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.num_fingertips = static_cast<std::uint8_t>(sensor_names.size());
  pod.force_filtered_valid = true;
  pod.force_guarded[0] = 1.0F;
  pod.force_guard_rejected[0] = 1;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, cols);

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()))
      << "header: " << hdr_os.str() << "\nrow: " << row_os.str();
}

// One tick, four quantities: the driver's raw force, the guarded triplet the LPF
// was fed, the filter output, and the guard verdict. A fill that dropped any of
// them (or wrote the guarded triplet over the raw slots) would still produce a
// well-formed row.
TEST(DeviceSensorLogPod, FillCarriesRawGuardedFilteredAndVerdictForSameTick) {
  rtc::ControllerState state{};
  state.num_devices = 2;
  state.devices[1].num_channels = 1;
  state.devices[1].inference_enable[0] = true;
  // Raw wire force = a 6.4 N fz spike (inference slots 1..3).
  state.devices[1].inference_data[1] = 0.1F;
  state.devices[1].inference_data[2] = -0.2F;
  state.devices[1].inference_data[3] = 6.4F;

  const std::array<float, 3> filtered{{0.1F, -0.2F, 0.03F}};
  const std::array<float, 3> guarded{{0.1F, -0.2F, 0.02F}};
  const std::array<std::uint8_t, 1> rejected{{1}};

  integrated_bringup::DeviceSensorLogPod pod{};
  integrated_bringup::FillDeviceSensorLogPod(
      state, 1, /*num_active_fingertips=*/1,
      integrated_bringup::ForceFilterLogView{filtered, guarded, rejected}, pod);

  // Raw is preserved verbatim — the guard must never launder the wire value.
  EXPECT_FLOAT_EQ(pod.inference_output[3], 6.4F);
  EXPECT_FLOAT_EQ(pod.force_guarded[2], 0.02F);
  EXPECT_FLOAT_EQ(pod.force_filtered[2], 0.03F);
  EXPECT_EQ(pod.force_guard_rejected[0], 1);
  EXPECT_TRUE(pod.force_filtered_valid);

  // A controller without the force LPF has no guard either: both blocks stay
  // zero and the shared valid flag says so.
  integrated_bringup::DeviceSensorLogPod no_filter{};
  integrated_bringup::FillDeviceSensorLogPod(state, 1, /*num_active_fingertips=*/1, {}, no_filter);
  EXPECT_FALSE(no_filter.force_filtered_valid);
  EXPECT_FLOAT_EQ(no_filter.force_guarded[2], 0.0F);
  EXPECT_EQ(no_filter.force_guard_rejected[0], 0);
  EXPECT_FLOAT_EQ(no_filter.inference_output[3], 6.4F) << "raw lane is independent of the guard";
}

// SPEC CHANGE (#440): this case used to assert `row_commas < hdr_commas`. That
// contract is precisely what produced a 59-column header over 3-column rows in
// logging_data/260808_2314 — 138,248 of them, with no error and no warning.
TEST(DeviceSensorLogPod, ShortRuntimeFingertipCountPadsRatherThanNarrowsTheRow) {
  const std::vector<std::string> sensor_names_4{"a", "b", "c", "d"};
  const auto cols = integrated_bringup::DeviceSensorLogColumnsFor(sensor_names_4);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names_4, cols);

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.num_fingertips = 2;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, cols);

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()))
      << "header: " << hdr_os.str() << "\nrow: " << row_os.str();
}

// The #440 session verbatim: config names 4 fingertips, the hand reports 0.
// The old writer collapsed the row to `t_relative_s, inference_valid,
// force_filtered_valid` — 3 fields under a 59-field header — and rtc_tools'
// detect_ft_labels, which resolves columns by header name, then read
// `force_filtered_valid` (constant 1) through the `ft_thumb_contact` index.
// A sensor-less run plotted as "thumb in contact for the whole session".
TEST(DeviceSensorLogPod, ZeroRuntimeFingertipsKeepsHeaderAndRowAligned) {
  const std::vector<std::string> sensor_names{"thumb", "index", "middle", "ring"};
  const auto cols =
      integrated_bringup::DeviceSensorLogColumnsFor(sensor_names, /*values_per_group=*/0);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.t_relative_s = 0.0;
  pod.num_fingertips = 0;
  pod.force_filtered_valid = true;  // the constant-1 column of the real session
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  ASSERT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;

  // Resolve ft_thumb_contact positionally, the way a header-driven reader does,
  // and confirm it now lands on an absent-sensor 0 rather than on the
  // force_filtered_valid flag.
  const auto column_at = [](const std::string& line, std::size_t idx) {
    std::istringstream ss(line);
    std::string field;
    for (std::size_t i = 0; i <= idx; ++i) {
      std::getline(ss, field, ',');
    }
    return field;
  };
  std::size_t contact_idx = 0;
  {
    std::istringstream ss(hdr);
    std::string col;
    for (std::size_t i = 0; std::getline(ss, col, ','); ++i) {
      if (col == "ft_thumb_contact") {
        contact_idx = i;
        break;
      }
    }
  }
  ASSERT_NE(contact_idx, 0u) << hdr;
  EXPECT_EQ(column_at(row, contact_idx), "0") << "row: " << row;

  // …and the row says why it is 0: the device reported no fingertips at all.
  EXPECT_EQ(row.substr(row.rfind(',') + 1), "0");
  EXPECT_EQ(hdr.substr(hdr.rfind(',') + 1), "num_fingertips");
}

// Mirror direction: the device reports MORE fingertips than the config named
// (the LEAP shape — tactile names absent, contact-force fingertips present).
// The row is truncated to the header's width rather than overrunning it, and
// num_fingertips records that data was left out.
TEST(DeviceSensorLogPod, ExcessRuntimeFingertipsTruncateToHeaderWidthAndSaySo) {
  const std::vector<std::string> sensor_names{"thumb", "index"};
  const auto cols =
      integrated_bringup::DeviceSensorLogColumnsFor(sensor_names, /*values_per_group=*/0);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, cols);

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.num_fingertips = 4;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row)) << "header: " << hdr_os.str();
  EXPECT_EQ(row.substr(row.rfind(',') + 1), "4") << row;
}

// Padding must not leak the device's packing. FillDeviceSensorLogPod copies the
// sensor lane at the CAPACITY stride while the writer indexes at the LAYOUT
// stride, so for a narrow layout the array tail past the reported fingertips
// can still hold live bytes — a padded slot that read the array would publish
// them under another fingertip's name.
TEST(DeviceSensorLogPod, PaddedSlotsAreZeroEvenWhenTheArrayTailHoldsData) {
  const std::vector<std::string> sensor_names{"thumb", "index"};
  const auto cols =
      integrated_bringup::DeviceSensorLogColumnsFor(sensor_names, /*values_per_group=*/2);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceSensorLogHeader(hdr_os, sensor_names, cols);

  integrated_bringup::DeviceSensorLogPod pod{};
  pod.num_fingertips = 1;
  pod.sensor_data_raw[0] = 11;
  pod.sensor_data_raw[1] = 22;
  pod.sensor_data_raw[2] = 33;  // index fingertip's slot at stride 2 — padding
  pod.sensor_data_raw[3] = 44;
  pod.inference_output[7] = 9.5F;  // fingertip 1's contact slot — padding
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceSensorLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row));
  EXPECT_NE(row.find("11"), std::string::npos) << row;  // reported slot survives
  EXPECT_EQ(row.find("33"), std::string::npos) << row;
  EXPECT_EQ(row.find("44"), std::string::npos) << row;
  EXPECT_EQ(row.find("9.5"), std::string::npos) << row;
}

TEST(DeviceStateLogPod, FirstColumnIsTRelativeS) {
  integrated_bringup::DeviceStateLogPod pod{};
  pod.t_relative_s = 0.123;
  std::ostringstream os;
  integrated_bringup::WriteDeviceStateLogRow(os, pod, {});
  EXPECT_EQ(os.str().find("0.123"), 0u);
}

TEST(DeviceSensorLogPod, FirstColumnIsTRelativeS) {
  integrated_bringup::DeviceSensorLogPod pod{};
  pod.t_relative_s = 7.5;
  std::ostringstream os;
  integrated_bringup::WriteDeviceSensorLogRow(os, pod, {});
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
  const auto cols = integrated_bringup::DeviceWbcLogColumnsFor(joint_names, {});
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/0, joint_names, {}, {}, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 0;
  pod.t_relative_s = 1.5;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
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
  const auto cols = integrated_bringup::DeviceWbcLogColumnsFor(joint_names, motor_names);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/1, joint_names, motor_names,
                                              fingertip_names, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 1;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  pod.num_motors = static_cast<std::uint8_t>(motor_names.size());
  pod.num_fingertips = static_cast<std::uint8_t>(fingertip_names.size());
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
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
  const auto cols = integrated_bringup::DeviceWbcLogColumnsFor(joint_names, {});
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/1, joint_names, {}, {}, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 1;
  pod.num_joints = static_cast<std::uint8_t>(joint_names.size());
  pod.num_motors = 0;
  pod.num_fingertips = 2;  // runtime count < kMaxFingertips, no names registered
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod, cols);

  EXPECT_TRUE(HeaderRowAligned(hdr, row_os.str())) << "header: " << hdr;
  // Fixed-width blocks → numeric-labelled columns present for |F| and each axis.
  EXPECT_NE(hdr.find("fingertip_force_0"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_force_7"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fx_0"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fy_7"), std::string::npos);
  EXPECT_NE(hdr.find("fingertip_fz_0"), std::string::npos);
}

// SPEC CHANGE (#440): this case used to assert the row was NARROWER than the
// header. The fingertip block of this same POD had already rejected that
// contract for the axis with no configured width; the joint / motor axes now
// close it the other way, by sizing the row from the configured width.
TEST(DeviceWbcLogPod, ShortRuntimeJointCountPadsRatherThanNarrowsTheRow) {
  const std::vector<std::string> joint_names_16 = NumberedJoints(16);
  const auto cols = integrated_bringup::DeviceWbcLogColumnsFor(joint_names_16, {});
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/0, joint_names_16, {}, {}, cols);
  const std::string hdr = hdr_os.str();

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 0;
  pod.num_joints = 6;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
  EXPECT_EQ(row.substr(row.rfind(',') + 1), "6") << row;
  // The arm role carries no hand-only counts.
  EXPECT_EQ(hdr.find("num_fingertips"), std::string::npos) << hdr;
}

// A hand whose backend reports nothing: every named joint / motor column must
// still be present so a header-driven reader cannot land on command_type.
TEST(DeviceWbcLogPod, ZeroRuntimeCountsKeepsHandHeaderAndRowAligned) {
  const std::vector<std::string> joint_names{"j0", "j1", "j2"};
  const std::vector<std::string> motor_names{"m0", "m1"};
  const auto cols = integrated_bringup::DeviceWbcLogColumnsFor(joint_names, motor_names);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDeviceWbcLogHeader(hdr_os, /*role=*/1, joint_names, motor_names, {},
                                              cols);

  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 1;
  pod.num_joints = 0;
  pod.num_motors = 0;
  pod.num_fingertips = 0;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(row_os, pod, cols);

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()))
      << "header: " << hdr_os.str() << "\nrow: " << row_os.str();
  EXPECT_NE(hdr_os.str().find("num_fingertips"), std::string::npos) << hdr_os.str();
}

TEST(DeviceWbcLogPod, AccelColumnRoundTrips) {
  const std::vector<std::string> joint_names{"a", "b"};
  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = 0;
  pod.num_joints = 2;
  pod.accelerations[0] = 3.25;
  pod.accelerations[1] = -1.5;
  std::ostringstream row_os;
  integrated_bringup::WriteDeviceWbcLogRow(
      row_os, pod, integrated_bringup::DeviceWbcLogColumnsFor(joint_names, {}));
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
  pod.qp_fail_count = 4;
  pod.kin_qp_fail_count = 5;
  pod.qp_status = 7;
  pod.qp_iterations = 123;
  pod.joint_box_inverted = 3;
  pod.lambda_opt[11] = 0.25;
  std::ostringstream row_os;
  integrated_bringup::WriteWbcDiagLogRow(row_os, pod, kNVars);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row));
  // Name → value, not just width: a header/row order mismatch reads a wrong
  // column silently in every (name-based) consumer. The distinct values catch
  // a swap among the int columns (#551).
  EXPECT_EQ(ColumnValue(hdr, row, "phase"), "closure");
  EXPECT_EQ(ColumnValue(hdr, row, "qp_converged"), "1");
  // Fallback split (decision 6): Dynamic (qp_fail_count) and Kinematic
  // (kin_qp_fail_count) QP failure counters are distinct CSV columns.
  EXPECT_EQ(ColumnValue(hdr, row, "qp_fail_count"), "4");
  EXPECT_EQ(ColumnValue(hdr, row, "kin_qp_fail_count"), "5");
  EXPECT_EQ(ColumnValue(hdr, row, "qp_status"), "7");
  EXPECT_EQ(ColumnValue(hdr, row, "qp_iterations"), "123");
  EXPECT_EQ(ColumnValue(hdr, row, "joint_box_inverted"), "3");
  EXPECT_EQ(ColumnValue(hdr, row, "lambda_11"), "0.25");
  EXPECT_EQ(ColumnValue(hdr, row, "num_contact_vars"), "12");
}

TEST(WbcDiagLogPod, DefaultRowSaysNoQpAttemptAndNoJointBox) {
  // −1 is the "nothing to report" value for both classification columns, so a
  // zero-initialised pod must not read as "SOLVED with 0 inverted rows".
  std::ostringstream hdr_os;
  integrated_bringup::WriteWbcDiagLogHeader(hdr_os, 0);
  integrated_bringup::WbcDiagLogPod pod{};
  std::ostringstream row_os;
  integrated_bringup::WriteWbcDiagLogRow(row_os, pod, 0);
  EXPECT_EQ(ColumnValue(hdr_os.str(), row_os.str(), "qp_status"), "-1");
  EXPECT_EQ(ColumnValue(hdr_os.str(), row_os.str(), "qp_iterations"), "0");
  EXPECT_EQ(ColumnValue(hdr_os.str(), row_os.str(), "joint_box_inverted"), "-1");
}

TEST(WbcDiagLogPod, PhaseStringTranslation) {
  integrated_bringup::WbcDiagLogPod pod{};
  pod.phase = 3;  // kClosure
  pod.num_contact_vars = 0;
  std::ostringstream os;
  integrated_bringup::WriteWbcDiagLogRow(os, pod, 0);
  EXPECT_NE(os.str().find("closure"), std::string::npos);
}

// SPEC CHANGE (#440): same axis, same fix — the λ block is sized by the
// configured QP dim both writers were handed, and the runtime count travels in
// its own column instead of in the row's width.
TEST(WbcDiagLogPod, ShortRuntimeContactVarCountPadsRatherThanNarrowsTheRow) {
  constexpr std::size_t kNVars = 12;
  std::ostringstream hdr_os;
  integrated_bringup::WriteWbcDiagLogHeader(hdr_os, kNVars);
  const std::string hdr = hdr_os.str();

  integrated_bringup::WbcDiagLogPod pod{};
  pod.num_contact_vars = 6;
  std::ostringstream row_os;
  integrated_bringup::WriteWbcDiagLogRow(row_os, pod, kNVars);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
  EXPECT_EQ(row.substr(row.rfind(',') + 1), "6") << row;
}

TEST(WbcDiagLogPod, FirstColumnIsTRelativeS) {
  integrated_bringup::WbcDiagLogPod pod{};
  pod.t_relative_s = 0.456;
  pod.num_contact_vars = 0;
  std::ostringstream os;
  integrated_bringup::WriteWbcDiagLogRow(os, pod, 0);
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

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
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

// ── TaskDiagLogPod (#310) ──────────────────────────────────────────────────

TEST(TaskDiagLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::TaskDiagLogPod>);
}

TEST(TaskDiagLogPod, HeaderColumnsMatchRow) {
  std::ostringstream hdr_os;
  integrated_bringup::WriteTaskDiagLogHeader(hdr_os);
  const std::string hdr = hdr_os.str();

  integrated_bringup::TaskDiagLogPod pod{};
  pod.t_relative_s = 1.25;
  pod.valid = true;
  pod.sigma_min = 0.004;
  pod.lambda_sq = 1e-4;
  pod.sigma0 = 0.02;
  pod.lambda_max = 0.05;
  pod.ik_ok = true;
  pod.control_6dof = true;
  std::ostringstream row_os;
  integrated_bringup::WriteTaskDiagLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
  EXPECT_NE(hdr.find("sigma_min"), std::string::npos);
  EXPECT_NE(hdr.find("lambda_sq"), std::string::npos);
  // σ₀/λ_max ride along so a stored CSV decodes without that run's YAML — the
  // gains are live-writable through set_gains()/the parameter callback.
  EXPECT_NE(hdr.find("sigma0"), std::string::npos);
  EXPECT_NE(hdr.find("lambda_max"), std::string::npos);
}

TEST(TaskDiagLogPod, FirstColumnIsTRelativeS) {
  std::ostringstream hdr_os;
  integrated_bringup::WriteTaskDiagLogHeader(hdr_os);
  EXPECT_EQ(hdr_os.str().rfind("t_relative_s", 0), 0u);
}

TEST(TaskDiagLogPod, DampingActiveTracksLambdaSqOnValidTicks) {
  // damping_active is the column the file exists for: it must be 1 exactly when
  // §6.5 damping is engaged on a tick that actually solved.
  integrated_bringup::TaskDiagLogPod pod{};
  pod.valid = true;
  pod.lambda_sq = 0.0;  // far from singular
  std::ostringstream off_os;
  integrated_bringup::WriteTaskDiagLogRow(off_os, pod);
  const std::string off = off_os.str();
  EXPECT_EQ(off.substr(off.rfind(',') + 1), "0");

  pod.lambda_sq = 2.5e-5;  // damping engaged
  std::ostringstream on_os;
  integrated_bringup::WriteTaskDiagLogRow(on_os, pod);
  const std::string on = on_os.str();
  EXPECT_EQ(on.substr(on.rfind(',') + 1), "1");
}

TEST(TaskDiagLogPod, StaleRowIsNotReportedAsDamping) {
  // A tick that skipped ComputeControl (E-STOP / unreadable arm / invalid
  // reorder) keeps the PREVIOUS tick's σ fields. valid=0 must both appear and
  // suppress damping_active, or a held tick reads as live damping.
  integrated_bringup::TaskDiagLogPod pod{};
  pod.valid = false;
  pod.lambda_sq = 2.5e-5;  // stale leftover from the last solved tick
  std::ostringstream row_os;
  integrated_bringup::WriteTaskDiagLogRow(row_os, pod);
  const std::string row = row_os.str();

  EXPECT_EQ(row.substr(row.rfind(',') + 1), "0") << "stale row must not claim damping: " << row;
  // valid is the 2nd column.
  const auto first = row.find(',');
  const auto second = row.find(',', first + 1);
  EXPECT_EQ(row.substr(first + 1, second - first - 1), "0");
}

// ── GraspDiagLogPod (#428) ──────────────────────────────────────────────────

TEST(GraspDiagLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::GraspDiagLogPod>);
}

TEST(GraspDiagLogPod, HeaderColumnsMatchRowColumns) {
  const std::vector<std::string> fingers{"thumb", "index", "middle"};

  std::ostringstream hdr_os;
  integrated_bringup::WriteGraspDiagLogHeader(hdr_os, fingers);
  integrated_bringup::GraspDiagLogPod pod{};
  pod.num_fingers = static_cast<std::uint8_t>(fingers.size());
  std::ostringstream row_os;
  integrated_bringup::WriteGraspDiagLogRow(row_os, pod, fingers.size());

  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()))
      << "header: " << hdr_os.str() << "\nrow: " << row_os.str();
}

// A hand with fewer sensors than the POD's capacity must not emit columns for
// fingers it cannot name: the row writer is driven by the same count the header
// writer got, so a mismatch is a silent column shift for every reader.
TEST(GraspDiagLogPod, HeaderAndRowAgreeForASingleFinger) {
  const std::vector<std::string> fingers{"thumb"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteGraspDiagLogHeader(hdr_os, fingers);
  integrated_bringup::GraspDiagLogPod pod{};
  std::ostringstream row_os;
  integrated_bringup::WriteGraspDiagLogRow(row_os, pod, fingers.size());
  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()));
}

// The finger name is stamped into every per-finger column, so a stored CSV
// decodes without that run's YAML (#234 P-14 rationale). Naming only the first
// block would leave the rest positional.
TEST(GraspDiagLogPod, HeaderStampsFingerNamesOnEveryQuantity) {
  const std::vector<std::string> fingers{"thumb", "index"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteGraspDiagLogHeader(hdr_os, fingers);
  const std::string hdr = hdr_os.str();

  for (const char* q :
       {"s_", "f_desired_", "f_measured_", "f_error_", "k_est_", "k_inst_raw_", "delta_s_",
        "delta_f_", "gain_scale_", "est_updated_", "integrator_frozen_", "contact_"}) {
    for (const auto& f : fingers) {
      const std::string col = std::string(q) + f;
      EXPECT_NE(hdr.find(col), std::string::npos) << "missing column " << col << " in " << hdr;
    }
  }
}

// The estimator's raw sample pair is the reason this channel exists — it cannot
// be recovered from the s/f columns because the estimator consumes s_{t-1} -
// s_{t-2} while a logged row carries s_t and s_{t-1} (#425). So the fill must
// copy what FingerState recorded, not recompute anything.
TEST(GraspDiagLogPod, FillCopiesTheEstimatorsOwnSamplePairAndPreClampRatio) {
  std::array<rtc::grasp::FingerState, 2> fingers{};
  fingers[0].s = 0.4;
  fingers[0].f_desired = 2.0;
  fingers[0].f_measured = 1.5;
  fingers[0].K_contact_est = 3.0;
  fingers[0].delta_s_used = 0.002;
  fingers[0].delta_f_used = 0.5;
  fingers[0].K_inst_raw = 250.0;  // far above K_est_max below — must survive
  fingers[0].estimate_updated = true;
  fingers[0].contact_detected = true;
  fingers[0].integrator_frozen = true;

  rtc::grasp::GraspParams params{};
  params.beta = 0.3;
  params.alpha_ema = 0.95;
  params.K_est_max = 5.0;

  integrated_bringup::GraspDiagLogPod pod{};
  integrated_bringup::FillGraspDiagLogPod(fingers, params, rtc::grasp::GraspPhase::kHolding, 2.5,
                                          /*t_relative_s=*/1.25, /*tick=*/77, pod);

  EXPECT_TRUE(pod.valid);
  EXPECT_EQ(pod.tick, 77U);
  EXPECT_EQ(pod.grasp_phase, static_cast<std::uint8_t>(rtc::grasp::GraspPhase::kHolding));
  EXPECT_FLOAT_EQ(pod.target_force, 2.5F);
  EXPECT_FLOAT_EQ(pod.K_est_max, 5.0F);
  EXPECT_FLOAT_EQ(pod.delta_s[0], 0.002F);
  EXPECT_FLOAT_EQ(pod.delta_f[0], 0.5F);
  // Pre-clamp: logging the post-clamp value instead would cap this at 5.0 and
  // erase exactly the spread #426 is arguing about.
  EXPECT_FLOAT_EQ(pod.k_inst_raw[0], 250.0F);
  EXPECT_EQ(pod.est_updated[0], 1U);
  EXPECT_FLOAT_EQ(pod.f_error[0], 0.5F);
  // gain_scale is 1/(1 + beta*K), the multiplier actually applied to Kp/Ki.
  EXPECT_FLOAT_EQ(pod.gain_scale[0], static_cast<float>(1.0 / (1.0 + 0.3 * 3.0)));
  // Finger 1 was left default — its block must stay zeroed rather than repeat
  // finger 0, which is the shape a copy-paste fill bug takes.
  EXPECT_FLOAT_EQ(pod.k_inst_raw[1], 0.0F);
  EXPECT_EQ(pod.est_updated[1], 0U);
}

// "No sample this tick" and "a sample that was rejected" are the same all-zeros
// row without this flag, and they mean different things for #426: the first is
// a converged loop, the second is a guard throwing data away.
TEST(GraspDiagLogPod, RejectedSampleIsDistinguishableFromNoSample) {
  std::array<rtc::grasp::FingerState, 1> rejected{};
  rejected[0].delta_s_used = 0.001;
  rejected[0].delta_f_used = -0.2;
  rejected[0].K_inst_raw = -200.0;  // negative: cleared the epsilon guard, failed K_inst > 0
  rejected[0].estimate_updated = false;

  rtc::grasp::GraspParams params{};
  integrated_bringup::GraspDiagLogPod pod{};
  integrated_bringup::FillGraspDiagLogPod(rejected, params, rtc::grasp::GraspPhase::kForceControl,
                                          1.0, 0.0, 1, pod);
  EXPECT_EQ(pod.est_updated[0], 0U);
  EXPECT_LT(pod.k_inst_raw[0], 0.0F) << "a rejected sample must still be visible in the file";
  EXPECT_NE(pod.delta_s[0], 0.0F);
}

TEST(GraspDiagLogPod, FingerNamesTruncateToTheGraspFingerCount) {
  const std::vector<std::string> sensors{"thumb", "index", "middle", "ring"};
  const auto names = integrated_bringup::GraspDiagFingerNames(sensors, 3);
  ASSERT_EQ(names.size(), 3U);
  EXPECT_EQ(names[2], "middle");
  // A hand reporting fewer sensors than configured grasp fingers yields the
  // shorter list — the header must not name a finger with no force lane.
  const auto short_names =
      integrated_bringup::GraspDiagFingerNames(std::vector<std::string>{"thumb"}, 3);
  EXPECT_EQ(short_names.size(), 1U);
}

// ── InferenceDiagLogPod ─────────────────────────────────────────────────────

TEST(InferenceDiagLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::InferenceDiagLogPod>);
}

TEST(InferenceDiagLogPod, HeaderColumnsMatchRowAndNameTheTips) {
  const std::vector<std::string> tips{"thumb", "index", "middle", "ring"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteInferenceDiagLogHeader(hdr_os, tips);
  const std::string hdr = hdr_os.str();

  integrated_bringup::InferenceDiagLogPod pod{};
  pod.held = true;
  pod.hold_reason = static_cast<std::uint8_t>(integrated_bringup::InferenceHoldReason::kObject);
  pod.num_tips = 4;
  pod.tip_force = {0.1, 0.2, 0.3, 0.4};
  std::ostringstream row_os;
  integrated_bringup::WriteInferenceDiagLogRow(row_os, pod, tips.size());
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
  EXPECT_EQ(hdr.rfind("t_relative_s", 0), 0U);
  EXPECT_NE(hdr.find(",force_thumb,force_index,force_middle,force_ring"), std::string::npos);
  // The reason is written as a code AND a name, so a stored file reads without
  // this header's enum.
  EXPECT_NE(row.find(",1,7,object,"), std::string::npos) << row;
}

TEST(InferenceDiagLogPod, AShortTipCountPadsRatherThanNarrowsTheRow) {
  const std::vector<std::string> tips{"thumb", "index", "middle", "ring"};
  std::ostringstream hdr_os;
  integrated_bringup::WriteInferenceDiagLogHeader(hdr_os, tips);
  integrated_bringup::InferenceDiagLogPod pod{};
  pod.num_tips = 1;
  pod.tip_force = {9.0, 9.0, 9.0, 9.0};  // beyond num_tips must NOT reach the file
  std::ostringstream row_os;
  integrated_bringup::WriteInferenceDiagLogRow(row_os, pod, tips.size());
  const std::string row = row_os.str();
  EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row));
  EXPECT_EQ(row.substr(row.size() - 8), ",9,0,0,0") << row;
}

TEST(InferenceDiagLogPod, EveryHoldReasonHasItsOwnName) {
  std::vector<std::string> names;
  for (std::size_t i = 0; i < integrated_bringup::kNumInferenceHoldReasons; ++i) {
    const auto name = std::string(integrated_bringup::InferenceHoldReasonName(
        static_cast<integrated_bringup::InferenceHoldReason>(i)));
    EXPECT_NE(name, "unknown") << "reason " << i << " has no name";
    EXPECT_EQ(std::count(names.begin(), names.end(), name), 0) << "duplicate name " << name;
    names.push_back(name);
  }
  // One past the table: the count constant and the enum must not drift apart.
  EXPECT_EQ(integrated_bringup::InferenceHoldReasonName(
                static_cast<integrated_bringup::InferenceHoldReason>(
                    integrated_bringup::kNumInferenceHoldReasons)),
            "unknown");
}

// ── DualArmDiagLogPod ───────────────────────────────────────────────────────

TEST(DualArmDiagLogPod, IsTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<integrated_bringup::DualArmDiagLogPod>);
}

TEST(DualArmDiagLogPod, HeaderColumnsMatchRowAndNameTheTasksAndJoints) {
  using integrated_bringup::DualArmDiagLogPod;
  const std::vector<std::string> tasks = {"right_hand", "left_hand"};
  const std::vector<std::string> joints = {"j_a", "j_b", "j_c"};
  const auto cols = integrated_bringup::DualArmDiagLogColumnsFor(tasks, joints);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDualArmDiagLogHeader(hdr_os, tasks, joints, cols);
  const std::string hdr = hdr_os.str();

  DualArmDiagLogPod pod{};
  pod.num_tasks = 2;
  pod.num_joints = 3;
  pod.tasks[1].err_lin = 0.004;
  pod.q_cmd[2] = 1.25;
  std::ostringstream row_os;
  integrated_bringup::WriteDualArmDiagLogRow(row_os, pod, cols);
  const std::string row = row_os.str();

  EXPECT_TRUE(HeaderRowAligned(hdr, row)) << "header: " << hdr << "\nrow: " << row;
  // The declared width is the one both writers produce.
  EXPECT_EQ(static_cast<std::size_t>(CountCommas(hdr)) + 1,
            integrated_bringup::DualArmDiagLogColumnCount(cols));
  EXPECT_EQ(hdr.rfind("t_relative_s,tick,", 0), 0U);
  for (const char* column :
       {"right_hand_err_lin", "left_hand_err_ang", "left_hand_ref_qw", "right_hand_cmd_z",
        "left_hand_meas_valid", "right_hand_reject_unknown_frame", "left_hand_drop_unusable",
        "q_cmd_j_c", "brake_static_infeasible", "track_err"}) {
    EXPECT_NE(hdr.find(column), std::string::npos) << column;
  }
  // The last column is the last joint's command.
  EXPECT_EQ(row.substr(row.rfind(',') + 1), "1.25");
}

TEST(DualArmDiagLogPod, RowWidthFollowsTheConfiguredNamesNotThePod) {
  using integrated_bringup::DualArmDiagLogPod;
  const std::vector<std::string> tasks = {"only"};
  const std::vector<std::string> joints = {"j_a", "j_b"};
  const auto cols = integrated_bringup::DualArmDiagLogColumnsFor(tasks, joints);
  std::ostringstream hdr_os;
  integrated_bringup::WriteDualArmDiagLogHeader(hdr_os, tasks, joints, cols);

  // A pod that claims more tasks and joints than the header has columns for
  // (and one that claims none) still writes the header's width.
  for (const std::uint8_t claimed : {std::uint8_t{0}, std::uint8_t{4}}) {
    DualArmDiagLogPod pod{};
    pod.num_tasks = claimed;
    pod.num_joints = static_cast<std::uint8_t>(claimed * 8);
    std::ostringstream row_os;
    integrated_bringup::WriteDualArmDiagLogRow(row_os, pod, cols);
    EXPECT_TRUE(HeaderRowAligned(hdr_os.str(), row_os.str()))
        << "claimed " << static_cast<int>(claimed);
  }

  // More names than the pod's capacity: both writers stop at the capacity.
  const std::vector<std::string> many_tasks(DualArmDiagLogPod::kMaxTasks + 2, "t");
  const std::vector<std::string> many_joints(DualArmDiagLogPod::kMaxJoints + 5, "j");
  const auto capped = integrated_bringup::DualArmDiagLogColumnsFor(many_tasks, many_joints);
  EXPECT_EQ(capped.tasks, DualArmDiagLogPod::kMaxTasks);
  EXPECT_EQ(capped.joints, DualArmDiagLogPod::kMaxJoints);
}

// ── Every column by name (#883) ─────────────────────────────────────────────
//
// HeaderRowAligned proves the two writers agree on width and names, and the
// ColumnValue spot checks above pin a handful of cells. Neither catches two
// row statements trading places when both cells hold the same value — the
// default 0, or two bools left false. The tests below give every column of a
// writer a value no other column has (strict mode), assert all of them by
// name, and walk each bool column on alone, so any such swap reads a wrong
// cell. Value rules: |v| < 10000 (the default ostream prints 6 significant
// digits), steps of 0.25 (exact in float), integer and floating columns from
// disjoint ranges (float 101 and int 101 both print "101").

namespace {

// Expected cells of one row, as (header name, printed value).
using Cells = std::vector<std::pair<std::string, std::string>>;

template <class T>
std::string Str(const T& v) {
  std::ostringstream os;
  os << v;
  return os.str();
}

// Every name in `expected` is a header column whose row cell prints its value,
// and every header column is in `expected`. `strict` (the writer tests, not
// the Fill round trips) also requires each non-bool cell to be neither "0" nor
// "1" and unlike every other non-bool cell, so swapping any two of them shows.
::testing::AssertionResult RowMatchesByName(const std::string& hdr, const std::string& row,
                                            const Cells& expected,
                                            const std::vector<std::string>& bools, bool strict) {
  const auto aligned = HeaderRowAligned(hdr, row);
  if (!aligned)
    return aligned;
  const auto h = SplitCsv(hdr);
  const auto r = SplitCsv(row);
  std::ostringstream err;
  std::map<std::string, std::string> want;
  for (const auto& [name, value] : expected)
    if (!want.emplace(name, value).second)
      err << "\n  '" << name << "' expected twice";
  for (const auto& name : bools)
    if (want.count(name) == 0)
      err << "\n  bool '" << name << "' has no expected value";
  for (const auto& [name, value] : expected) {
    const auto it = std::find(h.begin(), h.end(), name);
    if (it == h.end()) {
      err << "\n  '" << name << "' is not a header column";
      continue;
    }
    const auto& got = r[static_cast<std::size_t>(it - h.begin())];
    if (got != value)
      err << "\n  " << name << ": row has '" << got << "', expected '" << value << "'";
  }
  for (const auto& name : h)
    if (want.count(name) == 0)
      err << "\n  header column '" << name << "' is not asserted";
  if (strict) {
    std::map<std::string, std::string> owner;  // printed value → column
    for (const auto& [name, value] : expected) {
      if (std::find(bools.begin(), bools.end(), name) != bools.end())
        continue;
      if (value == "0" || value == "1")
        err << "\n  non-bool '" << name << "' expects '" << value << "' (reads as a bool)";
      const auto [it, fresh] = owner.emplace(value, name);
      if (!fresh)
        err << "\n  '" << name << "' and '" << it->second << "' both expect '" << value
            << "' (a swap between them would not show)";
    }
  }
  if (!err.str().empty())
    return ::testing::AssertionFailure() << err.str() << "\nheader: " << hdr << "\nrow: " << row;
  return ::testing::AssertionSuccess();
}

// One bool walk step: the cells `on` must read "1" after `set` is applied to
// the all-false base pod, every other bool cell "0".
template <class Pod>
struct BoolCase {
  std::vector<std::string> on;
  std::function<void(Pod&)> set;
};

// `write_row(pod)` returns the row. The base must print every bool as "0",
// each case must light exactly its `on` cells, and every bool must be lit by
// some case — two bool statements that trade places then light the wrong cell.
template <class Pod, class WriteRow>
::testing::AssertionResult BoolsAreOneHot(const std::string& hdr, const Pod& base,
                                          const WriteRow& write_row,
                                          const std::vector<std::string>& bools,
                                          const std::vector<BoolCase<Pod>>& cases) {
  std::ostringstream err;
  const auto check = [&](const std::string& label, const std::string& row,
                         const std::vector<std::string>& on) {
    const auto aligned = HeaderRowAligned(hdr, row);
    if (!aligned) {
      err << "\n  " << label << ": " << aligned.message();
      return;
    }
    for (const auto& name : bools) {
      const bool lit = std::find(on.begin(), on.end(), name) != on.end();
      const auto got = ColumnValue(hdr, row, name);
      if (got != (lit ? "1" : "0"))
        err << "\n  " << label << ": '" << name << "' reads '" << got << "', expected "
            << (lit ? "1" : "0");
    }
  };
  check("base", write_row(base), {});
  std::vector<std::string> covered;
  for (const auto& c : cases) {
    Pod pod = base;
    c.set(pod);
    std::string label = "case";
    for (const auto& name : c.on)
      label += " " + name;
    check(label, write_row(pod), c.on);
    covered.insert(covered.end(), c.on.begin(), c.on.end());
  }
  for (const auto& name : bools)
    if (std::find(covered.begin(), covered.end(), name) == covered.end())
      err << "\n  bool '" << name << "' is never lit";
  if (!err.str().empty())
    return ::testing::AssertionFailure() << err.str() << "\nheader: " << hdr;
  return ::testing::AssertionSuccess();
}

// Gives each field a value and records the cell the row must print for it.
// Real() hands out 100.25, 100.75, ... — always x.25 / x.75, so a floating cell
// never prints like an integer one; Int() takes its value from a range the
// caller keeps disjoint (and away from 0 / 1).
struct CellBuilder {
  Cells want;
  double next{100.25};

  template <class T>
  void Real(const std::string& name, T& field) {
    field = static_cast<T>(next);
    next += 0.5;
    want.emplace_back(name, Str(field));
  }

  template <class T>
  void Int(const std::string& name, T& field, long long value) {
    field = static_cast<T>(value);
    want.emplace_back(name, std::to_string(value));
  }

  void Bool(const std::string& name) { want.emplace_back(name, "0"); }
};

// A writer-test pod and the cells its row must print.
template <class Pod>
struct PodCells {
  Pod pod{};
  Cells want;
};

// One BoolCase per (column, bool member).
template <class Pod>
std::vector<BoolCase<Pod>> BoolMemberCases(
    const std::vector<std::pair<std::string, bool Pod::*>>& members) {
  std::vector<BoolCase<Pod>> cases;
  for (const auto& [name, member] : members)
    cases.push_back({{name}, [member = member](Pod& p) { p.*member = true; }});
  return cases;
}

}  // namespace

// ── WbcDiagLogPod, every column (#883) ──

namespace {

constexpr std::size_t kWbcDiagVars = 12;

std::string WbcDiagHeader() {
  std::ostringstream os;
  integrated_bringup::WriteWbcDiagLogHeader(os, kWbcDiagVars);
  return os.str();
}

std::string WbcDiagRow(const integrated_bringup::WbcDiagLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteWbcDiagLogRow(os, pod, kWbcDiagVars);
  return os.str();
}

// Every non-bool field distinct; integers from 2001 up, floating from 100.25 up.
integrated_bringup::WbcDiagLogPod WbcDiagBase() {
  integrated_bringup::WbcDiagLogPod pod{};
  pod.t_relative_s = 100.25;
  pod.phase = 3;
  pod.num_contact_vars = static_cast<std::uint8_t>(kWbcDiagVars);
  pod.solve_time_us = 100.5;
  pod.solve_levels = 2001;
  pod.qp_fail_count = 2002;
  pod.kin_qp_fail_count = 2003;
  pod.num_active_contacts = 2004;
  pod.max_force = 100.75f;
  pod.clik_tcp_err = 101.25;
  pod.clik_manipulability = 101.5;
  pod.qp_status = 2005;
  pod.qp_iterations = 2006;
  pod.joint_box_inverted = 2007;
  for (std::size_t i = 0; i < kWbcDiagVars; ++i)
    pod.lambda_opt[i] = 200.25 + 0.25 * static_cast<double>(i);
  return pod;
}

const std::vector<std::string> kWbcDiagBools = {"qp_converged", "grasp_detected", "clik_valid"};

}  // namespace

TEST(WbcDiagLogPod, EveryColumnByName) {
  Cells want = {{"t_relative_s", "100.25"},
                {"phase", "closure"},
                {"solve_time_us", "100.5"},
                {"qp_converged", "0"},
                {"solve_levels", "2001"},
                {"qp_fail_count", "2002"},
                {"kin_qp_fail_count", "2003"},
                {"num_active_contacts", "2004"},
                {"grasp_detected", "0"},
                {"max_force", "100.75"},
                {"clik_valid", "0"},
                {"clik_tcp_err", "101.25"},
                {"clik_manipulability", "101.5"},
                {"qp_status", "2005"},
                {"qp_iterations", "2006"},
                {"joint_box_inverted", "2007"},
                {"num_contact_vars", "12"}};
  for (std::size_t i = 0; i < kWbcDiagVars; ++i)
    want.emplace_back("lambda_" + std::to_string(i), Str(200.25 + 0.25 * static_cast<double>(i)));
  EXPECT_TRUE(RowMatchesByName(WbcDiagHeader(), WbcDiagRow(WbcDiagBase()), want, kWbcDiagBools,
                               /*strict=*/true));
}

TEST(WbcDiagLogPod, BoolColumnsAreOneHot) {
  using Pod = integrated_bringup::WbcDiagLogPod;
  EXPECT_TRUE(BoolsAreOneHot<Pod>(WbcDiagHeader(), WbcDiagBase(), WbcDiagRow, kWbcDiagBools,
                                  {{{"qp_converged"}, [](Pod& p) { p.qp_converged = true; }},
                                   {{"grasp_detected"}, [](Pod& p) { p.grasp_detected = true; }},
                                   {{"clik_valid"}, [](Pod& p) { p.clik_valid = true; }}}));
}

// ── DeviceStateLogPod, every column (#883) ──
//
// No bool columns, so no OneHot test: command_type / goal_type print as names.

namespace {

// 3 joints, 4 motors: the widths differ so a joint block read as a motor block
// (or the reverse) shows.
constexpr std::size_t kStateWbcJoints = 3;
constexpr std::size_t kStateWbcMotors = 4;

// Value of element `i` of array block `block`: floating, > 200, and unique over
// every (block, i) a row below holds.
double StateWbcValue(int block, std::size_t i) {
  return 200.25 + 10.0 * block + 0.25 * static_cast<double>(i);
}

template <std::size_t N>
void StateWbcFill(std::array<double, N>& a, int block, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i)
    a[i] = StateWbcValue(block, i);
}

// Expected cells of `prefix + suffix[i]` = StateWbcValue(block, i).
void StateWbcAdd(Cells& want, const std::string& prefix, const std::vector<std::string>& suffix,
                 int block) {
  for (std::size_t i = 0; i < suffix.size(); ++i)
    want.emplace_back(prefix + suffix[i], Str(StateWbcValue(block, i)));
}

const std::vector<std::string> kStateWbcTaskAxes = {"x", "y", "z", "roll", "pitch", "yaw"};

std::vector<std::string> StateWbcNames(const std::string& stem, std::size_t n) {
  std::vector<std::string> names;
  for (std::size_t i = 0; i < n; ++i)
    names.push_back(stem + std::to_string(i));
  return names;
}

const std::vector<std::string> kStateJointNames = StateWbcNames("jnt", kStateWbcJoints);
const std::vector<std::string> kStateMotorNames = StateWbcNames("mot", kStateWbcMotors);

integrated_bringup::DeviceStateLogColumns StateCols() {
  return integrated_bringup::DeviceStateLogColumnsFor(kStateJointNames, kStateMotorNames);
}

std::string StateHeader() {
  std::ostringstream os;
  integrated_bringup::WriteDeviceStateLogHeader(os, kStateJointNames, kStateMotorNames,
                                                StateCols());
  return os.str();
}

std::string StateRow(const integrated_bringup::DeviceStateLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteDeviceStateLogRow(os, pod, StateCols());
  return os.str();
}

// Block numbers: 0..6 joint arrays in header order, 7 task_goal, 8 task_pos,
// 9..11 motor arrays. `task_filled` false → the task cells hold the default 0.
Cells StateExpected(bool task_filled) {
  Cells want = {{"t_relative_s", "100.25"},
                {"command_type", "pd_feedforward"},
                {"goal_type", "task"},
                {"num_joints", Str(kStateWbcJoints)},
                {"num_motors", Str(kStateWbcMotors)}};
  StateWbcAdd(want, "actual_pos_", kStateJointNames, 0);
  StateWbcAdd(want, "actual_vel_", kStateJointNames, 1);
  StateWbcAdd(want, "effort_", kStateJointNames, 2);
  StateWbcAdd(want, "command_", kStateJointNames, 3);
  StateWbcAdd(want, "joint_goal_", kStateJointNames, 4);
  StateWbcAdd(want, "traj_pos_", kStateJointNames, 5);
  StateWbcAdd(want, "traj_vel_", kStateJointNames, 6);
  if (task_filled) {
    StateWbcAdd(want, "task_goal_", kStateWbcTaskAxes, 7);
    StateWbcAdd(want, "task_pos_", kStateWbcTaskAxes, 8);
  } else {
    for (const auto& axis : kStateWbcTaskAxes) {
      want.emplace_back("task_goal_" + axis, "0");
      want.emplace_back("task_pos_" + axis, "0");
    }
  }
  StateWbcAdd(want, "motor_pos_", kStateMotorNames, 9);
  StateWbcAdd(want, "motor_vel_", kStateMotorNames, 10);
  StateWbcAdd(want, "motor_eff_", kStateMotorNames, 11);
  return want;
}

integrated_bringup::DeviceStateLogPod StateBase() {
  integrated_bringup::DeviceStateLogPod pod{};
  pod.t_relative_s = 100.25;
  pod.num_joints = static_cast<std::uint8_t>(kStateWbcJoints);
  pod.num_motors = static_cast<std::uint8_t>(kStateWbcMotors);
  StateWbcFill(pod.actual_positions, 0, kStateWbcJoints);
  StateWbcFill(pod.actual_velocities, 1, kStateWbcJoints);
  StateWbcFill(pod.efforts, 2, kStateWbcJoints);
  StateWbcFill(pod.commands, 3, kStateWbcJoints);
  StateWbcFill(pod.joint_goal, 4, kStateWbcJoints);
  StateWbcFill(pod.trajectory_positions, 5, kStateWbcJoints);
  StateWbcFill(pod.trajectory_velocities, 6, kStateWbcJoints);
  StateWbcFill(pod.task_goal, 7, integrated_bringup::DeviceStateLogPod::kTaskDim);
  StateWbcFill(pod.actual_task_positions, 8, integrated_bringup::DeviceStateLogPod::kTaskDim);
  StateWbcFill(pod.motor_positions, 9, kStateWbcMotors);
  StateWbcFill(pod.motor_velocities, 10, kStateWbcMotors);
  StateWbcFill(pod.motor_efforts, 11, kStateWbcMotors);
  pod.command_type = 2;
  pod.goal_type = 1;
  return pod;
}

// A controller state and output carrying the same values as StateBase(), on
// device `device_idx`, for the real Fill.
void StateFillSources(std::size_t device_idx, rtc::ControllerState& state,
                      rtc::ControllerOutput& output) {
  state.t_relative_s = 100.25;
  state.num_devices = static_cast<int>(device_idx) + 1;
  output.num_devices = state.num_devices;
  auto& dev = state.devices[device_idx];
  auto& out = output.devices[device_idx];
  dev.num_channels = static_cast<int>(kStateWbcJoints);
  dev.num_motor_channels = static_cast<int>(kStateWbcMotors);
  StateWbcFill(dev.positions, 0, kStateWbcJoints);
  StateWbcFill(dev.velocities, 1, kStateWbcJoints);
  StateWbcFill(dev.efforts, 2, kStateWbcJoints);
  StateWbcFill(out.commands, 3, kStateWbcJoints);
  StateWbcFill(out.goal_positions, 4, kStateWbcJoints);
  StateWbcFill(out.trajectory_positions, 5, kStateWbcJoints);
  StateWbcFill(out.trajectory_velocities, 6, kStateWbcJoints);
  StateWbcFill(output.task_goal_positions, 7, kStateWbcTaskAxes.size());
  StateWbcFill(output.actual_task_positions, 8, kStateWbcTaskAxes.size());
  StateWbcFill(dev.motor_positions, 9, kStateWbcMotors);
  StateWbcFill(dev.motor_velocities, 10, kStateWbcMotors);
  StateWbcFill(dev.motor_efforts, 11, kStateWbcMotors);
  out.command_type = rtc::CommandType::kPdFeedforward;
  out.goal_type = rtc::GoalType::kTask;
}

}  // namespace

TEST(DeviceStateLogPod, EveryColumnByName) {
  EXPECT_TRUE(RowMatchesByName(StateHeader(), StateRow(StateBase()), StateExpected(true), {},
                               /*strict=*/true));
}

// Not strict: the Fill's cells are checked by name against the same values the
// writer test pins, so a Fill that routes a field to the wrong pod array reads
// a wrong cell.
TEST(DeviceStateLogPod, FillRoundTripsEveryColumnByName) {
  rtc::ControllerState state{};
  rtc::ControllerOutput output{};
  StateFillSources(0, state, output);
  integrated_bringup::DeviceStateLogPod pod{};
  integrated_bringup::FillDeviceStateLogPod(state, output, 0, pod);
  EXPECT_TRUE(RowMatchesByName(StateHeader(), StateRow(pod), StateExpected(true), {},
                               /*strict=*/false));
}

// Only the primary device (index 0) fills the task-space cells; a secondary
// device's row keeps the pod's zeros there and everything else is unchanged.
TEST(DeviceStateLogPod, FillSecondaryDeviceLeavesTaskCellsZeroByName) {
  rtc::ControllerState state{};
  rtc::ControllerOutput output{};
  StateFillSources(1, state, output);
  integrated_bringup::DeviceStateLogPod pod{};
  integrated_bringup::FillDeviceStateLogPod(state, output, 1, pod);
  EXPECT_TRUE(RowMatchesByName(StateHeader(), StateRow(pod), StateExpected(false), {},
                               /*strict=*/false));
}

// ── DeviceWbcLogPod, every column (#883) ──
//
// No bool columns, so no OneHot test: command_type / goal_type print as names.

namespace {

// Hand role: joints, motors and fingertips all differ.
constexpr std::size_t kWbcHandTips = 5;

const std::vector<std::string> kWbcJointNames = StateWbcNames("jnt", kStateWbcJoints);
const std::vector<std::string> kWbcMotorNames = StateWbcNames("mot", kStateWbcMotors);
// Fewer names than the 8 fixed slots: the rest are labelled by index.
const std::vector<std::string> kWbcTipNames = StateWbcNames("tip", kWbcHandTips);

integrated_bringup::DeviceWbcLogColumns WbcCols() {
  return integrated_bringup::DeviceWbcLogColumnsFor(kWbcJointNames, kWbcMotorNames);
}

std::string WbcHeader(std::uint8_t role) {
  std::ostringstream os;
  integrated_bringup::WriteDeviceWbcLogHeader(os, role, kWbcJointNames, kWbcMotorNames,
                                              kWbcTipNames, WbcCols());
  return os.str();
}

std::string WbcRow(const integrated_bringup::DeviceWbcLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteDeviceWbcLogRow(os, pod, WbcCols());
  return os.str();
}

// The 8 fingertip slot labels as the header writes them.
std::vector<std::string> WbcTipSlots() {
  std::vector<std::string> slots = kWbcTipNames;
  for (std::size_t i = slots.size(); i < integrated_bringup::DeviceWbcLogPod::kMaxFingertips; ++i)
    slots.push_back(std::to_string(i));
  return slots;
}

// Joint blocks 0..7 in header order (actual_pos, actual_vel, effort, accel,
// command, joint_goal, traj_pos, traj_vel); command_type / goal_type as names.
integrated_bringup::DeviceWbcLogPod WbcBase(std::uint8_t role) {
  integrated_bringup::DeviceWbcLogPod pod{};
  pod.role = role;
  pod.t_relative_s = 100.25;
  pod.num_joints = static_cast<std::uint8_t>(kStateWbcJoints);
  StateWbcFill(pod.actual_positions, 0, kStateWbcJoints);
  StateWbcFill(pod.actual_velocities, 1, kStateWbcJoints);
  StateWbcFill(pod.efforts, 2, kStateWbcJoints);
  StateWbcFill(pod.accelerations, 3, kStateWbcJoints);
  StateWbcFill(pod.commands, 4, kStateWbcJoints);
  StateWbcFill(pod.joint_goal, 5, kStateWbcJoints);
  StateWbcFill(pod.trajectory_positions, 6, kStateWbcJoints);
  StateWbcFill(pod.trajectory_velocities, 7, kStateWbcJoints);
  pod.command_type = 1;
  pod.goal_type = 1;
  return pod;
}

Cells WbcJointExpected() {
  Cells want = {{"t_relative_s", "100.25"},
                {"command_type", "torque"},
                {"goal_type", "task"},
                {"num_joints", Str(kStateWbcJoints)}};
  StateWbcAdd(want, "actual_pos_", kWbcJointNames, 0);
  StateWbcAdd(want, "actual_vel_", kWbcJointNames, 1);
  StateWbcAdd(want, "effort_", kWbcJointNames, 2);
  StateWbcAdd(want, "accel_", kWbcJointNames, 3);
  StateWbcAdd(want, "command_", kWbcJointNames, 4);
  StateWbcAdd(want, "joint_goal_", kWbcJointNames, 5);
  StateWbcAdd(want, "traj_pos_", kWbcJointNames, 6);
  StateWbcAdd(want, "traj_vel_", kWbcJointNames, 7);
  return want;
}

}  // namespace

TEST(DeviceWbcLogPod, ArmRoleEveryColumnByName) {
  using Pod = integrated_bringup::DeviceWbcLogPod;
  Pod pod = WbcBase(0);
  StateWbcFill(pod.task_goal, 8, Pod::kTaskDim);
  StateWbcFill(pod.actual_task_positions, 9, Pod::kTaskDim);
  StateWbcFill(pod.trajectory_task_positions, 10, Pod::kTaskDim);
  StateWbcFill(pod.trajectory_task_velocities, 11, Pod::kTaskDim);

  Cells want = WbcJointExpected();
  StateWbcAdd(want, "task_goal_", kStateWbcTaskAxes, 8);
  StateWbcAdd(want, "task_pos_", kStateWbcTaskAxes, 9);
  StateWbcAdd(want, "traj_task_pos_", kStateWbcTaskAxes, 10);
  StateWbcAdd(want, "traj_task_vel_", kStateWbcTaskAxes, 11);
  EXPECT_TRUE(RowMatchesByName(WbcHeader(0), WbcRow(pod), want, {}, /*strict=*/true));
}

TEST(DeviceWbcLogPod, HandRoleEveryColumnByName) {
  using Pod = integrated_bringup::DeviceWbcLogPod;
  Pod pod = WbcBase(1);
  pod.num_motors = static_cast<std::uint8_t>(kStateWbcMotors);
  pod.num_fingertips = static_cast<std::uint8_t>(kWbcHandTips);
  StateWbcFill(pod.motor_positions, 8, kStateWbcMotors);
  StateWbcFill(pod.motor_velocities, 9, kStateWbcMotors);
  StateWbcFill(pod.motor_efforts, 10, kStateWbcMotors);
  // All 8 slots of each fingertip block get a value, past num_fingertips too:
  // the writer emits the whole fixed-width block.
  StateWbcFill(pod.fingertip_force, 11, Pod::kMaxFingertips);
  StateWbcFill(pod.fingertip_force_x, 12, Pod::kMaxFingertips);
  StateWbcFill(pod.fingertip_force_y, 13, Pod::kMaxFingertips);
  StateWbcFill(pod.fingertip_force_z, 14, Pod::kMaxFingertips);

  Cells want = WbcJointExpected();
  want.emplace_back("num_motors", Str(kStateWbcMotors));
  want.emplace_back("num_fingertips", Str(kWbcHandTips));
  StateWbcAdd(want, "motor_pos_", kWbcMotorNames, 8);
  StateWbcAdd(want, "motor_vel_", kWbcMotorNames, 9);
  StateWbcAdd(want, "motor_eff_", kWbcMotorNames, 10);
  const auto slots = WbcTipSlots();
  StateWbcAdd(want, "fingertip_force_", slots, 11);
  StateWbcAdd(want, "fingertip_fx_", slots, 12);
  StateWbcAdd(want, "fingertip_fy_", slots, 13);
  StateWbcAdd(want, "fingertip_fz_", slots, 14);
  EXPECT_TRUE(RowMatchesByName(WbcHeader(1), WbcRow(pod), want, {}, /*strict=*/true));
}

// ── DeviceSensorLogPod, every column (#883) ──

namespace {

constexpr std::size_t kDeviceSensorFingertips = 3;

const std::vector<std::string>& DeviceSensorNames() {
  static const std::vector<std::string> names = {"thumb", "index", "middle"};
  return names;
}

std::string DeviceSensorHeader(std::size_t stride) {
  const auto cols = integrated_bringup::DeviceSensorLogColumnsFor(DeviceSensorNames(), stride);
  std::ostringstream os;
  integrated_bringup::WriteDeviceSensorLogHeader(os, DeviceSensorNames(), cols);
  return os.str();
}

std::string DeviceSensorRow(const integrated_bringup::DeviceSensorLogPod& pod, std::size_t stride) {
  const auto cols = integrated_bringup::DeviceSensorLogColumnsFor(DeviceSensorNames(), stride);
  std::ostringstream os;
  integrated_bringup::WriteDeviceSensorLogRow(os, pod, cols);
  return os.str();
}

// Sensor lanes packed at `stride` (the layout stride the writer indexes with);
// every slot a distinct value. Integers from 3001/4001 up, floating from
// 100.25 up, bools false.
integrated_bringup::DeviceSensorLogPod DeviceSensorBase(std::size_t stride) {
  integrated_bringup::DeviceSensorLogPod pod{};
  pod.t_relative_s = 99.25;
  pod.num_fingertips = static_cast<std::uint8_t>(kDeviceSensorFingertips);
  for (std::size_t i = 0; i < kDeviceSensorFingertips * stride; ++i) {
    pod.sensor_data_raw[i] = 3001 + static_cast<std::int32_t>(i);
    pod.sensor_data[i] = 4001 + static_cast<std::int32_t>(i);
  }
  for (std::size_t i = 0;
       i < kDeviceSensorFingertips * integrated_bringup::DeviceSensorLogPod::kFTValuesPerFingertip;
       ++i)
    pod.inference_output[i] = 100.25F + 0.25F * static_cast<float>(i);
  for (std::size_t i = 0;
       i < kDeviceSensorFingertips * integrated_bringup::DeviceSensorLogPod::kForceAxes; ++i) {
    pod.force_filtered[i] = 200.25F + 0.25F * static_cast<float>(i);
    pod.force_guarded[i] = 300.25F + 0.25F * static_cast<float>(i);
  }
  return pod;
}

const std::vector<std::string> kDeviceSensorBools = [] {
  std::vector<std::string> bools = {"inference_valid", "force_filtered_valid"};
  for (const auto& name : DeviceSensorNames())
    bools.push_back("ft_" + name + "_force_guard_rejected");
  return bools;
}();

// Expected cells for DeviceSensorBase(stride); `bools` = inference_valid,
// force_filtered_valid, then force_guard_rejected per tip.
Cells DeviceSensorWant(std::size_t stride, const std::array<const char*, 5>& bools) {
  namespace ib = integrated_bringup;
  Cells want = {{"t_relative_s", "99.25"},
                {"inference_valid", bools[0]},
                {"force_filtered_valid", bools[1]},
                {"num_fingertips", "3"}};
  const auto& names = DeviceSensorNames();
  for (std::size_t f = 0; f < names.size(); ++f) {
    for (std::size_t v = 0; v < stride; ++v) {
      const auto i = static_cast<int>(f * stride + v);
      want.emplace_back(names[f] + "_raw_" + std::to_string(v), std::to_string(3001 + i));
      want.emplace_back(names[f] + "_filt_" + std::to_string(v), std::to_string(4001 + i));
    }
    static constexpr std::array<const char*, ib::DeviceSensorLogPod::kFTValuesPerFingertip> kFt{
        "contact", "fx", "fy", "fz", "ux", "uy", "uz"};
    for (std::size_t v = 0; v < kFt.size(); ++v)
      want.emplace_back("ft_" + names[f] + "_" + kFt[v],
                        Str(100.25F + 0.25F * static_cast<float>(f * kFt.size() + v)));
    static constexpr std::array<const char*, 3> kAxes{"fx", "fy", "fz"};
    for (std::size_t a = 0; a < kAxes.size(); ++a) {
      const auto k = static_cast<float>(f * kAxes.size() + a);
      want.emplace_back("ft_" + names[f] + "_" + kAxes[a] + "_filt", Str(200.25F + 0.25F * k));
      want.emplace_back("ft_" + names[f] + "_" + kAxes[a] + "_guarded", Str(300.25F + 0.25F * k));
    }
    want.emplace_back("ft_" + names[f] + "_force_guard_rejected", bools[2 + f]);
  }
  return want;
}

constexpr std::array<const char*, 5> kDeviceSensorAllFalse = {"0", "0", "0", "0", "0"};

}  // namespace

TEST(DeviceSensorLogPod, EveryColumnByName) {
  constexpr std::size_t kStride = integrated_bringup::DeviceSensorLogPod::kSensorValuesPerFingertip;
  EXPECT_TRUE(RowMatchesByName(
      DeviceSensorHeader(kStride), DeviceSensorRow(DeviceSensorBase(kStride), kStride),
      DeviceSensorWant(kStride, kDeviceSensorAllFalse), kDeviceSensorBools, /*strict=*/true));
}

// A layout stride below the capacity: with stride == capacity an `f * stride + v`
// indexing slip in the writer reads the same slot and stays invisible.
TEST(DeviceSensorLogPod, EveryColumnByNameStride3) {
  constexpr std::size_t kStride = 3;
  static_assert(kStride < integrated_bringup::DeviceSensorLogPod::kSensorValuesPerFingertip);
  EXPECT_TRUE(RowMatchesByName(
      DeviceSensorHeader(kStride), DeviceSensorRow(DeviceSensorBase(kStride), kStride),
      DeviceSensorWant(kStride, kDeviceSensorAllFalse), kDeviceSensorBools, /*strict=*/true));
}

TEST(DeviceSensorLogPod, BoolColumnsAreOneHot) {
  using Pod = integrated_bringup::DeviceSensorLogPod;
  constexpr std::size_t kStride = Pod::kSensorValuesPerFingertip;
  std::vector<BoolCase<Pod>> cases = {
      {{"inference_valid"}, [](Pod& p) { p.inference_valid = true; }},
      {{"force_filtered_valid"}, [](Pod& p) { p.force_filtered_valid = true; }}};
  const auto& names = DeviceSensorNames();
  for (std::size_t f = 0; f < names.size(); ++f)
    cases.push_back({{"ft_" + names[f] + "_force_guard_rejected"},
                     [f](Pod& p) { p.force_guard_rejected[f] = 1; }});
  EXPECT_TRUE(BoolsAreOneHot<Pod>(
      DeviceSensorHeader(kStride), DeviceSensorBase(kStride),
      [](const Pod& p) { return DeviceSensorRow(p, kStride); }, kDeviceSensorBools, cases));
}

// The real Fill, then every column. Fill derives inference_valid from the
// per-group enable flags and force_filtered_valid from a non-empty filtered
// span; the verdict lane {0,1,0} puts the "1" on the index tip.
TEST(DeviceSensorLogPod, FillRoundTripsEveryColumnByName) {
  namespace ib = integrated_bringup;
  constexpr std::size_t kStride = ib::DeviceSensorLogPod::kSensorValuesPerFingertip;
  auto state = std::make_unique<rtc::ControllerState>();
  state->num_devices = 2;
  state->t_relative_s = 99.25;
  auto& dev = state->devices[1];
  dev.inference_enable[1] = true;
  for (std::size_t i = 0; i < kDeviceSensorFingertips * kStride; ++i) {
    dev.sensor_data_raw[i] = 3001 + static_cast<std::int32_t>(i);
    dev.sensor_data[i] = 4001 + static_cast<std::int32_t>(i);
  }
  for (std::size_t i = 0;
       i < kDeviceSensorFingertips * ib::DeviceSensorLogPod::kFTValuesPerFingertip; ++i)
    dev.inference_data[i] = 100.25F + 0.25F * static_cast<float>(i);
  std::array<float, 9> filtered{};
  std::array<float, 9> guarded{};
  for (std::size_t i = 0; i < filtered.size(); ++i) {
    filtered[i] = 200.25F + 0.25F * static_cast<float>(i);
    guarded[i] = 300.25F + 0.25F * static_cast<float>(i);
  }
  const std::array<std::uint8_t, 3> rejected = {0, 1, 0};

  ib::DeviceSensorLogPod pod{};
  ib::FillDeviceSensorLogPod(*state, 1, static_cast<int>(kDeviceSensorFingertips),
                             ib::ForceFilterLogView{filtered, guarded, rejected}, pod);

  EXPECT_TRUE(RowMatchesByName(DeviceSensorHeader(kStride), DeviceSensorRow(pod, kStride),
                               DeviceSensorWant(kStride, {"1", "1", "0", "1", "0"}),
                               kDeviceSensorBools,
                               /*strict=*/false));
}

// ── PullEstimatorLogPod, every column (#883) ──

namespace {

const std::vector<std::string>& PullRoles() {
  static const std::vector<std::string> roles = {"thumb", "index", "middle"};
  return roles;
}

std::string PullHeader() {
  std::ostringstream os;
  integrated_bringup::WritePullEstimatorLogHeader(os, PullRoles());
  return os.str();
}

std::string PullRow(const integrated_bringup::PullEstimatorLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WritePullEstimatorLogRow(os, pod);
  return os.str();
}

// Floating columns from 100.25 up (float-typed fields stream as float), ints
// from 2001, masks and enum bytes small and distinct (uint8 prints as a number).
integrated_bringup::PullEstimatorLogPod PullBase() {
  integrated_bringup::PullEstimatorLogPod pod{};
  auto& e = pod.estimate;
  pod.t_relative_s = 99.25;
  pod.tick = 2001;
  pod.force_prefilter = {100.25F, 100.5F, 100.75F};
  e.force = {101.25F, 101.5F, 101.75F};
  e.force_inplane = {102.25F, 102.5F};
  e.plane_normal = {103.25F, 103.5F, 103.75F};
  e.basis_x = {104.25F, 104.5F, 104.75F};
  e.basis_source = 3;
  e.invalid_reason = 4;
  e.magnitude = 105.25F;
  e.directional = 105.5F;
  e.friction_utilization = 105.75F;
  e.leakage_bound = 106.25F;
  e.valid_contact_count = 2002;
  e.contact_mask = 7;
  e.touch_mask = 11;
  pod.opposing_mask = 13;
  return pod;
}

const std::vector<std::string> kPullBools = {"valid", "slip_risk", "any_saturated",
                                             "baseline_applied"};

// `bools` = valid, slip_risk, any_saturated, baseline_applied.
Cells PullWant(const std::array<const char*, 4>& bools) {
  const std::string m = "[thumb|index|middle]";
  return {{"t_relative_s", "99.25"},
          {"tick", "2001"},
          {"force_prefilter_x", "100.25"},
          {"force_prefilter_y", "100.5"},
          {"force_prefilter_z", "100.75"},
          {"force_x", "101.25"},
          {"force_y", "101.5"},
          {"force_z", "101.75"},
          {"inplane_x", "102.25"},
          {"inplane_y", "102.5"},
          {"plane_normal_x", "103.25"},
          {"plane_normal_y", "103.5"},
          {"plane_normal_z", "103.75"},
          {"basis_x_x", "104.25"},
          {"basis_x_y", "104.5"},
          {"basis_x_z", "104.75"},
          {"basis_source", "3"},
          {"invalid_reason", "4"},
          {"magnitude", "105.25"},
          {"directional", "105.5"},
          {"friction_utilization", "105.75"},
          {"leakage_bound", "106.25"},
          {"valid_contact_count", "2002"},
          {"valid", bools[0]},
          {"slip_risk", bools[1]},
          {"any_saturated", bools[2]},
          {"baseline_applied", bools[3]},
          {"contact_mask" + m, "7"},
          {"touch_mask" + m, "11"},
          {"opposing_mask" + m, "13"}};
}

}  // namespace

TEST(PullEstimatorLogPod, EveryColumnByName) {
  EXPECT_TRUE(RowMatchesByName(PullHeader(), PullRow(PullBase()), PullWant({"0", "0", "0", "0"}),
                               kPullBools, /*strict=*/true));
}

TEST(PullEstimatorLogPod, BoolColumnsAreOneHot) {
  using Pod = integrated_bringup::PullEstimatorLogPod;
  EXPECT_TRUE(BoolsAreOneHot<Pod>(
      PullHeader(), PullBase(), PullRow, kPullBools,
      {{{"valid"}, [](Pod& p) { p.estimate.valid = true; }},
       {{"slip_risk"}, [](Pod& p) { p.estimate.slip_risk = true; }},
       {{"any_saturated"}, [](Pod& p) { p.estimate.any_saturated = true; }},
       {{"baseline_applied"}, [](Pod& p) { p.estimate.baseline_applied = true; }}}));
}

// The real Fill, then every column. Fill does not write opposing_mask (the
// wiring stamps it), so it is set on the pod beforehand. The bool pattern
// 1,0,1,0 makes every neighbouring pair differ.
TEST(PullEstimatorLogPod, FillRoundTripsEveryColumnByName) {
  rtc::grasp::PullEstimate est{};
  est.force_raw = Eigen::Vector3d(100.25, 100.5, 100.75);
  est.force_filtered = Eigen::Vector3d(101.25, 101.5, 101.75);
  est.force_inplane = Eigen::Vector2d(102.25, 102.5);
  est.plane_normal = Eigen::Vector3d(103.25, 103.5, 103.75);
  est.basis_x = Eigen::Vector3d(104.25, 104.5, 104.75);
  est.basis_source = rtc::grasp::PullBasisSource::kSeed;
  est.invalid_reason = rtc::grasp::PullInvalidReason::kInsufficientContacts;
  est.contact_mask = 7;
  est.touch_mask = 11;
  est.magnitude = 105.25;
  est.directional = 105.5;
  est.max_friction_utilization = 105.75;
  est.leakage_bound = 106.25;
  est.valid_contact_count = 2002;
  est.valid = true;
  est.slip_risk = false;
  est.any_saturated = true;
  est.baseline_applied = false;

  integrated_bringup::PullEstimatorLogPod pod{};
  pod.opposing_mask = 13;
  integrated_bringup::FillPullEstimatorLogPod(est, 99.25, /*tick=*/2001, pod);

  EXPECT_TRUE(RowMatchesByName(PullHeader(), PullRow(pod), PullWant({"1", "0", "1", "0"}),
                               kPullBools, /*strict=*/false));
}

// ── TaskDiagLogPod, every column (#883) ──

namespace {

std::string TaskDiagHeader() {
  std::ostringstream os;
  integrated_bringup::WriteTaskDiagLogHeader(os);
  return os.str();
}

std::string TaskDiagRow(const integrated_bringup::TaskDiagLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteTaskDiagLogRow(os, pod);
  return os.str();
}

// lambda_sq is negative on purpose: damping_active is derived as
// `valid && lambda_sq > 0`, so a positive value would light it with `valid`.
integrated_bringup::TaskDiagLogPod TaskDiagBase() {
  integrated_bringup::TaskDiagLogPod pod{};
  pod.t_relative_s = 100.25;
  pod.sigma_min = 100.5;
  pod.lambda_sq = -100.75;
  pod.sigma0 = 101.25;
  pod.lambda_max = 101.5;
  return pod;
}

const std::vector<std::string> kTaskDiagBools = {"valid", "ik_ok", "control_6dof",
                                                 "damping_active"};

}  // namespace

TEST(TaskDiagLogPod, EveryColumnByName) {
  const Cells want = {{"t_relative_s", "100.25"},
                      {"valid", "0"},
                      {"sigma_min", "100.5"},
                      {"lambda_sq", "-100.75"},
                      {"sigma0", "101.25"},
                      {"lambda_max", "101.5"},
                      {"ik_ok", "0"},
                      {"control_6dof", "0"},
                      {"damping_active", "0"}};
  EXPECT_TRUE(RowMatchesByName(TaskDiagHeader(), TaskDiagRow(TaskDiagBase()), want, kTaskDiagBools,
                               /*strict=*/true));
}

TEST(TaskDiagLogPod, BoolColumnsAreOneHot) {
  using Pod = integrated_bringup::TaskDiagLogPod;
  EXPECT_TRUE(
      BoolsAreOneHot<Pod>(TaskDiagHeader(), TaskDiagBase(), TaskDiagRow, kTaskDiagBools,
                          {{{"valid"}, [](Pod& p) { p.valid = true; }},
                           {{"ik_ok"}, [](Pod& p) { p.ik_ok = true; }},
                           {{"control_6dof"}, [](Pod& p) { p.control_6dof = true; }},
                           // damping_active is derived: it needs a solved tick and a positive λ².
                           {{"valid", "damping_active"}, [](Pod& p) {
                              p.valid = true;
                              p.lambda_sq = 100.75;
                            }}}));
}

// ── GraspDiagLogPod, every column (#883) ──

namespace {

const std::vector<std::string> kGraspDiagFingers = {"thumb", "index", "middle"};

// Per-finger families in header order; float families first, then the uint8 flags.
const std::vector<std::string> kGraspDiagFloatFamilies = {"s",       "f_desired", "f_measured",
                                                          "f_error", "k_est",     "k_inst_raw",
                                                          "delta_s", "delta_f",   "gain_scale"};
const std::vector<std::string> kGraspDiagFlagFamilies = {"est_updated", "integrator_frozen",
                                                         "contact"};

std::string GraspDiagHeader() {
  std::ostringstream os;
  integrated_bringup::WriteGraspDiagLogHeader(os, kGraspDiagFingers);
  return os.str();
}

std::string GraspDiagRow(const integrated_bringup::GraspDiagLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteGraspDiagLogRow(os, pod, kGraspDiagFingers.size());
  return os.str();
}

// Float of family k, finger i: 100.25 + 0.25 * (3k + i), all distinct.
float GraspDiagFloat(std::size_t family, std::size_t finger) {
  return static_cast<float>(100.25 + 0.25 * static_cast<double>(3 * family + finger));
}

// The uint8 flag columns are printed as numbers (the writer casts to unsigned),
// so they are given distinct values >= 2 and are not bool columns. Range 211..219.
std::uint8_t GraspDiagFlag(std::size_t family, std::size_t finger) {
  return static_cast<std::uint8_t>(211 + 3 * family + finger);
}

integrated_bringup::GraspDiagLogPod GraspDiagBase() {
  integrated_bringup::GraspDiagLogPod pod{};
  pod.t_relative_s = 90.25;
  pod.tick = 2001;
  pod.grasp_phase = 201;
  pod.target_force = 90.5F;
  pod.beta = 90.75F;
  pod.alpha_ema = 91.25F;
  pod.K_est_max = 91.5F;
  pod.num_fingers = static_cast<std::uint8_t>(kGraspDiagFingers.size());
  for (std::size_t i = 0; i < kGraspDiagFingers.size(); ++i) {
    pod.s[i] = GraspDiagFloat(0, i);
    pod.f_desired[i] = GraspDiagFloat(1, i);
    pod.f_measured[i] = GraspDiagFloat(2, i);
    pod.f_error[i] = GraspDiagFloat(3, i);
    pod.k_est[i] = GraspDiagFloat(4, i);
    pod.k_inst_raw[i] = GraspDiagFloat(5, i);
    pod.delta_s[i] = GraspDiagFloat(6, i);
    pod.delta_f[i] = GraspDiagFloat(7, i);
    pod.gain_scale[i] = GraspDiagFloat(8, i);
    pod.est_updated[i] = GraspDiagFlag(0, i);
    pod.integrator_frozen[i] = GraspDiagFlag(1, i);
    pod.contact[i] = GraspDiagFlag(2, i);
  }
  return pod;
}

const std::vector<std::string> kGraspDiagBools = {"valid"};

// Appends "<family>_<finger>" → value(family index, finger index) for each family.
template <class ValueFn>
void GraspDiagAddFamilies(Cells& want, const std::vector<std::string>& families,
                          std::size_t first_family, const ValueFn& value) {
  for (std::size_t k = 0; k < families.size(); ++k)
    for (std::size_t i = 0; i < kGraspDiagFingers.size(); ++i)
      want.emplace_back(families[k] + "_" + kGraspDiagFingers[i], value(first_family + k, i));
}

}  // namespace

TEST(GraspDiagLogPod, EveryColumnByName) {
  Cells want = {{"t_relative_s", "90.25"}, {"tick", "2001"},         {"valid", "0"},
                {"grasp_phase", "201"},    {"target_force", "90.5"}, {"beta", "90.75"},
                {"alpha_ema", "91.25"},    {"K_est_max", "91.5"}};
  GraspDiagAddFamilies(want, kGraspDiagFloatFamilies, 0,
                       [](std::size_t k, std::size_t i) { return Str(GraspDiagFloat(k, i)); });
  GraspDiagAddFamilies(want, kGraspDiagFlagFamilies, 0, [](std::size_t k, std::size_t i) {
    return Str(static_cast<unsigned>(GraspDiagFlag(k, i)));
  });
  EXPECT_TRUE(RowMatchesByName(GraspDiagHeader(), GraspDiagRow(GraspDiagBase()), want,
                               kGraspDiagBools, /*strict=*/true));
}

TEST(GraspDiagLogPod, BoolColumnsAreOneHot) {
  using Pod = integrated_bringup::GraspDiagLogPod;
  EXPECT_TRUE(BoolsAreOneHot<Pod>(GraspDiagHeader(), GraspDiagBase(), GraspDiagRow, kGraspDiagBools,
                                  {{{"valid"}, [](Pod& p) { p.valid = true; }}}));
}

// The real Fill, not a hand-built pod: every column must carry the controller
// state it is named for. beta = 0.5 with K = 6 / 14 / 30 makes gain_scale an
// exact 0.25 / 0.125 / 0.0625; f_error = f_desired - f_measured is exact too.
TEST(GraspDiagLogPod, FillRoundTripsEveryColumnByName) {
  std::array<rtc::grasp::FingerState, 3> fingers{};
  constexpr std::array<double, 3> kStiffness = {6.0, 14.0, 30.0};
  for (std::size_t i = 0; i < fingers.size(); ++i) {
    const double d = static_cast<double>(i);
    fingers[i].s = 0.25 * (d + 1.0);
    fingers[i].f_desired = 300.0 + d;
    fingers[i].f_measured = 50.5 + 0.25 * d;
    fingers[i].K_contact_est = kStiffness[i];
    fingers[i].K_inst_raw = 250.25 + d;
    fingers[i].delta_s_used = 0.0625 * (d + 1.0);
    fingers[i].delta_f_used = 2.5 + 0.25 * d;
    fingers[i].estimate_updated = (i == 0);
    fingers[i].integrator_frozen = (i == 1);
    fingers[i].contact_detected = (i == 2);
  }
  rtc::grasp::GraspParams params{};
  params.beta = 0.5;
  params.alpha_ema = 0.75;
  params.K_est_max = 40.0;

  integrated_bringup::GraspDiagLogPod pod{};
  integrated_bringup::FillGraspDiagLogPod(fingers, params, rtc::grasp::GraspPhase::kHolding, 2.75,
                                          /*t_relative_s=*/12.25, /*tick=*/77, pod);

  Cells want = {{"t_relative_s", "12.25"}, {"tick", "77"},           {"valid", "1"},
                {"grasp_phase", "4"},      {"target_force", "2.75"}, {"beta", "0.5"},
                {"alpha_ema", "0.75"},     {"K_est_max", "40"}};
  const std::vector<std::vector<std::string>> per_finger = {
      {"0.25", "0.5", "0.75"},         // s
      {"300", "301", "302"},           // f_desired
      {"50.5", "50.75", "51"},         // f_measured
      {"249.5", "250.25", "251"},      // f_error
      {"6", "14", "30"},               // k_est
      {"250.25", "251.25", "252.25"},  // k_inst_raw
      {"0.0625", "0.125", "0.1875"},   // delta_s
      {"2.5", "2.75", "3"},            // delta_f
      {"0.25", "0.125", "0.0625"},     // gain_scale
      {"1", "0", "0"},                 // est_updated
      {"0", "1", "0"},                 // integrator_frozen
      {"0", "0", "1"}};                // contact
  std::vector<std::string> families = kGraspDiagFloatFamilies;
  families.insert(families.end(), kGraspDiagFlagFamilies.begin(), kGraspDiagFlagFamilies.end());
  GraspDiagAddFamilies(want, families, 0,
                       [&](std::size_t k, std::size_t i) { return per_finger[k][i]; });
  EXPECT_TRUE(RowMatchesByName(GraspDiagHeader(), GraspDiagRow(pod), want, kGraspDiagBools,
                               /*strict=*/false));
}

// ── DualArmDiagLogPod, every column (#883) ──

namespace {

const std::vector<std::string> kDualArmDiagTasks = {"right_hand", "left_hand"};
const std::vector<std::string> kDualArmDiagJoints = {"j_a", "j_b", "j_c", "j_d", "j_e"};

const std::vector<std::string> kDualArmDiagGlobalBools = {
    "clik_ran",       "estop",      "fault_latched",    "body_readable",
    "hand_readable",  "reseeded",   "reached_solve",    "converged",
    "rejected_input", "non_finite", "command_mismatch", "accel_rows_violated",
    "brake_box_empty"};

std::vector<std::string> DualArmDiagBools() {
  auto bools = kDualArmDiagGlobalBools;
  for (const auto& n : kDualArmDiagTasks)
    for (const char* suffix : {"_valid", "_traj_active", "_meas_valid"})
      bools.push_back(n + suffix);
  return bools;
}

integrated_bringup::DualArmDiagLogColumns DualArmDiagCols() {
  return integrated_bringup::DualArmDiagLogColumnsFor(kDualArmDiagTasks, kDualArmDiagJoints);
}

std::string DualArmDiagHeader() {
  std::ostringstream os;
  integrated_bringup::WriteDualArmDiagLogHeader(os, kDualArmDiagTasks, kDualArmDiagJoints,
                                                DualArmDiagCols());
  return os.str();
}

std::string DualArmDiagRow(const integrated_bringup::DualArmDiagLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteDualArmDiagLogRow(os, pod, DualArmDiagCols());
  return os.str();
}

using DualArmPod = integrated_bringup::DualArmDiagLogPod;

// Every non-bool field its own value (integers from 2001), every bool false.
PodCells<DualArmPod> DualArmDiagBase() {
  PodCells<DualArmPod> f;
  CellBuilder b;
  auto& p = f.pod;
  long long next_int = 2001;
  const auto integer = [&](const std::string& name, auto& field) {
    b.Int(name, field, next_int++);
  };
  // Position then quaternion; the quaternion defaults to {1,0,0,0}, so each
  // component is overwritten.
  const auto pose = [&](const std::string& prefix, std::array<double, 3>& pos,
                        std::array<double, 4>& quat) {
    const char* axes[] = {"x", "y", "z"};
    for (std::size_t i = 0; i < 3; ++i)
      b.Real(prefix + axes[i], pos[i]);
    const char* comps[] = {"qw", "qx", "qy", "qz"};
    for (std::size_t i = 0; i < 4; ++i)
      b.Real(prefix + comps[i], quat[i]);
  };
  p.num_tasks = static_cast<std::uint8_t>(kDualArmDiagTasks.size());
  p.num_joints = static_cast<std::uint8_t>(kDualArmDiagJoints.size());
  b.Real("t_relative_s", p.t_relative_s);
  integer("tick", p.tick);
  // Hold and cause print as their numeric codes, so pick values no counter reaches.
  p.hold = DualArmPod::Hold::kNoModel;
  b.want.emplace_back("hold", "5");
  p.fault_cause = DualArmPod::FaultCause::kSeedOutsideBox;
  b.want.emplace_back("fault_cause", "3");
  // status defaults to -1; it must not read as a default here.
  integer("status", p.status);
  integer("iterations", p.iterations);
  b.Real("solve_time_us", p.solve_time_us);
  integer("accel_rows", p.accel_rows);
  integer("accel_rows_binding", p.accel_rows_binding);
  integer("fb_saturated", p.fb_saturated);
  integer("rot_near_pi", p.rot_near_pi);
  integer("brake_active", p.brake_active);
  integer("brake_static_infeasible", p.brake_static_infeasible);
  integer("qp_fail_streak", p.qp_fail_streak);
  b.Real("track_err", p.track_err);
  integer("group_goal_rejects", p.group_goal_rejects);
  for (std::size_t k = 0; k < kDualArmDiagTasks.size(); ++k) {
    auto& t = p.tasks[k];
    const std::string& n = kDualArmDiagTasks[k];
    integer(n + "_goal_sequence", t.goal_sequence);
    b.Real(n + "_err_lin", t.err_lin);
    b.Real(n + "_err_ang", t.err_ang);
    pose(n + "_ref_", t.ref_pos, t.ref_quat);
    pose(n + "_cmd_", t.cmd_pos, t.cmd_quat);
    pose(n + "_meas_", t.meas_pos, t.meas_quat);
    const char* ingress[] = {"_goals_accepted", "_reject_goal_type", "_reject_non_finite",
                             "_reject_unknown_frame"};
    for (std::size_t i = 0; i < 4; ++i)
      integer(n + ingress[i], t.ingress_counts[i]);
    const char* drops[] = {"_drop_stale", "_drop_unusable", "_drop_held"};
    for (std::size_t i = 0; i < 3; ++i)
      integer(n + drops[i], t.drop_counts[i]);
  }
  for (std::size_t i = 0; i < kDualArmDiagJoints.size(); ++i)
    b.Real("q_cmd_" + kDualArmDiagJoints[i], p.q_cmd[i]);
  for (const auto& name : DualArmDiagBools())
    b.Bool(name);
  f.want = b.want;
  return f;
}

}  // namespace

TEST(DualArmDiagLogPod, EveryColumnByName) {
  const auto f = DualArmDiagBase();
  EXPECT_TRUE(RowMatchesByName(DualArmDiagHeader(), DualArmDiagRow(f.pod), f.want,
                               DualArmDiagBools(), /*strict=*/true));
}

TEST(DualArmDiagLogPod, BoolColumnsAreOneHot) {
  using Pod = DualArmPod;
  auto cases = BoolMemberCases<Pod>({{"clik_ran", &Pod::clik_ran},
                                     {"estop", &Pod::estop},
                                     {"fault_latched", &Pod::fault_latched},
                                     {"body_readable", &Pod::body_readable},
                                     {"hand_readable", &Pod::hand_readable},
                                     {"reseeded", &Pod::reseeded},
                                     {"reached_solve", &Pod::reached_solve},
                                     {"converged", &Pod::converged},
                                     {"rejected_input", &Pod::rejected_input},
                                     {"non_finite", &Pod::non_finite},
                                     {"command_mismatch", &Pod::command_mismatch},
                                     {"accel_rows_violated", &Pod::accel_rows_violated},
                                     {"brake_box_empty", &Pod::brake_box_empty}});
  const std::pair<const char*, bool Pod::Task::*> per_task[] = {
      {"_valid", &Pod::Task::valid},
      {"_traj_active", &Pod::Task::traj_active},
      {"_meas_valid", &Pod::Task::meas_valid}};
  for (std::size_t k = 0; k < kDualArmDiagTasks.size(); ++k)
    for (const auto& [suffix, member] : per_task)
      cases.push_back({{kDualArmDiagTasks[k] + suffix},
                       [k, member = member](Pod& p) { p.tasks[k].*member = true; }});
  EXPECT_TRUE(BoolsAreOneHot<Pod>(DualArmDiagHeader(), DualArmDiagBase().pod, DualArmDiagRow,
                                  DualArmDiagBools(), cases));
}

// ── InferenceDiagLogPod, every column (#883) ──

namespace {

const std::vector<std::string> kInferenceDiagTips = {"thumb", "index", "middle", "ring", "little"};

const std::vector<std::string> kInferenceDiagBools = {
    "held", "policy_step", "reach_hold", "object_valid", "closed_held", "closed_singular"};

std::string InferenceDiagHeader() {
  std::ostringstream os;
  integrated_bringup::WriteInferenceDiagLogHeader(os, kInferenceDiagTips);
  return os.str();
}

std::string InferenceDiagRow(const integrated_bringup::InferenceDiagLogPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteInferenceDiagLogRow(os, pod, kInferenceDiagTips.size());
  return os.str();
}

// The pod's NaN defaults (reach_phase, tip_distance, object_age_s, object
// position, arm_lag_max) are all overridden; integers from 2001, floats from 100.25.
integrated_bringup::InferenceDiagLogPod InferenceDiagBase() {
  integrated_bringup::InferenceDiagLogPod pod{};
  pod.t_relative_s = 100.25;
  pod.iteration = 2001;
  pod.hold_reason = static_cast<std::uint8_t>(integrated_bringup::InferenceHoldReason::kObject);
  pod.inference_count = 2002;
  pod.reach_phase = 100.5;
  pod.tip_distance = 100.75;
  pod.object_age_s = 101.25;
  pod.object_position = {101.5, 101.75, 102.25};
  pod.closed_held_ticks = 2003;
  pod.closure_error = 102.5;
  pod.arm_lag_max = 102.75;
  pod.num_tips = static_cast<std::uint8_t>(kInferenceDiagTips.size());
  for (std::size_t i = 0; i < kInferenceDiagTips.size(); ++i)
    pod.tip_force[i] = 200.25 + 0.25 * static_cast<double>(i);
  return pod;
}

}  // namespace

TEST(InferenceDiagLogPod, EveryColumnByName) {
  Cells want = {{"t_relative_s", "100.25"},
                {"iteration", "2001"},
                {"held", "0"},
                {"hold_code", "7"},  // kObject
                {"hold_reason", "object"},
                {"policy_step", "0"},
                {"inference_count", "2002"},
                {"reach_phase", "100.5"},
                {"tip_distance", "100.75"},
                {"reach_hold", "0"},
                {"object_valid", "0"},
                {"object_age_s", "101.25"},
                {"object_x", "101.5"},
                {"object_y", "101.75"},
                {"object_z", "102.25"},
                {"closed_held", "0"},
                {"closed_held_ticks", "2003"},
                {"closed_singular", "0"},
                {"closure_error", "102.5"},
                {"arm_lag_max", "102.75"}};
  for (std::size_t i = 0; i < kInferenceDiagTips.size(); ++i)
    want.emplace_back("force_" + kInferenceDiagTips[i],
                      Str(200.25 + 0.25 * static_cast<double>(i)));
  EXPECT_TRUE(RowMatchesByName(InferenceDiagHeader(), InferenceDiagRow(InferenceDiagBase()), want,
                               kInferenceDiagBools, /*strict=*/true));
}

TEST(InferenceDiagLogPod, BoolColumnsAreOneHot) {
  using Pod = integrated_bringup::InferenceDiagLogPod;
  EXPECT_TRUE(BoolsAreOneHot<Pod>(
      InferenceDiagHeader(), InferenceDiagBase(), InferenceDiagRow, kInferenceDiagBools,
      {{{"held"}, [](Pod& p) { p.held = true; }},
       {{"policy_step"}, [](Pod& p) { p.policy_step = true; }},
       {{"reach_hold"}, [](Pod& p) { p.reach_hold = true; }},
       {{"object_valid"}, [](Pod& p) { p.object_valid = true; }},
       {{"closed_held"}, [](Pod& p) { p.closed_held = true; }},
       {{"closed_singular"}, [](Pod& p) { p.closed_singular = true; }}}));
}

// ── CatchingDiagLogPod, every column (#883) ──

namespace {

using CatchingPod = integrated_bringup::CatchingDiagLogPod;

// 3 arm joints and 2 tips: the widths differ, so a per-joint block read with the
// tip count (or the reverse) shows.
const std::vector<std::string> kCatchingTips = {"thumb", "index"};

std::vector<std::string> CatchingJoints() {
  return NumberedJoints(3);
}

integrated_bringup::CatchingDiagLogColumns CatchingCols() {
  return integrated_bringup::CatchingDiagLogColumnsFor(CatchingJoints(), kCatchingTips);
}

std::string CatchingHeader() {
  std::ostringstream os;
  integrated_bringup::WriteCatchingDiagLogHeader(os, CatchingJoints(), kCatchingTips,
                                                 CatchingCols());
  return os.str();
}

std::string CatchingRow(const CatchingPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteCatchingDiagLogRow(os, pod, CatchingCols());
  return os.str();
}

const std::vector<std::pair<std::string, bool CatchingPod::*>> kCatchingBoolMembers = {
    {"armed", &CatchingPod::armed},
    {"estop_active", &CatchingPod::estop_active},
    {"fault_latched", &CatchingPod::fault_latched},
    {"armable", &CatchingPod::armable},
    {"law_enabled", &CatchingPod::law_enabled},
    {"real_arm_config", &CatchingPod::real_arm_config},
    {"wait_pose_adopted", &CatchingPod::wait_pose_adopted},
    {"input_valid", &CatchingPod::input_valid},
    {"input_stale", &CatchingPod::input_stale},
    {"input_expired", &CatchingPod::input_expired},
    {"input_new", &CatchingPod::input_new},
    {"plan_valid", &CatchingPod::plan_valid},
    {"ref_valid", &CatchingPod::ref_valid},
    {"ref_saturated", &CatchingPod::ref_saturated},
    {"clik_ran", &CatchingPod::clik_ran},
    {"clik_converged", &CatchingPod::clik_converged},
    {"clik_command_mismatch", &CatchingPod::clik_command_mismatch},
    {"abort_stopped", &CatchingPod::abort_stopped},
    {"hand_phase_valid", &CatchingPod::hand_phase_valid},
    {"hand_timeout", &CatchingPod::hand_timeout},
    {"segment_judged", &CatchingPod::segment_judged},
    {"segment_following", &CatchingPod::segment_following},
    {"segment_held", &CatchingPod::segment_held},
};

std::vector<std::string> CatchingBools() {
  std::vector<std::string> bools;
  for (const auto& m : kCatchingBoolMembers)
    bools.push_back(m.first);
  for (const auto& tip : kCatchingTips) {
    bools.push_back("tip_contact_" + tip);
    bools.push_back("tip_fresh_" + tip);
  }
  return bools;
}

// Every non-bool field distinct (floating from 100.25, integers from 2001,
// uint8 columns from 201). The pod's non-zero defaults (input_stale, input_age_s,
// clik_status, segment_gate_joint, hand_effort_frac = NaN) are all overwritten,
// and every bool is false.
PodCells<CatchingPod> CatchingBase() {
  PodCells<CatchingPod> f;
  CellBuilder b;
  auto& p = f.pod;
  p.input_stale = false;
  b.Real("t_relative_s", p.t_relative_s);
  b.Int("tick", p.tick, 2001);
  b.Real("t_arm_s", p.t_arm_s);
  b.Int("mode", p.mode, 4);
  b.want.emplace_back("mode_name", "committed");
  b.Int("reason", p.reason, 13);
  b.want.emplace_back("reason_name", "abort_escalated");
  b.Int("outcome", p.outcome, 206);
  b.Int("input_n", p.input_n, 2002);
  b.Int("input_generation", p.input_generation, 2003);
  b.Int("input_snapshot_sequence", p.input_snapshot_sequence, 2004);
  b.Int("input_activation_generation", p.input_activation_generation, 2005);
  b.Real("input_age_s", p.input_age_s);
  p.input_age_s = -p.input_age_s;  // the field's default is negative ("never received")
  b.want.back().second = Str(p.input_age_s);
  b.Real("input_horizon_s", p.input_horizon_s);
  b.Int("plan_id", p.plan_id, 2006);
  b.Real("plan_t_c_s", p.plan_t_c_s);
  b.Real("plan_age_s", p.plan_age_s);
  const auto xyz = [&](const std::string& prefix, std::array<double, 3>& v) {
    b.Real(prefix + "_x", v[0]);
    b.Real(prefix + "_y", v[1]);
    b.Real(prefix + "_z", v[2]);
  };
  xyz("plan_p_c", p.plan_p_c);
  xyz("plan_a_d", p.plan_a_d);
  xyz("plan_v_c", p.plan_v_c);
  b.Real("plan_gamma_f", p.plan_gamma_f);
  b.Real("plan_w5", p.plan_w5);
  b.Real("plan_w6", p.plan_w6);
  b.Real("plan_sigma_c", p.plan_sigma_c);
  b.Real("plan_score", p.plan_score);
  b.Int("plan_reason", p.plan_reason, 201);
  xyz("ref_x", p.ref_x);
  xyz("ref_xd", p.ref_xd);
  xyz("ref_xdd", p.ref_xdd);
  xyz("ref_u_des", p.ref_u_des);
  xyz("ref_e", p.ref_e);
  xyz("ref_ed", p.ref_ed);
  b.Real("ref_gamma", p.ref_gamma);
  b.Real("ref_gamma_d", p.ref_gamma_d);
  b.Real("ref_gamma_dd", p.ref_gamma_dd);
  b.Int("clik_status", p.clik_status, -2);
  b.Int("clik_iterations", p.clik_iterations, 2007);
  b.Real("clik_solve_us", p.clik_solve_us);
  b.Int("qp_fail_streak", p.qp_fail_streak, 2008);
  b.Real("track_err_rad", p.track_err_rad);
  // Both enums have 4 values (0..3); 2 and 3 are the two that are neither 0/1.
  p.fault_cause = CatchingPod::FaultCause::kStopDeadline;
  b.want.emplace_back("fault_cause", "2");
  p.fault_reset_refused = CatchingPod::FaultResetRefusal::kVelocityUnreadable;
  b.want.emplace_back("fault_reset_refused", "3");
  b.Int("hand_phase", p.hand_phase, 202);
  b.Real("hand_rho", p.hand_rho);
  b.Int("hand_stalled_n", p.hand_stalled_n, 203);
  b.Real("hand_effort_frac", p.hand_effort_frac);  // default NaN
  b.Real("hand_blocked_s", p.hand_blocked_s);
  b.Int("outcome_source", p.outcome_source, 204);
  b.Int("segment_refusal", p.segment_refusal, 205);
  p.segment_event = CatchingPod::SegmentEvent::kNoSegment;
  b.want.emplace_back("segment_event", "9");
  b.Int("segment_seq", p.segment_seq, 2009);
  b.Int("segment_k0", p.segment_k0, 2010);
  xyz("segment_p_d", p.segment_p_d);
  xyz("segment_v_ff", p.segment_v_ff);
  b.Real("segment_rho", p.segment_rho);
  b.Real("segment_dq_max", p.segment_dq_max);
  b.Real("segment_dqd_max", p.segment_dqd_max);
  b.Int("segment_gate_joint", p.segment_gate_joint, -3);
  p.num_arm_joints = 3;
  p.num_tips = 2;
  const auto joints = CatchingJoints();
  for (std::size_t i = 0; i < joints.size(); ++i)
    b.Real("q_cmd_" + joints[i], p.q_cmd[i]);
  for (std::size_t i = 0; i < joints.size(); ++i)
    b.Real("q_meas_" + joints[i], p.q_meas[i]);
  for (std::size_t i = 0; i < joints.size(); ++i)
    b.Real("q_wait_" + joints[i], p.wait_pose[i]);
  for (std::size_t i = 0; i < kCatchingTips.size(); ++i)
    b.Real("tip_force_" + kCatchingTips[i], p.tip_force[i]);
  for (const auto& name : CatchingBools())
    b.Bool(name);
  for (std::size_t i = 0; i < kCatchingTips.size(); ++i)
    b.Real("tip_age_s_" + kCatchingTips[i], p.tip_age_s[i]);
  f.want = b.want;
  return f;
}

}  // namespace

TEST(CatchingDiagLogPod, EveryColumnByName) {
  const auto f = CatchingBase();
  EXPECT_TRUE(RowMatchesByName(CatchingHeader(), CatchingRow(f.pod), f.want, CatchingBools(),
                               /*strict=*/true));
}

TEST(CatchingDiagLogPod, BoolColumnsAreOneHot) {
  auto cases = BoolMemberCases<CatchingPod>(kCatchingBoolMembers);
  for (std::size_t i = 0; i < kCatchingTips.size(); ++i) {
    cases.push_back(
        {{"tip_contact_" + kCatchingTips[i]}, [i](CatchingPod& p) { p.tip_contact[i] = true; }});
    cases.push_back(
        {{"tip_fresh_" + kCatchingTips[i]}, [i](CatchingPod& p) { p.tip_fresh[i] = true; }});
  }
  EXPECT_TRUE(BoolsAreOneHot<CatchingPod>(CatchingHeader(), CatchingBase().pod, CatchingRow,
                                          CatchingBools(), cases));
}

// ── ComplianceDiagLogPod, every column (#883) ──

namespace {

using CompliancePod = integrated_bringup::ComplianceDiagLogPod;

std::string ComplianceHeader() {
  std::ostringstream os;
  integrated_bringup::WriteComplianceDiagLogHeader(os);
  return os.str();
}

std::string ComplianceRow(const CompliancePod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteComplianceDiagLogRow(os, pod);
  return os.str();
}

const std::vector<std::pair<std::string, bool CompliancePod::*>> kComplianceBoolMembers = {
    {"valid", &CompliancePod::valid},
    {"wrench_valid", &CompliancePod::wrench_valid},
    {"wrench_stale", &CompliancePod::wrench_stale},
    {"in_contact", &CompliancePod::in_contact},
    {"bias_calibrated", &CompliancePod::bias_calibrated},
    {"bias_gate_released", &CompliancePod::bias_gate_released},
    {"bias_begin", &CompliancePod::bias_begin},
    {"quality_low", &CompliancePod::quality_low},
    {"disp_limited", &CompliancePod::disp_limited},
    {"vel_limited", &CompliancePod::vel_limited},
    {"adm_finite", &CompliancePod::adm_finite},
};

std::vector<std::string> ComplianceBools() {
  std::vector<std::string> bools;
  for (const auto& m : kComplianceBoolMembers)
    bools.push_back(m.first);
  return bools;
}

// Every 6-block cell is its own value, so kp / kd / md (and wrench / x_tilde /
// nu_c) trading places or a block reading the wrong member shows.
PodCells<CompliancePod> ComplianceBase() {
  PodCells<CompliancePod> f;
  CellBuilder b;
  auto& p = f.pod;
  b.Real("t_relative_s", p.t_relative_s);
  b.Int("tick", p.tick, 2001);
  b.Int("wrench_source", p.wrench_source, 201);
  b.Int("fsm_state", p.fsm_state, 202);
  b.Real("alpha", p.alpha);
  const auto six = [&](const std::string& prefix, std::array<float, 6>& v,
                       const std::array<const char*, 6>& suffix) {
    for (std::size_t i = 0; i < 6; ++i)
      b.Real(prefix + suffix[i], v[i]);
  };
  const std::array<const char*, 6> kForce = {"_fx", "_fy", "_fz", "_tx", "_ty", "_tz"};
  const std::array<const char*, 6> kAxis = {"_x", "_y", "_z", "_rx", "_ry", "_rz"};
  six("wrench", p.wrench, kForce);
  b.Real("wrench_age", p.wrench_age);
  b.Real("wrench_fade", p.wrench_fade);
  b.Int("rejected_samples", p.rejected_samples, 2002);
  b.Int("invalid_reason", p.invalid_reason, 203);
  six("x_tilde", p.x_tilde, kAxis);
  six("nu_c", p.nu_c, kAxis);
  b.Real("task_origin_x", p.task_origin[0]);
  b.Real("task_origin_y", p.task_origin[1]);
  b.Real("task_origin_z", p.task_origin[2]);
  six("kp", p.kp, kAxis);
  six("kd", p.kd, kAxis);
  six("md", p.md, kAxis);
  b.Real("max_disp_lin", p.max_disp_lin);
  b.Real("max_disp_ang", p.max_disp_ang);
  for (const auto& name : ComplianceBools())
    b.Bool(name);
  f.want = b.want;
  return f;
}

}  // namespace

TEST(ComplianceDiagLogPod, EveryColumnByName) {
  const auto f = ComplianceBase();
  EXPECT_TRUE(RowMatchesByName(ComplianceHeader(), ComplianceRow(f.pod), f.want, ComplianceBools(),
                               /*strict=*/true));
}

TEST(ComplianceDiagLogPod, BoolColumnsAreOneHot) {
  EXPECT_TRUE(BoolsAreOneHot<CompliancePod>(
      ComplianceHeader(), ComplianceBase().pod, ComplianceRow, ComplianceBools(),
      BoolMemberCases<CompliancePod>(kComplianceBoolMembers)));
}

// ── MomentumObserverLogPod, every column (#883) ──

namespace {

using MomentumPod = integrated_bringup::MomentumObserverLogPod;

const std::vector<std::string> kMomentumJoints = {"q0", "q1", "q2"};

std::string MomentumHeader() {
  std::ostringstream os;
  integrated_bringup::WriteMomentumObserverLogHeader(os, kMomentumJoints);
  return os.str();
}

std::string MomentumRow(const MomentumPod& pod) {
  std::ostringstream os;
  integrated_bringup::WriteMomentumObserverLogRow(
      os, pod, integrated_bringup::MomentumObserverLogColumns(kMomentumJoints));
  return os.str();
}

const std::vector<std::pair<std::string, bool MomentumPod::*>> kMomentumBoolMembers = {
    {"valid", &MomentumPod::valid},
    {"payload_valid", &MomentumPod::payload_valid},
    {"inertial_valid", &MomentumPod::inertial_valid},
};

std::vector<std::string> MomentumBools() {
  std::vector<std::string> bools;
  for (const auto& m : kMomentumBoolMembers)
    bools.push_back(m.first);
  return bools;
}

PodCells<MomentumPod> MomentumBase() {
  PodCells<MomentumPod> f;
  CellBuilder b;
  auto& p = f.pod;
  b.Real("t_relative_s", p.t_relative_s);
  b.Int("tick", p.tick, 2001);
  for (std::size_t i = 0; i < kMomentumJoints.size(); ++i)
    b.Real("r_" + kMomentumJoints[i], p.residual[i]);
  b.Real("residual_inf_norm", p.residual_inf_norm);
  b.Int("invalid_reason", p.invalid_reason, 201);
  b.Int("ticks_since_seed", p.ticks_since_seed, 2002);
  const std::array<const char*, 6> kWrench = {"fx", "fy", "fz", "tx", "ty", "tz"};
  for (std::size_t i = 0; i < kWrench.size(); ++i)
    b.Real(std::string("payload_") + kWrench[i], p.payload_wrench[i]);
  b.Real("payload_mass", p.payload_mass);
  b.Real("payload_sigma_min", p.payload_sigma_min);
  b.Real("payload_lambda_sq", p.payload_lambda_sq);
  b.Real("payload_fit_error", p.payload_fit_error);
  b.Int("payload_reason", p.payload_reason, 202);
  b.Real("inertial_mass", p.inertial_mass);
  const std::array<const char*, 3> kAxis = {"x", "y", "z"};
  for (std::size_t i = 0; i < kAxis.size(); ++i)
    b.Real(std::string("inertial_mc") + kAxis[i], p.inertial_first_moment[i]);
  for (std::size_t i = 0; i < kAxis.size(); ++i)
    b.Real(std::string("inertial_c") + kAxis[i], p.inertial_com[i]);
  b.Real("inertial_sigma_min", p.inertial_sigma_min);
  b.Real("inertial_fit_error", p.inertial_fit_error);
  b.Int("inertial_rank", p.inertial_rank, 203);
  b.Int("inertial_reason", p.inertial_reason, 204);
  for (const auto& name : MomentumBools())
    b.Bool(name);
  f.want = b.want;
  return f;
}

}  // namespace

TEST(MomentumObserverLogPod, EveryColumnByName) {
  const auto f = MomentumBase();
  EXPECT_TRUE(RowMatchesByName(MomentumHeader(), MomentumRow(f.pod), f.want, MomentumBools(),
                               /*strict=*/true));
}

TEST(MomentumObserverLogPod, BoolColumnsAreOneHot) {
  EXPECT_TRUE(BoolsAreOneHot<MomentumPod>(MomentumHeader(), MomentumBase().pod, MomentumRow,
                                          MomentumBools(),
                                          BoolMemberCases<MomentumPod>(kMomentumBoolMembers)));
}
