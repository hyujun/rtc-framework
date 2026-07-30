#ifndef UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_
#define UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_

// UR5e-bringup POD mirror of rtc_msgs/DeviceSensorLog for controller-owned
// CSV logging. Defines the column set + per-row writer used by the
// generic rtc::ThreadCsvLogger<DeviceSensorLogPod>.
//
// The mirror is a SUPERSET, not a copy: `force_filtered` /
// `force_filtered_valid` have no DeviceSensorLog.msg counterpart. That is
// deliberate — the msg name appears only as the `msg_type:` key in the
// controller `logs:` YAML block and is never actually published, so this POD is
// the sole carrier and a CSV-only column costs no message ABI (E-3).
//
// Robot-specific caps (kMaxFingertips, kSensorValuesPerFingertip,
// kFTValuesPerFingertip) are integrated_bringup's own SSoT — they
// intentionally do NOT include udp_hand_driver and are NOT re-exported
// from rtc_base. The boundary contract with udp_hand_driver is the
// rtc_msgs FingertipSensor.msg / HandSensorState.msg named fields; the
// values below mirror that schema for assm_v1 hand by construction
// only (numerical equality, not compile-time dependency).
//
// SPSC constraint: trivially copyable (std::array, no vector/string).
//
// sensor_names captured at on_configure for header expansion only — not
// stored on the POD (would force per-tick string copy / break trivial
// copy).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <type_traits>

namespace integrated_bringup {

struct DeviceSensorLogPod {
  // ── Capacities (chosen for ur5e hand: 4 fingertips, 11 sensor values
  //    each — barometer + ToF) ─────────────────────────────────────────────
  static constexpr std::size_t kMaxFingertips = 8;
  static constexpr std::size_t kSensorValuesPerFingertip = 11;
  static constexpr std::size_t kMaxSensorValues = kMaxFingertips * kSensorValuesPerFingertip;  // 88
  static constexpr std::size_t kFTValuesPerFingertip = 7;  // contact + F(3) + u(3)
  static constexpr std::size_t kMaxInferenceValues = kMaxFingertips * kFTValuesPerFingertip;  // 56
  static constexpr std::size_t kForceAxes = 3;                                         // fx,fy,fz
  static constexpr std::size_t kMaxForceFilteredValues = kMaxFingertips * kForceAxes;  // 24

  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // ── Sizes (runtime) ──────────────────────────────────────────────────────
  std::uint8_t num_fingertips{0};
  bool inference_valid{false};
  /// True only when the producing controller actually runs the per-axis force
  /// LPF (joint / task). Controllers without it (wbc) leave `force_filtered`
  /// zero-filled, and a zero column is indistinguishable from "the filter
  /// output really was 0" downstream — this flag is what tells the two apart.
  bool force_filtered_valid{false};

  // ── Sensor data (raw + filtered, packed [fingertip *
  // kSensorValuesPerFingertip
  //    + value_index]) ─────────────────────────────────────────────────────
  std::array<std::int32_t, kMaxSensorValues> sensor_data_raw{};
  std::array<std::int32_t, kMaxSensorValues> sensor_data{};

  // ── Inference output (per-fingertip [contact, fx, fy, fz, ux, uy, uz]) ───
  std::array<float, kMaxInferenceValues> inference_output{};

  // ── Per-axis LPF'd force, packed [fingertip * 3 + axis] ──────────────────
  // The quantity contact_stop actually thresholds, kept beside the raw
  // inference_output slots 1..3 it derives from so a session can be read as
  // raw-vs-filtered without re-deriving the filter offline. Independent of
  // grasp_controller_type: the filter runs every tick regardless of mode, so
  // this column's meaning does not change with the run's configuration.
  std::array<float, kMaxForceFilteredValues> force_filtered{};
};

static_assert(std::is_trivially_copyable_v<DeviceSensorLogPod>,
              "DeviceSensorLogPod must be trivially copyable for SPSC ring");

/// Emit the entire CSV header line. `sensor_names` length determines the
/// per-fingertip column expansion (each fingertip gets `values_per_group` raw
/// + filtered and kFTValuesPerFingertip inference columns, plus the 3 LPF'd
/// force columns). The logger appends '\n'.
///
/// `values_per_group` is the device's runtime `DeviceSensorLayout` stride, NOT
/// the compile-time capacity. A force-only hand declares 0 there and gets no
/// raw/filt block at all — emitting the capacity unconditionally used to hand
/// such a device 2 * names * 11 permanently-zero columns that read downstream
/// as a barometer flatlining rather than as absent hardware. It defaults to the
/// capacity so a caller that has no layout to hand keeps the old columns.
inline void WriteDeviceSensorLogHeader(
    std::ostream& os, std::span<const std::string> sensor_names,
    std::size_t values_per_group = DeviceSensorLogPod::kSensorValuesPerFingertip) {
  os << "t_relative_s";

  // Per-fingertip raw/filtered (each fingertip → values_per_group values,
  // named e.g. "thumb_baro_0..7", "thumb_tof_0..2"). We don't know the value
  // semantics here, so just index them.
  const auto stride = std::min(values_per_group, DeviceSensorLogPod::kSensorValuesPerFingertip);
  auto emit_ft_block = [&](std::string_view kind) {
    for (const auto& name : sensor_names) {
      for (std::size_t v = 0; v < stride; ++v) {
        os << ',' << name << '_' << kind << '_' << v;
      }
    }
  };
  emit_ft_block("raw");
  emit_ft_block("filt");

  os << ",inference_valid";
  // Inference: per-fingertip 7 columns: contact, fx, fy, fz, ux, uy, uz
  static constexpr std::array<const char*, DeviceSensorLogPod::kFTValuesPerFingertip> kFtCols{
      "contact", "fx", "fy", "fz", "ux", "uy", "uz"};
  for (const auto& name : sensor_names) {
    for (const char* col : kFtCols) {
      os << ',' << "ft_" << name << '_' << col;
    }
  }

  // LPF'd force. Suffixed rather than prefixed so the existing `ft_<n>_fx`
  // detectors (which match on the trailing token) cannot pick these up as a
  // second set of raw-force columns.
  os << ",force_filtered_valid";
  static constexpr std::array<const char*, DeviceSensorLogPod::kForceAxes> kForceFiltCols{
      "fx_filt", "fy_filt", "fz_filt"};
  for (const auto& name : sensor_names) {
    for (const char* col : kForceFiltCols) {
      os << ',' << "ft_" << name << '_' << col;
    }
  }
}

/// Emit one row. Column count must agree with the header writer's
/// sensor_names span and `values_per_group` — caller's responsibility. Rows are
/// narrower than the header whenever the device reports fewer fingertips than
/// were named at configure time; that is the documented contract (see
/// test_device_log_pods.cpp RowRespectsRuntimeNumFingertips), not padding this
/// writer is missing.
inline void WriteDeviceSensorLogRow(
    std::ostream& os, const DeviceSensorLogPod& p,
    std::size_t values_per_group = DeviceSensorLogPod::kSensorValuesPerFingertip) {
  os << p.t_relative_s;

  const auto stride = std::min(values_per_group, DeviceSensorLogPod::kSensorValuesPerFingertip);
  const auto n_sensor = static_cast<std::size_t>(p.num_fingertips) * stride;
  for (std::size_t i = 0; i < n_sensor; ++i) {
    os << ',' << p.sensor_data_raw[i];
  }
  for (std::size_t i = 0; i < n_sensor; ++i) {
    os << ',' << p.sensor_data[i];
  }

  os << ',' << (p.inference_valid ? 1 : 0);

  const auto n_inf =
      static_cast<std::size_t>(p.num_fingertips) * DeviceSensorLogPod::kFTValuesPerFingertip;
  for (std::size_t i = 0; i < n_inf; ++i) {
    os << ',' << p.inference_output[i];
  }

  os << ',' << (p.force_filtered_valid ? 1 : 0);
  const auto n_filt = static_cast<std::size_t>(p.num_fingertips) * DeviceSensorLogPod::kForceAxes;
  for (std::size_t i = 0; i < n_filt; ++i) {
    os << ',' << p.force_filtered[i];
  }
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_
