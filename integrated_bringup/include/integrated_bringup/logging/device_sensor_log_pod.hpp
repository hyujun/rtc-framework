#ifndef UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_
#define UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_

// UR5e-bringup POD mirror of rtc_msgs/DeviceSensorLog for controller-owned
// CSV logging. Defines the column set + per-row writer used by the
// generic rtc::ThreadCsvLogger<DeviceSensorLogPod>.
//
// Robot-specific caps (kMaxFingertips, kSensorValuesPerFingertip,
// kFTValuesPerFingertip) live HERE in integrated_bringup, not in rtc_base.
// SPSC constraint: trivially copyable (std::array, no vector/string).
//
// sensor_names captured at on_configure for header expansion only — not
// stored on the POD (would force per-tick string copy / break trivial
// copy).

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

  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // ── Sizes (runtime) ──────────────────────────────────────────────────────
  std::uint8_t num_fingertips{0};
  bool inference_valid{false};

  // ── Sensor data (raw + filtered, packed [fingertip *
  // kSensorValuesPerFingertip
  //    + value_index]) ─────────────────────────────────────────────────────
  std::array<std::int32_t, kMaxSensorValues> sensor_data_raw{};
  std::array<std::int32_t, kMaxSensorValues> sensor_data{};

  // ── Inference output (per-fingertip [contact, fx, fy, fz, ux, uy, uz]) ───
  std::array<float, kMaxInferenceValues> inference_output{};
};

static_assert(std::is_trivially_copyable_v<DeviceSensorLogPod>,
              "DeviceSensorLogPod must be trivially copyable for SPSC ring");

/// Emit the entire CSV header line. `sensor_names` length determines the
/// per-fingertip column expansion (each fingertip gets
/// kSensorValuesPerFingertip raw + filtered + kFTValuesPerFingertip
/// inference columns). The logger appends '\n'.
inline void WriteDeviceSensorLogHeader(std::ostream& os,
                                       std::span<const std::string> sensor_names) {
  os << "t_relative_s";

  // Per-fingertip raw/filtered (each fingertip → kSensorValuesPerFingertip
  // values, named e.g. "thumb_baro_0..7", "thumb_tof_0..2"). We don't know
  // the value semantics here, so just index them.
  auto emit_ft_block = [&](std::string_view kind) {
    for (const auto& name : sensor_names) {
      for (std::size_t v = 0; v < DeviceSensorLogPod::kSensorValuesPerFingertip; ++v) {
        os << ',' << name << '_' << kind << '_' << v;
      }
    }
  };
  emit_ft_block("raw");
  emit_ft_block("filt");

  os << ",inference_valid";
  // Inference: per-fingertip 7 columns: contact, fx, fy, fz, ux, uy, uz
  static constexpr const char* kFtCols[] = {"contact", "fx", "fy", "fz", "ux", "uy", "uz"};
  for (const auto& name : sensor_names) {
    for (const char* col : kFtCols) {
      os << ',' << "ft_" << name << '_' << col;
    }
  }
}

/// Emit one row. Column count must agree with the header writer's
/// sensor_names span — caller's responsibility.
inline void WriteDeviceSensorLogRow(std::ostream& os, const DeviceSensorLogPod& p) {
  os << p.t_relative_s;

  const auto n_sensor =
      static_cast<std::size_t>(p.num_fingertips) * DeviceSensorLogPod::kSensorValuesPerFingertip;
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
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_
