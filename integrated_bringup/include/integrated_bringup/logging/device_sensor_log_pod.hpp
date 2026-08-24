#ifndef UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_
#define UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_

// UR5e-bringup POD mirror of rtc_msgs/DeviceSensorLog for controller-owned
// CSV logging. Defines the column set + per-row writer used by the
// generic rtc::ThreadCsvLogger<DeviceSensorLogPod>.
//
// The mirror is a SUPERSET, not a copy: `force_filtered` /
// `force_filtered_valid` / `force_guarded` / `force_guard_rejected` have no
// DeviceSensorLog.msg counterpart. That is deliberate — the msg name appears
// only as the `msg_type:` key in the controller `logs:` YAML block and is never
// actually published, so this POD is the sole carrier and a CSV-only column
// costs no message ABI (E-3).
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

  // ── Guarded LPF input, packed [fingertip * 3 + axis] ─────────────────────
  // What the LPF above was actually fed this tick: the raw triplet, or the last
  // accepted one when the delta-spike guard held (FingertipForceGuard). Logged
  // beside raw + filtered because the guard's effect is otherwise
  // unreconstructible offline — from raw and filtered alone one cannot tell a
  // held tick from a filter that simply lagged. Gated by the same
  // `force_filtered_valid` flag: a controller without the force LPF has no
  // guard either, so the two blocks are present or absent together.
  std::array<float, kMaxForceFilteredValues> force_guarded{};
  /// 1 on ticks where the guard replaced the raw triplet by the held one.
  /// uint8 rather than bool so the CSV column is unambiguously 0/1 and the POD
  /// stays trivially copyable with a stable size.
  std::array<std::uint8_t, kMaxFingertips> force_guard_rejected{};
};

static_assert(std::is_trivially_copyable_v<DeviceSensorLogPod>,
              "DeviceSensorLogPod must be trivially copyable for SPSC ring");

/// Column geometry for one registered DeviceSensorLog channel, captured ONCE
/// at registration and handed to BOTH writers.
///
/// Bundled rather than passed as two positional sizes for the reason
/// ForceFilterLogView is bundled (pod_fill.hpp): header and row must be sized
/// from the same numbers, and a call site that handed one writer a different
/// width than the other would still compile and still produce a well-formed
/// file — just one whose columns mean something other than what the header
/// says.
///
/// `fingertips` is a CONFIGURED width (sensor_names, clamped to the POD
/// capacity), never the per-tick runtime count. The header is written once at
/// first Open, before any pod exists, so a row sized by `pod.num_fingertips`
/// diverges from it the moment the device reports a different number than the
/// config named. That divergence is not hypothetical: a session whose hand
/// reported 0 fingertips wrote a 59-column header over 3-column rows for
/// 138,248 rows, and a reader resolving columns by header name then read
/// `force_filtered_valid` (constant 1) as `ft_thumb_contact` — an absent
/// sensor lane plotted as "thumb in contact for the whole session" (#440).
///
/// Rows therefore always carry `fingertips` blocks: slots the device did not
/// report this tick carry 0, and the trailing `num_fingertips` column records
/// what it actually reported, so a zero block is legible as absence rather
/// than as a measured zero.
struct DeviceSensorLogColumns {
  /// Fingertip-block width. Both writers emit exactly this many blocks.
  std::size_t fingertips{0};
  /// Device's runtime DeviceSensorLayout stride for the raw/filt block.
  std::size_t values_per_group{DeviceSensorLogPod::kSensorValuesPerFingertip};
};

/// Derive the geometry from the configured names + the device's runtime
/// DeviceSensorLayout stride. The single place either width is computed, so
/// the two writers cannot be handed different numbers.
///
/// `values_per_group` is the runtime stride, NOT the compile-time capacity. A
/// force-only hand declares 0 there and gets no raw/filt block at all —
/// emitting the capacity unconditionally used to hand such a device
/// 2 * names * 11 permanently-zero columns that read downstream as a barometer
/// flatlining rather than as absent hardware. It defaults to the capacity so a
/// caller that has no layout to hand keeps the old columns.
[[nodiscard]] inline DeviceSensorLogColumns DeviceSensorLogColumnsFor(
    std::span<const std::string> sensor_names,
    std::size_t values_per_group = DeviceSensorLogPod::kSensorValuesPerFingertip) {
  return {std::min(sensor_names.size(), DeviceSensorLogPod::kMaxFingertips),
          std::min(values_per_group, DeviceSensorLogPod::kSensorValuesPerFingertip)};
}

/// Emit the entire CSV header line. Exactly `cols.fingertips` per-fingertip
/// blocks are emitted (each fingertip gets `cols.values_per_group` raw +
/// filtered and kFTValuesPerFingertip inference columns, plus 3 LPF'd force,
/// 3 guarded LPF-input and 1 guard-verdict column), followed by the trailing
/// `num_fingertips` column. The logger appends '\n'.
///
/// `sensor_names` supplies labels only — it does not size anything. A slot
/// past the end of the list falls back to `ft<index>`, so the emitted width is
/// `cols.fingertips` no matter what the two arguments say about each other.
/// DeviceSensorLogColumnsFor keeps the fallback unreachable in practice.
inline void WriteDeviceSensorLogHeader(std::ostream& os, std::span<const std::string> sensor_names,
                                       const DeviceSensorLogColumns& cols) {
  os << "t_relative_s";

  const auto n_ft = std::min(cols.fingertips, DeviceSensorLogPod::kMaxFingertips);
  const auto stride =
      std::min(cols.values_per_group, DeviceSensorLogPod::kSensorValuesPerFingertip);
  // Header write is once-per-file and off the RT path, so building the label
  // string here is free.
  auto label = [&](std::size_t f) {
    return f < sensor_names.size() ? sensor_names[f] : ("ft" + std::to_string(f));
  };

  // Per-fingertip raw/filtered (each fingertip → values_per_group values,
  // named e.g. "thumb_baro_0..7", "thumb_tof_0..2"). We don't know the value
  // semantics here, so just index them.
  auto emit_ft_block = [&](std::string_view kind) {
    for (std::size_t f = 0; f < n_ft; ++f) {
      for (std::size_t v = 0; v < stride; ++v) {
        os << ',' << label(f) << '_' << kind << '_' << v;
      }
    }
  };
  emit_ft_block("raw");
  emit_ft_block("filt");

  os << ",inference_valid";
  // Inference: per-fingertip 7 columns: contact, fx, fy, fz, ux, uy, uz
  static constexpr std::array<const char*, DeviceSensorLogPod::kFTValuesPerFingertip> kFtCols{
      "contact", "fx", "fy", "fz", "ux", "uy", "uz"};
  for (std::size_t f = 0; f < n_ft; ++f) {
    for (const char* col : kFtCols) {
      os << ',' << "ft_" << label(f) << '_' << col;
    }
  }

  // LPF'd force. Suffixed rather than prefixed so the existing `ft_<n>_fx`
  // detectors (which match on the trailing token) cannot pick these up as a
  // second set of raw-force columns.
  os << ",force_filtered_valid";
  static constexpr std::array<const char*, DeviceSensorLogPod::kForceAxes> kForceFiltCols{
      "fx_filt", "fy_filt", "fz_filt"};
  for (std::size_t f = 0; f < n_ft; ++f) {
    for (const char* col : kForceFiltCols) {
      os << ',' << "ft_" << label(f) << '_' << col;
    }
  }

  // Guarded LPF input + guard verdict. APPENDED, never inserted: an older
  // reader that positionally consumed the columns above keeps working, and the
  // `_guarded` / `_force_guard_rejected` suffixes stay clear of the trailing
  // `_fx` / `_contact` tokens the raw-force and fingertip-label detectors match
  // on (rtc_tools plotting/columns/detect.py).
  static constexpr std::array<const char*, DeviceSensorLogPod::kForceAxes> kForceGuardedCols{
      "fx_guarded", "fy_guarded", "fz_guarded"};
  for (std::size_t f = 0; f < n_ft; ++f) {
    for (const char* col : kForceGuardedCols) {
      os << ',' << "ft_" << label(f) << '_' << col;
    }
  }
  for (std::size_t f = 0; f < n_ft; ++f) {
    os << ',' << "ft_" << label(f) << "_force_guard_rejected";
  }

  // Appended last, same append-only rule as the guard block above. This is the
  // column that keeps a padded (or truncated) row self-describing — see
  // DeviceSensorLogColumns.
  os << ",num_fingertips";
}

/// Emit one row. Width is taken from the SAME `cols` the header was written
/// with, so the two agree by construction rather than by the caller's care.
///
/// Fingertip slots the device did not report this tick are emitted as 0 rather
/// than skipped; `num_fingertips` (last column) says how many were real. A
/// device reporting MORE fingertips than the config named is truncated to the
/// header's width — visible in the same column, since it then exceeds the
/// number of blocks present.
inline void WriteDeviceSensorLogRow(std::ostream& os, const DeviceSensorLogPod& p,
                                    const DeviceSensorLogColumns& cols) {
  os << p.t_relative_s;

  const auto stride =
      std::min(cols.values_per_group, DeviceSensorLogPod::kSensorValuesPerFingertip);
  const auto n_ft = std::min(cols.fingertips, DeviceSensorLogPod::kMaxFingertips);
  // Slots [reported, n_ft) are padding. Padding is written as a literal 0
  // instead of reading the array: FillDeviceSensorLogPod copies the device's
  // sensor lane at the CAPACITY stride while this writer indexes at the
  // LAYOUT stride, so for a narrow layout the array tail past the reported
  // fingertips can still hold live bytes from the device's packing.
  const auto reported = std::min(static_cast<std::size_t>(p.num_fingertips), n_ft);

  auto emit_sensor_block =
      [&](const std::array<std::int32_t, DeviceSensorLogPod::kMaxSensorValues>& a) {
        for (std::size_t f = 0; f < n_ft; ++f) {
          for (std::size_t v = 0; v < stride; ++v) {
            os << ',' << (f < reported ? a[f * stride + v] : 0);
          }
        }
      };
  emit_sensor_block(p.sensor_data_raw);
  emit_sensor_block(p.sensor_data);

  os << ',' << (p.inference_valid ? 1 : 0);
  for (std::size_t f = 0; f < n_ft; ++f) {
    for (std::size_t v = 0; v < DeviceSensorLogPod::kFTValuesPerFingertip; ++v) {
      os << ','
         << (f < reported ? p.inference_output[f * DeviceSensorLogPod::kFTValuesPerFingertip + v]
                          : 0.0F);
    }
  }

  os << ',' << (p.force_filtered_valid ? 1 : 0);
  auto emit_force_block =
      [&](const std::array<float, DeviceSensorLogPod::kMaxForceFilteredValues>& a) {
        for (std::size_t f = 0; f < n_ft; ++f) {
          for (std::size_t v = 0; v < DeviceSensorLogPod::kForceAxes; ++v) {
            os << ',' << (f < reported ? a[f * DeviceSensorLogPod::kForceAxes + v] : 0.0F);
          }
        }
      };
  emit_force_block(p.force_filtered);
  emit_force_block(p.force_guarded);
  for (std::size_t f = 0; f < n_ft; ++f) {
    os << ',' << ((f < reported && p.force_guard_rejected[f] != 0) ? 1 : 0);
  }

  os << ',' << static_cast<unsigned>(p.num_fingertips);
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_LOGGING_DEVICE_SENSOR_LOG_POD_HPP_
