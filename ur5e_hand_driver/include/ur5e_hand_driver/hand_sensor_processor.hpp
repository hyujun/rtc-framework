#ifndef UR5E_HAND_DRIVER_HAND_SENSOR_PROCESSOR_HPP_
#define UR5E_HAND_DRIVER_HAND_SENSOR_PROCESSOR_HPP_

// HandSensorProcessor: sensor post-processing pipeline.
//
// Owns BesselFilter (baro/tof LPF), SensorRateEstimator, and
// SlidingTrendDetector (one-shot drift detection).
//
// Called from HandController's EventLoop on every sensor cycle.
// All hot-path methods are noexcept (RT-safe).

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include "rtc_base/types/types.hpp"
#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_base/filters/sensor_rate_estimator.hpp"
#include "rtc_base/filters/sliding_trend_detector.hpp"
#include "ur5e_hand_driver/hand_logging.hpp"

namespace rtc {

struct HandSensorProcessorConfig {
  int  num_fingertips{0};
  int  sensor_decimation{1};
  bool tof_lpf_enabled{false};
  double tof_lpf_cutoff_hz{15.0};
  bool baro_lpf_enabled{false};
  double baro_lpf_cutoff_hz{30.0};
  bool drift_detection_enabled{false};
  double drift_threshold{5.0};
  int drift_window_size{2500};
};

class HandSensorProcessor {
 public:
  explicit HandSensorProcessor(const HandSensorProcessorConfig& cfg) noexcept
      : num_fingertips_(cfg.num_fingertips),
        sensor_decimation_(cfg.sensor_decimation < 1 ? 1 : cfg.sensor_decimation),
        tof_lpf_enabled_(cfg.tof_lpf_enabled),
        tof_lpf_cutoff_hz_(cfg.tof_lpf_cutoff_hz),
        baro_lpf_enabled_(cfg.baro_lpf_enabled),
        baro_lpf_cutoff_hz_(cfg.baro_lpf_cutoff_hz),
        drift_detection_enabled_(cfg.drift_detection_enabled),
        drift_threshold_(cfg.drift_threshold),
        drift_window_size_(cfg.drift_window_size) {}

  // Initialize rate estimator and filters. Call once from Start().
  void Init() noexcept {
    if (num_fingertips_ <= 0) return;

    constexpr double kNominalRateHz = 500.0;
    sensor_rate_estimator_.Init(kNominalRateHz);

    const double nominal_effective_rate =
        kNominalRateHz / static_cast<double>(sensor_decimation_);

    const auto logger = ::ur5e_hand_driver::logging::SensorLogger();

    if (baro_lpf_enabled_) {
      try {
        baro_filter_.Init(baro_lpf_cutoff_hz_, nominal_effective_rate);
        baro_filter_active_ = true;
      } catch (...) {
        baro_filter_active_ = false;
        RCLCPP_WARN(logger, "Barometer LPF init failed (disabled)");
      }
    }

    if (tof_lpf_enabled_) {
      try {
        tof_filter_.Init(tof_lpf_cutoff_hz_, nominal_effective_rate);
        tof_filter_active_ = true;
      } catch (...) {
        tof_filter_active_ = false;
        RCLCPP_WARN(logger, "TOF LPF init failed (disabled)");
      }
    }

    if (drift_detection_enabled_) {
      drift_detector_.Init(
          static_cast<std::size_t>(drift_window_size_),
          drift_threshold_);
    }

    RCLCPP_INFO(logger,
                "SensorProcessor initialized: baro_lpf=%s, tof_lpf=%s, drift=%s",
                baro_filter_active_ ? "ON" : "OFF",
                tof_filter_active_ ? "ON" : "OFF",
                drift_detection_enabled_ ? "ON" : "OFF");
  }

  // Called every sensor cycle: rate estimator tick + delayed BesselFilter re-init.
  void PreFilter() noexcept {
    sensor_rate_estimator_.Tick();

    if (!filter_reinited_ && sensor_rate_estimator_.warmed_up()) {
      const double actual_rate =
          sensor_rate_estimator_.rate_hz()
          / static_cast<double>(sensor_decimation_);
      // Replaced RCLCPP_WARN_ONCE with WARN_THROTTLE on the RT path: _ONCE
      // still pays the formatting allocation on the first hit and offers no
      // defense against repeated firing if the filter init keeps throwing
      // before filter_reinited_ flips. The throttle bounds the worst case to
      // one log per kThrottleIdleMs while still surfacing the failure.
      if (baro_filter_active_) {
        try {
          baro_filter_.Init(baro_lpf_cutoff_hz_, actual_rate);
        } catch (...) {
          RCLCPP_WARN_THROTTLE(::ur5e_hand_driver::logging::SensorLogger(),
                               filter_warn_clock_,
                               ::ur5e_hand_driver::logging::kThrottleIdleMs,
                               "Barometer BesselFilter re-init failed at actual rate %.1f Hz",
                               actual_rate);
        }
      }
      if (tof_filter_active_) {
        try {
          tof_filter_.Init(tof_lpf_cutoff_hz_, actual_rate);
        } catch (...) {
          RCLCPP_WARN_THROTTLE(::ur5e_hand_driver::logging::SensorLogger(),
                               filter_warn_clock_,
                               ::ur5e_hand_driver::logging::kThrottleIdleMs,
                               "TOF BesselFilter re-init failed at actual rate %.1f Hz",
                               actual_rate);
        }
      }
      filter_reinited_ = true;
    }
  }

  // Apply baro/tof LPF in-place.
  void ApplyFilters(
      std::array<int32_t, kMaxHandSensors>& sensor_data) noexcept {
    // Barometer LPF (8 channels per fingertip)
    if (baro_filter_active_) {
      std::array<double, kMaxBaroChannels> baro_input{};
      for (int f = 0; f < num_fingertips_; ++f) {
        const int base = f * kSensorValuesPerFingertip;
        for (int b = 0; b < kBarometerCount; ++b) {
          baro_input[static_cast<std::size_t>(f * kBarometerCount + b)] =
              static_cast<double>(sensor_data[static_cast<std::size_t>(base + b)]);
        }
      }
      const auto filtered = baro_filter_.Apply(baro_input);
      for (int f = 0; f < num_fingertips_; ++f) {
        const int base = f * kSensorValuesPerFingertip;
        for (int b = 0; b < kBarometerCount; ++b) {
          const double v = filtered[static_cast<std::size_t>(f * kBarometerCount + b)];
          sensor_data[static_cast<std::size_t>(base + b)] =
              static_cast<int32_t>(std::round(v));
        }
      }
    }

    // TOF LPF (3 channels per fingertip)
    if (tof_filter_active_) {
      std::array<double, kMaxTofChannels> tof_input{};
      for (int f = 0; f < num_fingertips_; ++f) {
        const int base = f * kSensorValuesPerFingertip + kBarometerCount;
        for (int t = 0; t < kTofCount; ++t) {
          tof_input[static_cast<std::size_t>(f * kTofCount + t)] =
              static_cast<double>(sensor_data[static_cast<std::size_t>(base + t)]);
        }
      }
      const auto filtered = tof_filter_.Apply(tof_input);
      for (int f = 0; f < num_fingertips_; ++f) {
        const int base = f * kSensorValuesPerFingertip + kBarometerCount;
        for (int t = 0; t < kTofCount; ++t) {
          const double v = filtered[static_cast<std::size_t>(f * kTofCount + t)];
          sensor_data[static_cast<std::size_t>(base + t)] =
              static_cast<int32_t>(std::round(v));
        }
      }
    }
  }

  // One-shot drift detection on raw baro data.
  void DetectDrift(
      const std::array<int32_t, kMaxHandSensors>& sensor_data_raw) noexcept {
    if (!drift_detection_enabled_) return;

    // Phase 1: accumulate until window full
    if (!drift_detector_.window_full()) {
      std::array<double, kMaxBaroChannels> baro_raw{};
      for (int f = 0; f < num_fingertips_; ++f) {
        const int sensor_base = f * kSensorValuesPerFingertip;
        const int baro_base   = f * kBarometerCount;
        for (int b = 0; b < kBarometerCount; ++b) {
          baro_raw[static_cast<std::size_t>(baro_base + b)] =
              static_cast<double>(
                  sensor_data_raw[static_cast<std::size_t>(sensor_base + b)]);
        }
      }

      auto result = drift_detector_.Update(baro_raw);

      // Phase 2: window full -> cache result (one-shot)
      if (result.window_full) {
        drift_result_ = result;
        const int num_baro = num_fingertips_ * kBarometerCount;
        drift_detected_ = false;
        for (int i = 0; i < num_baro; ++i) {
          if (result.drift_flags[static_cast<std::size_t>(i)]) {
            drift_detected_ = true;
            break;
          }
        }
      }
    }

    // Phase 3: throttled warning (1Hz)
    if (drift_detected_) {
      ThrottledDriftWarning();
    }
  }

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] double actual_sensor_rate_hz() const noexcept {
    return sensor_rate_estimator_.rate_hz();
  }

 private:
  // RT hot path (EventLoop thread). A single aggregated WARN (3 args, fixed
  // length) replaces the former per-channel loop of up to kMaxBaroChannels
  // RCLCPP_WARN calls. Only the count and the first flagged channel are
  // reported so the format string has a bounded length and never truncates;
  // full per-channel slopes are available via drift_result_ for any consumer
  // that needs them outside the log stream.
  void ThrottledDriftWarning() noexcept {
    const int num_baro = num_fingertips_ * kBarometerCount;
    int flagged_count = 0;
    int first_id = -1;
    double first_slope = 0.0;
    for (int i = 0; i < num_baro; ++i) {
      if (drift_result_.drift_flags[static_cast<std::size_t>(i)]) {
        if (flagged_count == 0) {
          first_id = i;
          first_slope = drift_result_.slopes[static_cast<std::size_t>(i)];
        }
        ++flagged_count;
      }
    }
    if (flagged_count == 0) return;

    RCLCPP_WARN_THROTTLE(::ur5e_hand_driver::logging::SensorLogger(),
                         drift_warn_clock_,
                         ::ur5e_hand_driver::logging::kThrottleSlowMs,
                         "Barometer drift: %d ch flagged (first id=%d slope=%.3f)",
                         flagged_count, first_id, first_slope);
  }

  int  num_fingertips_;
  int  sensor_decimation_;

  // LPF config
  bool   tof_lpf_enabled_;
  double tof_lpf_cutoff_hz_;
  bool   baro_lpf_enabled_;
  double baro_lpf_cutoff_hz_;

  // LPF state
  bool baro_filter_active_{false};
  BesselFilterN<kMaxBaroChannels> baro_filter_{};
  bool tof_filter_active_{false};
  BesselFilterN<kMaxTofChannels> tof_filter_{};

  // Rate estimator
  SensorRateEstimator sensor_rate_estimator_;
  bool filter_reinited_{false};

  // Drift detection
  bool   drift_detection_enabled_;
  double drift_threshold_;
  int    drift_window_size_;
  SlidingTrendDetector<kMaxBaroChannels, 2500> drift_detector_;
  SlidingTrendDetector<kMaxBaroChannels, 2500>::Result drift_result_{};
  bool   drift_detected_{false};

  // Throttle clocks for RT-path warnings. filter_warn_clock_ replaces
  // RCLCPP_WARN_ONCE on the PreFilter path; drift_warn_clock_ backs the
  // single aggregated WARN in ThrottledDriftWarning (no per-channel loop).
  rclcpp::Clock filter_warn_clock_{RCL_STEADY_TIME};
  rclcpp::Clock drift_warn_clock_{RCL_STEADY_TIME};
};

}  // namespace rtc

#endif  // UR5E_HAND_DRIVER_HAND_SENSOR_PROCESSOR_HPP_
