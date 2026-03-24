#ifndef UR5E_HAND_DRIVER_HAND_TIMING_PROFILER_HPP_
#define UR5E_HAND_DRIVER_HAND_TIMING_PROFILER_HPP_

// HandTimingProfiler: EventLoop per-phase timing measurement.
//
// Measures and accumulates per-phase durations for the HandController EventLoop.
// Inherits TimingProfilerBase<50,100,2000> for histogram/percentile logic.
//
// Phase measurements:
//   Individual: write -> read_pos -> read_vel -> read_sensor -> total
//   Bulk:       write -> read_all_motor -> read_all_sensor -> total
//
// Thread safety:
//   Update() is called only from the EventLoop thread (single producer).
//   GetStats() / Summary() can be called from any thread (relaxed atomic).

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "rtc_base/timing/timing_profiler_base.hpp"

namespace rtc {

class HandTimingProfiler : public TimingProfilerBase<50, 100, 2000> {
 public:
  using PhaseStats = typename TimingProfilerBase<50, 100, 2000>::PhaseStats;

  struct Stats : BaseStats {
    // Per-phase (individual mode)
    PhaseStats write;
    PhaseStats read_pos;
    PhaseStats read_vel;
    PhaseStats read_sensor;
    uint64_t   sensor_cycle_count{0};

    // Per-phase (bulk mode)
    PhaseStats read_all_motor;
    PhaseStats read_all_sensor;
    bool       is_bulk_mode{false};

    // Per-mode cycle counts
    uint64_t   individual_count{0};
    uint64_t   bulk_count{0};

    // Sensor processing phase (filter + drift detection, excl. FT)
    PhaseStats sensor_proc;

    // F/T inference phase
    PhaseStats ft_infer;
    uint64_t   ft_infer_count{0};
  };

  // Per-tick measurement (filled by EventLoop, passed to Update())
  struct PhaseTiming {
    double write_us{0.0};
    // Individual mode phases
    double read_pos_us{0.0};
    double read_vel_us{0.0};
    double read_sensor_us{0.0};
    // Bulk mode phases
    double read_all_motor_us{0.0};
    double read_all_sensor_us{0.0};
    double sensor_proc_us{0.0};
    double ft_infer_us{0.0};
    double total_us{0.0};
    bool   is_sensor_cycle{false};
    bool   is_bulk_mode{false};
  };

  // ── Update (single-producer: EventLoop thread) ────────────────────────────

  void Update(const PhaseTiming& t) noexcept {
    UpdateTotal(t.total_us);

    UpdatePhase(write_sum_, write_min_, write_max_, t.write_us);

    if (t.is_bulk_mode) {
      is_bulk_mode_.store(true, std::memory_order_relaxed);
      bulk_count_.fetch_add(1, std::memory_order_relaxed);
      UpdatePhase(read_all_motor_sum_, read_all_motor_min_, read_all_motor_max_,
                  t.read_all_motor_us);
      if (t.is_sensor_cycle) {
        sensor_cycle_count_.fetch_add(1, std::memory_order_relaxed);
        UpdatePhase(read_all_sensor_sum_, read_all_sensor_min_, read_all_sensor_max_,
                    t.read_all_sensor_us);
      }
    } else {
      individual_count_.fetch_add(1, std::memory_order_relaxed);
      UpdatePhase(read_pos_sum_, read_pos_min_, read_pos_max_, t.read_pos_us);
      UpdatePhase(read_vel_sum_, read_vel_min_, read_vel_max_, t.read_vel_us);
      if (t.is_sensor_cycle) {
        sensor_cycle_count_.fetch_add(1, std::memory_order_relaxed);
        UpdatePhase(read_sensor_sum_, read_sensor_min_, read_sensor_max_,
                    t.read_sensor_us);
      }
    }

    if (t.is_sensor_cycle) {
      UpdatePhase(sensor_proc_sum_, sensor_proc_min_, sensor_proc_max_,
                  t.sensor_proc_us);
    }

    if (t.ft_infer_us > 0.0) {
      ft_infer_count_.fetch_add(1, std::memory_order_relaxed);
      UpdatePhase(ft_infer_sum_, ft_infer_min_, ft_infer_max_, t.ft_infer_us);
    }
  }

  // ── Statistics snapshot ───────────────────────────────────────────────────

  [[nodiscard]] Stats GetStats() const noexcept {
    Stats s;
    static_cast<BaseStats&>(s) = GetBaseStats();

    s.individual_count = individual_count_.load(std::memory_order_relaxed);
    s.bulk_count = bulk_count_.load(std::memory_order_relaxed);

    if (s.count > 0) {
      s.write = LoadPhaseStats(write_sum_, write_min_, write_max_, s.count);
    }

    if (s.individual_count > 0) {
      s.read_pos = LoadPhaseStats(read_pos_sum_, read_pos_min_, read_pos_max_,
                                  s.individual_count);
      s.read_vel = LoadPhaseStats(read_vel_sum_, read_vel_min_, read_vel_max_,
                                  s.individual_count);
    }

    s.is_bulk_mode = is_bulk_mode_.load(std::memory_order_relaxed);
    if (s.bulk_count > 0) {
      s.read_all_motor = LoadPhaseStats(read_all_motor_sum_, read_all_motor_min_,
                                        read_all_motor_max_, s.bulk_count);
    }

    s.sensor_cycle_count = sensor_cycle_count_.load(std::memory_order_relaxed);
    if (s.sensor_cycle_count > 0) {
      if (s.is_bulk_mode) {
        s.read_all_sensor = LoadPhaseStats(read_all_sensor_sum_, read_all_sensor_min_,
                                           read_all_sensor_max_, s.sensor_cycle_count);
      } else {
        s.read_sensor = LoadPhaseStats(read_sensor_sum_, read_sensor_min_,
                                       read_sensor_max_, s.sensor_cycle_count);
      }
    }

    if (s.sensor_cycle_count > 0) {
      s.sensor_proc = LoadPhaseStats(sensor_proc_sum_, sensor_proc_min_,
                                     sensor_proc_max_, s.sensor_cycle_count);
    }

    s.ft_infer_count = ft_infer_count_.load(std::memory_order_relaxed);
    if (s.ft_infer_count > 0) {
      s.ft_infer = LoadPhaseStats(ft_infer_sum_, ft_infer_min_, ft_infer_max_,
                                  s.ft_infer_count);
    }

    return s;
  }

  // ── Human-readable summary ────────────────────────────────────────────────

  [[nodiscard]] std::string Summary() const noexcept {
    const Stats s = GetStats();
    if (s.count == 0) { return "HandUDP timing: no data"; }

    const double over_pct = static_cast<double>(s.over_budget) * 100.0 /
                            static_cast<double>(s.count);
    char buf[512];
    if (s.is_bulk_mode) {
      std::snprintf(
          buf, sizeof(buf),
          "HandUDP timing [bulk]: count=%lu  mean=%.0f\xc2\xb5s  min=%.0f\xc2\xb5s"
          "  max=%.0f\xc2\xb5s  p95=%.0f\xc2\xb5s  p99=%.0f\xc2\xb5s"
          "  over_budget=%lu (%.1f%%)"
          "  | write=%.0f  all_motor=%.0f  all_sensor=%.0f  proc=%.0f\xc2\xb5s",
          static_cast<unsigned long>(s.count),
          s.mean_us, s.min_us, s.max_us, s.p95_us, s.p99_us,
          static_cast<unsigned long>(s.over_budget), over_pct,
          s.write.mean_us, s.read_all_motor.mean_us, s.read_all_sensor.mean_us,
          s.sensor_proc.mean_us);
    } else {
      std::snprintf(
          buf, sizeof(buf),
          "HandUDP timing: count=%lu  mean=%.0f\xc2\xb5s  min=%.0f\xc2\xb5s"
          "  max=%.0f\xc2\xb5s  p95=%.0f\xc2\xb5s  p99=%.0f\xc2\xb5s"
          "  over_budget=%lu (%.1f%%)"
          "  | write=%.0f  pos=%.0f  vel=%.0f  sensor=%.0f  proc=%.0f\xc2\xb5s",
          static_cast<unsigned long>(s.count),
          s.mean_us, s.min_us, s.max_us, s.p95_us, s.p99_us,
          static_cast<unsigned long>(s.over_budget), over_pct,
          s.write.mean_us, s.read_pos.mean_us, s.read_vel.mean_us,
          s.read_sensor.mean_us, s.sensor_proc.mean_us);
    }

    if (s.ft_infer_count > 0) {
      const std::size_t len = std::strlen(buf);
      std::snprintf(buf + len, sizeof(buf) - len,
                    "  ft=%.0f\xc2\xb5s", s.ft_infer.mean_us);
    }

    return std::string(buf);
  }

  void Reset() noexcept {
    ResetBase();
    ResetPhase(write_sum_, write_min_, write_max_);
    ResetPhase(read_pos_sum_, read_pos_min_, read_pos_max_);
    ResetPhase(read_vel_sum_, read_vel_min_, read_vel_max_);
    ResetPhase(read_sensor_sum_, read_sensor_min_, read_sensor_max_);
    ResetPhase(read_all_motor_sum_, read_all_motor_min_, read_all_motor_max_);
    ResetPhase(read_all_sensor_sum_, read_all_sensor_min_, read_all_sensor_max_);
    sensor_cycle_count_.store(0, std::memory_order_relaxed);
    is_bulk_mode_.store(false, std::memory_order_relaxed);
    individual_count_.store(0, std::memory_order_relaxed);
    bulk_count_.store(0, std::memory_order_relaxed);
    ResetPhase(sensor_proc_sum_, sensor_proc_min_, sensor_proc_max_);
    ResetPhase(ft_infer_sum_, ft_infer_min_, ft_infer_max_);
    ft_infer_count_.store(0, std::memory_order_relaxed);
  }

 private:
  std::atomic<double> write_sum_{0.0};
  std::atomic<double> write_min_{1e9};
  std::atomic<double> write_max_{0.0};
  std::atomic<double> read_pos_sum_{0.0};
  std::atomic<double> read_pos_min_{1e9};
  std::atomic<double> read_pos_max_{0.0};
  std::atomic<double> read_vel_sum_{0.0};
  std::atomic<double> read_vel_min_{1e9};
  std::atomic<double> read_vel_max_{0.0};
  std::atomic<double> read_sensor_sum_{0.0};
  std::atomic<double> read_sensor_min_{1e9};
  std::atomic<double> read_sensor_max_{0.0};
  std::atomic<double> read_all_motor_sum_{0.0};
  std::atomic<double> read_all_motor_min_{1e9};
  std::atomic<double> read_all_motor_max_{0.0};
  std::atomic<double> read_all_sensor_sum_{0.0};
  std::atomic<double> read_all_sensor_min_{1e9};
  std::atomic<double> read_all_sensor_max_{0.0};
  std::atomic<uint64_t> sensor_cycle_count_{0};
  std::atomic<bool> is_bulk_mode_{false};
  std::atomic<uint64_t> individual_count_{0};
  std::atomic<uint64_t> bulk_count_{0};
  std::atomic<double> sensor_proc_sum_{0.0};
  std::atomic<double> sensor_proc_min_{1e9};
  std::atomic<double> sensor_proc_max_{0.0};
  std::atomic<double> ft_infer_sum_{0.0};
  std::atomic<double> ft_infer_min_{1e9};
  std::atomic<double> ft_infer_max_{0.0};
  std::atomic<uint64_t> ft_infer_count_{0};
};

}  // namespace rtc

#endif  // UR5E_HAND_DRIVER_HAND_TIMING_PROFILER_HPP_
