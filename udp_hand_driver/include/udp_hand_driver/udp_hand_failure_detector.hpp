#ifndef UDP_HAND_DRIVER_UDP_HAND_FAILURE_DETECTOR_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_FAILURE_DETECTOR_HPP_

// Non-RT failure detector for hand data.
//
// Runs its own 50 Hz std::jthread.  Reads the latest UdpHandState from
// UdpHandController and checks two failure conditions:
//   1. All-zero data for N consecutive checks
//   2. Duplicate (unchanged) data for N consecutive checks
//
// On failure, invokes a registered callback (typically triggers global E-Stop).

#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/types/types.hpp"
#include "udp_hand_driver/udp_hand_controller.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>
#include <utility>

namespace udp_hand_driver {

struct UdpHandFailureDetectorConfig {
  int failure_threshold{5};         ///< 연속 감지 횟수 임계값
  bool check_motor{true};           ///< 모터 위치 데이터 검사
  bool check_sensor{true};          ///< 센서 데이터 검사
  bool sensor_force_layout{false};  ///< 1b: sensor_data(int32) 대신 sensor_force 의 fx,fy,fz 검사
  double min_rate_hz{30.0};         ///< 최소 허용 polling rate
  int rate_fail_threshold{5};       ///< 연속 N회 미달 시 failure
  bool check_link{true};            ///< UDP 링크 상태 검사
  // Per-channel strict link-down (#1): a channel latches link-down when ITS
  // consecutive-failure streak reaches the threshold at ITS effective attempt
  // rate. Motor/joint are attempted every comm cycle → link_fail_threshold;
  // sensor is attempted every sensor_decimation-th comm cycle → the shorter
  // link_fail_threshold_sensor (= comm threshold / sensor_decimation) so the
  // sensor channel latches within the same wall-clock budget, not sensor_dec×
  // later.
  int link_fail_threshold{10};         ///< motor/joint 연속 실패 임계 (comm-rate cycles)
  int link_fail_threshold_sensor{10};  ///< sensor 연속 실패 임계 (sensor-rate cycles)
  double startup_grace_ms{0.0};       ///< 기동 후 이 시간까지 RATE 판정 유예 (0=즉시)
  double link_startup_grace_ms{0.0};  ///< 기동 후 이 시간까지 LINK 판정 유예 (0=즉시, #2)
};

// Convert a time-based link-fail timeout (ms) into the consecutive-failure cycle
// count that CheckLink consumes, expressed in a CHANNEL's own attempt cadence.
// The comm loop reads at loop_rate_hz / comm_decimation (decimated skip cycles
// do no I/O and no failure accounting). A channel attempted every comm cycle
// (motor/joint) uses extra_decimation = 1; a channel attempted every N-th comm
// cycle (sensor, N = sensor_decimation) uses extra_decimation = N so the same
// wall-clock budget maps to fewer of its own attempts. So
//   cycles = timeout_ms/1000 · loop_rate_hz / (comm_decimation · extra_decimation),
// floored at 1 (a positive timeout must map to at least one attempt). The node
// owns this conversion so the detector's thresholds stay pure cycle counts and
// the detector need not know the loop rate. Free function for direct unit testing.
[[nodiscard]] inline int LinkFailCyclesFromTimeoutMs(double timeout_ms, double loop_rate_hz,
                                                     int comm_decimation,
                                                     int extra_decimation = 1) noexcept {
  const int dec = ClampCommDecimation(comm_decimation);
  const int extra = ClampCommDecimation(extra_decimation);
  const double cycles =
      timeout_ms / 1000.0 * loop_rate_hz / (static_cast<double>(dec) * static_cast<double>(extra));
  return std::max(1, static_cast<int>(std::lround(cycles)));
}

class UdpHandFailureDetector {
 public:
  using Config = UdpHandFailureDetectorConfig;
  using FailureCallback = std::function<void(const std::string&)>;

  /// @param controller   Reference to the UdpHandController to monitor.
  /// @param cfg          Detection configuration.
  /// @param thread_cfg   Thread scheduling / CPU affinity configuration.
  explicit UdpHandFailureDetector(UdpHandController& controller, Config cfg = Config{},
                                  rtc::ThreadConfig thread_cfg = rtc::kNrtLoggingConfig)
      : controller_(controller), cfg_(cfg), thread_cfg_(thread_cfg) {}

  ~UdpHandFailureDetector() { Stop(); }

  // Non-copyable, non-movable
  UdpHandFailureDetector(const UdpHandFailureDetector&) = delete;
  UdpHandFailureDetector& operator=(const UdpHandFailureDetector&) = delete;

  /// Register a callback invoked when a failure is detected.
  void SetFailureCallback(FailureCallback cb) { on_failure_ = std::move(cb); }

  /// Start the 50 Hz detector thread.
  void Start() {
    if (running_.load(std::memory_order_relaxed))
      return;
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::jthread([this](std::stop_token st) { DetectLoop(st); });
    RCLCPP_DEBUG(::udp_hand_driver::logging::FailureLogger(), "Detector thread started (50 Hz)");
  }

  /// Stop the detector thread.
  void Stop() {
    running_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) {
      thread_.request_stop();
      thread_.join();
    }
    RCLCPP_DEBUG(::udp_hand_driver::logging::FailureLogger(), "Detector thread stopped");
  }

  [[nodiscard]] bool failed() const noexcept { return failed_.load(std::memory_order_relaxed); }

 private:
  void DetectLoop(std::stop_token st) {
    using std::chrono_literals::operator""ms;
    // Detector runs on the nrt_logging config (SCHED_OTHER), so the strict
    // verbose apply is equivalent to the former graceful fallback here while
    // giving the same "[INFO] Thread '<name>' configured:" line as every other
    // runtime thread. The loop continues regardless of the result.
    (void)rtc::ApplyThreadConfigVerbose(thread_cfg_);
    loop_start_ = std::chrono::steady_clock::now();
    prev_rate_check_ = loop_start_;
    prev_cycle_count_ = controller_.cycle_count();

    while (!st.stop_requested() && running_.load(std::memory_order_relaxed)) {
      // Comm-loop-alive gate (#6). When the CommLoop self-exits on an external
      // E-Stop (or is being torn down by on_deactivate) it stores running_=false
      // and stops advancing cycle_count_ / publishing state — but this detector
      // jthread keeps polling until on_deactivate joins it. Re-evaluating the
      // frozen counter/state would spuriously raise hand_polling_rate_low (rate
      // 0), hand_motor_duplicate (unchanged frozen state), and redundantly
      // re-invoke the failure callback (SaveCommStats) — all AFTER the real cause
      // (the external E-Stop) already fired. Skip every check while the loop is
      // inactive. A genuine silent stall keeps running_=true, so CheckRate still
      // catches it; running_=false means an intentional stop, nothing to detect.
      if (!controller_.IsRunning()) {
        std::this_thread::sleep_for(20ms);
        continue;
      }
      {
        // L1 span covers one detector pass — the 20 ms pacing sleep below
        // stays outside so slices read as check cost, not the 50 Hz cadence.
        RTC_TRACE_SCOPE("hand_detector_tick");
        const UdpHandState state = controller_.GetLatestState();
        if (state.valid) {
          RTC_TRACE_SCOPE("hand_detector_check");
          Check(state);
        }
        // Startup grace, split per #2. The RATE grace (long) suppresses the
        // polling-rate check while the loop reaches steady cadence; while suppressed
        // keep the rate baseline fresh so the first post-grace CheckRate measures a
        // clean interval. The LINK grace (independent, much shorter) only spans the
        // ARP/firmware-boot first-packet transient, so a genuinely dead link latches
        // well below the CM device_timeout instead of waiting out the rate warmup.
        // Motor/sensor data-validity checks above are unaffected (they run once a
        // state read has arrived).
        if (InGrace(cfg_.startup_grace_ms)) {
          prev_rate_check_ = std::chrono::steady_clock::now();
          prev_cycle_count_ = controller_.cycle_count();
        } else {
          RTC_TRACE_SCOPE("hand_detector_rate");
          CheckRate();
        }
        if (cfg_.check_link && !InGrace(cfg_.link_startup_grace_ms)) {
          RTC_TRACE_SCOPE("hand_detector_link");
          CheckLink();
        }
      }
      std::this_thread::sleep_for(20ms);  // ~50 Hz
    }
  }

  // True while still inside a grace window of the given width measured from
  // loop_start_. grace ≤ 0 → always false (grace disabled). Shared by the rate
  // and link startup-grace gates (#2), which pass different widths.
  [[nodiscard]] bool InGrace(double grace_ms) const {
    if (grace_ms <= 0.0) {
      return false;
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - loop_start_)
            .count();
    return elapsed_ms < grace_ms;
  }

  void Check(const UdpHandState& state) {
    if (cfg_.check_motor) {
      CheckMotor(state);
    }
    if (cfg_.check_sensor) {
      CheckSensor(state);
    }
  }

  void CheckMotor(const UdpHandState& state) {
    // Motor-space read is protocol-optional (1b skips it entirely). Without a
    // fresh motor read the buffer stays zero/stale, which would otherwise
    // false-trigger all-zero and duplicate — gate on the per-cycle validity flag
    // so 1b (motor_valid always false) is a no-op and 1a ignores dropped reads.
    if (!state.motor_valid) {
      return;
    }

    const auto& pos = state.motor_positions;

    // All-zero check
    bool all_zero = true;
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumHandMotors); ++i) {
      if (pos[i] != 0.0f) {
        all_zero = false;
        break;
      }
    }
    if (all_zero) {
      ++motor_zero_count_;
    } else {
      motor_zero_count_ = 0;
    }

    // Duplicate check
    if (prev_motor_valid_ && pos == prev_motor_) {
      ++motor_dup_count_;
    } else {
      motor_dup_count_ = 0;
    }
    prev_motor_ = pos;
    prev_motor_valid_ = true;

    if (motor_zero_count_ >= cfg_.failure_threshold) {
      RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                  "Motor all-zero detected (count=%d, threshold=%d)", motor_zero_count_,
                  cfg_.failure_threshold);
      RaiseFailure("hand_motor_all_zero (count=" + std::to_string(motor_zero_count_) + ")");
    }
    if (motor_dup_count_ >= cfg_.failure_threshold) {
      RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                  "Motor duplicate detected (count=%d, threshold=%d)", motor_dup_count_,
                  cfg_.failure_threshold);
      RaiseFailure("hand_motor_duplicate (count=" + std::to_string(motor_dup_count_) + ")");
    }
  }

  void CheckSensor(const UdpHandState& state) {
    // Freshness gate. The detector (~50 Hz) polls faster than the decimated
    // sensor rate, so most polls carry the SAME published snapshot. Evaluate the
    // all-zero / duplicate heuristics ONLY when a fresh validated sensor read has
    // advanced the sequence; otherwise a byte-identical republish would read as a
    // false duplicate (#4), and a zero-init snapshot before the first read (seq
    // still 0) would read as false all-zero (#3). A genuine firmware stall — same
    // value returned on every REAL read — still advances the seq each cycle, so
    // it is caught here as a true duplicate.
    if (state.sensor_seq == prev_sensor_seq_) {
      return;
    }
    prev_sensor_seq_ = state.sensor_seq;

    bool all_zero = false;
    bool duplicate = false;
    if (cfg_.sensor_force_layout) {
      EvaluateForceSensor(state, all_zero, duplicate);
    } else {
      EvaluateRawSensor(state, all_zero, duplicate);
    }

    if (all_zero) {
      ++sensor_zero_count_;
    } else {
      sensor_zero_count_ = 0;
    }
    if (duplicate) {
      ++sensor_dup_count_;
    } else {
      sensor_dup_count_ = 0;
    }

    if (sensor_zero_count_ >= cfg_.failure_threshold) {
      RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                  "Sensor all-zero detected (count=%d, threshold=%d)", sensor_zero_count_,
                  cfg_.failure_threshold);
      RaiseFailure("hand_sensor_all_zero (count=" + std::to_string(sensor_zero_count_) + ")");
    }
    if (sensor_dup_count_ >= cfg_.failure_threshold) {
      RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                  "Sensor duplicate detected (count=%d, threshold=%d)", sensor_dup_count_,
                  cfg_.failure_threshold);
      RaiseFailure("hand_sensor_duplicate (count=" + std::to_string(sensor_dup_count_) + ")");
    }
  }

  // 1a path: barometer/ToF int32 buffer (sensor_data). All-zero + exact
  // duplicate over num_fingertips × kSensorValuesPerFingertip values.
  void EvaluateRawSensor(const UdpHandState& state, bool& all_zero, bool& duplicate) {
    const auto& sens = state.sensor_data;
    all_zero = true;
    const int num_sensors = state.num_fingertips * udp_hand_driver::kSensorValuesPerFingertip;
    for (int i = 0; i < num_sensors; ++i) {
      if (sens[static_cast<std::size_t>(i)] != 0) {
        all_zero = false;
        break;
      }
    }
    duplicate = prev_sensor_valid_ && sens == prev_sensor_;
    prev_sensor_ = sens;
    prev_sensor_valid_ = true;
  }

  // 1b path: per-fingertip firmware force (sensor_force). Only fx,fy,fz (offsets
  // 0,1,2 within the 6-value [fx,fy,fz,Lx,Ly,Temp] stride) are the real datum —
  // Lx/Ly/Temp are firmware placeholders and are excluded so a constant
  // placeholder can neither mask a genuine all-zero nor gate the duplicate
  // comparison. Firmware emits ADC noise at rest, so an exact-0 force block means
  // a dead/disconnected sensor and a bit-frozen fx,fy,fz block means a stalled
  // feed — both distinguishable from normal no-contact operation.
  void EvaluateForceSensor(const UdpHandState& state, bool& all_zero, bool& duplicate) {
    const int nf = std::min(state.num_fingertips, udp_hand_driver::kMaxFingertips);
    if (nf <= 0) {
      all_zero = false;
      duplicate = false;
      return;
    }
    constexpr std::size_t kStride =
        static_cast<std::size_t>(udp_hand_driver::kP1bValuesPerFingertip);
    all_zero = true;
    duplicate = prev_force_valid_;
    for (int f = 0; f < nf; ++f) {
      for (std::size_t j = 0; j < 3; ++j) {
        const std::size_t idx = static_cast<std::size_t>(f) * kStride + j;
        const float v = state.sensor_force[idx];
        if (v != 0.0f) {
          all_zero = false;
        }
        if (prev_force_valid_ && v != prev_sensor_force_[idx]) {
          duplicate = false;
        }
      }
    }
    prev_sensor_force_ = state.sensor_force;
    prev_force_valid_ = true;
  }

  void CheckRate() {
    const auto now = std::chrono::steady_clock::now();
    const double dt_sec = std::chrono::duration<double>(now - prev_rate_check_).count();
    if (dt_sec < 0.001)
      return;  // 너무 짧은 간격 무시

    const std::size_t current_cycles = controller_.cycle_count();
    const double rate_hz = static_cast<double>(current_cycles - prev_cycle_count_) / dt_sec;

    prev_rate_check_ = now;
    prev_cycle_count_ = current_cycles;

    if (rate_hz < cfg_.min_rate_hz) {
      ++rate_fail_count_;
    } else {
      rate_fail_count_ = 0;
    }

    if (rate_fail_count_ >= cfg_.rate_fail_threshold) {
      RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                  "Polling rate low: %.1f Hz (min=%.1f Hz, fail_count=%d)", rate_hz,
                  cfg_.min_rate_hz, rate_fail_count_);
      RaiseFailure("hand_polling_rate_low (rate=" + std::to_string(rate_hz) +
                   " Hz, min=" + std::to_string(cfg_.min_rate_hz) + " Hz)");
    }
  }

  void CheckLink() {
    const auto comm_thr = static_cast<uint32_t>(std::max(0, cfg_.link_fail_threshold));
    const auto sensor_thr = static_cast<uint32_t>(std::max(0, cfg_.link_fail_threshold_sensor));
    if (!controller_.LinkDown(comm_thr, sensor_thr)) {
      return;
    }
    // Name the latched channels + their streaks for the log/E-STOP reason.
    // Non-RT detector thread — string building is fine here.
    const std::string detail = LinkDownChannelDetail(comm_thr, sensor_thr);
    RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(), "UDP link down (%s)", detail.c_str());
    if (!link_dump_done_) {
      link_dump_done_ = true;
      DumpLinkForensics();
    }
    RaiseFailure("hand_udp_link_down (" + detail + ")");
  }

  // Build a "kind=streak/threshold" list of the channels that have latched, for
  // the link-down log + E-STOP reason. Detector thread only (allocates).
  [[nodiscard]] std::string LinkDownChannelDetail(uint32_t comm_thr, uint32_t sensor_thr) const {
    struct Ch {
      RequestKind kind;
      uint32_t thr;
    };

    const std::array<Ch, 4> channels = {{{RequestKind::kMotorRead, comm_thr},
                                         {RequestKind::kJointRead, comm_thr},
                                         {RequestKind::kSensorRead, sensor_thr},
                                         {RequestKind::kBulkSensor, sensor_thr}}};
    std::string detail;
    for (const auto& ch : channels) {
      const uint32_t streak = controller_.channel_consec_fail(ch.kind);
      if (ch.thr > 0 && streak >= ch.thr) {
        if (!detail.empty()) {
          detail += ", ";
        }
        detail += kRequestKindNames[static_cast<std::size_t>(ch.kind)];
        detail += "=" + std::to_string(streak) + "/" + std::to_string(ch.thr);
      }
    }
    return detail;
  }

  // One-shot forensic dump at link-down: per-request-kind comm stats table +
  // the last CycleOutcomeRing::kCapacity cycle outcomes. Non-RT detector
  // thread — string building / unthrottled WARN are fine here.
  void DumpLinkForensics() {
    const UdpHandCommStats stats = controller_.comm_stats();
    RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                "Link-down forensics: per-request stats "
                "(last_unexpected_cmd=0x%02X last_unexpected_len=%u):",
                static_cast<unsigned>(stats.last_unexpected_cmd),
                static_cast<unsigned>(stats.last_unexpected_len));
    for (int k = 0; k < kNumRequestKinds; ++k) {
      const auto& pk = stats.per_kind[static_cast<std::size_t>(k)];
      RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                  "  %-11s ok=%lu timeout=%lu error=%lu cmd_mismatch=%lu "
                  "mode_mismatch=%lu short_or_decode=%lu",
                  kRequestKindNames[static_cast<std::size_t>(k)], static_cast<unsigned long>(pk.ok),
                  static_cast<unsigned long>(pk.timeout), static_cast<unsigned long>(pk.error),
                  static_cast<unsigned long>(pk.cmd_mismatch),
                  static_cast<unsigned long>(pk.mode_mismatch),
                  static_cast<unsigned long>(pk.short_or_decode));
    }

    // Ring decode, oldest → newest: "seq:attempted>ok" per cycle, hex masks
    // (bit i = RequestKind i), chunked to keep individual log lines short.
    const CycleOutcomeRing ring = controller_.GetCycleOutcomeRing();
    const uint32_t n = std::min(ring.count, CycleOutcomeRing::kCapacity);
    constexpr uint32_t kPerLine = 16;
    std::string line;
    for (uint32_t i = 0; i < n; ++i) {
      const uint32_t idx = (ring.count - n + i) % CycleOutcomeRing::kCapacity;
      const auto& e = ring.entries[idx];
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%u:%02X>%02X ", e.cycle_seq,
                    static_cast<unsigned>(e.attempted_mask), static_cast<unsigned>(e.ok_mask));
      line += buf;
      if ((i + 1) % kPerLine == 0 || i + 1 == n) {
        RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(), "  cycles[%u..%u]: %s",
                    i - (i % kPerLine) + 1, i + 1, line.c_str());
        line.clear();
      }
    }
    if (n == 0) {
      RCLCPP_WARN(::udp_hand_driver::logging::FailureLogger(),
                  "  cycle outcome ring empty (no EventLoop cycles recorded)");
    }
  }

  void RaiseFailure(const std::string& reason) {
    bool expected = false;
    if (!failed_.compare_exchange_strong(expected, true))
      return;
    if (on_failure_) {
      on_failure_(reason);
    }
  }

  UdpHandController& controller_;
  Config cfg_;
  rtc::ThreadConfig thread_cfg_;
  FailureCallback on_failure_;

  std::jthread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> failed_{false};

  // Motor state
  std::array<float, kNumHandMotors> prev_motor_{};
  bool prev_motor_valid_{false};
  int motor_zero_count_{0};
  int motor_dup_count_{0};

  // Sensor state (1a: int32 barometer/ToF buffer)
  std::array<int32_t, udp_hand_driver::kMaxHandSensors> prev_sensor_{};
  bool prev_sensor_valid_{false};
  // Sensor state (1b: per-fingertip float force buffer)
  std::array<float, udp_hand_driver::kMaxSensorForceValues> prev_sensor_force_{};
  bool prev_force_valid_{false};
  int sensor_zero_count_{0};
  int sensor_dup_count_{0};
  // Last sensor-freshness sequence evaluated. Starts at 0 == the zero-init
  // UdpHandState::sensor_seq, so pre-first-read polls are skipped (see
  // CheckSensor). Bumped to the published seq on each evaluated cycle.
  uint32_t prev_sensor_seq_{0};

  // Rate monitoring state
  int rate_fail_count_{0};
  std::size_t prev_cycle_count_{0};
  std::chrono::steady_clock::time_point prev_rate_check_{};

  // DetectLoop start time — baseline for the startup grace window.
  std::chrono::steady_clock::time_point loop_start_{};

  // Link-down forensic dump one-shot latch (detector thread only)
  bool link_dump_done_{false};
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_UDP_HAND_FAILURE_DETECTOR_HPP_
