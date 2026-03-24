#ifndef UR5E_HAND_DRIVER_HAND_CONTROLLER_HPP_
#define UR5E_HAND_DRIVER_HAND_CONTROLLER_HPP_

// High-level hand controller: event-driven UDP driver.
//
// Uses a single UDP socket for both send and receive.
// Driven by ControlLoop events (pipeline mode):
//   Phase 4 of tick N: SendCommandAndRequestStates(cmd)
//     → Hand thread wakes and executes (individual mode):
//       1. Write position  (0x01) + recv echo → send 43B, recv 43B (header cmd만 검증)
//       2. Read position   (0x11)             → send  3B, recv 43B
//       3. Read velocity   (0x12)             → send  3B, recv 43B
//       4. Read sensor 0-3 (0x14..0x17) × 4   → send  3B, recv 67B
//     → Hand thread (bulk mode):
//       1. Write position  (0x01) + recv echo → send 43B, recv 43B (header cmd만 검증)
//       2. Read all motors (0x10)             → send  3B, recv 123B
//       3. Read all sensors (0x19)            → send  3B, recv 259B
//     → State ready for tick N+1
//   Phase 1 of tick N+1: GetLatestState() returns pre-fetched state
//
// RT safety:
//   - All hot-path operations are allocation-free.
//   - No printf/stdout on the EventLoop thread.
//   - Shared state uses SeqLock (lock-free) instead of mutex.

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/logging.hpp>

#include "rtc_base/types/types.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/timing/timing_profiler_base.hpp"
#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_base/filters/sensor_rate_estimator.hpp"
#include "rtc_base/filters/sliding_trend_detector.hpp"
#include "ur5e_hand_driver/fingertip_ft_inferencer.hpp"
#include "ur5e_hand_driver/hand_packets.hpp"
#include "ur5e_hand_driver/hand_udp_codec.hpp"

namespace rtc {

// ── Communication mode ────────────────────────────────────────────────────────
enum class HandCommunicationMode {
  kIndividual,  // 0x11 pos + 0x12 vel + 0x14~0x17 sensors (기존)
  kBulk,        // 0x10 all motors + 0x19 all sensors
};

// ── HandTimingProfiler ────────────────────────────────────────────────────────
//
// EventLoop의 per-phase 소요시간을 측정하고 히스토그램 기반 통계를 유지.
// TimingProfilerBase<50,100,2000>을 상속하여 공통 histogram/percentile 로직 재사용.
//
// Phase별 측정:
//   Individual: write → read_pos → read_vel → read_sensor → total
//   Bulk:       write → read_all_motor → read_all_sensor → total
//
// Thread safety:
//   Update()는 EventLoop 스레드에서만 호출 (single producer).
//   GetStats() / Summary()는 어떤 스레드에서든 호출 가능 (relaxed atomic).
//
class HandTimingProfiler : public TimingProfilerBase<50, 100, 2000> {
 public:
  using PhaseStats = typename TimingProfilerBase<50, 100, 2000>::PhaseStats;

  // 전체 통계 스냅샷 (BaseStats 확장)
  struct Stats : BaseStats {
    // Per-phase 평균 (individual mode)
    PhaseStats write;
    PhaseStats read_pos;
    PhaseStats read_vel;
    PhaseStats read_sensor;
    uint64_t   sensor_cycle_count{0};  // 센서 읽기가 실행된 cycle 수

    // Per-phase 평균 (bulk mode)
    PhaseStats read_all_motor;
    PhaseStats read_all_sensor;
    bool       is_bulk_mode{false};

    // Per-mode cycle counts (모드 전환 시 정확한 mean 계산용)
    uint64_t   individual_count{0};
    uint64_t   bulk_count{0};

    // Sensor processing phase (filter + drift detection, excl. FT)
    PhaseStats sensor_proc;

    // F/T inference phase
    PhaseStats ft_infer;
    uint64_t   ft_infer_count{0};  // FT 추론이 실행된 cycle 수
  };

  // Per-tick 측정값 (EventLoop에서 채워서 Update()에 전달)
  struct PhaseTiming {
    double write_us{0.0};
    // Individual mode phases
    double read_pos_us{0.0};
    double read_vel_us{0.0};
    double read_sensor_us{0.0};  // 0 if sensor decimation skipped
    // Bulk mode phases
    double read_all_motor_us{0.0};
    double read_all_sensor_us{0.0};  // 0 if sensor decimation skipped
    double sensor_proc_us{0.0};      // filter + drift detection (excl. FT)
    double ft_infer_us{0.0};  // 0 if FT inference disabled or not run
    double total_us{0.0};
    bool   is_sensor_cycle{false};
    bool   is_bulk_mode{false};
  };

  // ── Update (single-producer: EventLoop thread) ────────────────────────────

  void Update(const PhaseTiming& t) noexcept {
    // Base class: total histogram/min/max/sum 업데이트
    UpdateTotal(t.total_us);

    // Per-phase 누적
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

    // Sensor processing phase (공통 — bulk/individual 무관, sensor cycle만)
    if (t.is_sensor_cycle) {
      UpdatePhase(sensor_proc_sum_, sensor_proc_min_, sensor_proc_max_,
                  t.sensor_proc_us);
    }

    // F/T inference phase (공통 — bulk/individual 무관)
    if (t.ft_infer_us > 0.0) {
      ft_infer_count_.fetch_add(1, std::memory_order_relaxed);
      UpdatePhase(ft_infer_sum_, ft_infer_min_, ft_infer_max_, t.ft_infer_us);
    }
  }

  // ── Statistics snapshot ───────────────────────────────────────────────────

  [[nodiscard]] Stats GetStats() const noexcept {
    Stats s;
    // Base stats (count, min, max, mean, stddev, percentiles, histogram)
    static_cast<BaseStats&>(s) = GetBaseStats();

    s.individual_count = individual_count_.load(std::memory_order_relaxed);
    s.bulk_count = bulk_count_.load(std::memory_order_relaxed);

    if (s.count > 0) {
      // write는 모든 cycle에서 실행 → 전체 count로 나눔
      s.write = LoadPhaseStats(write_sum_, write_min_, write_max_, s.count);
    }

    // Individual mode phases — individual_count로 나눔
    if (s.individual_count > 0) {
      s.read_pos = LoadPhaseStats(read_pos_sum_, read_pos_min_, read_pos_max_,
                                  s.individual_count);
      s.read_vel = LoadPhaseStats(read_vel_sum_, read_vel_min_, read_vel_max_,
                                  s.individual_count);
    }

    s.is_bulk_mode = is_bulk_mode_.load(std::memory_order_relaxed);
    // Bulk mode phases — bulk_count로 나눔
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

    // Sensor processing stats (filter + drift detection)
    if (s.sensor_cycle_count > 0) {
      s.sensor_proc = LoadPhaseStats(sensor_proc_sum_, sensor_proc_min_,
                                     sensor_proc_max_, s.sensor_cycle_count);
    }

    // F/T inference stats
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

    // FT inference stats (조건부 출력)
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
  // Per-phase 누적 (sum + min + max)
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
  // Bulk mode phase accumulators
  std::atomic<double> read_all_motor_sum_{0.0};
  std::atomic<double> read_all_motor_min_{1e9};
  std::atomic<double> read_all_motor_max_{0.0};
  std::atomic<double> read_all_sensor_sum_{0.0};
  std::atomic<double> read_all_sensor_min_{1e9};
  std::atomic<double> read_all_sensor_max_{0.0};
  std::atomic<uint64_t> sensor_cycle_count_{0};
  std::atomic<bool> is_bulk_mode_{false};
  // Per-mode cycle counts
  std::atomic<uint64_t> individual_count_{0};
  std::atomic<uint64_t> bulk_count_{0};
  // Sensor processing phase accumulators (filter + drift detection)
  std::atomic<double> sensor_proc_sum_{0.0};
  std::atomic<double> sensor_proc_min_{1e9};
  std::atomic<double> sensor_proc_max_{0.0};
  // F/T inference phase accumulators
  std::atomic<double> ft_infer_sum_{0.0};
  std::atomic<double> ft_infer_min_{1e9};
  std::atomic<double> ft_infer_max_{0.0};
  std::atomic<uint64_t> ft_infer_count_{0};
};

class HandController {
 public:
  using StateCallback = std::function<void(const HandState&, const FingertipFTState&)>;

  // 통신 통계 (recv 성공/타임아웃/에러 및 총 사이클 수)
  struct HandCommStats {
    uint64_t recv_ok{0};
    uint64_t recv_timeout{0};
    uint64_t recv_error{0};
    uint64_t cmd_mismatch{0};         // response cmd != request cmd (stale packet)
    uint64_t mode_mismatch{0};        // sensor response mode != kRaw
    uint64_t total_cycles{0};
    uint64_t event_skip_count{0};     // EventLoop busy 중 skip된 이벤트 수
  };

  explicit HandController(
      std::string target_ip,
      int target_port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig,
      double recv_timeout_ms = 10.0,
      bool /*enable_write_ack*/ = false,  // deprecated: echo is always consumed
      int sensor_decimation = 1,
      int num_fingertips = kDefaultNumFingertips,
      bool use_fake_hand = false,
      const std::vector<std::string>& fingertip_names = {},
      HandCommunicationMode communication_mode = HandCommunicationMode::kIndividual,
      bool tof_lpf_enabled = false,
      double tof_lpf_cutoff_hz = 15.0,
      bool baro_lpf_enabled = false,
      double baro_lpf_cutoff_hz = 30.0,
      FingertipFTInferencer::Config ft_config = {},
      bool drift_detection_enabled = false,
      double drift_threshold = 5.0,
      int drift_window_size = 2500) noexcept
      : target_ip_(std::move(target_ip)),
        target_port_(target_port),
        thread_cfg_(thread_cfg),
        recv_timeout_ms_(recv_timeout_ms),
        sensor_decimation_(sensor_decimation < 1 ? 1 : sensor_decimation),
        num_fingertips_(num_fingertips > kMaxFingertips ? kMaxFingertips
                       : (num_fingertips < 0 ? 0 : num_fingertips)),
        use_fake_hand_(use_fake_hand),
        fingertip_names_(fingertip_names.empty()
                         ? kDefaultFingertipNames
                         : fingertip_names),
        communication_mode_(communication_mode),
        tof_lpf_enabled_(tof_lpf_enabled),
        tof_lpf_cutoff_hz_(tof_lpf_cutoff_hz),
        baro_lpf_enabled_(baro_lpf_enabled),
        baro_lpf_cutoff_hz_(baro_lpf_cutoff_hz),
        ft_config_(std::move(ft_config)),
        drift_detection_enabled_(drift_detection_enabled),
        drift_threshold_(drift_threshold),
        drift_window_size_(drift_window_size)
  {
    // fingertip_names 갯수로 num_fingertips_ 결정
    const int name_count = static_cast<int>(fingertip_names_.size());
    if (name_count < num_fingertips_) {
      num_fingertips_ = name_count;
    }
  }

  ~HandController() { Stop(); }

  HandController(const HandController&)            = delete;
  HandController& operator=(const HandController&) = delete;
  HandController(HandController&&)                 = delete;
  HandController& operator=(HandController&&)      = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────

  [[nodiscard]] bool Start() {
    // F/T 추론기 초기화 (fake/real 공통 — EventLoop 불필요)
    RCLCPP_INFO(rclcpp::get_logger("HandController"),
                "FT config: enabled=%d, num_fingertips=%d, "
                "model_paths.size=%zu, calibration_enabled=%d, calibration_samples=%d",
                ft_config_.enabled ? 1 : 0, num_fingertips_,
                ft_config_.model_paths.size(),
                ft_config_.calibration_enabled ? 1 : 0,
                ft_config_.calibration_samples);
    if (ft_config_.enabled && num_fingertips_ > 0) {
      for (std::size_t i = 0; i < ft_config_.model_paths.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("HandController"),
                    "FT model_path[%zu]=\"%s\"",
                    i, ft_config_.model_paths[i].c_str());
      }
      ft_inferencer_ = std::make_unique<FingertipFTInferencer>();
      try {
        ft_inferencer_->InitFT(ft_config_);
        ft_enabled_ = ft_inferencer_->is_initialized();
        RCLCPP_INFO(rclcpp::get_logger("HandController"),
                    "FT init OK: initialized=%d, num_active_models=%d, calibrated=%d",
                    ft_inferencer_->is_initialized() ? 1 : 0,
                    ft_inferencer_->num_models(),
                    ft_inferencer_->is_calibrated() ? 1 : 0);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("HandController"),
                     "FT init FAILED: %s", e.what());
        ft_enabled_ = false;
        ft_inferencer_.reset();
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("HandController"),
                     "FT init FAILED: unknown exception");
        ft_enabled_ = false;
        ft_inferencer_.reset();
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HandController"),
                  "FT inference SKIPPED: enabled=%d, num_fingertips=%d",
                  ft_config_.enabled ? 1 : 0, num_fingertips_);
    }

    if (use_fake_hand_) {
      // Fake mode: UDP 소켓/스레드 없이 echo-back만 수행
      running_.store(true, std::memory_order_release);
      return true;
    }

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) return false;

    // Resolve target address.
    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port   = htons(static_cast<uint16_t>(target_port_));
    if (inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) <= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    // 소형 패킷(43~67B) 전용 소켓 — 버퍼를 명시적으로 설정하여
    // rmem_default/wmem_default 커널 기본값에 의존하지 않음.
    constexpr int kUdpBufSize = 65536;  // 64 KB
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &kUdpBufSize, sizeof(kUdpBufSize));
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &kUdpBufSize, sizeof(kUdpBufSize));

    // Sub-ms recv timeout: ppoll() 기반 (hrtimer on PREEMPT_RT).
    // SO_RCVTIMEO는 jiffies 해상도(HZ=1000→1ms)로 sub-ms 불가능.
    // ppoll은 struct timespec (ns 단위) → hrtimer → µs 정밀도 제공.
    // 안전망으로 SO_RCVTIMEO도 100ms로 설정 (ppoll 실패 시 무한 블록 방지).
    {
      struct timeval tv{};
      tv.tv_sec  = 0;
      tv.tv_usec = 100'000;  // 100ms safety fallback
      setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    // 센서 초기화: NN → RAW 모드 전환 (num_fingertips > 0일 때만)
    if (num_fingertips_ > 0) {
      sensor_init_ok_ = InitializeSensors();
    }

    // 센서 rate estimator 초기화 (nominal 500Hz, warmup 후 실측 rate로 filter 재초기화)
    if (num_fingertips_ > 0) {
      constexpr double kNominalRateHz = 500.0;
      sensor_rate_estimator_.Init(kNominalRateHz);

      // 센서 LPF 초기 Init (nominal rate 기반 — warmup 후 실측 rate로 재초기화)
      const double nominal_effective_rate =
          kNominalRateHz / static_cast<double>(sensor_decimation_);

      if (baro_lpf_enabled_) {
        try {
          baro_filter_.Init(baro_lpf_cutoff_hz_, nominal_effective_rate);
          baro_filter_active_ = true;
        } catch (...) {
          baro_filter_active_ = false;
        }
      }

      if (tof_lpf_enabled_) {
        try {
          tof_filter_.Init(tof_lpf_cutoff_hz_, nominal_effective_rate);
          tof_filter_active_ = true;
        } catch (...) {
          tof_filter_active_ = false;
        }
      }

      // Drift detector 초기화 (one-shot: window_size 샘플 축적 후 1회 판정)
      if (drift_detection_enabled_) {
        drift_detector_.Init(
            static_cast<std::size_t>(drift_window_size_),
            drift_threshold_);
      }
    }

    running_.store(true, std::memory_order_release);
    event_thread_ = std::jthread([this](std::stop_token st) {
      EventLoop(std::move(st));
    });

    return true;
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    if (use_fake_hand_) { return; }  // fake mode: 소켓/스레드 없음
    event_thread_.request_stop();
    // condvar에 대기 중인 스레드를 깨움
    event_cv_.notify_all();
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  // ── Callback ───────────────────────────────────────────────────────────

  void SetCallback(StateCallback cb) noexcept {
    callback_ = std::move(cb);
  }

  // ── E-Stop flag (shared with RtControllerNode) ─────────────────────────

  void SetEstopFlag(std::atomic<bool>* flag) noexcept {
    estop_flag_ = flag;
  }

  // ── Event-driven API (called from ControlLoop) ─────────────────────────

  /// ControlLoop Phase 4에서 호출: 명령 전송 + 다음 tick용 state 읽기를 요청.
  /// Hand thread를 깨워서 write → read position → read velocity → read sensors 수행.
  /// Non-blocking — condvar notify만 하고 즉시 리턴 (~1µs).
  /// EventLoop이 busy면 skip하고 event_skip_count 증가 (이전 state 유지).
  void SendCommandAndRequestStates(
      const std::array<float, kNumHandMotors>& cmd) noexcept {
    // Fake mode: command를 즉시 position으로 echo-back (통신 없음)
    if (use_fake_hand_) {
      HandState fake_state{};
      std::copy(cmd.begin(), cmd.end(), fake_state.motor_positions.begin());
      fake_state.num_fingertips = num_fingertips_;
      fake_state.valid = true;

      // Fake mode F/T 추론 (센서 데이터는 0 — 파이프라인 테스트용)
      if (ft_enabled_ && ft_inferencer_) {
        if (!ft_inferencer_->is_calibrated()) {
          static_cast<void>(ft_inferencer_->FeedCalibration(
              fake_state.sensor_data, num_fingertips_));
        } else {
          auto ft_result = ft_inferencer_->Infer(
              fake_state.sensor_data, num_fingertips_);
          ft_seqlock_.Store(ft_result);
        }
      }

      state_seqlock_.Store(fake_state);
      if (callback_) {
        callback_(fake_state, ft_seqlock_.Load());
      }
      cycle_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    if (busy_.load(std::memory_order_acquire)) {
      event_skip_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    {
      std::lock_guard lock(event_mutex_);
      staged_cmd_ = cmd;
      event_pending_ = true;
    }
    event_cv_.notify_one();
  }

  // ── Legacy API (standalone hand_udp_node 호환) ─────────────────────────

  void SetTargetPositions(const std::array<float, kNumHandMotors>& positions) noexcept {
    SendCommandAndRequestStates(positions);
  }

  // ── State access ───────────────────────────────────────────────────────

  [[nodiscard]] HandState GetLatestState() const noexcept {
    return state_seqlock_.Load();
  }

  [[nodiscard]] std::array<float, kNumHandMotors> GetLatestPositions() const noexcept {
    return state_seqlock_.Load().motor_positions;
  }

  [[nodiscard]] bool IsRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
  }

  // ── F/T inference accessors ──────────────────────────────────────────────

  [[nodiscard]] FingertipFTState GetLatestFTState() const noexcept {
    return ft_seqlock_.Load();
  }

  [[nodiscard]] bool ft_inference_enabled() const noexcept {
    return ft_enabled_;
  }

  /// Returns true if sensor initialization (NN→RAW) succeeded.
  [[nodiscard]] bool IsSensorInitialized() const noexcept {
    return sensor_init_ok_;
  }

  [[nodiscard]] std::size_t cycle_count() const noexcept {
    return cycle_count_.load(std::memory_order_relaxed);
  }

  // recv() 타임아웃/에러 발생 횟수 (any thread, relaxed).
  [[nodiscard]] uint64_t recv_error_count() const noexcept {
    return recv_error_count_.load(std::memory_order_relaxed);
  }

  // EventLoop busy 중 skip된 이벤트 수
  [[nodiscard]] uint64_t event_skip_count() const noexcept {
    return event_skip_count_.load(std::memory_order_relaxed);
  }

  // 연속 recv 전체 실패 횟수 (0이면 link alive)
  [[nodiscard]] uint64_t consecutive_recv_failures() const noexcept {
    return consecutive_recv_failures_.load(std::memory_order_relaxed);
  }

  // 통신 통계 스냅샷 반환 (struct copy — relaxed read로 충분)
  [[nodiscard]] HandCommStats comm_stats() const noexcept {
    HandCommStats stats = comm_stats_;
    stats.event_skip_count = event_skip_count_.load(std::memory_order_relaxed);
    return stats;
  }

  // 타이밍 통계 스냅샷 반환
  [[nodiscard]] HandTimingProfiler::Stats timing_stats() const noexcept {
    return timing_profiler_.GetStats();
  }

  // 타이밍 요약 문자열 반환 (RCLCPP_INFO 출력용)
  [[nodiscard]] std::string TimingSummary() const noexcept {
    return timing_profiler_.Summary();
  }

  // 통신 모드 반환
  [[nodiscard]] HandCommunicationMode communication_mode() const noexcept {
    return communication_mode_;
  }

  // 설정된 recv timeout (ms) 반환
  [[nodiscard]] double recv_timeout_ms() const noexcept {
    return recv_timeout_ms_;
  }

  // 실측 sensor sampling rate (Hz). SensorRateEstimator 기반.
  [[nodiscard]] double actual_sensor_rate_hz() const noexcept {
    return sensor_rate_estimator_.rate_hz();
  }

 private:
  // Sub-ms precision recv using ppoll (hrtimer on PREEMPT_RT kernels).
  // SO_RCVTIMEO uses schedule_timeout() → jiffies resolution (1ms on HZ=1000).
  // ppoll uses struct timespec (ns) → hrtimer → true µs-level precision.
  // Returns bytes received, or -1 with errno=EAGAIN on timeout.
  [[nodiscard]] ssize_t RecvWithTimeout(
      void* buf, std::size_t len) noexcept {
    struct pollfd pfd{};
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    struct timespec ts{};
    const long total_ns = static_cast<long>(recv_timeout_ms_ * 1'000'000.0);
    ts.tv_sec  = total_ns / 1'000'000'000;
    ts.tv_nsec = total_ns % 1'000'000'000;

    const int ret = ::ppoll(&pfd, 1, &ts, nullptr);
    if (ret <= 0) {
      errno = EAGAIN;
      return -1;
    }
    return ::recv(socket_fd_, buf, len, MSG_DONTWAIT);
  }

  // Send raw bytes and receive into a buffer. Returns bytes received, or -1 on error.
  [[nodiscard]] ssize_t SendAndRecvRaw(
      const uint8_t* send_data, std::size_t send_len,
      uint8_t* recv_data, std::size_t recv_len) noexcept {
    const ssize_t sent = sendto(
        socket_fd_, send_data, send_len, 0,
        reinterpret_cast<const sockaddr*>(&target_addr_),
        sizeof(target_addr_));
    if (sent < 0) return -1;

    const ssize_t recvd = RecvWithTimeout(recv_data, recv_len);
    if (recvd < 0) {
      recv_error_count_.fetch_add(1, std::memory_order_relaxed);
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        ++comm_stats_.recv_timeout;
      } else {
        ++comm_stats_.recv_error;
      }
    } else {
      ++comm_stats_.recv_ok;
    }
    return recvd;
  }

  // Request a motor read command and decode 10 floats.
  // Sends header only (3 bytes) — data payload is not needed for read requests.
  // Verifies response cmd matches the request to reject stale packets.
  // 첫 recv만 ppoll 대기(hrtimer), cmd 불일치 시 non-blocking retry로 stale 소비.
  [[nodiscard]] bool RequestMotorRead(
      hand_packets::Command cmd,
      std::array<float, hand_packets::kMotorDataCount>& out) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};  // 3B header only
    std::array<uint8_t, hand_packets::kMotorPacketSize> recv_buf{};

    hand_udp_codec::EncodeMotorReadRequest(cmd, send_buf);

    const ssize_t sent = sendto(
        socket_fd_, send_buf.data(), send_buf.size(), 0,
        reinterpret_cast<const sockaddr*>(&target_addr_),
        sizeof(target_addr_));
    if (sent < 0) return false;

    // 최대 3회 시도: 첫 recv만 ppoll 대기, 이후는 non-blocking (stale packet 소비)
    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd = (attempt == 0)
          ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
          : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          recv_error_count_.fetch_add(1, std::memory_order_relaxed);
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ++comm_stats_.recv_timeout;
          } else {
            ++comm_stats_.recv_error;
          }
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(hand_packets::kMotorPacketSize)) continue;

      uint8_t cmd_out, mode_out;
      if (!hand_udp_codec::DecodeMotorResponse(
              recv_buf.data(), static_cast<std::size_t>(recvd),
              cmd_out, mode_out, out)) {
        continue;
      }
      // Verify response cmd matches request — reject stale/echo packets
      if (cmd_out == static_cast<uint8_t>(cmd)) {
        return true;
      }
      ++comm_stats_.cmd_mismatch;
    }
    return false;
  }

  // Send a set-sensor-mode command (CMD=0x04, 3B send, 3B recv echo).
  // Must be called once after sensor power-on to switch from NN to RAW mode.
  // Returns true only when response mode field confirms the requested mode.
  [[nodiscard]] bool RequestSetSensorMode(
      hand_packets::SensorMode sensor_mode) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kSensorRequestSize> recv_buf{};

    hand_udp_codec::EncodeSetSensorMode(sensor_mode, send_buf);
    const ssize_t recvd = SendAndRecvRaw(
        send_buf.data(), send_buf.size(),
        recv_buf.data(), recv_buf.size());
    if (recvd < static_cast<ssize_t>(hand_packets::kSensorRequestSize)) {
      return false;  // 수신 실패 (타임아웃)
    }
    // Response mode 필드(offset 2)가 요청한 모드와 일치하는지 검증
    return recv_buf[2] == static_cast<uint8_t>(sensor_mode);
  }

  // Sensor initialization: switch from NN (power-on default) to RAW mode.
  // Retries up to max_retries times with retry_interval_ms between attempts.
  // Returns true only when the response confirms RAW mode.
  [[nodiscard]] bool InitializeSensors(
      int max_retries = 5,
      int retry_interval_ms = 100) noexcept {
    for (int attempt = 0; attempt < max_retries; ++attempt) {
      if (RequestSetSensorMode(hand_packets::SensorMode::kRaw)) {
        return true;  // Response mode == kRaw 확인됨
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(retry_interval_ms));
    }
    return false;  // max_retries 초과 — 초기화 실패
  }

  // Drain stale UDP responses from the socket buffer (non-blocking).
  void DrainStaleResponses() noexcept {
    std::array<uint8_t, hand_packets::kMaxPacketSize> discard{};
    for (int i = 0; i < 8; ++i) {
      const ssize_t r = ::recv(socket_fd_, discard.data(), discard.size(),
                               MSG_DONTWAIT);
      if (r <= 0) break;  // 버퍼 비어있으면 종료
    }
  }

  // Request a sensor read command (3B send) and decode 11 useful raw int32 values (67B recv).
  // MODE field in request carries the desired sensor mode (default kRaw).
  // Response cmd must match the request cmd; stale responses are discarded via non-blocking retry.
  //
  // 첫 recv만 ppoll 대기(hrtimer)으로 HW 응답 대기.
  // cmd 불일치 시 retry recv는 MSG_DONTWAIT로 즉시 반환하여 timeout spike 방지.
  [[nodiscard]] bool RequestSensorRead(
      hand_packets::Command cmd,
      std::array<int32_t, kSensorValuesPerFingertip>& out,
      hand_packets::SensorMode sensor_mode = hand_packets::SensorMode::kRaw,
      uint8_t* response_mode = nullptr,
      uint8_t* response_cmd = nullptr) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kSensorResponseSize> recv_buf{};

    hand_udp_codec::EncodeSensorReadRequest(cmd, send_buf, sensor_mode);

    const ssize_t sent = sendto(
        socket_fd_, send_buf.data(), send_buf.size(), 0,
        reinterpret_cast<const sockaddr*>(&target_addr_),
        sizeof(target_addr_));
    if (sent < 0) return false;

    // 최대 3회 시도: 첫 recv만 ppoll 대기, 이후는 non-blocking
    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      // attempt 0: ppoll + recv (hrtimer 기반 sub-ms 정밀도)
      // attempt 1+: non-blocking recv — 소켓 버퍼의 stale 패킷만 소비
      const ssize_t recvd = (attempt == 0)
          ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
          : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          recv_error_count_.fetch_add(1, std::memory_order_relaxed);
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ++comm_stats_.recv_timeout;
          } else {
            ++comm_stats_.recv_error;
          }
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(hand_packets::kSensorResponseSize)) continue;

      uint8_t cmd_out, mode_out;
      const bool ok = hand_udp_codec::DecodeSensorResponseRaw(
          recv_buf.data(), static_cast<std::size_t>(recvd),
          cmd_out, mode_out, out);
      if (!ok) continue;

      if (response_mode) { *response_mode = mode_out; }
      if (response_cmd)  { *response_cmd = cmd_out; }

      if (cmd_out != static_cast<uint8_t>(cmd)) {
        // cmd 불일치 — stale 응답, non-blocking으로 다음 패킷 시도
        ++comm_stats_.cmd_mismatch;
        continue;
      }
      // sensor response mode는 반드시 kRaw여야 함
      if (mode_out != static_cast<uint8_t>(sensor_mode)) {
        ++comm_stats_.mode_mismatch;
        return false;  // mode 불일치는 HW 상태 문제 — retry 무의미
      }
      return true;
    }
    return false;
  }

  // Request bulk motor read (cmd=0x10): 3B send, 123B recv.
  // Extracts positions[10] and velocities[10] from grouped response.
  // Verifies response cmd matches 0x10 to reject stale packets.
  // 첫 recv만 ppoll 대기(hrtimer), cmd 불일치 시 non-blocking retry로 stale 소비.
  [[nodiscard]] bool RequestAllMotorRead(
      std::array<float, hand_packets::kMotorDataCount>& positions,
      std::array<float, hand_packets::kMotorDataCount>& velocities) noexcept {
    std::array<uint8_t, hand_packets::kAllMotorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kAllMotorResponseSize> recv_buf{};

    hand_udp_codec::EncodeReadAllMotorsRequest(send_buf);

    const ssize_t sent = sendto(
        socket_fd_, send_buf.data(), send_buf.size(), 0,
        reinterpret_cast<const sockaddr*>(&target_addr_),
        sizeof(target_addr_));
    if (sent < 0) return false;

    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd = (attempt == 0)
          ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
          : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          recv_error_count_.fetch_add(1, std::memory_order_relaxed);
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ++comm_stats_.recv_timeout;
          } else {
            ++comm_stats_.recv_error;
          }
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(hand_packets::kAllMotorResponseSize)) continue;

      uint8_t cmd_out, mode_out;
      if (!hand_udp_codec::DecodeAllMotorResponse(
              recv_buf.data(), static_cast<std::size_t>(recvd),
              cmd_out, mode_out, positions, velocities)) {
        continue;
      }
      if (cmd_out == static_cast<uint8_t>(hand_packets::Command::kReadAllMotors)) {
        return true;
      }
      ++comm_stats_.cmd_mismatch;
    }
    return false;
  }

  // Request bulk sensor read (cmd=0x19): 3B send, 259B recv.
  // Extracts all fingertip sensor data into flat buffer.
  // Verifies response cmd matches 0x19 to reject stale packets.
  // 첫 recv만 ppoll 대기(hrtimer), cmd 불일치 시 non-blocking retry로 stale 소비.
  [[nodiscard]] bool RequestAllSensorRead(
      int32_t* out, int num_fingertips,
      hand_packets::SensorMode sensor_mode = hand_packets::SensorMode::kRaw) noexcept {
    std::array<uint8_t, hand_packets::kAllSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kAllSensorResponseSize> recv_buf{};

    hand_udp_codec::EncodeReadAllSensorsRequest(send_buf, sensor_mode);

    const ssize_t sent = sendto(
        socket_fd_, send_buf.data(), send_buf.size(), 0,
        reinterpret_cast<const sockaddr*>(&target_addr_),
        sizeof(target_addr_));
    if (sent < 0) return false;

    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd = (attempt == 0)
          ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
          : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          recv_error_count_.fetch_add(1, std::memory_order_relaxed);
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ++comm_stats_.recv_timeout;
          } else {
            ++comm_stats_.recv_error;
          }
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(hand_packets::kAllSensorResponseSize)) continue;

      uint8_t cmd_out, mode_out;
      if (!hand_udp_codec::DecodeAllSensorResponseRaw(
              recv_buf.data(), static_cast<std::size_t>(recvd),
              cmd_out, mode_out, out, num_fingertips)) {
        continue;
      }
      if (cmd_out != static_cast<uint8_t>(hand_packets::Command::kReadAllSensors)) {
        ++comm_stats_.cmd_mismatch;
        continue;
      }
      // sensor response mode는 반드시 요청한 mode(kRaw)여야 함
      if (mode_out != static_cast<uint8_t>(sensor_mode)) {
        ++comm_stats_.mode_mismatch;
        return false;  // mode 불일치는 HW 상태 문제 — retry 무의미
      }
      return true;
    }
    return false;
  }

  // Event-driven loop: condvar 대기 → ControlLoop 이벤트 수신 시 write + read 수행.
  // Pipeline: tick N의 Phase 4에서 write + read → tick N+1의 Phase 1에서 state 사용.
  //
  // Communication mode:
  //   Individual: write+echo → 0x11 pos → 0x12 vel → 0x14~0x17 sensors (6 round-trips)
  //   Bulk:       write+echo → 0x10 all motors → 0x19 all sensors (3 round-trips)
  //
  // WritePosition echo: 하드웨어가 43B echo를 반환하므로 항상 수신.
  //   echo의 data 부분은 의미 없음 — header cmd(0x01)만 검증하여 소켓 버퍼 drain.
  //   모든 read는 request → response → cmd 검증 패턴을 따름.
  //
  // Sensor decimation: sensor_decimation_ 주기마다 센서 읽기 수행.
  //   decimation=1 → 매 cycle (기존), decimation=4 → 4 cycle마다.
  //   센서 skip 시 이전 sensor_data 유지.
  //
  // RT safety: No printf/stdout — debug data available via HandTimingProfiler.
  void EventLoop(std::stop_token stop_token) {
    (void)ApplyThreadConfig(thread_cfg_);

    const bool is_bulk = (communication_mode_ == HandCommunicationMode::kBulk);

    std::array<uint8_t, hand_packets::kMotorPacketSize> send_buf{};
    std::array<uint8_t, hand_packets::kMotorPacketSize> echo_buf{};
    std::array<float, hand_packets::kMotorDataCount> motor_pos_buf{};
    std::array<float, hand_packets::kMotorDataCount> motor_vel_buf{};
    std::array<int32_t, kSensorValuesPerFingertip> sensor_raw_buf{};
    std::array<float, kNumHandMotors> pending_cmd{};

    // Sensor decimation: 이전 cycle의 sensor_data를 유지하기 위한 버퍼
    std::array<int32_t, kMaxHandSensors> cached_sensor_data{};
    int sensor_cycle_counter = 0;

    while (!stop_token.stop_requested()) {
      // condvar 대기 — ControlLoop의 SendCommandAndRequestStates()가 깨움
      {
        std::unique_lock lock(event_mutex_);
        event_cv_.wait(lock, [&] {
          return event_pending_ || stop_token.stop_requested();
        });
        if (stop_token.stop_requested()) break;
        pending_cmd = staged_cmd_;
        event_pending_ = false;
      }

      // busy 플래그 — 이 동안 도착하는 이벤트는 skip됨
      busy_.store(true, std::memory_order_release);

      // E-Stop 체크: 전역 E-Stop 발생 시 zero 명령 전송 후 루프 종료
      if (estop_flag_ && estop_flag_->load(std::memory_order_acquire)) {
        std::array<float, kNumHandMotors> zeros{};
        std::array<uint8_t, hand_packets::kMotorPacketSize> zero_buf{};
        hand_udp_codec::EncodeWritePosition(zeros, zero_buf);
        sendto(socket_fd_, zero_buf.data(), zero_buf.size(), 0,
               reinterpret_cast<const sockaddr*>(&target_addr_),
               sizeof(target_addr_));
        busy_.store(false, std::memory_order_release);
        break;
      }

      HandState state{};
      bool any_recv_ok = false;
      double ft_infer_elapsed_us = 0.0;
      std::array<int32_t, kMaxHandSensors> cached_sensor_data_raw{};

      // ── Timing: t0 = EventLoop 시작 ──
      const auto t0 = std::chrono::steady_clock::now();

      // 1. Write position + recv echo (소켓 버퍼 drain, header cmd만 검증)
      //    echo = 43B, 하드웨어 반환. data 부분은 의미 없음 — cmd(0x01)만 확인.
      hand_udp_codec::EncodeWritePosition(pending_cmd, send_buf);
      sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
             reinterpret_cast<const sockaddr*>(&target_addr_),
             sizeof(target_addr_));
      bool echo_ok = false;
      {
        const ssize_t recvd = RecvWithTimeout(echo_buf.data(), echo_buf.size());
        if (recvd >= static_cast<ssize_t>(hand_packets::kHeaderSize)) {
          echo_ok = (echo_buf[1] == static_cast<uint8_t>(
                         hand_packets::Command::kWritePosition));
          if (!echo_ok) {
            ++comm_stats_.cmd_mismatch;
          }
          ++comm_stats_.recv_ok;
        } else if (recvd < 0) {
          recv_error_count_.fetch_add(1, std::memory_order_relaxed);
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ++comm_stats_.recv_timeout;
          } else {
            ++comm_stats_.recv_error;
          }
        }
      }

      const auto t1 = std::chrono::steady_clock::now();  // write+echo 완료

      // ── Sensor decimation counter (공통) ──
      ++sensor_cycle_counter;
      const bool is_sensor_cycle = (sensor_cycle_counter >= sensor_decimation_);
      if (is_sensor_cycle) sensor_cycle_counter = 0;

      if (is_bulk) {
        // ══════════════════════════════════════════════════════════════════
        // Bulk mode: echo(position) + 0x10 all motors + 0x19 all sensors
        // ══════════════════════════════════════════════════════════════════

        // 2. Read all motors (0x10, 3B → 123B) — position + velocity + current
        if (RequestAllMotorRead(motor_pos_buf, motor_vel_buf)) {
          std::copy_n(motor_pos_buf.begin(), kNumHandMotors,
                      state.motor_positions.begin());
          std::copy_n(motor_vel_buf.begin(), kNumHandMotors,
                      state.motor_velocities.begin());
          any_recv_ok = true;
        }

        const auto t2 = std::chrono::steady_clock::now();  // read_all_motor 완료

        // 3. Read all sensors (0x19, 3B → 259B) — UDP I/O only
        if (is_sensor_cycle) {
          if (RequestAllSensorRead(cached_sensor_data.data(), num_fingertips_,
                                   hand_packets::SensorMode::kRaw)) {
            any_recv_ok = true;
          }
        }

        const auto t3 = std::chrono::steady_clock::now();  // sensor I/O 완료 (순수 UDP)

        // 4. Sensor processing (filter + drift detection + FT inference)
        if (is_sensor_cycle) {
          cached_sensor_data_raw = cached_sensor_data;   // raw 사본 보존 (LPF 이전)

          // SensorRateEstimator + BesselFilter delayed re-init
          SensorProcessingPreFilter();

          ApplySensorFilters(cached_sensor_data);

          // One-shot drift detection (raw 데이터 기반)
          OneShotDriftDetection(cached_sensor_data_raw);

          // F/T 추론 (캘리브레이션 또는 추론)
          if (ft_enabled_) {
            if (!ft_inferencer_->is_calibrated()) {
              static_cast<void>(ft_inferencer_->FeedCalibration(cached_sensor_data, num_fingertips_));
            } else {
              const auto ft_t0 = std::chrono::steady_clock::now();
              auto ft_result = ft_inferencer_->Infer(cached_sensor_data, num_fingertips_);
              const auto ft_t1 = std::chrono::steady_clock::now();
              ft_infer_elapsed_us = std::chrono::duration<double, std::micro>(ft_t1 - ft_t0).count();
              ft_seqlock_.Store(ft_result);
            }
          }
        }

        const auto t4 = std::chrono::steady_clock::now();  // processing 완료

        // 항상 캐시된 센서 데이터를 state에 복사
        state.sensor_data_raw = cached_sensor_data_raw;
        state.sensor_data = cached_sensor_data;
        state.num_fingertips = num_fingertips_;
        state.valid = any_recv_ok;

        // Update shared state (lock-free SeqLock)
        state_seqlock_.Store(state);
        if (callback_) { callback_(state, ft_seqlock_.Load()); }

        const auto t5 = std::chrono::steady_clock::now();  // 전체 완료

        // ── Timing profiler 업데이트 (bulk) ──
        HandTimingProfiler::PhaseTiming pt;
        pt.is_bulk_mode       = true;
        pt.write_us           = std::chrono::duration<double, std::micro>(t1 - t0).count();
        pt.read_all_motor_us  = std::chrono::duration<double, std::micro>(t2 - t1).count();
        pt.read_all_sensor_us = std::chrono::duration<double, std::micro>(t3 - t2).count();
        pt.sensor_proc_us     = std::chrono::duration<double, std::micro>(t4 - t3).count()
                                - ft_infer_elapsed_us;  // FT 시간 제외 (별도 추적)
        pt.ft_infer_us        = ft_infer_elapsed_us;
        pt.total_us           = std::chrono::duration<double, std::micro>(t5 - t0).count();
        pt.is_sensor_cycle    = is_sensor_cycle;
        timing_profiler_.Update(pt);

      } else {
        // ══════════════════════════════════════════════════════════════════
        // Individual mode: echo(cmd확인) + 0x11 pos + 0x12 vel + 0x14~0x17 sensors
        // 모든 read는 request → response → cmd 검증 패턴을 따름
        // ══════════════════════════════════════════════════════════════════

        // 2. Read position (motor, 3B → 43B) — request & response cmd 검증
        if (RequestMotorRead(hand_packets::Command::kReadPosition, motor_pos_buf)) {
          std::copy_n(motor_pos_buf.begin(), kNumHandMotors,
                      state.motor_positions.begin());
          any_recv_ok = true;
        }

        const auto t2 = std::chrono::steady_clock::now();  // read_pos 완료

        // 3. Read velocity (motor, 3B → 43B)
        if (RequestMotorRead(hand_packets::Command::kReadVelocity, motor_vel_buf)) {
          std::copy_n(motor_vel_buf.begin(), kNumHandMotors,
                      state.motor_velocities.begin());
          any_recv_ok = true;
        }

        const auto t3 = std::chrono::steady_clock::now();  // read_vel 완료

        // 4. Read sensors — UDP I/O only (decimation 적용)
        if (is_sensor_cycle) {
          for (int i = 0; i < num_fingertips_; ++i) {
            auto cmd = hand_packets::SensorCommand(i);
            if (RequestSensorRead(cmd, sensor_raw_buf,
                                  hand_packets::SensorMode::kRaw)) {
              std::copy_n(sensor_raw_buf.begin(), kSensorValuesPerFingertip,
                          cached_sensor_data.begin() + i * kSensorValuesPerFingertip);
              any_recv_ok = true;
            }
          }
        }

        const auto t4 = std::chrono::steady_clock::now();  // sensor I/O 완료 (순수 UDP)

        // 5. Sensor processing (filter + drift detection + FT inference)
        if (is_sensor_cycle) {
          cached_sensor_data_raw = cached_sensor_data;   // raw 사본 보존 (LPF 이전)

          // SensorRateEstimator + BesselFilter delayed re-init
          SensorProcessingPreFilter();

          ApplySensorFilters(cached_sensor_data);

          // One-shot drift detection (raw 데이터 기반)
          OneShotDriftDetection(cached_sensor_data_raw);

          // F/T 추론 (캘리브레이션 또는 추론)
          if (ft_enabled_) {
            if (!ft_inferencer_->is_calibrated()) {
              static_cast<void>(ft_inferencer_->FeedCalibration(cached_sensor_data, num_fingertips_));
            } else {
              const auto ft_t0 = std::chrono::steady_clock::now();
              auto ft_result = ft_inferencer_->Infer(cached_sensor_data, num_fingertips_);
              const auto ft_t1 = std::chrono::steady_clock::now();
              ft_infer_elapsed_us = std::chrono::duration<double, std::micro>(ft_t1 - ft_t0).count();
              ft_seqlock_.Store(ft_result);
            }
          }
        }

        const auto t5 = std::chrono::steady_clock::now();  // processing 완료

        // 항상 캐시된 센서 데이터를 state에 복사
        state.sensor_data_raw = cached_sensor_data_raw;
        state.sensor_data = cached_sensor_data;
        state.num_fingertips = num_fingertips_;
        state.valid = any_recv_ok;

        // Update shared state (lock-free SeqLock)
        state_seqlock_.Store(state);
        if (callback_) { callback_(state, ft_seqlock_.Load()); }

        const auto t6 = std::chrono::steady_clock::now();  // 전체 완료

        // ── Timing profiler 업데이트 (individual) ──
        HandTimingProfiler::PhaseTiming pt;
        pt.write_us       = std::chrono::duration<double, std::micro>(t1 - t0).count();
        pt.read_pos_us    = std::chrono::duration<double, std::micro>(t2 - t1).count();
        pt.read_vel_us    = std::chrono::duration<double, std::micro>(t3 - t2).count();
        pt.read_sensor_us = std::chrono::duration<double, std::micro>(t4 - t3).count();
        pt.sensor_proc_us = std::chrono::duration<double, std::micro>(t5 - t4).count()
                            - ft_infer_elapsed_us;  // FT 시간 제외 (별도 추적)
        pt.ft_infer_us    = ft_infer_elapsed_us;
        pt.total_us       = std::chrono::duration<double, std::micro>(t6 - t0).count();
        pt.is_sensor_cycle = is_sensor_cycle;
        timing_profiler_.Update(pt);
      }

      ++comm_stats_.total_cycles;
      cycle_count_.fetch_add(1, std::memory_order_relaxed);

      // UDP link health: 연속 실패 카운터 업데이트
      if (any_recv_ok) {
        consecutive_recv_failures_.store(0, std::memory_order_relaxed);
      } else {
        consecutive_recv_failures_.fetch_add(1, std::memory_order_relaxed);
      }

      busy_.store(false, std::memory_order_release);
    }
  }

  // ── SensorRateEstimator tick + BesselFilter delayed re-init ──────────────
  // sensor cycle마다 호출. warmup 완료 후 실측 rate로 filter 계수를 1회 재초기화.
  void SensorProcessingPreFilter() noexcept {
    sensor_rate_estimator_.Tick();

    // BesselFilter delayed re-init: warmup 완료 후 실측 rate로 1회 재초기화
    if (!filter_reinited_ && sensor_rate_estimator_.warmed_up()) {
      const double actual_rate =
          sensor_rate_estimator_.rate_hz()
          / static_cast<double>(sensor_decimation_);
      if (baro_filter_active_) {
        try {
          baro_filter_.Init(baro_lpf_cutoff_hz_, actual_rate);
        } catch (...) {}
      }
      if (tof_filter_active_) {
        try {
          tof_filter_.Init(tof_lpf_cutoff_hz_, actual_rate);
        } catch (...) {}
      }
      filter_reinited_ = true;
    }
  }

  // ── One-shot drift detection (raw baro 데이터 기반) ────────────────────
  // window_size 샘플 축적 후 1회 판정. 이후 Update() 호출 중단.
  // drift 발견 시 1Hz 간격으로 stderr 경고 (노드 수명 동안 지속).
  void OneShotDriftDetection(
      const std::array<int32_t, kMaxHandSensors>& sensor_data_raw) noexcept {
    if (!drift_detection_enabled_) return;

    // Phase 1: 축적 (window가 가득 찰 때까지)
    if (!drift_detector_.window_full()) {
      // Extract baro raw into flat double array (8ch × num_fingertips)
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

      // Phase 2: window 가득 참 → 결과 캐시 (1회만 실행)
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

    // Phase 3: drift 발견 시 1Hz throttled warning (노드 수명 동안 지속)
    if (drift_detected_) {
      ThrottledDriftWarning();
    }
  }

  // ── Throttled drift warning (1Hz, stderr) ──────────────────────────────
  // NOTE: fprintf on RT thread is not strictly RT-safe, but drift is an
  // abnormal condition and 1Hz rate is acceptable for operator notification.
  void ThrottledDriftWarning() noexcept {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_drift_warn_time_ < std::chrono::seconds(1)) return;
    last_drift_warn_time_ = now;

    const int num_baro = num_fingertips_ * kBarometerCount;
    for (int i = 0; i < num_baro; ++i) {
      if (drift_result_.drift_flags[static_cast<std::size_t>(i)]) {
        fprintf(stderr,
                "[WARN] barometer sensor(id:%d) detect drift (slope=%.3f)\n",
                i, drift_result_.slopes[static_cast<std::size_t>(i)]);
      }
    }
  }

  // ── 센서 LPF (noexcept — EventLoop hot path에서 호출) ────────────────────
  void ApplySensorFilters(
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

  std::string  target_ip_;
  int          target_port_;
  int          socket_fd_{-1};
  ThreadConfig thread_cfg_;
  double       recv_timeout_ms_;
  sockaddr_in  target_addr_{};

  std::atomic<bool> running_{false};
  StateCallback     callback_;

  int  sensor_decimation_;     // N cycle마다 센서 읽기 (1=매번, 4=4cycle마다)
  int  num_fingertips_;        // fingertip_names 갯수로 결정된 센서 읽기 대상 수
  bool use_fake_hand_;         // true: echo-back mock (UDP 소켓 미생성)
  std::vector<std::string> fingertip_names_;  // YAML에서 로드된 fingertip 이름 목록
  HandCommunicationMode communication_mode_;  // individual (기존) 또는 bulk (0x10/0x19)
  bool sensor_init_ok_{false}; // 센서 초기화 (NN→RAW) 성공 여부

  // ── 센서 LPF 설정 (initializer list 순서와 일치) ──
  bool   tof_lpf_enabled_{false};
  double tof_lpf_cutoff_hz_{15.0};
  bool   baro_lpf_enabled_{false};
  double baro_lpf_cutoff_hz_{30.0};

  bool   baro_filter_active_{false};  // Init 성공 후 true
  BesselFilterN<kMaxBaroChannels> baro_filter_{};
  bool   tof_filter_active_{false};   // Init 성공 후 true
  BesselFilterN<kMaxTofChannels> tof_filter_{};

  // 전역 E-Stop 플래그 (RtControllerNode에서 설정, null이면 체크하지 않음)
  std::atomic<bool>* estop_flag_{nullptr};

  // Event synchronisation — ControlLoop가 notify, EventLoop이 wait
  std::mutex              event_mutex_;
  std::condition_variable event_cv_;
  bool                    event_pending_{false};
  std::array<float, kNumHandMotors> staged_cmd_{};

  // EventLoop busy 플래그 — busy 중 도착하는 이벤트는 skip됨
  std::atomic<bool>     busy_{false};
  std::atomic<uint64_t> event_skip_count_{0};

  SeqLock<HandState> state_seqlock_{};
  std::atomic<std::size_t> cycle_count_{0};

  // recv() 타임아웃/에러 카운터 (모든 send-recv 실패 시 증가)
  std::atomic<uint64_t> recv_error_count_{0};

  // 연속 recv 전체 실패 카운터 (cycle 내 모든 recv 실패 시 증가, 1개라도 성공 시 0)
  std::atomic<uint64_t> consecutive_recv_failures_{0};

  // 통신 통계 (EventLoop 스레드에서만 쓰기, 외부에서 struct copy로 읽기)
  HandCommStats comm_stats_;

  // 타이밍 프로파일러 (EventLoop에서 write, 외부에서 read)
  HandTimingProfiler timing_profiler_;

  // ── F/T 추론기 ──
  FingertipFTInferencer::Config ft_config_;
  std::unique_ptr<FingertipFTInferencer> ft_inferencer_;
  SeqLock<FingertipFTState> ft_seqlock_{};  // EventLoop(writer) ↔ 외부(reader)
  bool ft_enabled_{false};

  // ── SensorRateEstimator (실측 sampling rate) ──
  SensorRateEstimator sensor_rate_estimator_;
  bool filter_reinited_{false};  // BesselFilter re-init 완료 플래그

  // ── Drift detection (one-shot) ──
  bool   drift_detection_enabled_{false};
  double drift_threshold_{5.0};
  int    drift_window_size_{2500};
  SlidingTrendDetector<kMaxBaroChannels, 2500> drift_detector_;
  SlidingTrendDetector<kMaxBaroChannels, 2500>::Result drift_result_{};
  bool   drift_detected_{false};
  std::chrono::steady_clock::time_point last_drift_warn_time_{};

  std::jthread event_thread_;
};

}  // namespace rtc

#endif  // UR5E_HAND_DRIVER_HAND_CONTROLLER_HPP_
