#ifndef UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
#define UR5E_HAND_UDP_HAND_CONTROLLER_HPP_

// High-level hand controller: event-driven UDP driver.
//
// Uses a single UDP socket for both send and receive.
// Driven by ControlLoop events (pipeline mode):
//   Phase 4 of tick N: SendCommandAndRequestStates(cmd)
//     → Hand thread wakes and executes:
//       1. Write position  (0x01)            → send 43B
//       2. Read position   (0x11)            → send 43B, recv 43B
//       3. Read velocity   (0x12)            → send 43B, recv 43B
//       4. Read sensor 0-3 (0x14..0x17) × 4  → send  3B, recv 67B
//     → State ready for tick N+1
//   Phase 1 of tick N+1: GetLatestState() returns pre-fetched state
//
// All hot-path operations are allocation-free for RT safety.

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "ur5e_rt_base/types/types.hpp"
#include "ur5e_rt_base/threading/thread_config.hpp"
#include "ur5e_rt_base/threading/thread_utils.hpp"
#include "ur5e_hand_udp/hand_packets.hpp"
#include "ur5e_hand_udp/hand_udp_codec.hpp"

namespace ur5e_rt_controller {

// ── HandTimingProfiler ────────────────────────────────────────────────────────
//
// EventLoop의 per-phase 소요시간을 측정하고 히스토그램 기반 통계를 유지.
//
// Phase별 측정:
//   write → read_pos → read_vel → read_sensor → total (condvar wake ~ busy=false)
//
// Thread safety:
//   Update()는 EventLoop 스레드에서만 호출 (single producer).
//   GetStats() / Summary()는 어떤 스레드에서든 호출 가능 (relaxed atomic).
//
class HandTimingProfiler {
 public:
  // 히스토그램: 0–5000µs, 100µs 버킷. 마지막 버킷은 overflow (≥5000µs).
  static constexpr int    kBuckets = 50;
  static constexpr int    kBucketWidthUs = 100;
  static constexpr double kBudgetUs = 2000.0;  // 500Hz = 2ms 예산

  // Per-phase 평균 통계
  struct PhaseStats {
    double mean_us{0.0};
    double min_us{0.0};
    double max_us{0.0};
  };

  // 전체 통계 스냅샷
  struct Stats {
    uint64_t count{0};
    double   min_us{0.0};
    double   max_us{0.0};
    double   mean_us{0.0};
    double   stddev_us{0.0};
    double   p95_us{0.0};
    double   p99_us{0.0};
    double   last_us{0.0};
    uint64_t over_budget{0};
    std::array<uint64_t, kBuckets + 1> histogram{};

    // Per-phase 평균
    PhaseStats write;
    PhaseStats read_pos;
    PhaseStats read_vel;
    PhaseStats read_sensor;
    uint64_t   sensor_cycle_count{0};  // 센서 읽기가 실행된 cycle 수
  };

  // Per-tick 측정값 (EventLoop에서 채워서 Update()에 전달)
  struct PhaseTiming {
    double write_us{0.0};
    double read_pos_us{0.0};
    double read_vel_us{0.0};
    double read_sensor_us{0.0};  // 0 if sensor decimation skipped
    double total_us{0.0};
    bool   is_sensor_cycle{false};
  };

  // ── Update (single-producer: EventLoop thread) ────────────────────────────

  void Update(const PhaseTiming& t) noexcept {
    count_.fetch_add(1, std::memory_order_relaxed);

    AtomicMin(min_us_, t.total_us);
    AtomicMax(max_us_, t.total_us);
    last_us_.store(t.total_us, std::memory_order_relaxed);

    // Single-producer RMW — relaxed load+store is safe
    sum_us_.store(
        sum_us_.load(std::memory_order_relaxed) + t.total_us,
        std::memory_order_relaxed);
    sum_sq_us_.store(
        sum_sq_us_.load(std::memory_order_relaxed) + t.total_us * t.total_us,
        std::memory_order_relaxed);

    if (t.total_us > kBudgetUs) {
      over_budget_.fetch_add(1, std::memory_order_relaxed);
    }

    const int bucket = std::min(
        static_cast<int>(t.total_us / static_cast<double>(kBucketWidthUs)),
        kBuckets);
    histogram_[static_cast<std::size_t>(bucket)].fetch_add(
        1, std::memory_order_relaxed);

    // Per-phase 누적
    UpdatePhase(write_sum_, write_min_, write_max_, t.write_us);
    UpdatePhase(read_pos_sum_, read_pos_min_, read_pos_max_, t.read_pos_us);
    UpdatePhase(read_vel_sum_, read_vel_min_, read_vel_max_, t.read_vel_us);

    if (t.is_sensor_cycle) {
      sensor_cycle_count_.fetch_add(1, std::memory_order_relaxed);
      UpdatePhase(read_sensor_sum_, read_sensor_min_, read_sensor_max_,
                  t.read_sensor_us);
    }
  }

  // ── Statistics snapshot ───────────────────────────────────────────────────

  [[nodiscard]] Stats GetStats() const noexcept {
    Stats s;
    s.count = count_.load(std::memory_order_relaxed);
    s.min_us = min_us_.load(std::memory_order_relaxed);
    s.max_us = max_us_.load(std::memory_order_relaxed);
    s.last_us = last_us_.load(std::memory_order_relaxed);
    s.over_budget = over_budget_.load(std::memory_order_relaxed);

    if (s.count > 0) {
      s.mean_us = sum_us_.load(std::memory_order_relaxed) /
                  static_cast<double>(s.count);
      const double var =
          sum_sq_us_.load(std::memory_order_relaxed) /
          static_cast<double>(s.count) - s.mean_us * s.mean_us;
      s.stddev_us = (var > 0.0) ? std::sqrt(var) : 0.0;

      s.write = LoadPhaseStats(write_sum_, write_min_, write_max_, s.count);
      s.read_pos = LoadPhaseStats(read_pos_sum_, read_pos_min_, read_pos_max_,
                                  s.count);
      s.read_vel = LoadPhaseStats(read_vel_sum_, read_vel_min_, read_vel_max_,
                                  s.count);
    }

    s.sensor_cycle_count = sensor_cycle_count_.load(std::memory_order_relaxed);
    if (s.sensor_cycle_count > 0) {
      s.read_sensor = LoadPhaseStats(read_sensor_sum_, read_sensor_min_,
                                     read_sensor_max_, s.sensor_cycle_count);
    }

    for (int b = 0; b <= kBuckets; ++b) {
      s.histogram[static_cast<std::size_t>(b)] =
          histogram_[static_cast<std::size_t>(b)].load(
              std::memory_order_relaxed);
    }
    ComputePercentiles(s);
    return s;
  }

  // ── Human-readable summary ────────────────────────────────────────────────

  [[nodiscard]] std::string Summary() const noexcept {
    const Stats s = GetStats();
    if (s.count == 0) { return "HandUDP timing: no data"; }

    const double over_pct = static_cast<double>(s.over_budget) * 100.0 /
                            static_cast<double>(s.count);
    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        "HandUDP timing: count=%lu  mean=%.0f\xc2\xb5s  min=%.0f\xc2\xb5s"
        "  max=%.0f\xc2\xb5s  p95=%.0f\xc2\xb5s  p99=%.0f\xc2\xb5s"
        "  over_budget=%lu (%.1f%%)"
        "  | write=%.0f  pos=%.0f  vel=%.0f  sensor=%.0f\xc2\xb5s",
        static_cast<unsigned long>(s.count),
        s.mean_us, s.min_us, s.max_us, s.p95_us, s.p99_us,
        static_cast<unsigned long>(s.over_budget), over_pct,
        s.write.mean_us, s.read_pos.mean_us, s.read_vel.mean_us,
        s.read_sensor.mean_us);
    return std::string(buf);
  }

  void Reset() noexcept {
    count_.store(0, std::memory_order_relaxed);
    min_us_.store(1e9, std::memory_order_relaxed);
    max_us_.store(0.0, std::memory_order_relaxed);
    last_us_.store(0.0, std::memory_order_relaxed);
    sum_us_.store(0.0, std::memory_order_relaxed);
    sum_sq_us_.store(0.0, std::memory_order_relaxed);
    over_budget_.store(0, std::memory_order_relaxed);
    for (auto& b : histogram_) b.store(0, std::memory_order_relaxed);

    write_sum_.store(0.0, std::memory_order_relaxed);
    write_min_.store(1e9, std::memory_order_relaxed);
    write_max_.store(0.0, std::memory_order_relaxed);
    read_pos_sum_.store(0.0, std::memory_order_relaxed);
    read_pos_min_.store(1e9, std::memory_order_relaxed);
    read_pos_max_.store(0.0, std::memory_order_relaxed);
    read_vel_sum_.store(0.0, std::memory_order_relaxed);
    read_vel_min_.store(1e9, std::memory_order_relaxed);
    read_vel_max_.store(0.0, std::memory_order_relaxed);
    read_sensor_sum_.store(0.0, std::memory_order_relaxed);
    read_sensor_min_.store(1e9, std::memory_order_relaxed);
    read_sensor_max_.store(0.0, std::memory_order_relaxed);
    sensor_cycle_count_.store(0, std::memory_order_relaxed);
  }

 private:
  static void ComputePercentiles(Stats& s) noexcept {
    if (s.count == 0) return;
    const uint64_t p95_target = (s.count * 95) / 100;
    const uint64_t p99_target = (s.count * 99) / 100;

    uint64_t cumulative = 0;
    bool p95_done = false, p99_done = false;

    for (int b = 0; b <= kBuckets; ++b) {
      cumulative += s.histogram[static_cast<std::size_t>(b)];
      if (!p95_done && cumulative >= p95_target) {
        s.p95_us = static_cast<double>(b * kBucketWidthUs);
        p95_done = true;
      }
      if (!p99_done && cumulative >= p99_target) {
        s.p99_us = static_cast<double>(b * kBucketWidthUs);
        p99_done = true;
      }
      if (p95_done && p99_done) break;
    }
  }

  static void AtomicMin(std::atomic<double>& a, double v) noexcept {
    double old = a.load(std::memory_order_relaxed);
    while (v < old &&
           !a.compare_exchange_weak(old, v, std::memory_order_relaxed)) {}
  }

  static void AtomicMax(std::atomic<double>& a, double v) noexcept {
    double old = a.load(std::memory_order_relaxed);
    while (v > old &&
           !a.compare_exchange_weak(old, v, std::memory_order_relaxed)) {}
  }

  void UpdatePhase(std::atomic<double>& sum, std::atomic<double>& min_val,
                   std::atomic<double>& max_val, double us) noexcept {
    sum.store(sum.load(std::memory_order_relaxed) + us,
              std::memory_order_relaxed);
    AtomicMin(min_val, us);
    AtomicMax(max_val, us);
  }

  [[nodiscard]] static PhaseStats LoadPhaseStats(
      const std::atomic<double>& sum, const std::atomic<double>& min_val,
      const std::atomic<double>& max_val, uint64_t count) noexcept {
    PhaseStats ps;
    ps.mean_us = sum.load(std::memory_order_relaxed) /
                 static_cast<double>(count);
    ps.min_us = min_val.load(std::memory_order_relaxed);
    ps.max_us = max_val.load(std::memory_order_relaxed);
    return ps;
  }

  // Total 통계
  std::atomic<uint64_t> count_{0};
  std::atomic<double>   min_us_{1e9};
  std::atomic<double>   max_us_{0.0};
  std::atomic<double>   last_us_{0.0};
  std::atomic<double>   sum_us_{0.0};
  std::atomic<double>   sum_sq_us_{0.0};
  std::atomic<uint64_t> over_budget_{0};
  std::array<std::atomic<uint64_t>, kBuckets + 1> histogram_{};

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
  std::atomic<uint64_t> sensor_cycle_count_{0};
};

class HandController {
 public:
  using StateCallback = std::function<void(const HandState&)>;

  // 통신 통계 (recv 성공/타임아웃/에러 및 총 사이클 수)
  struct HandCommStats {
    uint64_t recv_ok{0};
    uint64_t recv_timeout{0};
    uint64_t recv_error{0};
    uint64_t total_cycles{0};
    uint64_t event_skip_count{0};     // EventLoop busy 중 skip된 이벤트 수
  };

  explicit HandController(
      std::string target_ip,
      int target_port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig,
      int recv_timeout_ms = 10,
      bool enable_write_ack = false,
      int sensor_decimation = 1,
      int num_fingertips = kDefaultNumFingertips,
      bool use_fake_hand = false) noexcept
      : target_ip_(std::move(target_ip)),
        target_port_(target_port),
        thread_cfg_(thread_cfg),
        recv_timeout_ms_(recv_timeout_ms),
        enable_write_ack_(enable_write_ack),
        sensor_decimation_(sensor_decimation < 1 ? 1 : sensor_decimation),
        num_fingertips_(num_fingertips > kMaxFingertips ? kMaxFingertips
                       : (num_fingertips < 0 ? 0 : num_fingertips)),
        use_fake_hand_(use_fake_hand) {}

  ~HandController() { Stop(); }

  HandController(const HandController&)            = delete;
  HandController& operator=(const HandController&) = delete;
  HandController(HandController&&)                 = delete;
  HandController& operator=(HandController&&)      = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────

  [[nodiscard]] bool Start() {
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

    // SO_RCVTIMEO: YAML에서 설정 가능 (기본값 10ms).
    // recv() 타임아웃으로 stop_token 체크 + 통신 실패 감지 가능.
    struct timeval tv{};
    tv.tv_sec  = recv_timeout_ms_ / 1000;
    tv.tv_usec = (recv_timeout_ms_ % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

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
      std::lock_guard lock(state_mutex_);
      std::copy(cmd.begin(), cmd.end(), latest_state_.motor_positions.begin());
      // velocities = 0, sensor_data = 0 (기본값 유지)
      latest_state_.num_fingertips = num_fingertips_;
      latest_state_.valid = true;
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

  [[nodiscard]] HandState GetLatestState() const {
    std::lock_guard lock(state_mutex_);
    return latest_state_;
  }

  [[nodiscard]] std::array<float, kNumHandMotors> GetLatestPositions() const {
    std::lock_guard lock(state_mutex_);
    return latest_state_.motor_positions;
  }

  [[nodiscard]] bool IsRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
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

 private:
  // Send raw bytes and receive into a buffer. Returns bytes received, or -1 on error.
  [[nodiscard]] ssize_t SendAndRecvRaw(
      const uint8_t* send_data, std::size_t send_len,
      uint8_t* recv_data, std::size_t recv_len) noexcept {
    const ssize_t sent = sendto(
        socket_fd_, send_data, send_len, 0,
        reinterpret_cast<const sockaddr*>(&target_addr_),
        sizeof(target_addr_));
    if (sent < 0) return -1;

    const ssize_t recvd = ::recv(socket_fd_, recv_data, recv_len, 0);
    if (recvd < 0) {
      // EAGAIN/EWOULDBLOCK = SO_RCVTIMEO expired → increment error counter
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
  [[nodiscard]] bool RequestMotorRead(
      hand_packets::Command cmd,
      std::array<float, hand_packets::kMotorDataCount>& out) noexcept {
    std::array<uint8_t, hand_packets::kMotorPacketSize> send_buf{};
    std::array<uint8_t, hand_packets::kMotorPacketSize> recv_buf{};

    hand_udp_codec::EncodeReadRequest(cmd, send_buf);
    const ssize_t recvd = SendAndRecvRaw(
        send_buf.data(), send_buf.size(),
        recv_buf.data(), recv_buf.size());
    if (recvd < static_cast<ssize_t>(hand_packets::kMotorPacketSize)) return false;

    uint8_t cmd_out, mode_out;
    return hand_udp_codec::DecodeMotorResponse(
        recv_buf.data(), static_cast<std::size_t>(recvd),
        cmd_out, mode_out, out);
  }

  // Send a set-sensor-mode command (CMD=0x04, 3B send, 3B recv echo).
  // Must be called once after sensor power-on to switch from NN to RAW mode.
  [[nodiscard]] bool RequestSetSensorMode(
      hand_packets::SensorMode sensor_mode) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kSensorRequestSize> recv_buf{};

    hand_udp_codec::EncodeSetSensorMode(sensor_mode, send_buf);
    const ssize_t recvd = SendAndRecvRaw(
        send_buf.data(), send_buf.size(),
        recv_buf.data(), recv_buf.size());
    return recvd >= static_cast<ssize_t>(hand_packets::kSensorRequestSize);
  }

  // Request a sensor read command (3B send) and decode 11 useful values (67B recv).
  // MODE field in request carries the desired sensor mode (default kRaw).
  // Response mode_out indicates the actual sensor mode used.
  [[nodiscard]] bool RequestSensorRead(
      hand_packets::Command cmd,
      std::array<float, kSensorValuesPerFingertip>& out,
      hand_packets::SensorMode sensor_mode = hand_packets::SensorMode::kRaw,
      uint8_t* response_mode = nullptr) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kSensorResponseSize> recv_buf{};

    hand_udp_codec::EncodeSensorReadRequest(cmd, send_buf, sensor_mode);
    const ssize_t recvd = SendAndRecvRaw(
        send_buf.data(), send_buf.size(),
        recv_buf.data(), recv_buf.size());
    if (recvd < static_cast<ssize_t>(hand_packets::kSensorResponseSize)) return false;

    uint8_t cmd_out, mode_out;
    const bool ok = hand_udp_codec::DecodeSensorResponse(
        recv_buf.data(), static_cast<std::size_t>(recvd),
        cmd_out, mode_out, out);
    if (ok && response_mode) {
      *response_mode = mode_out;
    }
    return ok;
  }

  // Event-driven loop: condvar 대기 → ControlLoop 이벤트 수신 시 write + read 수행.
  // Pipeline: tick N의 Phase 4에서 write + read → tick N+1의 Phase 1에서 state 사용.
  //
  // Sensor decimation: sensor_decimation_ 주기마다 센서 읽기 수행.
  //   decimation=1 → 매 cycle (기존), decimation=4 → 4 cycle마다.
  //   센서 skip 시 이전 sensor_data 유지.
  void EventLoop(std::stop_token stop_token) {
    ApplyThreadConfig(thread_cfg_);

    std::array<uint8_t, hand_packets::kMotorPacketSize> send_buf{};
    std::array<float, hand_packets::kMotorDataCount> motor_float_buf{};
    std::array<float, kSensorValuesPerFingertip> sensor_float_buf{};
    std::array<float, kNumHandMotors> pending_cmd{};

    // Sensor decimation: 이전 cycle의 sensor_data를 유지하기 위한 버퍼
    std::array<float, kMaxHandSensors> cached_sensor_data{};
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

      // ── Timing: t0 = EventLoop 시작 ──
      const auto t0 = std::chrono::steady_clock::now();

      // 1. Write position (이번 tick 명령)
      hand_udp_codec::EncodeWritePosition(pending_cmd, send_buf);
      sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
             reinterpret_cast<const sockaddr*>(&target_addr_),
             sizeof(target_addr_));
      if (enable_write_ack_) {
        std::array<uint8_t, hand_packets::kMotorPacketSize> ack_buf{};
        recvfrom(socket_fd_, ack_buf.data(), ack_buf.size(), 0,
                 nullptr, nullptr);
      }

      const auto t1 = std::chrono::steady_clock::now();  // write 완료

      // 2. Read position (motor, 43B ↔ 43B) — 다음 tick용 state
      if (RequestMotorRead(hand_packets::Command::kReadPosition, motor_float_buf)) {
        std::copy_n(motor_float_buf.begin(), kNumHandMotors,
                    state.motor_positions.begin());
        any_recv_ok = true;
      }

      const auto t2 = std::chrono::steady_clock::now();  // read_pos 완료

      // 3. Read velocity (motor, 43B ↔ 43B)
      if (RequestMotorRead(hand_packets::Command::kReadVelocity, motor_float_buf)) {
        std::copy_n(motor_float_buf.begin(), kNumHandMotors,
                    state.motor_velocities.begin());
        any_recv_ok = true;
      }

      const auto t3 = std::chrono::steady_clock::now();  // read_vel 완료

      // 4. Read sensors — sensor_decimation_ cycle마다 수행, 나머지는 캐시 사용
      ++sensor_cycle_counter;
      const bool is_sensor_cycle = (sensor_cycle_counter >= sensor_decimation_);
      if (is_sensor_cycle) {
        sensor_cycle_counter = 0;
        for (int i = 0; i < num_fingertips_; ++i) {
          auto cmd = hand_packets::SensorCommand(i);
          if (RequestSensorRead(cmd, sensor_float_buf)) {
            std::copy_n(sensor_float_buf.begin(), kSensorValuesPerFingertip,
                        cached_sensor_data.begin() + i * kSensorValuesPerFingertip);
            any_recv_ok = true;
          }
        }
      }

      const auto t4 = std::chrono::steady_clock::now();  // read_sensor 완료

      // 항상 캐시된 센서 데이터를 state에 복사 (읽었든 안 읽었든)
      state.sensor_data = cached_sensor_data;
      state.num_fingertips = num_fingertips_;

      // state.valid = true only if at least one recv succeeded
      state.valid = any_recv_ok;

      // Update shared state (다음 tick의 Phase 1에서 읽힘)
      {
        std::lock_guard lock(state_mutex_);
        latest_state_ = state;
      }
      if (callback_) {
        callback_(state);
      }

      const auto t5 = std::chrono::steady_clock::now();  // 전체 완료

      // ── Timing profiler 업데이트 ──
      HandTimingProfiler::PhaseTiming pt;
      pt.write_us      = std::chrono::duration<double, std::micro>(t1 - t0).count();
      pt.read_pos_us   = std::chrono::duration<double, std::micro>(t2 - t1).count();
      pt.read_vel_us   = std::chrono::duration<double, std::micro>(t3 - t2).count();
      pt.read_sensor_us = std::chrono::duration<double, std::micro>(t4 - t3).count();
      pt.total_us       = std::chrono::duration<double, std::micro>(t5 - t0).count();
      pt.is_sensor_cycle = is_sensor_cycle;
      timing_profiler_.Update(pt);

      ++comm_stats_.total_cycles;
      cycle_count_.fetch_add(1, std::memory_order_relaxed);
      busy_.store(false, std::memory_order_release);
    }
  }

  std::string  target_ip_;
  int          target_port_;
  int          socket_fd_{-1};
  ThreadConfig thread_cfg_;
  int          recv_timeout_ms_;
  sockaddr_in  target_addr_{};

  std::atomic<bool> running_{false};
  StateCallback     callback_;

  bool enable_write_ack_;
  int  sensor_decimation_;     // N cycle마다 센서 읽기 (1=매번, 4=4cycle마다)
  int  num_fingertips_;        // YAML에서 설정된 fingertip 수
  bool use_fake_hand_;         // true: echo-back mock (UDP 소켓 미생성)

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

  mutable std::mutex state_mutex_;
  HandState          latest_state_{};
  std::atomic<std::size_t> cycle_count_{0};

  // recv() 타임아웃/에러 카운터 (모든 send-recv 실패 시 증가)
  std::atomic<uint64_t> recv_error_count_{0};

  // 통신 통계 (EventLoop 스레드에서만 쓰기, 외부에서 struct copy로 읽기)
  HandCommStats comm_stats_;

  // 타이밍 프로파일러 (EventLoop에서 write, 외부에서 read)
  HandTimingProfiler timing_profiler_;

  std::jthread event_thread_;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
