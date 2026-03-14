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

#include <array>
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
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
      int sensor_decimation = 1) noexcept
      : target_ip_(std::move(target_ip)),
        target_port_(target_port),
        thread_cfg_(thread_cfg),
        recv_timeout_ms_(recv_timeout_ms),
        enable_write_ack_(enable_write_ack),
        sensor_decimation_(sensor_decimation < 1 ? 1 : sensor_decimation) {}

  ~HandController() { Stop(); }

  HandController(const HandController&)            = delete;
  HandController& operator=(const HandController&) = delete;
  HandController(HandController&&)                 = delete;
  HandController& operator=(HandController&&)      = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────

  [[nodiscard]] bool Start() {
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
    std::array<float, kNumFingertips * kSensorValuesPerFingertip> cached_sensor_data{};
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

      // 2. Read position (motor, 43B ↔ 43B) — 다음 tick용 state
      if (RequestMotorRead(hand_packets::Command::kReadPosition, motor_float_buf)) {
        std::copy_n(motor_float_buf.begin(), kNumHandMotors,
                    state.motor_positions.begin());
        any_recv_ok = true;
      }

      // 3. Read velocity (motor, 43B ↔ 43B)
      if (RequestMotorRead(hand_packets::Command::kReadVelocity, motor_float_buf)) {
        std::copy_n(motor_float_buf.begin(), kNumHandMotors,
                    state.motor_velocities.begin());
        any_recv_ok = true;
      }

      // 4. Read sensors — sensor_decimation_ cycle마다 수행, 나머지는 캐시 사용
      ++sensor_cycle_counter;
      if (sensor_cycle_counter >= sensor_decimation_) {
        sensor_cycle_counter = 0;
        for (int i = 0; i < kNumFingertips; ++i) {
          auto cmd = hand_packets::SensorCommand(i);
          if (RequestSensorRead(cmd, sensor_float_buf)) {
            std::copy_n(sensor_float_buf.begin(), kSensorValuesPerFingertip,
                        cached_sensor_data.begin() + i * kSensorValuesPerFingertip);
            any_recv_ok = true;
          }
        }
      }
      // 항상 캐시된 센서 데이터를 state에 복사 (읽었든 안 읽었든)
      state.sensor_data = cached_sensor_data;

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

  std::jthread event_thread_;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
