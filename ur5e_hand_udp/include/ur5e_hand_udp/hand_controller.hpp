#ifndef UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
#define UR5E_HAND_UDP_HAND_CONTROLLER_HPP_

// High-level hand controller: request-response driver over UDP.
//
// Uses a single UDP socket for both send and receive.
// Polling loop per cycle:
//   1. Write position  (0x01)            → send 43B
//   2. Read position   (0x11)            → send 43B, recv 43B
//   3. Read velocity   (0x12)            → send 43B, recv 43B
//   4. Read sensor 0-3 (0x14..0x17) × 4  → send  3B, recv 67B
//
// All hot-path operations are allocation-free for RT safety.

#include <array>
#include <atomic>
#include <cerrno>
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

  explicit HandController(
      std::string target_ip,
      int target_port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig,
      int recv_timeout_ms = 10,
      bool enable_write_ack = false) noexcept
      : target_ip_(std::move(target_ip)),
        target_port_(target_port),
        thread_cfg_(thread_cfg),
        recv_timeout_ms_(recv_timeout_ms),
        enable_write_ack_(enable_write_ack) {}

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
    poll_thread_ = std::jthread([this](std::stop_token st) {
      PollLoop(std::move(st));
    });

    return true;
  }

  void Stop() noexcept {
    running_.store(false, std::memory_order_release);
    poll_thread_.request_stop();
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

  // ── Command (thread-safe write from ROS2 callback) ─────────────────────

  void SetTargetPositions(const std::array<float, kNumHandMotors>& positions) noexcept {
    std::lock_guard lock(cmd_mutex_);
    target_positions_ = positions;
    has_new_command_   = true;
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

  void PollLoop(std::stop_token stop_token) {
    ApplyThreadConfig(thread_cfg_);

    std::array<uint8_t, hand_packets::kMotorPacketSize> send_buf{};
    std::array<float, hand_packets::kMotorDataCount> motor_float_buf{};
    std::array<float, kSensorValuesPerFingertip> sensor_float_buf{};

    while (!stop_token.stop_requested()) {
      // E-Stop 체크: 전역 E-Stop 발생 시 zero 명령 전송 후 루프 종료
      if (estop_flag_ && estop_flag_->load(std::memory_order_acquire)) {
        // zero 명령 전송
        std::array<float, kNumHandMotors> zeros{};
        std::array<uint8_t, hand_packets::kMotorPacketSize> zero_buf{};
        hand_udp_codec::EncodeWritePosition(zeros, zero_buf);
        sendto(socket_fd_, zero_buf.data(), zero_buf.size(), 0,
               reinterpret_cast<const sockaddr*>(&target_addr_),
               sizeof(target_addr_));
        break;
      }

      HandState state{};
      bool any_recv_ok = false;

      // 1. Write position (if new command available).
      {
        std::lock_guard lock(cmd_mutex_);
        if (has_new_command_) {
          hand_udp_codec::EncodeWritePosition(target_positions_, send_buf);
          sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
                 reinterpret_cast<const sockaddr*>(&target_addr_),
                 sizeof(target_addr_));
          // Optional ACK: wait for hardware to echo back after write.
          // Enable only when hand firmware supports write ACK.
          if (enable_write_ack_) {
            std::array<uint8_t, hand_packets::kMotorPacketSize> ack_buf{};
            recvfrom(socket_fd_, ack_buf.data(), ack_buf.size(), 0,
                     nullptr, nullptr);
          }
          has_new_command_ = false;
        }
      }

      // 2. Read position (motor, 43B ↔ 43B).
      if (RequestMotorRead(hand_packets::Command::kReadPosition, motor_float_buf)) {
        std::copy_n(motor_float_buf.begin(), kNumHandMotors,
                    state.motor_positions.begin());
        any_recv_ok = true;
      }

      // 3. Read velocity (motor, 43B ↔ 43B).
      if (RequestMotorRead(hand_packets::Command::kReadVelocity, motor_float_buf)) {
        std::copy_n(motor_float_buf.begin(), kNumHandMotors,
                    state.motor_velocities.begin());
        any_recv_ok = true;
      }

      // 4. Read sensors (4 fingertips, 3B → 67B each).
      for (int i = 0; i < kNumFingertips; ++i) {
        auto cmd = hand_packets::SensorCommand(i);
        if (RequestSensorRead(cmd, sensor_float_buf)) {
          std::copy_n(sensor_float_buf.begin(), kSensorValuesPerFingertip,
                      state.sensor_data.begin() + i * kSensorValuesPerFingertip);
          any_recv_ok = true;
        }
      }

      // state.valid = true only if at least one recv succeeded
      state.valid = any_recv_ok;

      // Update shared state.
      {
        std::lock_guard lock(state_mutex_);
        latest_state_ = state;
      }
      if (callback_) {
        callback_(state);
      }

      cycle_count_.fetch_add(1, std::memory_order_relaxed);
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

  // 전역 E-Stop 플래그 (RtControllerNode에서 설정, null이면 체크하지 않음)
  std::atomic<bool>* estop_flag_{nullptr};

  mutable std::mutex cmd_mutex_;
  std::array<float, kNumHandMotors> target_positions_{};
  bool has_new_command_{false};

  mutable std::mutex state_mutex_;
  HandState          latest_state_{};
  std::atomic<std::size_t> cycle_count_{0};

  // recv() 타임아웃/에러 카운터 (모든 send-recv 실패 시 증가)
  std::atomic<uint64_t> recv_error_count_{0};

  std::jthread poll_thread_;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
