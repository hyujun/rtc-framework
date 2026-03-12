#ifndef UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
#define UR5E_HAND_UDP_HAND_CONTROLLER_HPP_

// High-level hand controller: request-response driver over UDP.
//
// Uses a single UDP socket for both send and receive.
// Polling loop per cycle:
//   1. Write position  (0x01)
//   2. Read position   (0x11) → recv
//   3. Read velocity   (0x12) → recv
//   4. Read sensor 0-3 (0x14..0x17) → recv × 4
//
// All hot-path operations are allocation-free for RT safety.

#include <array>
#include <atomic>
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
      const ThreadConfig& thread_cfg = kUdpRecvConfig) noexcept
      : target_ip_(std::move(target_ip)),
        target_port_(target_port),
        thread_cfg_(thread_cfg) {}

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

    // Recv timeout for stop_token check (10 ms for fast response).
    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 10'000;
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

 private:
  // Send a packet and receive a response. Returns false on timeout/error.
  [[nodiscard]] bool SendAndRecv(
      const std::array<uint8_t, hand_packets::kPacketSize>& send_buf,
      std::array<uint8_t, hand_packets::kPacketSize>& recv_buf) noexcept {
    const ssize_t sent = sendto(
        socket_fd_, send_buf.data(), send_buf.size(), 0,
        reinterpret_cast<const sockaddr*>(&target_addr_),
        sizeof(target_addr_));
    if (sent < 0) return false;

    const ssize_t recvd = recv(socket_fd_, recv_buf.data(), recv_buf.size(), 0);
    if (recvd < static_cast<ssize_t>(hand_packets::kPacketSize)) return false;

    return true;
  }

  // Request a read command and decode 10 floats.
  [[nodiscard]] bool RequestRead(
      hand_packets::Command cmd,
      std::array<float, hand_packets::kDataCount>& out) noexcept {
    std::array<uint8_t, hand_packets::kPacketSize> send_buf{};
    std::array<uint8_t, hand_packets::kPacketSize> recv_buf{};

    hand_udp_codec::EncodeReadRequest(cmd, send_buf);
    if (!SendAndRecv(send_buf, recv_buf)) return false;

    uint8_t cmd_out;
    return hand_udp_codec::DecodeResponse(recv_buf.data(), recv_buf.size(),
                                          cmd_out, out);
  }

  void PollLoop(std::stop_token stop_token) {
    ApplyThreadConfig(thread_cfg_);

    std::array<uint8_t, hand_packets::kPacketSize> send_buf{};
    std::array<float, hand_packets::kDataCount> float_buf{};

    while (!stop_token.stop_requested()) {
      HandState state{};

      // 1. Write position (if new command available).
      {
        std::lock_guard lock(cmd_mutex_);
        if (has_new_command_) {
          hand_udp_codec::EncodeWritePosition(target_positions_, send_buf);
          sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
                 reinterpret_cast<const sockaddr*>(&target_addr_),
                 sizeof(target_addr_));
          has_new_command_ = false;
        }
      }

      // 2. Read position.
      if (RequestRead(hand_packets::Command::kReadPosition, float_buf)) {
        std::copy_n(float_buf.begin(), kNumHandMotors,
                    state.motor_positions.begin());
      }

      // 3. Read velocity.
      if (RequestRead(hand_packets::Command::kReadVelocity, float_buf)) {
        std::copy_n(float_buf.begin(), kNumHandMotors,
                    state.motor_velocities.begin());
      }

      // 4. Read sensors (4 fingertips).
      for (int i = 0; i < kNumFingertips; ++i) {
        auto cmd = hand_packets::SensorCommand(i);
        if (RequestRead(cmd, float_buf)) {
          std::copy_n(float_buf.begin(), kSensorValuesPerFingertip,
                      state.sensor_data.begin() + i * kSensorValuesPerFingertip);
        }
      }

      state.valid = true;

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
  sockaddr_in  target_addr_{};

  std::atomic<bool> running_{false};
  StateCallback     callback_;

  mutable std::mutex cmd_mutex_;
  std::array<float, kNumHandMotors> target_positions_{};
  bool has_new_command_{false};

  mutable std::mutex state_mutex_;
  HandState          latest_state_{};
  std::atomic<std::size_t> cycle_count_{0};

  std::jthread poll_thread_;
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_HAND_UDP_HAND_CONTROLLER_HPP_
