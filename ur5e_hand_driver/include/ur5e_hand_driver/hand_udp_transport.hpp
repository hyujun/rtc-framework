#ifndef UR5E_HAND_DRIVER_HAND_UDP_TRANSPORT_HPP_
#define UR5E_HAND_DRIVER_HAND_UDP_TRANSPORT_HPP_

// HandUdpTransport: low-level UDP socket management and protocol requests.
//
// Owns the UDP socket and provides typed request-response methods for
// the hand protocol (motor read/write, sensor read, bulk read).
//
// All hot-path methods are noexcept and allocation-free (RT-safe).
// Uses ppoll for sub-ms recv timeout (hrtimer on PREEMPT_RT kernels).

#include <array>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/logging.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rtc_base/types/types.hpp"
#include "ur5e_hand_driver/hand_logging.hpp"
#include "ur5e_hand_driver/hand_packets.hpp"
#include "ur5e_hand_driver/hand_udp_codec.hpp"

namespace rtc {

// Communication statistics (recv success/timeout/error and total cycles)
struct HandCommStats {
  uint64_t recv_ok{0};
  uint64_t recv_timeout{0};
  uint64_t recv_error{0};
  uint64_t cmd_mismatch{0};
  uint64_t mode_mismatch{0};
  uint64_t total_cycles{0};
  uint64_t event_skip_count{0};
};

class HandUdpTransport {
 public:
  explicit HandUdpTransport(std::string target_ip, int target_port,
                            double recv_timeout_ms) noexcept
      : target_ip_(std::move(target_ip)),
        target_port_(target_port),
        recv_timeout_ms_(recv_timeout_ms) {}

  ~HandUdpTransport() { Close(); }

  HandUdpTransport(const HandUdpTransport&) = delete;
  HandUdpTransport& operator=(const HandUdpTransport&) = delete;
  HandUdpTransport(HandUdpTransport&&) = delete;
  HandUdpTransport& operator=(HandUdpTransport&&) = delete;

  // ── Socket lifecycle ──────────────────────────────────────────────────────

  [[nodiscard]] bool Open() noexcept {
    const auto logger = ::ur5e_hand_driver::logging::TransportLogger();

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(logger, "UDP socket creation failed (errno=%d: %s)",
                   errno, strerror(errno));
      return false;
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port   = htons(static_cast<uint16_t>(target_port_));
    if (inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) <= 0) {
      RCLCPP_ERROR(logger, "Invalid target IP address: %s", target_ip_.c_str());
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    constexpr int kUdpBufSize = 65536;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &kUdpBufSize, sizeof(kUdpBufSize)) != 0) {
      RCLCPP_WARN(logger, "setsockopt SO_RCVBUF failed (errno=%d)", errno);
    }
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &kUdpBufSize, sizeof(kUdpBufSize)) != 0) {
      RCLCPP_WARN(logger, "setsockopt SO_SNDBUF failed (errno=%d)", errno);
    }

    // Safety fallback SO_RCVTIMEO (ppoll is primary timeout mechanism)
    {
      struct timeval tv{};
      tv.tv_sec  = 0;
      tv.tv_usec = 100'000;  // 100ms safety fallback
      if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
        RCLCPP_WARN(logger, "setsockopt SO_RCVTIMEO failed (errno=%d)", errno);
      }
    }

    RCLCPP_INFO(logger, "UDP socket opened: %s:%d (recv_timeout=%.2fms)",
                target_ip_.c_str(), target_port_, recv_timeout_ms_);
    return true;
  }

  void Close() noexcept {
    if (socket_fd_ >= 0) {
      RCLCPP_DEBUG(::ur5e_hand_driver::logging::TransportLogger(),
                   "UDP socket closed (%s:%d)", target_ip_.c_str(), target_port_);
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  [[nodiscard]] bool is_open() const noexcept { return socket_fd_ >= 0; }

  // ── Protocol requests ─────────────────────────────────────────────────────

  // Write position command + recv echo. Returns true if echo cmd matches.
  [[nodiscard]] bool WritePositionWithEcho(
      const std::array<float, kNumHandMotors>& cmd,
      std::array<uint8_t, hand_packets::kMotorPacketSize>& send_buf,
      std::array<uint8_t, hand_packets::kMotorPacketSize>& echo_buf,
      hand_packets::JointMode joint_mode = hand_packets::JointMode::kMotor) noexcept {
    hand_udp_codec::EncodeWritePosition(cmd, send_buf, joint_mode);
    sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
           reinterpret_cast<const sockaddr*>(&target_addr_),
           sizeof(target_addr_));

    const ssize_t recvd = RecvWithTimeout(echo_buf.data(), echo_buf.size());
    if (recvd >= static_cast<ssize_t>(hand_packets::kHeaderSize)) {
      const bool echo_ok = (echo_buf[1] == static_cast<uint8_t>(
                                hand_packets::Command::kWritePosition));
      if (!echo_ok) {
        ++comm_stats_.cmd_mismatch;
      }
      ++comm_stats_.recv_ok;
      return echo_ok;
    }
    if (recvd < 0) {
      recv_error_count_.fetch_add(1, std::memory_order_relaxed);
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        ++comm_stats_.recv_timeout;
      } else {
        ++comm_stats_.recv_error;
      }
    }
    return false;
  }

  // Send a fire-and-forget write position (e.g. for E-Stop zero command).
  void WritePositionFireAndForget(
      const std::array<float, kNumHandMotors>& cmd,
      hand_packets::JointMode joint_mode = hand_packets::JointMode::kMotor) noexcept {
    std::array<uint8_t, hand_packets::kMotorPacketSize> buf{};
    hand_udp_codec::EncodeWritePosition(cmd, buf, joint_mode);
    sendto(socket_fd_, buf.data(), buf.size(), 0,
           reinterpret_cast<const sockaddr*>(&target_addr_),
           sizeof(target_addr_));
  }

  // Request motor read (individual: 0x11 pos, 0x12 vel). 3B send, 43B recv.
  [[nodiscard]] bool RequestMotorRead(
      hand_packets::Command cmd,
      std::array<float, hand_packets::kMotorDataCount>& out,
      hand_packets::JointMode joint_mode = hand_packets::JointMode::kMotor,
      hand_packets::JointMode* received_mode = nullptr) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kMotorPacketSize> recv_buf{};

    hand_udp_codec::EncodeMotorReadRequest(cmd, send_buf, joint_mode);

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

      if (recvd < static_cast<ssize_t>(hand_packets::kMotorPacketSize)) continue;

      uint8_t cmd_out, mode_out;
      if (!hand_udp_codec::DecodeMotorResponse(
              recv_buf.data(), static_cast<std::size_t>(recvd),
              cmd_out, mode_out, out)) {
        continue;
      }
      if (cmd_out != static_cast<uint8_t>(cmd)) {
        ++comm_stats_.cmd_mismatch;
        continue;
      }
      if (mode_out != static_cast<uint8_t>(joint_mode)) {
        ++comm_stats_.mode_mismatch;
        return false;
      }
      if (received_mode) {
        *received_mode = static_cast<hand_packets::JointMode>(mode_out);
      }
      return true;
    }
    return false;
  }

  // Request sensor read (individual: 0x14~0x17). 3B send, 67B recv.
  [[nodiscard]] bool RequestSensorRead(
      hand_packets::Command cmd,
      std::array<int32_t, kSensorValuesPerFingertip>& out,
      hand_packets::SensorMode sensor_mode = hand_packets::SensorMode::kRaw) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kSensorResponseSize> recv_buf{};

    hand_udp_codec::EncodeSensorReadRequest(cmd, send_buf, sensor_mode);

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

      if (recvd < static_cast<ssize_t>(hand_packets::kSensorResponseSize)) continue;

      uint8_t cmd_out, mode_out;
      const bool ok = hand_udp_codec::DecodeSensorResponseRaw(
          recv_buf.data(), static_cast<std::size_t>(recvd),
          cmd_out, mode_out, out);
      if (!ok) continue;

      if (cmd_out != static_cast<uint8_t>(cmd)) {
        ++comm_stats_.cmd_mismatch;
        continue;
      }
      if (mode_out != static_cast<uint8_t>(sensor_mode)) {
        ++comm_stats_.mode_mismatch;
        return false;
      }
      return true;
    }
    return false;
  }

  // Request bulk motor read (cmd=0x10). 3B send, 123B recv.
  [[nodiscard]] bool RequestAllMotorRead(
      std::array<float, hand_packets::kMotorDataCount>& positions,
      std::array<float, hand_packets::kMotorDataCount>& velocities,
      std::array<float, hand_packets::kMotorDataCount>& currents,
      hand_packets::JointMode joint_mode = hand_packets::JointMode::kMotor,
      hand_packets::JointMode* received_mode = nullptr) noexcept {
    std::array<uint8_t, hand_packets::kAllMotorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kAllMotorResponseSize> recv_buf{};

    hand_udp_codec::EncodeReadAllMotorsRequest(send_buf, joint_mode);

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
              cmd_out, mode_out, positions, velocities, currents)) {
        continue;
      }
      if (cmd_out != static_cast<uint8_t>(hand_packets::Command::kReadAllMotors)) {
        ++comm_stats_.cmd_mismatch;
        continue;
      }
      if (mode_out != static_cast<uint8_t>(joint_mode)) {
        ++comm_stats_.mode_mismatch;
        return false;
      }
      if (received_mode) {
        *received_mode = static_cast<hand_packets::JointMode>(mode_out);
      }
      return true;
    }
    return false;
  }

  // Request bulk sensor read (cmd=0x19). 3B send, 259B recv.
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
      if (mode_out != static_cast<uint8_t>(sensor_mode)) {
        ++comm_stats_.mode_mismatch;
        return false;
      }
      return true;
    }
    return false;
  }

  // Set sensor mode (CMD=0x04, 3B send, 3B recv echo).
  [[nodiscard]] bool RequestSetSensorMode(
      hand_packets::SensorMode sensor_mode) noexcept {
    std::array<uint8_t, hand_packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, hand_packets::kSensorRequestSize> recv_buf{};

    hand_udp_codec::EncodeSetSensorMode(sensor_mode, send_buf);
    const ssize_t recvd = SendAndRecvRaw(
        send_buf.data(), send_buf.size(),
        recv_buf.data(), recv_buf.size());
    if (recvd < static_cast<ssize_t>(hand_packets::kSensorRequestSize)) {
      return false;
    }
    return recv_buf[2] == static_cast<uint8_t>(sensor_mode);
  }

  // Sensor initialization: switch from NN to RAW mode with retries.
  [[nodiscard]] bool InitializeSensors(
      int max_retries = 5,
      int retry_interval_ms = 100) noexcept {
    const auto logger = ::ur5e_hand_driver::logging::TransportLogger();
    for (int attempt = 0; attempt < max_retries; ++attempt) {
      if (RequestSetSensorMode(hand_packets::SensorMode::kRaw)) {
        RCLCPP_INFO(logger, "Sensor mode set to RAW (attempt %d/%d)",
                    attempt + 1, max_retries);
        return true;
      }
      RCLCPP_WARN(logger, "Sensor mode switch retry %d/%d failed",
                  attempt + 1, max_retries);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(retry_interval_ms));
    }
    RCLCPP_ERROR(logger, "Sensor mode switch to RAW failed after %d retries",
                 max_retries);
    return false;
  }

  // Drain stale UDP responses from the socket buffer (non-blocking).
  void DrainStaleResponses() noexcept {
    std::array<uint8_t, hand_packets::kMaxPacketSize> discard{};
    for (int i = 0; i < 8; ++i) {
      const ssize_t r = ::recv(socket_fd_, discard.data(), discard.size(),
                               MSG_DONTWAIT);
      if (r <= 0) break;
    }
  }

  // ── Statistics accessors ──────────────────────────────────────────────────

  [[nodiscard]] const HandCommStats& comm_stats() const noexcept {
    return comm_stats_;
  }
  [[nodiscard]] HandCommStats& comm_stats_mut() noexcept {
    return comm_stats_;
  }

  [[nodiscard]] uint64_t recv_error_count() const noexcept {
    return recv_error_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] double recv_timeout_ms() const noexcept {
    return recv_timeout_ms_;
  }

 private:
  // Sub-ms precision recv using ppoll (hrtimer on PREEMPT_RT kernels).
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

  // Send raw bytes and receive into a buffer. Returns bytes received.
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

  std::string  target_ip_;
  int          target_port_;
  int          socket_fd_{-1};
  double       recv_timeout_ms_;
  sockaddr_in  target_addr_{};

  HandCommStats comm_stats_;
  std::atomic<uint64_t> recv_error_count_{0};
};

}  // namespace rtc

#endif  // UR5E_HAND_DRIVER_HAND_UDP_TRANSPORT_HPP_
