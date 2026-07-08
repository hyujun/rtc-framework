#ifndef UDP_HAND_DRIVER_UDP_HAND_TRANSPORT_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_TRANSPORT_HPP_

// UdpHandTransport: low-level UDP socket management and protocol requests.
//
// Owns the UDP socket and provides typed request-response methods for
// the hand protocol (motor read/write, sensor read, bulk read).
//
// All hot-path methods are noexcept and allocation-free (RT-safe).
// Uses ppoll for sub-ms recv timeout (hrtimer on PREEMPT_RT kernels).

#include "rtc_base/types/types.hpp"
#include "udp_hand_driver/udp_hand_codec.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <rclcpp/logging.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <utility>

namespace udp_hand_driver {

// Request kind for per-kind statistics attribution. The caller disambiguates
// joint- vs motor-space reads (same request method / cmd byte, different MODE),
// which the transport cannot infer from the wire format alone. Values are bit
// positions in the controller's cycle-outcome masks — keep them dense from 0.
enum class RequestKind : uint8_t {
  kWriteEcho = 0,   // 0x01 write position + echo
  kMotorRead = 1,   // 0x10/0x11/0x12 motor-space read
  kJointRead = 2,   // 0x10/0x11 joint-space read
  kSensorRead = 3,  // 0x14~0x17 individual sensor read
  kBulkSensor = 4,  // 0x19 bulk sensor read
  kSetMode = 5,     // 0x04 set sensor mode
};

inline constexpr int kNumRequestKinds = 6;

// Stable short names for logs / stats JSON, indexed by RequestKind.
inline constexpr std::array<const char*, kNumRequestKinds> kRequestKindNames = {
    "write_echo", "motor_read", "joint_read", "sensor_read", "bulk_sensor", "set_mode"};

// Bit for a RequestKind in the cycle-outcome attempted/ok masks.
[[nodiscard]] constexpr uint8_t RequestKindBit(RequestKind kind) noexcept {
  return static_cast<uint8_t>(1u << static_cast<unsigned>(kind));
}

// Communication statistics (recv success/timeout/error and total cycles).
// Single writer (EventLoop); other threads read racily — same relaxed pattern
// as the pre-existing aggregate fields.
struct UdpHandCommStats {
  uint64_t recv_ok{0};
  uint64_t recv_timeout{0};
  uint64_t recv_error{0};
  uint64_t cmd_mismatch{0};
  uint64_t mode_mismatch{0};
  uint64_t total_cycles{0};
  uint64_t event_skip_count{0};
  uint64_t comm_decimation_skip_count{0};  // cycles skipped by comm_decimation

  // Per-request-kind breakdown. Unlike aggregate recv_ok (any first-attempt
  // packet arrival), per-kind `ok` counts request-level success — the method
  // returned validated data. `short_or_decode` counts short packets and codec
  // decode failures (both drop the packet and retry).
  struct PerKind {
    uint64_t ok{0};
    uint64_t timeout{0};
    uint64_t error{0};
    uint64_t cmd_mismatch{0};
    uint64_t mode_mismatch{0};
    uint64_t short_or_decode{0};
  };

  std::array<PerKind, kNumRequestKinds> per_kind{};

  // Forensics for the most recent rejected packet: the CMD byte of the last
  // cmd_mismatch (a stale echo of the *previous* request implies desync), and
  // the byte count of the last rejected recv (mismatch or short).
  uint8_t last_unexpected_cmd{0};
  uint16_t last_unexpected_len{0};
};

class UdpHandTransport {
 public:
  explicit UdpHandTransport(std::string target_ip, int target_port, double recv_timeout_ms) noexcept
      : target_ip_(std::move(target_ip)),
        target_port_(target_port),
        recv_timeout_ms_(recv_timeout_ms) {}

  ~UdpHandTransport() { Close(); }

  UdpHandTransport(const UdpHandTransport&) = delete;
  UdpHandTransport& operator=(const UdpHandTransport&) = delete;
  UdpHandTransport(UdpHandTransport&&) = delete;
  UdpHandTransport& operator=(UdpHandTransport&&) = delete;

  // ── Socket lifecycle ──────────────────────────────────────────────────────

  [[nodiscard]] bool Open() noexcept {
    const auto logger = ::udp_hand_driver::logging::TransportLogger();

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(logger, "UDP socket creation failed (errno=%d: %s)", errno, strerror(errno));
      return false;
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(static_cast<uint16_t>(target_port_));
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
      struct timeval tv {};

      tv.tv_sec = 0;
      tv.tv_usec = 100'000;  // 100ms safety fallback
      if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
        RCLCPP_WARN(logger, "setsockopt SO_RCVTIMEO failed (errno=%d)", errno);
      }
    }

    RCLCPP_INFO(logger, "UDP socket opened: %s:%d (recv_timeout=%.2fms)", target_ip_.c_str(),
                target_port_, recv_timeout_ms_);
    return true;
  }

  void Close() noexcept {
    if (socket_fd_ >= 0) {
      RCLCPP_DEBUG(::udp_hand_driver::logging::TransportLogger(), "UDP socket closed (%s:%d)",
                   target_ip_.c_str(), target_port_);
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  [[nodiscard]] bool is_open() const noexcept { return socket_fd_ >= 0; }

  // ── Protocol requests ─────────────────────────────────────────────────────

  // Write position command + recv echo. Returns true if echo cmd matches.
  [[nodiscard]] bool WritePositionWithEcho(
      const std::array<float, kNumHandMotors>& cmd,
      std::array<uint8_t, packets::kMotorPacketSize>& send_buf,
      std::array<uint8_t, packets::kMotorPacketSize>& echo_buf,
      packets::JointMode joint_mode = packets::JointMode::kMotor) noexcept {
    codec::EncodeWritePosition(cmd, send_buf, joint_mode);
    sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
           reinterpret_cast<const sockaddr*>(&target_addr_), sizeof(target_addr_));

    const ssize_t recvd = RecvWithTimeout(echo_buf.data(), echo_buf.size());
    if (recvd >= static_cast<ssize_t>(packets::kHeaderSize)) {
      const bool echo_ok = (echo_buf[1] == static_cast<uint8_t>(packets::Command::kWritePosition));
      if (!echo_ok) {
        CountCmdMismatch(RequestKind::kWriteEcho, echo_buf[1], recvd);
      } else {
        ++PerKindStats(RequestKind::kWriteEcho).ok;
      }
      ++comm_stats_.recv_ok;
      return echo_ok;
    }
    if (recvd < 0) {
      CountRecvFail(RequestKind::kWriteEcho);
    } else {
      CountShortOrDecode(RequestKind::kWriteEcho, recvd);
    }
    return false;
  }

  // Send a fire-and-forget write position (e.g. for E-Stop zero command).
  void WritePositionFireAndForget(
      const std::array<float, kNumHandMotors>& cmd,
      packets::JointMode joint_mode = packets::JointMode::kMotor) noexcept {
    std::array<uint8_t, packets::kMotorPacketSize> buf{};
    codec::EncodeWritePosition(cmd, buf, joint_mode);
    sendto(socket_fd_, buf.data(), buf.size(), 0, reinterpret_cast<const sockaddr*>(&target_addr_),
           sizeof(target_addr_));
  }

  // Request motor read (individual: 0x11 pos, 0x12 vel). 3B send, 43B recv.
  // `kind` attributes per-kind stats: the caller distinguishes motor- vs
  // joint-space use of this method (identical wire format, different MODE).
  [[nodiscard]] bool RequestMotorRead(packets::Command cmd,
                                      std::array<float, packets::kMotorDataCount>& out,
                                      packets::JointMode joint_mode = packets::JointMode::kMotor,
                                      packets::JointMode* received_mode = nullptr,
                                      RequestKind kind = RequestKind::kMotorRead) noexcept {
    std::array<uint8_t, packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, packets::kMotorPacketSize> recv_buf{};

    codec::EncodeMotorReadRequest(cmd, send_buf, joint_mode);

    const ssize_t sent =
        sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
               reinterpret_cast<const sockaddr*>(&target_addr_), sizeof(target_addr_));
    if (sent < 0)
      return false;

    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd =
          (attempt == 0) ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
                         : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          CountRecvFail(kind);
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(packets::kMotorPacketSize)) {
        CountShortOrDecode(kind, recvd);
        continue;
      }

      uint8_t cmd_out, mode_out;
      if (!codec::DecodeMotorResponse(recv_buf.data(), static_cast<std::size_t>(recvd), cmd_out,
                                      mode_out, out)) {
        CountShortOrDecode(kind, recvd);
        continue;
      }
      if (cmd_out != static_cast<uint8_t>(cmd)) {
        CountCmdMismatch(kind, cmd_out, recvd);
        continue;
      }
      if (verify_response_mode_ && mode_out != static_cast<uint8_t>(joint_mode)) {
        CountModeMismatch(kind);
        return false;
      }
      if (received_mode) {
        *received_mode = static_cast<packets::JointMode>(mode_out);
      }
      ++PerKindStats(kind).ok;
      return true;
    }
    return false;
  }

  // Request sensor read (individual: 0x14~0x17). 3B send, 67B recv.
  [[nodiscard]] bool RequestSensorRead(
      packets::Command cmd, std::array<int32_t, udp_hand_driver::kSensorValuesPerFingertip>& out,
      packets::SensorMode sensor_mode = packets::SensorMode::kRaw) noexcept {
    std::array<uint8_t, packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, packets::kSensorResponseSize> recv_buf{};

    codec::EncodeSensorReadRequest(cmd, send_buf, sensor_mode);

    const ssize_t sent =
        sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
               reinterpret_cast<const sockaddr*>(&target_addr_), sizeof(target_addr_));
    if (sent < 0)
      return false;

    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd =
          (attempt == 0) ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
                         : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          CountRecvFail(RequestKind::kSensorRead);
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(packets::kSensorResponseSize)) {
        CountShortOrDecode(RequestKind::kSensorRead, recvd);
        continue;
      }

      uint8_t cmd_out, mode_out;
      const bool ok = codec::DecodeSensorResponseRaw(
          recv_buf.data(), static_cast<std::size_t>(recvd), cmd_out, mode_out, out);
      if (!ok) {
        CountShortOrDecode(RequestKind::kSensorRead, recvd);
        continue;
      }

      if (cmd_out != static_cast<uint8_t>(cmd)) {
        CountCmdMismatch(RequestKind::kSensorRead, cmd_out, recvd);
        continue;
      }
      if (verify_response_mode_ && mode_out != static_cast<uint8_t>(sensor_mode)) {
        CountModeMismatch(RequestKind::kSensorRead);
        return false;
      }
      ++PerKindStats(RequestKind::kSensorRead).ok;
      return true;
    }
    return false;
  }

  // Request bulk motor read (cmd=0x10). 3B send, 123B recv.
  // `kind` attributes per-kind stats: the caller distinguishes motor- vs
  // joint-space use of this method (identical wire format, different MODE).
  [[nodiscard]] bool RequestAllMotorRead(std::array<float, packets::kMotorDataCount>& positions,
                                         std::array<float, packets::kMotorDataCount>& velocities,
                                         std::array<float, packets::kMotorDataCount>& currents,
                                         packets::JointMode joint_mode = packets::JointMode::kMotor,
                                         packets::JointMode* received_mode = nullptr,
                                         RequestKind kind = RequestKind::kMotorRead) noexcept {
    std::array<uint8_t, packets::kAllMotorRequestSize> send_buf{};
    std::array<uint8_t, packets::kAllMotorResponseSize> recv_buf{};

    codec::EncodeReadAllMotorsRequest(send_buf, joint_mode);

    const ssize_t sent =
        sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
               reinterpret_cast<const sockaddr*>(&target_addr_), sizeof(target_addr_));
    if (sent < 0)
      return false;

    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd =
          (attempt == 0) ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
                         : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          CountRecvFail(kind);
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(packets::kAllMotorResponseSize)) {
        CountShortOrDecode(kind, recvd);
        continue;
      }

      uint8_t cmd_out, mode_out;
      if (!codec::DecodeAllMotorResponse(recv_buf.data(), static_cast<std::size_t>(recvd), cmd_out,
                                         mode_out, positions, velocities, currents)) {
        CountShortOrDecode(kind, recvd);
        continue;
      }
      if (cmd_out != static_cast<uint8_t>(packets::Command::kReadAllMotors)) {
        CountCmdMismatch(kind, cmd_out, recvd);
        continue;
      }
      if (verify_response_mode_ && mode_out != static_cast<uint8_t>(joint_mode)) {
        CountModeMismatch(kind);
        return false;
      }
      if (received_mode) {
        *received_mode = static_cast<packets::JointMode>(mode_out);
      }
      ++PerKindStats(kind).ok;
      return true;
    }
    return false;
  }

  // Request bulk sensor read (cmd=0x19). 3B send, 259B recv.
  [[nodiscard]] bool RequestAllSensorRead(
      int32_t* out, int num_fingertips,
      packets::SensorMode sensor_mode = packets::SensorMode::kRaw) noexcept {
    std::array<uint8_t, packets::kAllSensorRequestSize> send_buf{};
    std::array<uint8_t, packets::kAllSensorResponseSize> recv_buf{};

    codec::EncodeReadAllSensorsRequest(send_buf, sensor_mode);

    const ssize_t sent =
        sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
               reinterpret_cast<const sockaddr*>(&target_addr_), sizeof(target_addr_));
    if (sent < 0)
      return false;

    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd =
          (attempt == 0) ? RecvWithTimeout(recv_buf.data(), recv_buf.size())
                         : ::recv(socket_fd_, recv_buf.data(), recv_buf.size(), MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          CountRecvFail(RequestKind::kBulkSensor);
        }
        return false;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(packets::kAllSensorResponseSize)) {
        CountShortOrDecode(RequestKind::kBulkSensor, recvd);
        continue;
      }

      uint8_t cmd_out, mode_out;
      if (!codec::DecodeAllSensorResponseRaw(recv_buf.data(), static_cast<std::size_t>(recvd),
                                             cmd_out, mode_out, out, num_fingertips)) {
        CountShortOrDecode(RequestKind::kBulkSensor, recvd);
        continue;
      }
      if (cmd_out != static_cast<uint8_t>(packets::Command::kReadAllSensors)) {
        CountCmdMismatch(RequestKind::kBulkSensor, cmd_out, recvd);
        continue;
      }
      if (verify_response_mode_ && mode_out != static_cast<uint8_t>(sensor_mode)) {
        CountModeMismatch(RequestKind::kBulkSensor);
        return false;
      }
      ++PerKindStats(RequestKind::kBulkSensor).ok;
      return true;
    }
    return false;
  }

  // Request bulk sensor read (cmd=0x19) returning the RAW response bytes.
  // Send 3B, recv up to `capacity` into `buf`. Validates recvd >= expected_size
  // and the echoed header (cmd==0x19, mode==sensor_mode). Returns the received
  // byte count on success, -1 on timeout / short read / header mismatch.
  //
  // Unlike RequestAllSensorRead (which decodes the 1a 259B layout inline), this
  // leaves decoding to the injected SensorProtocol so the response size/layout
  // can vary by firmware version (1a=259B, 1b=99B). The request itself is
  // identical across versions.
  [[nodiscard]] ssize_t RequestBulkSensorRaw(
      uint8_t* buf, std::size_t capacity, std::size_t expected_size,
      packets::SensorMode sensor_mode = packets::SensorMode::kRaw) noexcept {
    std::array<uint8_t, packets::kAllSensorRequestSize> send_buf{};
    codec::EncodeReadAllSensorsRequest(send_buf, sensor_mode);

    const ssize_t sent =
        sendto(socket_fd_, send_buf.data(), send_buf.size(), 0,
               reinterpret_cast<const sockaddr*>(&target_addr_), sizeof(target_addr_));
    if (sent < 0)
      return -1;

    constexpr int kMaxAttempts = 3;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      const ssize_t recvd = (attempt == 0) ? RecvWithTimeout(buf, capacity)
                                           : ::recv(socket_fd_, buf, capacity, MSG_DONTWAIT);
      if (recvd < 0) {
        if (attempt == 0) {
          CountRecvFail(RequestKind::kBulkSensor);
        }
        return -1;
      }
      if (attempt == 0) {
        ++comm_stats_.recv_ok;
      }

      if (recvd < static_cast<ssize_t>(expected_size)) {
        CountShortOrDecode(RequestKind::kBulkSensor, recvd);
        continue;
      }
      if (buf[1] != static_cast<uint8_t>(packets::Command::kReadAllSensors)) {
        CountCmdMismatch(RequestKind::kBulkSensor, buf[1], recvd);
        continue;
      }
      if (verify_bulk_sensor_mode_ && buf[2] != static_cast<uint8_t>(sensor_mode)) {
        CountModeMismatch(RequestKind::kBulkSensor);
        return -1;
      }
      ++PerKindStats(RequestKind::kBulkSensor).ok;
      return recvd;
    }
    return -1;
  }

  // Set sensor mode (CMD=0x04, 3B send, 3B recv echo).
  [[nodiscard]] bool RequestSetSensorMode(packets::SensorMode sensor_mode) noexcept {
    std::array<uint8_t, packets::kSensorRequestSize> send_buf{};
    std::array<uint8_t, packets::kSensorRequestSize> recv_buf{};

    codec::EncodeSetSensorMode(sensor_mode, send_buf);
    const ssize_t recvd = SendAndRecvRaw(send_buf.data(), send_buf.size(), recv_buf.data(),
                                         recv_buf.size(), RequestKind::kSetMode);
    if (recvd < static_cast<ssize_t>(packets::kSensorRequestSize)) {
      if (recvd >= 0) {
        CountShortOrDecode(RequestKind::kSetMode, recvd);
      }
      return false;
    }
    // cmd floor: reject a wrong-command echo regardless of MODE gating, so the
    // 1b MODE-don't-care path still validates that the firmware acknowledged the
    // set-mode command (not just "any 3 bytes = success").
    if (recv_buf[1] != static_cast<uint8_t>(packets::Command::kSetSensorMode)) {
      CountCmdMismatch(RequestKind::kSetMode, recv_buf[1], recvd);
      return false;
    }
    if (verify_response_mode_ && recv_buf[2] != static_cast<uint8_t>(sensor_mode)) {
      CountModeMismatch(RequestKind::kSetMode);
      return false;
    }
    ++PerKindStats(RequestKind::kSetMode).ok;
    return true;
  }

  // Sensor initialization: switch from NN to RAW mode with retries.
  [[nodiscard]] bool InitializeSensors(int max_retries = 5, int retry_interval_ms = 100) noexcept {
    const auto logger = ::udp_hand_driver::logging::TransportLogger();
    for (int attempt = 0; attempt < max_retries; ++attempt) {
      if (RequestSetSensorMode(packets::SensorMode::kRaw)) {
        RCLCPP_INFO(logger, "Sensor mode set to RAW (attempt %d/%d)", attempt + 1, max_retries);
        return true;
      }
      RCLCPP_WARN(logger, "Sensor mode switch retry %d/%d failed", attempt + 1, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
    }
    RCLCPP_ERROR(logger, "Sensor mode switch to RAW failed after %d retries", max_retries);
    return false;
  }

  // ── Statistics accessors ──────────────────────────────────────────────────

  [[nodiscard]] const UdpHandCommStats& comm_stats() const noexcept { return comm_stats_; }

  [[nodiscard]] UdpHandCommStats& comm_stats_mut() noexcept { return comm_stats_; }

  // MODE-byte validation gate for the joint / motor / set-sensor-mode responses.
  // Default true = strict (both 1a and 1b echo the requested joint/sensor mode
  // accurately on these paths). Injected from SensorProtocol::VerifiesResponseMode().
  void set_verify_response_mode(bool v) noexcept { verify_response_mode_ = v; }

  [[nodiscard]] bool verify_response_mode() const noexcept { return verify_response_mode_; }

  // MODE-byte validation gate for the bulk-sensor (0x19) response specifically.
  // Default true; 1b sets this false because its bulk-sensor response fills the
  // MODE byte with an arbitrary value, so a MODE mismatch there is meaningless
  // and must not drop otherwise-valid sensor data. Injected from
  // SensorProtocol::VerifiesBulkSensorResponseMode(); independent of the
  // joint/set-mode gate above.
  void set_verify_bulk_sensor_mode(bool v) noexcept { verify_bulk_sensor_mode_ = v; }

  [[nodiscard]] bool verify_bulk_sensor_mode() const noexcept { return verify_bulk_sensor_mode_; }

  [[nodiscard]] uint64_t recv_error_count() const noexcept {
    return recv_error_count_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] double recv_timeout_ms() const noexcept { return recv_timeout_ms_; }

 private:
  // ── Per-kind stat counting (hot path: counter increments only) ────────────

  [[nodiscard]] UdpHandCommStats::PerKind& PerKindStats(RequestKind kind) noexcept {
    return comm_stats_.per_kind[static_cast<std::size_t>(kind)];
  }

  // First-attempt recv failure: classify timeout (EAGAIN) vs socket error.
  void CountRecvFail(RequestKind kind) noexcept {
    recv_error_count_.fetch_add(1, std::memory_order_relaxed);
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      ++comm_stats_.recv_timeout;
      ++PerKindStats(kind).timeout;
    } else {
      ++comm_stats_.recv_error;
      ++PerKindStats(kind).error;
    }
  }

  void CountCmdMismatch(RequestKind kind, uint8_t got_cmd, ssize_t recvd) noexcept {
    ++comm_stats_.cmd_mismatch;
    ++PerKindStats(kind).cmd_mismatch;
    comm_stats_.last_unexpected_cmd = got_cmd;
    comm_stats_.last_unexpected_len = static_cast<uint16_t>(recvd);
  }

  void CountModeMismatch(RequestKind kind) noexcept {
    ++comm_stats_.mode_mismatch;
    ++PerKindStats(kind).mode_mismatch;
  }

  void CountShortOrDecode(RequestKind kind, ssize_t recvd) noexcept {
    ++PerKindStats(kind).short_or_decode;
    comm_stats_.last_unexpected_len = static_cast<uint16_t>(recvd);
  }

  // Sub-ms precision recv using ppoll (hrtimer on PREEMPT_RT kernels).
  [[nodiscard]] ssize_t RecvWithTimeout(void* buf, std::size_t len) noexcept {
    struct pollfd pfd {};

    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    struct timespec ts {};

    const long total_ns = static_cast<long>(recv_timeout_ms_ * 1'000'000.0);
    ts.tv_sec = total_ns / 1'000'000'000;
    ts.tv_nsec = total_ns % 1'000'000'000;

    const int ret = ::ppoll(&pfd, 1, &ts, nullptr);
    if (ret <= 0) {
      errno = EAGAIN;
      return -1;
    }
    return ::recv(socket_fd_, buf, len, MSG_DONTWAIT);
  }

  // Send raw bytes and receive into a buffer. Returns bytes received.
  [[nodiscard]] ssize_t SendAndRecvRaw(const uint8_t* send_data, std::size_t send_len,
                                       uint8_t* recv_data, std::size_t recv_len,
                                       RequestKind kind) noexcept {
    const ssize_t sent =
        sendto(socket_fd_, send_data, send_len, 0, reinterpret_cast<const sockaddr*>(&target_addr_),
               sizeof(target_addr_));
    if (sent < 0)
      return -1;

    const ssize_t recvd = RecvWithTimeout(recv_data, recv_len);
    if (recvd < 0) {
      CountRecvFail(kind);
    } else {
      ++comm_stats_.recv_ok;
    }
    return recvd;
  }

  std::string target_ip_;
  int target_port_;
  int socket_fd_{-1};
  double recv_timeout_ms_;
  sockaddr_in target_addr_{};

  UdpHandCommStats comm_stats_;
  std::atomic<uint64_t> recv_error_count_{0};
  bool verify_response_mode_{
      true};  // strict MODE echo check (joint/set-mode); injected per protocol capability
  bool verify_bulk_sensor_mode_{
      true};  // strict MODE echo check (bulk-sensor 0x19); injected per protocol capability
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_UDP_HAND_TRANSPORT_HPP_
