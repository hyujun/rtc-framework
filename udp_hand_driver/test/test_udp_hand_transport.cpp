// Unit tests for hand_udp_transport.hpp — UdpHandCommStats, transport lifecycle,
// and mode validation on request-response via loopback UDP.

#include "udp_hand_driver/udp_hand_transport.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

namespace udp_hand_driver::test {

using namespace packets;

// ── Helper: loopback UDP device simulator ──────────────────────────────────────

class LoopbackDevice {
 public:
  LoopbackDevice() {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    EXPECT_GE(fd_, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = 0;  // OS picks port
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    EXPECT_EQ(bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);

    socklen_t len = sizeof(addr);
    getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
  }

  ~LoopbackDevice() {
    if (fd_ >= 0)
      close(fd_);
  }

  [[nodiscard]] int port() const { return port_; }

  // Receive a request, then send back a crafted response.
  void RespondWith(const uint8_t* data, std::size_t len) {
    std::array<uint8_t, kMaxPacketSize> req{};
    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    ::recvfrom(fd_, req.data(), req.size(), 0, reinterpret_cast<sockaddr*>(&client_addr),
               &addr_len);
    ::sendto(fd_, data, len, 0, reinterpret_cast<const sockaddr*>(&client_addr), addr_len);
  }

  // Learn the transport's ephemeral source address from one received packet
  // (the packet itself is dropped). Enables Send() pre-injection below.
  void CaptureClientAddr() {
    std::array<uint8_t, kMaxPacketSize> req{};
    client_addr_len_ = sizeof(client_addr_);
    ::recvfrom(fd_, req.data(), req.size(), 0, reinterpret_cast<sockaddr*>(&client_addr_),
               &client_addr_len_);
  }

  // Send a raw packet to the captured client address without waiting for a
  // request — used to pre-inject stale packets into the transport's socket
  // buffer (desync simulation). Requires a prior CaptureClientAddr().
  void Send(const uint8_t* data, std::size_t len) {
    ::sendto(fd_, data, len, 0, reinterpret_cast<const sockaddr*>(&client_addr_), client_addr_len_);
  }

 private:
  int fd_{-1};
  int port_{0};
  sockaddr_in client_addr_{};
  socklen_t client_addr_len_{sizeof(sockaddr_in)};
};

// Per-kind stats index shorthand.
constexpr std::size_t KindIdx(RequestKind k) {
  return static_cast<std::size_t>(k);
}

// ── RequestMotorRead mode validation ────────────────────────────────────────────

TEST(HandUdpTransportModeValidation, MotorRead_ModeMatch_ReturnsTrue) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  // Craft a valid motor response with kMotor mode
  MotorPacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadPosition);
  response.mode = static_cast<uint8_t>(JointMode::kMotor);
  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    response.data[i] = FloatToUint32(static_cast<float>(i));
  }
  std::array<uint8_t, kMotorPacketSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kMotorPacketSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<float, kMotorDataCount> out{};
  const bool result = transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kMotor);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

TEST(HandUdpTransportModeValidation, MotorRead_ModeMismatch_ReturnsFalse) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  // Respond with kJoint mode when kMotor was requested
  MotorPacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadPosition);
  response.mode = static_cast<uint8_t>(JointMode::kJoint);  // MISMATCH
  std::array<uint8_t, kMotorPacketSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kMotorPacketSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<float, kMotorDataCount> out{};
  const bool result = transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kMotor);
  dev_thread.join();

  EXPECT_FALSE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

// ── RequestAllMotorRead mode validation ─────────────────────────────────────────

TEST(HandUdpTransportModeValidation, AllMotorRead_ModeMatch_ReturnsTrue) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  AllMotorResponsePacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadAllMotors);
  response.mode = static_cast<uint8_t>(JointMode::kJoint);
  for (std::size_t i = 0; i < kAllMotorDataCount; ++i) {
    response.data[i] = FloatToUint32(static_cast<float>(i));
  }
  std::array<uint8_t, kAllMotorResponseSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kAllMotorResponseSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<float, kMotorDataCount> pos{}, vel{}, cur{};
  const bool result = transport.RequestAllMotorRead(pos, vel, cur, JointMode::kJoint);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

TEST(HandUdpTransportModeValidation, AllMotorRead_ModeMismatch_ReturnsFalse) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  // Respond with kMotor mode when kJoint was requested
  AllMotorResponsePacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadAllMotors);
  response.mode = static_cast<uint8_t>(JointMode::kMotor);  // MISMATCH
  std::array<uint8_t, kAllMotorResponseSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kAllMotorResponseSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<float, kMotorDataCount> pos{}, vel{}, cur{};
  const bool result = transport.RequestAllMotorRead(pos, vel, cur, JointMode::kJoint);
  dev_thread.join();

  EXPECT_FALSE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

// ── MODE gates (mechanism) ──────────────────────────────────────────────────────
// The transport has two independent MODE-validation gates: verify_response_mode
// (joint / motor / set-mode paths) and verify_bulk_sensor_mode (the bulk-sensor
// 0x19 path). With a gate off, a MODE-mismatched response on that path must be
// accepted and must NOT bump mode_mismatch. 1b runs with the bulk-sensor gate
// off (its 0x19 response echoes an arbitrary MODE) while keeping the joint/set-
// mode gate on; these exercise both gates and their independence directly.

TEST(HandUdpTransportModeGate, DefaultIsStrict) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  EXPECT_TRUE(transport.verify_response_mode());     // default strict (joint/set-mode)
  EXPECT_TRUE(transport.verify_bulk_sensor_mode());  // default strict (bulk-sensor)
  transport.set_verify_response_mode(false);
  transport.set_verify_bulk_sensor_mode(false);
  EXPECT_FALSE(transport.verify_response_mode());
  EXPECT_FALSE(transport.verify_bulk_sensor_mode());
}

TEST(HandUdpTransportModeGate, AllMotorRead_ModeMismatch_GateOff_Accepts) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_response_mode(false);  // gate off

  // Mode deliberately differs from the requested kJoint.
  AllMotorResponsePacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadAllMotors);
  response.mode = static_cast<uint8_t>(JointMode::kMotor);  // arbitrary / mismatched
  for (std::size_t i = 0; i < kAllMotorDataCount; ++i) {
    response.data[i] = FloatToUint32(static_cast<float>(i));
  }
  std::array<uint8_t, kAllMotorResponseSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kAllMotorResponseSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<float, kMotorDataCount> pos{}, vel{}, cur{};
  const bool result = transport.RequestAllMotorRead(pos, vel, cur, JointMode::kJoint);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

// Helper: craft a 1b bulk-sensor response with a deliberately arbitrary MODE byte.
static void FillBulkSensorArbitraryMode(std::array<uint8_t, kP1bSensorResponseSize>& resp_buf) {
  resp_buf[0] = kDeviceId;
  resp_buf[1] = static_cast<uint8_t>(Command::kReadAllSensors);
  resp_buf[2] = 0x7F;  // arbitrary MODE (requested kRaw = 0x00)
}

TEST(HandUdpTransportModeGate, BulkSensorRaw_ModeMismatch_Default_Rejects) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  // Default: both gates strict — an arbitrary MODE on the bulk-sensor path is rejected.

  std::array<uint8_t, kP1bSensorResponseSize> resp_buf{};
  FillBulkSensorArbitraryMode(resp_buf);
  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<uint8_t, kP1bSensorResponseSize> buf{};
  const ssize_t recvd = transport.RequestBulkSensorRaw(buf.data(), buf.size(),
                                                       kP1bSensorResponseSize, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_EQ(recvd, -1);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

TEST(HandUdpTransportModeGate, BulkSensorRaw_ModeMismatch_BulkGateOff_Accepts) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_bulk_sensor_mode(false);  // 1b: bulk-sensor gate off

  std::array<uint8_t, kP1bSensorResponseSize> resp_buf{};
  FillBulkSensorArbitraryMode(resp_buf);
  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<uint8_t, kP1bSensorResponseSize> buf{};
  const ssize_t recvd = transport.RequestBulkSensorRaw(buf.data(), buf.size(),
                                                       kP1bSensorResponseSize, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_EQ(recvd, static_cast<ssize_t>(kP1bSensorResponseSize));
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

// The two gates are independent: turning off the joint/set-mode gate must NOT
// relax the bulk-sensor path (guards against a single-flag regression).
TEST(HandUdpTransportModeGate, BulkSensorRaw_JointGateOff_BulkGateOn_StillRejects) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_response_mode(false);  // joint/set-mode gate off, bulk gate still on

  std::array<uint8_t, kP1bSensorResponseSize> resp_buf{};
  FillBulkSensorArbitraryMode(resp_buf);
  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<uint8_t, kP1bSensorResponseSize> buf{};
  const ssize_t recvd = transport.RequestBulkSensorRaw(buf.data(), buf.size(),
                                                       kP1bSensorResponseSize, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_EQ(recvd, -1);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

TEST(HandUdpTransportModeGate, BulkSensorRaw_GateOff_StillRejectsWrongCmd) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_bulk_sensor_mode(false);

  // Wrong cmd must still be rejected even with the MODE gate off (cmd floor).
  std::array<uint8_t, kP1bSensorResponseSize> resp_buf{};
  resp_buf[0] = kDeviceId;
  resp_buf[1] = static_cast<uint8_t>(Command::kReadAllMotors);  // WRONG cmd
  resp_buf[2] = 0x00;

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<uint8_t, kP1bSensorResponseSize> buf{};
  const ssize_t recvd = transport.RequestBulkSensorRaw(buf.data(), buf.size(),
                                                       kP1bSensorResponseSize, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_EQ(recvd, -1);
  EXPECT_EQ(transport.comm_stats().cmd_mismatch, 1u);
}

// ── RequestSetSensorMode cmd floor + MODE gate ──────────────────────────────────

TEST(HandUdpTransportSetSensorMode, WrongCmdEcho_Rejected) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  // Echo a wrong command — the cmd floor rejects it regardless of MODE gating.
  std::array<uint8_t, kSensorRequestSize> resp_buf{};
  resp_buf[0] = kDeviceId;
  resp_buf[1] = static_cast<uint8_t>(Command::kReadAllSensors);  // WRONG cmd
  resp_buf[2] = static_cast<uint8_t>(SensorMode::kRaw);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });
  const bool result = transport.RequestSetSensorMode(SensorMode::kRaw);
  dev_thread.join();

  EXPECT_FALSE(result);
}

TEST(HandUdpTransportSetSensorMode, ModeMismatch_GateOff_Accepts) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_response_mode(false);  // gate off

  // Correct cmd echo but arbitrary MODE — accepted with the gate off.
  std::array<uint8_t, kSensorRequestSize> resp_buf{};
  resp_buf[0] = kDeviceId;
  resp_buf[1] = static_cast<uint8_t>(Command::kSetSensorMode);
  resp_buf[2] = 0x5A;  // arbitrary MODE (requested kRaw)

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });
  const bool result = transport.RequestSetSensorMode(SensorMode::kRaw);
  dev_thread.join();

  EXPECT_TRUE(result);
}

// ── UdpHandCommStats defaults ─────────────────────────────────────────────────

TEST(UdpHandCommStats, DefaultValues) {
  UdpHandCommStats stats{};
  EXPECT_EQ(stats.recv_ok, 0u);
  EXPECT_EQ(stats.recv_timeout, 0u);
  EXPECT_EQ(stats.recv_error, 0u);
  EXPECT_EQ(stats.cmd_mismatch, 0u);
  EXPECT_EQ(stats.mode_mismatch, 0u);
  EXPECT_EQ(stats.total_cycles, 0u);
}

// ── Construction ───────────────────────────────────────────────────────────

TEST(UdpHandTransport, Construction_NotOpen) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  EXPECT_FALSE(transport.is_open());
}

TEST(UdpHandTransport, RecvTimeoutMs_StoredCorrectly) {
  UdpHandTransport transport("127.0.0.1", 55151, 0.4);
  EXPECT_DOUBLE_EQ(transport.recv_timeout_ms(), 0.4);
}

TEST(UdpHandTransport, CommStats_InitiallyZero) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  const auto& stats = transport.comm_stats();
  EXPECT_EQ(stats.recv_ok, 0u);
  EXPECT_EQ(stats.recv_timeout, 0u);
  EXPECT_EQ(stats.recv_error, 0u);
  EXPECT_EQ(stats.cmd_mismatch, 0u);
  EXPECT_EQ(stats.total_cycles, 0u);
}

TEST(UdpHandTransport, RecvErrorCount_InitiallyZero) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  EXPECT_EQ(transport.recv_error_count(), 0u);
}

// ── Open / Close lifecycle ─────────────────────────────────────────────────

TEST(UdpHandTransport, OpenClose_Loopback) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  ASSERT_TRUE(transport.Open());
  EXPECT_TRUE(transport.is_open());
  transport.Close();
  EXPECT_FALSE(transport.is_open());
}

TEST(UdpHandTransport, DoubleClose_Safe) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  ASSERT_TRUE(transport.Open());
  transport.Close();
  transport.Close();  // Should not crash
  EXPECT_FALSE(transport.is_open());
}

TEST(UdpHandTransport, Open_InvalidIP_Fails) {
  UdpHandTransport transport("999.999.999.999", 55151, 10.0);
  EXPECT_FALSE(transport.Open());
  EXPECT_FALSE(transport.is_open());
}

TEST(UdpHandTransport, DestructorClosesSocket) {
  {
    UdpHandTransport transport("127.0.0.1", 55151, 10.0);
    ASSERT_TRUE(transport.Open());
    // Destructor should close without crash
  }
}

// ── CommStats mutable access ───────────────────────────────────────────────

TEST(UdpHandTransport, CommStatsMut_Writable) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  transport.comm_stats_mut().total_cycles = 42;
  EXPECT_EQ(transport.comm_stats().total_cycles, 42u);
}

// ── WritePositionFireAndForget on open socket ──────────────────────────────

TEST(UdpHandTransport, WritePositionFireAndForget_NoRecv) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  ASSERT_TRUE(transport.Open());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 1.0f;
  // Fire-and-forget: should not block or crash (no receiver)
  transport.WritePositionFireAndForget(cmd);
  transport.Close();
}

// ── Per-request-kind statistics ─────────────────────────────────────────────
// Link-down forensics: each request kind gets its own {ok, timeout, error,
// cmd_mismatch, mode_mismatch, short_or_decode} bucket, and the transport
// records the CMD byte / length of the most recent rejected packet.

TEST(HandUdpTransportPerKind, Defaults_AllZero) {
  UdpHandCommStats stats{};
  for (int k = 0; k < kNumRequestKinds; ++k) {
    const auto& pk = stats.per_kind[static_cast<std::size_t>(k)];
    EXPECT_EQ(pk.ok, 0u);
    EXPECT_EQ(pk.timeout, 0u);
    EXPECT_EQ(pk.error, 0u);
    EXPECT_EQ(pk.cmd_mismatch, 0u);
    EXPECT_EQ(pk.mode_mismatch, 0u);
    EXPECT_EQ(pk.short_or_decode, 0u);
  }
  EXPECT_EQ(stats.last_unexpected_cmd, 0u);
  EXPECT_EQ(stats.last_unexpected_len, 0u);
}

TEST(HandUdpTransportPerKind, Timeout_AttributedToCallerKind) {
  // Silent device: request times out. The caller-specified kind (kJointRead)
  // must receive the timeout — not the method-default kMotorRead bucket.
  LoopbackDevice device;  // bound but never responds
  UdpHandTransport transport("127.0.0.1", device.port(), 5.0);
  ASSERT_TRUE(transport.Open());

  std::array<float, kMotorDataCount> out{};
  EXPECT_FALSE(transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kJoint, nullptr,
                                          RequestKind::kJointRead));

  const auto& stats = transport.comm_stats();
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kJointRead)].timeout, 1u);
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kMotorRead)].timeout, 0u);
  EXPECT_EQ(stats.recv_timeout, 1u);
}

TEST(HandUdpTransportPerKind, JointVsMotor_SeparateBuckets) {
  LoopbackDevice device;  // never responds
  UdpHandTransport transport("127.0.0.1", device.port(), 5.0);
  ASSERT_TRUE(transport.Open());

  std::array<float, kMotorDataCount> out{};
  EXPECT_FALSE(transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kMotor, nullptr,
                                          RequestKind::kMotorRead));
  EXPECT_FALSE(transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kJoint, nullptr,
                                          RequestKind::kJointRead));

  const auto& stats = transport.comm_stats();
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kMotorRead)].timeout, 1u);
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kJointRead)].timeout, 1u);
  EXPECT_EQ(stats.recv_timeout, 2u);
}

TEST(HandUdpTransportPerKind, CmdMismatch_RecordsLastUnexpectedCmd) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  // Full-size motor packet echoing the WRONG command (velocity for a position
  // request) — the stale-echo/desync signature.
  MotorPacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadVelocity);  // WRONG
  response.mode = static_cast<uint8_t>(JointMode::kMotor);
  std::array<uint8_t, kMotorPacketSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kMotorPacketSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<float, kMotorDataCount> out{};
  const bool result = transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kMotor,
                                                 nullptr, RequestKind::kMotorRead);
  dev_thread.join();

  EXPECT_FALSE(result);
  const auto& stats = transport.comm_stats();
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kMotorRead)].cmd_mismatch, 1u);
  EXPECT_EQ(stats.last_unexpected_cmd, static_cast<uint8_t>(Command::kReadVelocity));
  EXPECT_EQ(stats.last_unexpected_len, kMotorPacketSize);
}

TEST(HandUdpTransportPerKind, StalePreinjected_RetryRecovers) {
  // Desync simulation: a stale wrong-cmd packet already sits in the socket
  // buffer when the request is issued. Attempt 0 consumes and rejects it
  // (cmd_mismatch), the MSG_DONTWAIT retry picks up the fresh response.
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  // Teach the device the transport's source address, then pre-inject.
  std::array<float, kNumHandMotors> cmd{};
  transport.WritePositionFireAndForget(cmd);
  device.CaptureClientAddr();

  MotorPacket stale{};
  stale.id = kDeviceId;
  stale.cmd = static_cast<uint8_t>(Command::kWritePosition);  // stale echo
  stale.mode = static_cast<uint8_t>(JointMode::kMotor);
  std::array<uint8_t, kMotorPacketSize> stale_buf{};
  std::memcpy(stale_buf.data(), &stale, kMotorPacketSize);
  device.Send(stale_buf.data(), stale_buf.size());

  MotorPacket fresh{};
  fresh.id = kDeviceId;
  fresh.cmd = static_cast<uint8_t>(Command::kReadPosition);
  fresh.mode = static_cast<uint8_t>(JointMode::kMotor);
  for (std::size_t i = 0; i < kMotorDataCount; ++i) {
    fresh.data[i] = FloatToUint32(static_cast<float>(i));
  }
  std::array<uint8_t, kMotorPacketSize> fresh_buf{};
  std::memcpy(fresh_buf.data(), &fresh, kMotorPacketSize);
  device.Send(fresh_buf.data(), fresh_buf.size());

  // Both packets are queued before the request: deterministic retry path.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::array<float, kMotorDataCount> out{};
  const bool result = transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kMotor,
                                                 nullptr, RequestKind::kMotorRead);

  EXPECT_TRUE(result);
  const auto& stats = transport.comm_stats();
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kMotorRead)].ok, 1u);
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kMotorRead)].cmd_mismatch, 1u);
  EXPECT_EQ(stats.last_unexpected_cmd, static_cast<uint8_t>(Command::kWritePosition));
}

TEST(HandUdpTransportPerKind, ShortPacket_CountsShortOrDecode) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  // Header-only response (3 B) to a motor read expecting kMotorPacketSize.
  std::array<uint8_t, kHeaderSize> short_buf{};
  short_buf[0] = kDeviceId;
  short_buf[1] = static_cast<uint8_t>(Command::kReadPosition);

  std::thread dev_thread([&]() { device.RespondWith(short_buf.data(), short_buf.size()); });

  std::array<float, kMotorDataCount> out{};
  const bool result = transport.RequestMotorRead(Command::kReadPosition, out, JointMode::kMotor,
                                                 nullptr, RequestKind::kMotorRead);
  dev_thread.join();

  EXPECT_FALSE(result);
  const auto& stats = transport.comm_stats();
  EXPECT_EQ(stats.per_kind[KindIdx(RequestKind::kMotorRead)].short_or_decode, 1u);
  EXPECT_EQ(stats.last_unexpected_len, kHeaderSize);
}

TEST(HandUdpTransportPerKind, WriteEcho_OkCounted) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  std::array<uint8_t, kHeaderSize> echo{};
  echo[0] = kDeviceId;
  echo[1] = static_cast<uint8_t>(Command::kWritePosition);

  std::thread dev_thread([&]() { device.RespondWith(echo.data(), echo.size()); });

  std::array<float, kNumHandMotors> cmd{};
  std::array<uint8_t, kMotorPacketSize> send_buf{};
  std::array<uint8_t, kMotorPacketSize> echo_buf{};
  const bool result = transport.WritePositionWithEcho(cmd, send_buf, echo_buf);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().per_kind[KindIdx(RequestKind::kWriteEcho)].ok, 1u);
}

TEST(HandUdpTransportPerKind, BulkSensorTimeout_AttributedToBulkSensor) {
  LoopbackDevice device;  // never responds
  UdpHandTransport transport("127.0.0.1", device.port(), 5.0);
  ASSERT_TRUE(transport.Open());

  std::array<uint8_t, kP1bSensorResponseSize> buf{};
  EXPECT_EQ(transport.RequestBulkSensorRaw(buf.data(), buf.size(), kP1bSensorResponseSize,
                                           SensorMode::kRaw),
            -1);
  EXPECT_EQ(transport.comm_stats().per_kind[KindIdx(RequestKind::kBulkSensor)].timeout, 1u);
}

}  // namespace udp_hand_driver::test
