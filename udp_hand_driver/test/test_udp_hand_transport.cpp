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

 private:
  int fd_{-1};
  int port_{0};
};

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

// ── RequestAllSensorRead mode validation (already correct, regression test) ─────

TEST(HandUdpTransportModeValidation, AllSensorRead_ModeMismatch_ReturnsFalse) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  AllSensorResponsePacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadAllSensors);
  response.mode = static_cast<uint8_t>(SensorMode::kNn);  // MISMATCH (requested kRaw)
  std::array<uint8_t, kAllSensorResponseSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kAllSensorResponseSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<int32_t,
             udp_hand_driver::kDefaultNumFingertips * udp_hand_driver::kSensorValuesPerFingertip>
      out{};
  const bool result = transport.RequestAllSensorRead(
      out.data(), udp_hand_driver::kDefaultNumFingertips, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_FALSE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

TEST(HandUdpTransportModeValidation, AllSensorRead_ModeMatch_ReturnsTrue) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  AllSensorResponsePacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadAllSensors);
  response.mode = static_cast<uint8_t>(SensorMode::kRaw);
  std::array<uint8_t, kAllSensorResponseSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kAllSensorResponseSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<int32_t,
             udp_hand_driver::kDefaultNumFingertips * udp_hand_driver::kSensorValuesPerFingertip>
      out{};
  const bool result = transport.RequestAllSensorRead(
      out.data(), udp_hand_driver::kDefaultNumFingertips, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

// ── verify_response_mode gate (1b MODE don't-care) ──────────────────────────────
// 1b firmware fills the MODE byte with arbitrary values and only ever runs raw
// mode, so the transport is told to skip MODE validation. A MODE-mismatched
// response must then be accepted and must NOT bump mode_mismatch.

TEST(HandUdpTransportModeGate, DefaultIsStrict) {
  UdpHandTransport transport("127.0.0.1", 55151, 10.0);
  EXPECT_TRUE(transport.verify_response_mode());  // default strict (1a)
  transport.set_verify_response_mode(false);
  EXPECT_FALSE(transport.verify_response_mode());
}

TEST(HandUdpTransportModeGate, AllMotorRead_ModeMismatch_GateOff_Accepts) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_response_mode(false);  // 1b: MODE is don't-care

  // Mode deliberately differs from the requested kJoint (as 1b firmware would).
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

TEST(HandUdpTransportModeGate, BulkSensorRaw_ModeMismatch_GateOff_Accepts) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_response_mode(false);  // 1b: MODE is don't-care

  // 99-byte 1b bulk-sensor response with a valid cmd but an arbitrary MODE byte.
  std::array<uint8_t, kP1bSensorResponseSize> resp_buf{};
  resp_buf[0] = kDeviceId;
  resp_buf[1] = static_cast<uint8_t>(Command::kReadAllSensors);
  resp_buf[2] = 0x7F;  // arbitrary MODE (requested kRaw = 0x00)

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<uint8_t, kP1bSensorResponseSize> buf{};
  const ssize_t recvd = transport.RequestBulkSensorRaw(buf.data(), buf.size(),
                                                       kP1bSensorResponseSize, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_EQ(recvd, static_cast<ssize_t>(kP1bSensorResponseSize));
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

TEST(HandUdpTransportModeGate, BulkSensorRaw_GateOff_StillRejectsWrongCmd) {
  LoopbackDevice device;
  UdpHandTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());
  transport.set_verify_response_mode(false);

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
  transport.set_verify_response_mode(false);  // 1b

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
  EXPECT_EQ(stats.event_skip_count, 0u);
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

}  // namespace udp_hand_driver::test
