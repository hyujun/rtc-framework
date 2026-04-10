// Unit tests for hand_udp_transport.hpp — mode validation on request-response.
//
// Tier 2: Uses loopback UDP sockets to verify that transport methods correctly
// reject responses whose mode field doesn't match the requested mode.

#include <gtest/gtest.h>

#include "ur5e_hand_driver/hand_udp_transport.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rtc::test {

using namespace hand_packets;

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
    if (fd_ >= 0) close(fd_);
  }

  [[nodiscard]] int port() const { return port_; }

  // Receive a request, then send back a crafted response.
  void RespondWith(const uint8_t* data, std::size_t len) {
    std::array<uint8_t, kMaxPacketSize> req{};
    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    ::recvfrom(fd_, req.data(), req.size(), 0,
               reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
    ::sendto(fd_, data, len, 0,
             reinterpret_cast<const sockaddr*>(&client_addr), addr_len);
  }

 private:
  int fd_{-1};
  int port_{0};
};

// ── RequestMotorRead mode validation ────────────────────────────────────────────

TEST(HandUdpTransportModeValidation, MotorRead_ModeMatch_ReturnsTrue) {
  LoopbackDevice device;
  HandUdpTransport transport("127.0.0.1", device.port(), 50.0);
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
  const bool result = transport.RequestMotorRead(
      Command::kReadPosition, out, JointMode::kMotor);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

TEST(HandUdpTransportModeValidation, MotorRead_ModeMismatch_ReturnsFalse) {
  LoopbackDevice device;
  HandUdpTransport transport("127.0.0.1", device.port(), 50.0);
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
  const bool result = transport.RequestMotorRead(
      Command::kReadPosition, out, JointMode::kMotor);
  dev_thread.join();

  EXPECT_FALSE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

// ── RequestAllMotorRead mode validation ─────────────────────────────────────────

TEST(HandUdpTransportModeValidation, AllMotorRead_ModeMatch_ReturnsTrue) {
  LoopbackDevice device;
  HandUdpTransport transport("127.0.0.1", device.port(), 50.0);
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
  const bool result = transport.RequestAllMotorRead(
      pos, vel, cur, JointMode::kJoint);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

TEST(HandUdpTransportModeValidation, AllMotorRead_ModeMismatch_ReturnsFalse) {
  LoopbackDevice device;
  HandUdpTransport transport("127.0.0.1", device.port(), 50.0);
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
  const bool result = transport.RequestAllMotorRead(
      pos, vel, cur, JointMode::kJoint);
  dev_thread.join();

  EXPECT_FALSE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

// ── RequestAllSensorRead mode validation (already correct, regression test) ─────

TEST(HandUdpTransportModeValidation, AllSensorRead_ModeMismatch_ReturnsFalse) {
  LoopbackDevice device;
  HandUdpTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  AllSensorResponsePacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadAllSensors);
  response.mode = static_cast<uint8_t>(SensorMode::kNn);  // MISMATCH (requested kRaw)
  std::array<uint8_t, kAllSensorResponseSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kAllSensorResponseSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<int32_t, kDefaultNumFingertips * kSensorValuesPerFingertip> out{};
  const bool result = transport.RequestAllSensorRead(
      out.data(), kDefaultNumFingertips, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_FALSE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 1u);
}

TEST(HandUdpTransportModeValidation, AllSensorRead_ModeMatch_ReturnsTrue) {
  LoopbackDevice device;
  HandUdpTransport transport("127.0.0.1", device.port(), 50.0);
  ASSERT_TRUE(transport.Open());

  AllSensorResponsePacket response{};
  response.id = kDeviceId;
  response.cmd = static_cast<uint8_t>(Command::kReadAllSensors);
  response.mode = static_cast<uint8_t>(SensorMode::kRaw);
  std::array<uint8_t, kAllSensorResponseSize> resp_buf{};
  std::memcpy(resp_buf.data(), &response, kAllSensorResponseSize);

  std::thread dev_thread([&]() { device.RespondWith(resp_buf.data(), resp_buf.size()); });

  std::array<int32_t, kDefaultNumFingertips * kSensorValuesPerFingertip> out{};
  const bool result = transport.RequestAllSensorRead(
      out.data(), kDefaultNumFingertips, SensorMode::kRaw);
  dev_thread.join();

  EXPECT_TRUE(result);
  EXPECT_EQ(transport.comm_stats().mode_mismatch, 0u);
}

}  // namespace rtc::test
