// ── test_canfd_loopback.cpp ─────────────────────────────────────────────────
// Integration tests for rtc::CanFdTransport over a virtual CAN interface
// (vcan0). Tests skip when vcan0 is absent or its MTU is below CANFD_MTU
// (i.e. the interface is not FD-capable) so plain CI stays green.
//
// Local setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set up vcan0
//
// Covers:
//   - Open failure on invalid interface
//   - 64-byte FD payload round-trip with payload integrity
//   - >64-byte send rejection and the 64-byte boundary
//   - Mixed reception: an FD-enabled socket receives classic (CAN_MTU) and
//     FD (CANFD_MTU) frames on the same socket
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_communication/can/can_transport.hpp"
#include "rtc_communication/can/canfd_transport.hpp"

#include <gtest/gtest.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <numeric>

namespace {

constexpr char kVcanInterface[] = "vcan0";

// Returns the interface MTU, or -1 if the interface does not exist.
[[nodiscard]] int GetInterfaceMtu(const char* name) {
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0)
    return -1;
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
  const int rc = ::ioctl(fd, SIOCGIFMTU, &ifr);
  ::close(fd);
  return (rc < 0) ? -1 : ifr.ifr_mtu;
}

#define SKIP_IF_NO_FD_VCAN()                                           \
  if (if_nametoindex(kVcanInterface) == 0) {                           \
    GTEST_SKIP() << "vcan0 not available (sudo modprobe vcan && "      \
                    "sudo ip link add dev vcan0 type vcan && "         \
                    "sudo ip link set up vcan0)";                      \
  }                                                                    \
  if (GetInterfaceMtu(kVcanInterface) < static_cast<int>(CANFD_MTU)) { \
    GTEST_SKIP() << "vcan0 is not CAN FD capable (MTU < " << CANFD_MTU \
                 << "); sudo ip link set vcan0 mtu 72";                \
  }

// ── CanFdTransport: invalid interface fails Open ────────────────────────────
TEST(CanFdTransportTest, OpenFailsOnInvalidInterface) {
  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = "nonexistent999";
  rtc::CanFdTransport transport(cfg);
  EXPECT_FALSE(transport.Open());
  EXPECT_FALSE(transport.is_open());
}

// ── CanFdTransport: 64-byte FD payload round-trip ───────────────────────────
TEST(CanFdTransportTest, VcanFdPayloadRoundTrip) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransportConfig recv_cfg{};
  recv_cfg.interface_name = kVcanInterface;
  recv_cfg.rx_can_id = 0x200;
  recv_cfg.rx_can_mask = CAN_SFF_MASK;
  recv_cfg.recv_timeout_ms = 500;
  rtc::CanFdTransport receiver(recv_cfg);
  ASSERT_TRUE(receiver.Open());
  EXPECT_TRUE(receiver.is_open());

  rtc::CanFdTransportConfig send_cfg{};
  send_cfg.interface_name = kVcanInterface;
  send_cfg.tx_can_id = 0x200;
  rtc::CanFdTransport sender(send_cfg);
  ASSERT_TRUE(sender.Open());

  std::array<uint8_t, 64> payload{};
  std::iota(payload.begin(), payload.end(), uint8_t{0});
  ASSERT_EQ(sender.Send(payload), static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 128> buf{};
  const ssize_t n = receiver.Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), payload.data(), payload.size()));

  receiver.Close();
  EXPECT_FALSE(receiver.is_open());
}

// ── CanFdTransport: rejects >64-byte payload, accepts the boundary ──────────
TEST(CanFdTransportTest, OversizedSendRejected) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  cfg.tx_can_id = 0x300;
  rtc::CanFdTransport transport(cfg);
  ASSERT_TRUE(transport.Open());

  const std::array<uint8_t, 65> too_big{};
  EXPECT_LT(transport.Send(too_big), 0);  // 65 bytes > CANFD_MAX_DLEN

  const std::array<uint8_t, 64> max_ok{};
  EXPECT_EQ(transport.Send(max_ok), static_cast<ssize_t>(max_ok.size()));  // boundary passes
}

// ── CanFdTransport: FD socket receives classic and FD frames mixed ──────────
TEST(CanFdTransportTest, MixedClassicAndFdReception) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransportConfig recv_cfg{};
  recv_cfg.interface_name = kVcanInterface;
  recv_cfg.rx_can_id = 0x400;
  recv_cfg.rx_can_mask = CAN_SFF_MASK;
  recv_cfg.recv_timeout_ms = 500;
  rtc::CanFdTransport receiver(recv_cfg);
  ASSERT_TRUE(receiver.Open());

  // Classic frame from a CanTransport sender (arrives as CAN_MTU).
  rtc::CanTransportConfig classic_cfg{};
  classic_cfg.interface_name = kVcanInterface;
  classic_cfg.tx_can_id = 0x400;
  rtc::CanTransport classic_sender(classic_cfg);
  ASSERT_TRUE(classic_sender.Open());

  const std::array<uint8_t, 8> classic_payload{1, 2, 3, 4, 5, 6, 7, 8};
  ASSERT_EQ(classic_sender.Send(classic_payload), static_cast<ssize_t>(classic_payload.size()));

  std::array<uint8_t, 128> buf{};
  ASSERT_EQ(receiver.Recv(buf), static_cast<ssize_t>(classic_payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), classic_payload.data(), classic_payload.size()));

  // FD frame (16 bytes > classic max, arrives as CANFD_MTU).
  rtc::CanFdTransportConfig fd_cfg{};
  fd_cfg.interface_name = kVcanInterface;
  fd_cfg.tx_can_id = 0x400;
  rtc::CanFdTransport fd_sender(fd_cfg);
  ASSERT_TRUE(fd_sender.Open());

  std::array<uint8_t, 16> fd_payload{};
  std::iota(fd_payload.begin(), fd_payload.end(), uint8_t{0xA0});
  ASSERT_EQ(fd_sender.Send(fd_payload), static_cast<ssize_t>(fd_payload.size()));

  ASSERT_EQ(receiver.Recv(buf), static_cast<ssize_t>(fd_payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), fd_payload.data(), fd_payload.size()));
}

}  // namespace
