// ── test_can_loopback.cpp ───────────────────────────────────────────────────
// Integration tests for rtc::CanSocket and rtc::CanTransport over a virtual
// CAN interface (vcan0). Tests that need real frame traffic GTEST_SKIP() when
// vcan0 is absent so plain CI stays green.
//
// Local setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set up vcan0
//
// Covers:
//   - Construction/Open/Bind lifecycle and invalid-interface failure
//   - Frame round-trip between two sockets on vcan0 with payload integrity
//   - SO_RCVTIMEO returns -1 on timeout
//   - RAII destructor closes the underlying file descriptor
//   - CanTransport payload round-trip, >8-byte send rejection, rx filter,
//     extended-frame IDs, receive_own_messages, is_open() lifecycle
// ─────────────────────────────────────────────────────────────────────────────
#include "can_test_support.hpp"
#include "rtc_communication/can/can_socket.hpp"
#include "rtc_communication/can/can_transport.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <linux/can.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>

namespace {

using rtc::test::kVcanInterface;

// ── CanSocket: lifecycle ────────────────────────────────────────────────────
TEST(CanSocketTest, DefaultConstructedIsNotOpen) {
  rtc::CanSocket sock;
  EXPECT_FALSE(sock.is_open());
  EXPECT_LT(sock.fd(), 0);
}

TEST(CanSocketTest, BindInvalidInterfaceFails) {
  rtc::CanSocket sock;
  EXPECT_FALSE(sock.Bind("nonexistent999"));
  EXPECT_FALSE(sock.is_open());  // Bind failure closes the socket
}

// ── CanSocket: RAII closes fd on destruction ────────────────────────────────
TEST(CanSocketTest, DestructorClosesFd) {
  int captured_fd = -1;
  {
    rtc::CanSocket sock;
    ASSERT_TRUE(sock.Open());
    captured_fd = sock.fd();
    ASSERT_GE(captured_fd, 0);
    EXPECT_GE(::fcntl(captured_fd, F_GETFD), 0);
  }
  errno = 0;
  EXPECT_EQ(::fcntl(captured_fd, F_GETFD), -1);
  EXPECT_EQ(errno, EBADF);
}

// ── CanSocket: frame round-trip over vcan0 ──────────────────────────────────
TEST(CanSocketTest, VcanFrameRoundTrip) {
  SKIP_IF_NO_VCAN();

  rtc::CanSocket receiver;
  ASSERT_TRUE(receiver.Bind(kVcanInterface));
  receiver.SetRecvTimeout(500);

  rtc::CanSocket sender;
  ASSERT_TRUE(sender.Bind(kVcanInterface));

  can_frame tx{};
  tx.can_id = 0x123;
  tx.len = 4;
  tx.data[0] = 0xDE;
  tx.data[1] = 0xAD;
  tx.data[2] = 0xBE;
  tx.data[3] = 0xEF;
  ASSERT_GT(sender.SendFrame(tx), 0);

  can_frame rx{};
  ASSERT_EQ(receiver.RecvFrame(rx), static_cast<ssize_t>(sizeof(rx)));
  EXPECT_EQ(rx.can_id, tx.can_id);
  EXPECT_EQ(rx.len, tx.len);
  EXPECT_EQ(0, std::memcmp(rx.data, tx.data, tx.len));
}

// ── CanSocket: SO_RCVTIMEO triggers timeout return ──────────────────────────
TEST(CanSocketTest, RecvTimeoutReturnsError) {
  SKIP_IF_NO_VCAN();

  rtc::CanSocket receiver;
  ASSERT_TRUE(receiver.Bind(kVcanInterface));
  receiver.SetRecvTimeout(20);  // 20 ms

  can_frame rx{};
  const auto t0 = std::chrono::steady_clock::now();
  const ssize_t n = receiver.RecvFrame(rx);
  const auto t1 = std::chrono::steady_clock::now();

  EXPECT_LT(n, 0);  // EAGAIN/EWOULDBLOCK on timeout
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  EXPECT_GE(elapsed_ms, 15);   // honoured the timeout (loose lower bound)
  EXPECT_LT(elapsed_ms, 500);  // didn't block indefinitely
}

// ── CanTransport: invalid interface fails Open ──────────────────────────────
TEST(CanTransportTest, OpenFailsOnInvalidInterface) {
  rtc::CanTransportConfig cfg{};
  cfg.interface_name = "nonexistent999";
  rtc::CanTransport transport(cfg);
  EXPECT_FALSE(transport.Open());
  EXPECT_FALSE(transport.is_open());
}

// ── CanTransport: payload round-trip + is_open lifecycle ────────────────────
TEST(CanTransportTest, VcanPayloadRoundTrip) {
  SKIP_IF_NO_VCAN();

  rtc::CanTransportConfig recv_cfg{};
  recv_cfg.interface_name = kVcanInterface;
  recv_cfg.rx_can_id = 0x123;
  recv_cfg.rx_can_mask = CAN_SFF_MASK;
  recv_cfg.recv_timeout_ms = 500;
  rtc::CanTransport receiver(recv_cfg);
  ASSERT_TRUE(receiver.Open());
  EXPECT_TRUE(receiver.is_open());

  rtc::CanTransportConfig send_cfg{};
  send_cfg.interface_name = kVcanInterface;
  send_cfg.tx_can_id = 0x123;
  rtc::CanTransport sender(send_cfg);
  ASSERT_TRUE(sender.Open());

  const std::array<uint8_t, 8> payload{0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67};
  ASSERT_EQ(sender.Send(payload), static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 64> buf{};
  const ssize_t n = receiver.Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), payload.data(), payload.size()));

  receiver.Close();
  EXPECT_FALSE(receiver.is_open());
}

// ── CanTransport: classic CAN rejects >8-byte payload ───────────────────────
TEST(CanTransportTest, OversizedSendRejected) {
  SKIP_IF_NO_VCAN();

  rtc::CanTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  cfg.tx_can_id = 0x100;
  rtc::CanTransport transport(cfg);
  ASSERT_TRUE(transport.Open());

  const std::array<uint8_t, 9> too_big{};
  EXPECT_LT(transport.Send(too_big), 0);  // 9 bytes > CAN_MAX_DLEN

  const std::array<uint8_t, 8> max_ok{};
  EXPECT_EQ(transport.Send(max_ok), static_cast<ssize_t>(max_ok.size()));  // boundary passes
}

// ── CanTransport: rx filter drops non-matching IDs ──────────────────────────
TEST(CanTransportTest, FilterRejectsOtherIdAcceptsMatch) {
  SKIP_IF_NO_VCAN();

  rtc::CanTransportConfig recv_cfg{};
  recv_cfg.interface_name = kVcanInterface;
  recv_cfg.rx_can_id = 0x123;
  recv_cfg.rx_can_mask = CAN_SFF_MASK;
  recv_cfg.recv_timeout_ms = 50;
  rtc::CanTransport receiver(recv_cfg);
  ASSERT_TRUE(receiver.Open());

  rtc::CanTransportConfig other_cfg{};
  other_cfg.interface_name = kVcanInterface;
  other_cfg.tx_can_id = 0x456;  // does not match the receiver filter
  rtc::CanTransport other_sender(other_cfg);
  ASSERT_TRUE(other_sender.Open());

  const std::array<uint8_t, 4> payload{1, 2, 3, 4};
  ASSERT_EQ(other_sender.Send(payload), static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 64> buf{};
  EXPECT_LT(receiver.Recv(buf), 0);  // filtered out -> times out

  rtc::CanTransportConfig match_cfg{};
  match_cfg.interface_name = kVcanInterface;
  match_cfg.tx_can_id = 0x123;
  rtc::CanTransport match_sender(match_cfg);
  ASSERT_TRUE(match_sender.Open());

  ASSERT_EQ(match_sender.Send(payload), static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(receiver.Recv(buf), static_cast<ssize_t>(payload.size()));
}

// ── CanTransport: extended-frame (29-bit) IDs round-trip ────────────────────
TEST(CanTransportTest, ExtendedFrameRoundTrip) {
  SKIP_IF_NO_VCAN();

  rtc::CanTransportConfig recv_cfg{};
  recv_cfg.interface_name = kVcanInterface;
  recv_cfg.rx_can_id = 0x1ABCDEF;  // > 11-bit, needs EFF
  recv_cfg.rx_can_mask = CAN_EFF_MASK;
  recv_cfg.extended_frame = true;
  recv_cfg.recv_timeout_ms = 500;
  rtc::CanTransport receiver(recv_cfg);
  ASSERT_TRUE(receiver.Open());

  rtc::CanTransportConfig send_cfg{};
  send_cfg.interface_name = kVcanInterface;
  send_cfg.tx_can_id = 0x1ABCDEF;
  send_cfg.extended_frame = true;
  rtc::CanTransport sender(send_cfg);
  ASSERT_TRUE(sender.Open());

  const std::array<uint8_t, 3> payload{0xAA, 0xBB, 0xCC};
  ASSERT_EQ(sender.Send(payload), static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 64> buf{};
  ASSERT_EQ(receiver.Recv(buf), static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), payload.data(), payload.size()));
}

// ── CanTransport: RTR frames are dropped even without a filter ──────────────
TEST(CanTransportTest, RtrFrameDropped) {
  SKIP_IF_NO_VCAN();

  rtc::CanTransportConfig recv_cfg{};
  recv_cfg.interface_name = kVcanInterface;  // rx_can_mask = 0: no filter
  recv_cfg.recv_timeout_ms = 50;
  rtc::CanTransport receiver(recv_cfg);
  ASSERT_TRUE(receiver.Open());

  rtc::CanSocket rtr_sender;
  ASSERT_TRUE(rtr_sender.Bind(kVcanInterface));

  can_frame rtr{};
  rtr.can_id = 0x123 | CAN_RTR_FLAG;
  rtr.len = 4;  // requested length; RTR frames carry no data
  ASSERT_GT(rtr_sender.SendFrame(rtr), 0);

  std::array<uint8_t, 64> buf{};
  EXPECT_LT(receiver.Recv(buf), 0);  // RTR dropped -> times out
}

// ── CanTransport: receive_own_messages echoes own frames ────────────────────
TEST(CanTransportTest, ReceiveOwnMessages) {
  SKIP_IF_NO_VCAN();

  rtc::CanTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  cfg.tx_can_id = 0x321;
  cfg.receive_own_messages = true;
  cfg.recv_timeout_ms = 500;
  rtc::CanTransport transport(cfg);
  ASSERT_TRUE(transport.Open());

  const std::array<uint8_t, 2> payload{0x55, 0x66};
  ASSERT_EQ(transport.Send(payload), static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 64> buf{};
  ASSERT_EQ(transport.Recv(buf), static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), payload.data(), payload.size()));
}

}  // namespace
