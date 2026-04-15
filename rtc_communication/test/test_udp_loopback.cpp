// ── test_udp_loopback.cpp ───────────────────────────────────────────────────
// Integration tests for rtc::UdpSocket and rtc::UdpTransport over 127.0.0.1
// loopback. Validates the contract that allows higher-level Transceiver code
// to assume reliable, allocation-free, RT-safe I/O semantics.
//
// Covers:
//   - Bind/Connect/Send/Recv round-trip with payload integrity
//   - SO_RCVTIMEO blocks for the configured duration and returns -1 on timeout
//   - RAII destructor closes the underlying file descriptor
//   - UdpTransport bind+connect lifecycle and is_open() reporting
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_communication/udp/udp_socket.hpp"
#include "rtc_communication/udp/udp_transport.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>

namespace {

// Reads the kernel-assigned port from a bound socket fd.
[[nodiscard]] int GetBoundPort(int fd) {
  sockaddr_in addr{};
  socklen_t   len = sizeof(addr);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) < 0) {
    return -1;
  }
  return ntohs(addr.sin_port);
}

// ── UdpSocket: bind + send + recv loopback ──────────────────────────────────
TEST(UdpSocketTest, LoopbackSendRecvRoundTrip) {
  rtc::UdpSocket receiver;
  ASSERT_TRUE(receiver.Bind("127.0.0.1", 0));  // kernel picks free port
  receiver.SetRecvTimeout(500);                // 500 ms safety bound

  const int port = GetBoundPort(receiver.fd());
  ASSERT_GT(port, 0);

  rtc::UdpSocket sender;
  ASSERT_TRUE(sender.Connect("127.0.0.1", port));

  const std::array<uint8_t, 8> payload{0xDE, 0xAD, 0xBE, 0xEF,
                                        0x01, 0x23, 0x45, 0x67};
  const ssize_t n_sent = sender.Send(payload);
  ASSERT_EQ(n_sent, static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 64> recv_buf{};
  const ssize_t n_recv = receiver.Recv(recv_buf);
  ASSERT_EQ(n_recv, static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0,
            std::memcmp(recv_buf.data(), payload.data(), payload.size()));
}

// ── UdpSocket: SO_RCVTIMEO triggers timeout return ──────────────────────────
TEST(UdpSocketTest, RecvTimeoutReturnsError) {
  rtc::UdpSocket receiver;
  ASSERT_TRUE(receiver.Bind("127.0.0.1", 0));
  receiver.SetRecvTimeout(20);  // 20 ms

  std::array<uint8_t, 32> buf{};
  const auto t0   = std::chrono::steady_clock::now();
  const ssize_t n = receiver.Recv(buf);
  const auto t1   = std::chrono::steady_clock::now();

  EXPECT_LT(n, 0);  // EAGAIN/EWOULDBLOCK on timeout
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  EXPECT_GE(elapsed_ms, 15);   // honoured the timeout (loose lower bound)
  EXPECT_LT(elapsed_ms, 500);  // didn't block indefinitely
}

// ── UdpSocket: Send before Connect is rejected ──────────────────────────────
TEST(UdpSocketTest, SendWithoutConnectFails) {
  rtc::UdpSocket sock;
  ASSERT_TRUE(sock.Open());
  const std::array<uint8_t, 4> payload{1, 2, 3, 4};
  EXPECT_LT(sock.Send(payload), 0);
}

// ── UdpSocket: RAII closes fd on destruction ────────────────────────────────
TEST(UdpSocketTest, DestructorClosesFd) {
  int captured_fd = -1;
  {
    rtc::UdpSocket sock;
    ASSERT_TRUE(sock.Open());
    captured_fd = sock.fd();
    ASSERT_GE(captured_fd, 0);
    // fd is alive here — fcntl on a live fd returns >= 0
    EXPECT_GE(::fcntl(captured_fd, F_GETFD), 0);
  }
  // After scope exit, the fd must be closed: F_GETFD returns -1 with EBADF.
  errno = 0;
  EXPECT_EQ(::fcntl(captured_fd, F_GETFD), -1);
  EXPECT_EQ(errno, EBADF);
}

// ── UdpTransport: bind + connect lifecycle ──────────────────────────────────
TEST(UdpTransportTest, BindAndConnectLoopbackRoundTrip) {
  // UdpTransport::Open() skips bind when bind_port == 0 (treats it as
  // "send-only"). Discover a free port by binding a temporary socket, close
  // it, then hand the port to UdpTransport. (Tiny TOCTOU window; acceptable
  // for a single-process localhost test.)
  int picked_port = -1;
  {
    rtc::UdpSocket probe;
    ASSERT_TRUE(probe.Bind("127.0.0.1", 0));
    picked_port = GetBoundPort(probe.fd());
    ASSERT_GT(picked_port, 0);
  }

  // Step 1: bind a receiver transport on the chosen port.
  rtc::UdpTransportConfig recv_cfg{};
  recv_cfg.bind_address    = "127.0.0.1";
  recv_cfg.bind_port       = picked_port;
  recv_cfg.recv_timeout_ms = 500;
  rtc::UdpTransport recv_transport(recv_cfg);
  ASSERT_TRUE(recv_transport.Open());
  EXPECT_TRUE(recv_transport.is_open());

  const int port = picked_port;

  // Step 2: build a sender transport pointing at the receiver's port.
  rtc::UdpTransportConfig send_cfg{};
  send_cfg.target_address = "127.0.0.1";
  send_cfg.target_port    = port;
  rtc::UdpTransport send_transport(send_cfg);
  ASSERT_TRUE(send_transport.Open());

  // Step 3: round-trip.
  const std::array<uint8_t, 6> payload{0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
  ASSERT_EQ(send_transport.Send(payload),
            static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 64> buf{};
  const ssize_t n = recv_transport.Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), payload.data(), payload.size()));

  recv_transport.Close();
  EXPECT_FALSE(recv_transport.is_open());
}

}  // namespace
