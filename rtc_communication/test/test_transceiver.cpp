// ── test_transceiver.cpp ────────────────────────────────────────────────────
// Integration tests for rtc::Transceiver<Codec> driven by FakeCodec over a
// 127.0.0.1 loopback. The Transceiver owns a UdpTransport receiver and a
// jthread decode loop; these tests validate the full StartRecv → external
// send → GetLatestState() → Stop pipeline.
//
// Covers:
//   - Lifecycle: StartRecv() spins up the recv jthread; Stop() joins cleanly
//   - Decode path: external sender's payload reaches latest_state_
//   - Callback path: registered callback is invoked per decoded packet
//   - Send path: Transceiver::Send() reaches an external receiver
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_communication/transceiver.hpp"
#include "rtc_communication/udp/udp_socket.hpp"
#include "rtc_communication/udp/udp_transport.hpp"
#include "test/fake_codec.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

namespace {

using rtc::test::FakeCodec;
using rtc::test::FakePacket;

constexpr auto kPollInterval = std::chrono::milliseconds(2);
constexpr auto kPollDeadline = std::chrono::milliseconds(500);

[[nodiscard]] int GetBoundPort(int fd) {
  sockaddr_in addr{};
  socklen_t   len = sizeof(addr);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) < 0) {
    return -1;
  }
  return ntohs(addr.sin_port);
}

// UdpTransport::Open() skips bind when bind_port == 0. To get a kernel-
// assigned free port, briefly bind a probe UdpSocket, capture its port, then
// close it and hand the port to UdpTransport. Tiny TOCTOU window — acceptable
// for a single-process localhost test.
[[nodiscard]] int PickFreeLoopbackPort() {
  rtc::UdpSocket probe;
  if (!probe.Bind("127.0.0.1", 0)) return -1;
  return GetBoundPort(probe.fd());
}

// Spins until predicate() returns true or the deadline elapses.
template <typename Pred>
[[nodiscard]] bool WaitUntil(Pred pred) {
  const auto deadline = std::chrono::steady_clock::now() + kPollDeadline;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(kPollInterval);
  }
  return false;
}

// ── Transceiver: receives a packet sent by an external UDP socket ───────────
TEST(TransceiverTest, ReceivesExternalPacket) {
  const int port = PickFreeLoopbackPort();
  ASSERT_GT(port, 0);

  rtc::UdpTransportConfig cfg{};
  cfg.bind_address    = "127.0.0.1";
  cfg.bind_port       = port;
  cfg.recv_timeout_ms = 50;  // short so Stop() returns promptly

  auto transport = std::make_unique<rtc::UdpTransport>(cfg);
  rtc::Transceiver<FakeCodec> rx(std::move(transport));

  ASSERT_TRUE(rx.StartRecv());

  rtc::UdpSocket sender;
  ASSERT_TRUE(sender.Connect("127.0.0.1", port));

  const FakePacket pkt{.sequence = 42, .payload = -7};
  std::array<uint8_t, sizeof(FakePacket)> buf{};
  std::memcpy(buf.data(), &pkt, sizeof(FakePacket));
  ASSERT_EQ(sender.Send(buf), static_cast<ssize_t>(sizeof(FakePacket)));

  ASSERT_TRUE(WaitUntil([&] { return rx.recv_count() >= 1; }))
      << "Transceiver did not decode any packet within deadline";

  const FakePacket got = rx.GetLatestState();
  EXPECT_EQ(got.sequence, 42u);
  EXPECT_EQ(got.payload, -7);

  rx.Stop();
  EXPECT_FALSE(rx.IsRunning());
}

// ── Transceiver: invokes the registered callback per decoded packet ─────────
TEST(TransceiverTest, InvokesCallbackOnDecode) {
  const int port = PickFreeLoopbackPort();
  ASSERT_GT(port, 0);

  rtc::UdpTransportConfig cfg{};
  cfg.bind_address    = "127.0.0.1";
  cfg.bind_port       = port;
  cfg.recv_timeout_ms = 50;

  auto transport = std::make_unique<rtc::UdpTransport>(cfg);
  rtc::Transceiver<FakeCodec> rx(std::move(transport));

  std::atomic<int> cb_count{0};
  std::atomic<int32_t> last_payload{0};
  rx.SetCallback([&](const FakePacket& p) {
    last_payload.store(p.payload, std::memory_order_relaxed);
    cb_count.fetch_add(1, std::memory_order_relaxed);
  });

  ASSERT_TRUE(rx.StartRecv());

  rtc::UdpSocket sender;
  ASSERT_TRUE(sender.Connect("127.0.0.1", port));

  for (int i = 1; i <= 3; ++i) {
    const FakePacket pkt{.sequence = static_cast<uint32_t>(i),
                         .payload  = i * 100};
    std::array<uint8_t, sizeof(FakePacket)> buf{};
    std::memcpy(buf.data(), &pkt, sizeof(FakePacket));
    ASSERT_EQ(sender.Send(buf), static_cast<ssize_t>(sizeof(FakePacket)));
  }

  ASSERT_TRUE(WaitUntil([&] {
    return cb_count.load(std::memory_order_relaxed) >= 3;
  }));
  EXPECT_EQ(last_payload.load(), 300);

  rx.Stop();
}

// ── Transceiver: short / oversized datagrams are silently dropped ───────────
TEST(TransceiverTest, ShortDatagramIsIgnored) {
  const int port = PickFreeLoopbackPort();
  ASSERT_GT(port, 0);

  rtc::UdpTransportConfig cfg{};
  cfg.bind_address    = "127.0.0.1";
  cfg.bind_port       = port;
  cfg.recv_timeout_ms = 50;

  auto transport = std::make_unique<rtc::UdpTransport>(cfg);
  rtc::Transceiver<FakeCodec> rx(std::move(transport));

  ASSERT_TRUE(rx.StartRecv());

  rtc::UdpSocket sender;
  ASSERT_TRUE(sender.Connect("127.0.0.1", port));

  const std::array<uint8_t, 3> truncated{1, 2, 3};
  ASSERT_EQ(sender.Send(truncated), 3);

  // Wait briefly to confirm no packet is decoded.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(rx.recv_count(), 0u);

  rx.Stop();
}

// ── Transceiver: Send() path reaches an external receiver ───────────────────
TEST(TransceiverTest, SendPathReachesExternalReceiver) {
  // External receiver socket bound on a free loopback port.
  rtc::UdpSocket external_recv;
  ASSERT_TRUE(external_recv.Bind("127.0.0.1", 0));
  external_recv.SetRecvTimeout(500);
  const int recv_port = GetBoundPort(external_recv.fd());
  ASSERT_GT(recv_port, 0);

  // Transceiver wired to send-only (no bind).
  rtc::UdpTransportConfig cfg{};
  cfg.target_address = "127.0.0.1";
  cfg.target_port    = recv_port;

  auto transport = std::make_unique<rtc::UdpTransport>(cfg);
  rtc::Transceiver<FakeCodec> tx(std::move(transport));

  // Open transport without spinning a recv thread (no bind port configured).
  ASSERT_TRUE(tx.transport()->Open());

  const FakePacket pkt{.sequence = 99, .payload = 12345};
  ASSERT_TRUE(tx.Send(pkt));
  EXPECT_EQ(tx.send_count(), 1u);

  std::array<uint8_t, sizeof(FakePacket)> buf{};
  const ssize_t n = external_recv.Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(sizeof(FakePacket)));

  FakePacket got{};
  std::memcpy(&got, buf.data(), sizeof(FakePacket));
  EXPECT_EQ(got.sequence, 99u);
  EXPECT_EQ(got.payload, 12345);
}

}  // namespace
