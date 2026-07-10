// ── test_rs485_transport.cpp ────────────────────────────────────────────────
// Integration tests for rtc::Rs485Transport's stateful length-prefixed framer,
// driven over a PTY loopback with ROBOTIS Dynamixel Protocol 2.0 shaped frames
// (sync FF FF FD 00, id[1], length[2 LE], instruction/params, CRC16 [2 LE]).
//
// The transport is vendor-neutral: it restores frame boundaries from the sync
// prefix and length field only. CRC validation lives in the codec, so a
// Dynamixel-shaped test codec here doubles as proof that the framer hands the
// codec exactly one complete, correctly-bounded frame per Recv().
//
// Covers:
//   - Single frame round-trip
//   - Garbage bytes before the header are discarded (resync)
//   - Back-to-back frames are returned one per Recv()
//   - A frame split across two writes is reassembled
//   - Recv() returns -1 on timeout with no data
//   - Recv() deadline bounds a continuous non-frame byte stream
//   - Recv() returns -1 (not a truncated frame) when the caller buffer is
//     smaller than the frame, and the stream recovers on the next frame
//   - recv_timeout_ms <= 0 disables the timeout (blocking read) instead of
//     busy-polling
//   - Fixed-length framing (empty sync, length_size 0)
//   - discard_echo consumes the transmitted frame's self-echo
//   - Transceiver<DynamixelTestCodec> over Rs485Transport: full stack decode
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_communication/packet_codec.hpp"
#include "rtc_communication/rs485/rs485_transport.hpp"
#include "rtc_communication/transceiver.hpp"
#include "serial_test_support.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

namespace {

using rtc::Rs485Transport;
using rtc::Rs485TransportConfig;
using rtc::test::OpenPtyPair;
using rtc::test::PtyPair;

// Dynamixel Protocol 2.0 CRC-16 (poly 0x8005, MSB-first, init 0x0000).
[[nodiscard]] uint16_t DynamixelCrc16(std::span<const uint8_t> data) {
  uint16_t crc = 0;
  for (const uint8_t byte : data) {
    crc ^= static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x8005)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// Builds a Protocol 2.0 status packet: FF FF FD 00 | id | len(LE) | 0x55 |
// error | params... | crc(LE). length = instruction + error + params + crc.
[[nodiscard]] std::vector<uint8_t> BuildStatusFrame(uint8_t id, uint8_t error,
                                                    std::span<const uint8_t> params) {
  std::vector<uint8_t> f{0xFF, 0xFF, 0xFD, 0x00, id};
  const uint16_t length =
      static_cast<uint16_t>(1 /*inst*/ + 1 /*error*/ + params.size() + 2 /*crc*/);
  f.push_back(static_cast<uint8_t>(length & 0xFF));
  f.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));
  f.push_back(0x55);  // status instruction
  f.push_back(error);
  f.insert(f.end(), params.begin(), params.end());
  const uint16_t crc = DynamixelCrc16(std::span<const uint8_t>(f.data(), f.size()));
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return f;
}

// A fixed 4-param status packet, used to satisfy Transceiver's fixed-size
// RecvPacket contract. (Dynamixel status length varies with param count; the
// codec models one specific response shape, e.g. a 4-byte present-position.)
#pragma pack(push, 1)

struct DxlStatus4 {
  uint8_t header[4];
  uint8_t id;
  uint16_t length;
  uint8_t instruction;
  uint8_t error;
  uint8_t params[4];
  uint16_t crc;
};

#pragma pack(pop)
static_assert(std::is_trivially_copyable_v<DxlStatus4>);
static_assert(sizeof(DxlStatus4) == 15);

struct DynamixelTestCodec {
  using RecvPacket = DxlStatus4;
  using SendPacket = DxlStatus4;  // shape reuse; send path not exercised here.

  struct State {
    uint8_t id{0};
    uint8_t error{0};
    std::array<uint8_t, 4> params{};
    bool valid{false};
  };

  [[nodiscard]] static bool Decode(std::span<const uint8_t> buf, State& out) noexcept {
    if (buf.size() < sizeof(DxlStatus4))
      return false;
    DxlStatus4 pkt{};
    std::memcpy(&pkt, buf.data(), sizeof(pkt));
    // Validate CRC over all bytes preceding the 2-byte CRC field.
    const uint16_t want = DynamixelCrc16(std::span<const uint8_t>(buf.data(), sizeof(pkt) - 2));
    if (want != pkt.crc)
      return false;
    out.id = pkt.id;
    out.error = pkt.error;
    std::memcpy(out.params.data(), pkt.params, sizeof(pkt.params));
    out.valid = true;
    return true;
  }
};

static_assert(rtc::PacketCodec<DynamixelTestCodec>);

// Builds a transport bound to the PTY slave with framer defaults suited to the
// PTY (no RS485 ioctl, no drain).
[[nodiscard]] std::unique_ptr<Rs485Transport> MakeTransport(const std::string& device,
                                                            int timeout_ms = 500) {
  Rs485TransportConfig cfg;
  cfg.device = device;
  cfg.recv_timeout_ms = timeout_ms;
  cfg.rs485_enabled = false;
  cfg.drain_after_send = false;
  return std::make_unique<Rs485Transport>(cfg);
}

// ── Framer: single frame round-trip ─────────────────────────────────────────
TEST(Rs485TransportTest, SingleFrameRoundTrip) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path);
  ASSERT_TRUE(transport->Open());

  const std::array<uint8_t, 4> params{0x11, 0x22, 0x33, 0x44};
  const auto frame = BuildStatusFrame(0x01, 0x00, params);
  ASSERT_EQ(::write(pty.master_fd, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

  std::array<uint8_t, 64> buf{};
  const ssize_t n = transport->Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(frame.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), frame.data(), frame.size()));

  ::close(pty.master_fd);
}

// ── Framer: garbage before the header is discarded ──────────────────────────
TEST(Rs485TransportTest, GarbageBeforeHeaderResyncs) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path);
  ASSERT_TRUE(transport->Open());

  const std::array<uint8_t, 5> junk{0x00, 0xAB, 0xFF, 0xFD, 0x99};  // includes near-sync noise
  ASSERT_EQ(::write(pty.master_fd, junk.data(), junk.size()), static_cast<ssize_t>(junk.size()));

  const std::array<uint8_t, 2> params{0x55, 0x66};
  const auto frame = BuildStatusFrame(0x02, 0x00, params);
  ASSERT_EQ(::write(pty.master_fd, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

  std::array<uint8_t, 64> buf{};
  const ssize_t n = transport->Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(frame.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), frame.data(), frame.size()));

  ::close(pty.master_fd);
}

// ── Framer: back-to-back frames returned one per Recv() ─────────────────────
TEST(Rs485TransportTest, BackToBackFramesSplitPerRecv) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path);
  ASSERT_TRUE(transport->Open());

  const std::array<uint8_t, 1> p1{0xA0};
  const std::array<uint8_t, 3> p2{0xB0, 0xB1, 0xB2};
  const auto f1 = BuildStatusFrame(0x03, 0x00, p1);
  const auto f2 = BuildStatusFrame(0x04, 0x01, p2);

  std::vector<uint8_t> both;
  both.insert(both.end(), f1.begin(), f1.end());
  both.insert(both.end(), f2.begin(), f2.end());
  ASSERT_EQ(::write(pty.master_fd, both.data(), both.size()), static_cast<ssize_t>(both.size()));

  std::array<uint8_t, 64> buf{};
  ssize_t n1 = transport->Recv(buf);
  ASSERT_EQ(n1, static_cast<ssize_t>(f1.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), f1.data(), f1.size()));

  ssize_t n2 = transport->Recv(buf);
  ASSERT_EQ(n2, static_cast<ssize_t>(f2.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), f2.data(), f2.size()));

  ::close(pty.master_fd);
}

// ── Framer: a frame split across two writes is reassembled ──────────────────
TEST(Rs485TransportTest, FrameSplitAcrossReadsReassembles) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path, /*timeout_ms=*/1000);
  ASSERT_TRUE(transport->Open());

  const std::array<uint8_t, 4> params{0xC0, 0xC1, 0xC2, 0xC3};
  const auto frame = BuildStatusFrame(0x05, 0x00, params);
  const std::size_t split = frame.size() / 2;

  ASSERT_EQ(::write(pty.master_fd, frame.data(), split), static_cast<ssize_t>(split));

  // Deliver the tail shortly after, while Recv() is blocked awaiting more bytes.
  std::thread writer([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const ssize_t rest = static_cast<ssize_t>(frame.size() - split);
    (void)!::write(pty.master_fd, frame.data() + split, static_cast<std::size_t>(rest));
  });

  std::array<uint8_t, 64> buf{};
  const ssize_t n = transport->Recv(buf);
  writer.join();

  ASSERT_EQ(n, static_cast<ssize_t>(frame.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), frame.data(), frame.size()));

  ::close(pty.master_fd);
}

// ── Framer: Recv() times out to -1 with no data ─────────────────────────────
TEST(Rs485TransportTest, RecvTimesOutToMinusOne) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path, /*timeout_ms=*/100);
  ASSERT_TRUE(transport->Open());

  std::array<uint8_t, 64> buf{};
  const auto t0 = std::chrono::steady_clock::now();
  const ssize_t n = transport->Recv(buf);
  const auto t1 = std::chrono::steady_clock::now();

  EXPECT_EQ(n, -1);
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  EXPECT_LT(elapsed_ms, 2000);

  ::close(pty.master_fd);
}

// ── Framer: deadline bounds Recv() under a continuous garbage stream ────────
// A trickle of non-frame bytes arriving faster than the per-read VTIME window
// must not pin Recv(): the overall deadline (recv_timeout_ms + one VTIME
// window slack) has to trip even though every individual read succeeds.
TEST(Rs485TransportTest, RecvDeadlineBoundsGarbageStream) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path, /*timeout_ms=*/100);
  ASSERT_TRUE(transport->Open());

  std::atomic<bool> stop{false};
  std::thread pump([&] {
    const std::array<uint8_t, 8> junk{};  // 0x00s: never matches the sync
    while (!stop.load()) {
      (void)!::write(pty.master_fd, junk.data(), junk.size());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  std::array<uint8_t, 64> buf{};
  const auto t0 = std::chrono::steady_clock::now();
  const ssize_t n = transport->Recv(buf);
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
          .count();

  stop.store(true);
  pump.join();

  EXPECT_EQ(n, -1);
  // Deadline 100 ms + at most one extra VTIME window (100 ms) + CI slack.
  EXPECT_LT(elapsed_ms, 700);

  ::close(pty.master_fd);
}

// ── Framer: oversize frame yields -1, stream recovers on the next frame ─────
TEST(Rs485TransportTest, OversizeFrameReturnsErrorAndRecovers) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path);
  ASSERT_TRUE(transport->Open());

  const std::vector<uint8_t> big_params(40, 0xEE);
  const auto big = BuildStatusFrame(0x08, 0x00, big_params);
  const std::array<uint8_t, 2> small_params{0x10, 0x20};
  const auto small = BuildStatusFrame(0x09, 0x00, small_params);

  std::vector<uint8_t> both;
  both.insert(both.end(), big.begin(), big.end());
  both.insert(both.end(), small.begin(), small.end());
  ASSERT_EQ(::write(pty.master_fd, both.data(), both.size()), static_cast<ssize_t>(both.size()));

  std::array<uint8_t, 16> buf{};  // smaller than big (51 B), larger than small (13 B)
  ASSERT_LT(buf.size(), big.size());
  ASSERT_GE(buf.size(), small.size());

  EXPECT_EQ(transport->Recv(buf), -1);  // oversize frame dropped, not truncated

  const ssize_t n = transport->Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(small.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), small.data(), small.size()));

  ::close(pty.master_fd);
}

// ── Timeout <= 0 blocks (socket SO_RCVTIMEO=0 semantics) instead of polling ─
TEST(Rs485TransportTest, ZeroTimeoutBlocksUntilFrame) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  auto transport = MakeTransport(pty.slave_path, /*timeout_ms=*/0);
  ASSERT_TRUE(transport->Open());

  const std::array<uint8_t, 2> params{0x77, 0x88};
  const auto frame = BuildStatusFrame(0x0A, 0x00, params);
  std::thread writer([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    (void)!::write(pty.master_fd, frame.data(), frame.size());
  });

  std::array<uint8_t, 64> buf{};
  const auto t0 = std::chrono::steady_clock::now();
  const ssize_t n = transport->Recv(buf);
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
          .count();
  writer.join();

  // The old VMIN=0/VTIME=0 mapping returned -1 immediately (busy poll); the
  // blocking mapping waits for the late frame instead.
  ASSERT_EQ(n, static_cast<ssize_t>(frame.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), frame.data(), frame.size()));
  EXPECT_GE(elapsed_ms, 100);

  ::close(pty.master_fd);
}

// ── Fixed-length framing: empty sync, length_size 0 ─────────────────────────
TEST(Rs485TransportTest, FixedLengthFramingReturnsWholeFrames) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  constexpr std::size_t kFrameSize = 8;
  Rs485TransportConfig cfg;
  cfg.device = pty.slave_path;
  cfg.recv_timeout_ms = 500;
  cfg.rs485_enabled = false;
  cfg.drain_after_send = false;
  cfg.sync_pattern.clear();
  cfg.length_offset = 0;
  cfg.length_size = 0;
  cfg.length_adjust = kFrameSize;
  Rs485Transport transport(cfg);
  ASSERT_TRUE(transport.Open());

  std::array<uint8_t, 2 * kFrameSize> two{};
  for (std::size_t i = 0; i < two.size(); ++i)
    two[i] = static_cast<uint8_t>(i);
  ASSERT_EQ(::write(pty.master_fd, two.data(), two.size()), static_cast<ssize_t>(two.size()));

  std::array<uint8_t, 64> buf{};
  ssize_t n1 = transport.Recv(buf);
  ASSERT_EQ(n1, static_cast<ssize_t>(kFrameSize));
  EXPECT_EQ(0, std::memcmp(buf.data(), two.data(), kFrameSize));

  ssize_t n2 = transport.Recv(buf);
  ASSERT_EQ(n2, static_cast<ssize_t>(kFrameSize));
  EXPECT_EQ(0, std::memcmp(buf.data(), two.data() + kFrameSize, kFrameSize));

  ::close(pty.master_fd);
}

// ── discard_echo: Send() consumes the echoed frame, Recv() sees the reply ───
// Simulates a 2-wire bus whose receiver stays enabled during transmit: the
// "echo" (a byte-exact copy of the instruction frame) is pre-loaded into the
// rx path before Send(), followed by the device response. With discard_echo,
// Send() must consume exactly the echoed bytes so Recv() returns the response,
// not the echo.
TEST(Rs485TransportTest, DiscardEchoSkipsSentFrame) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  Rs485TransportConfig cfg;
  cfg.device = pty.slave_path;
  cfg.recv_timeout_ms = 500;
  cfg.rs485_enabled = false;
  cfg.drain_after_send = false;
  cfg.discard_echo = true;
  Rs485Transport transport(cfg);
  ASSERT_TRUE(transport.Open());

  const std::array<uint8_t, 2> req_params{0x01, 0x02};
  const auto request = BuildStatusFrame(0x0B, 0x00, req_params);
  const std::array<uint8_t, 2> rsp_params{0x03, 0x04};
  const auto response = BuildStatusFrame(0x0C, 0x00, rsp_params);

  // Echo arrives on the rx path (master -> slave) as the frame is transmitted,
  // then the device response follows.
  ASSERT_EQ(::write(pty.master_fd, request.data(), request.size()),
            static_cast<ssize_t>(request.size()));
  ASSERT_EQ(::write(pty.master_fd, response.data(), response.size()),
            static_cast<ssize_t>(response.size()));

  ASSERT_EQ(transport.Send(request), static_cast<ssize_t>(request.size()));

  std::array<uint8_t, 64> buf{};
  const ssize_t n = transport.Recv(buf);
  ASSERT_EQ(n, static_cast<ssize_t>(response.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), response.data(), response.size()));

  ::close(pty.master_fd);
}

// ── Full stack: Transceiver<DynamixelTestCodec> over Rs485Transport ─────────
TEST(Rs485TransportTest, TransceiverDecodesFramedStatus) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  rtc::Transceiver<DynamixelTestCodec> xcvr(MakeTransport(pty.slave_path));
  ASSERT_TRUE(xcvr.StartRecv());

  const std::array<uint8_t, 4> params{0xDE, 0xAD, 0xBE, 0xEF};
  const auto frame = BuildStatusFrame(0x07, 0x00, params);
  ASSERT_EQ(::write(pty.master_fd, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  while (xcvr.recv_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_GT(xcvr.recv_count(), 0u);

  const auto state = xcvr.GetLatestState();
  EXPECT_TRUE(state.valid);
  EXPECT_EQ(state.id, 0x07);
  EXPECT_EQ(0, std::memcmp(state.params.data(), params.data(), params.size()));

  xcvr.Stop();
  ::close(pty.master_fd);
}

}  // namespace
