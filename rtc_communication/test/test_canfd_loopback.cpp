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
//   - Frame API: SendFrame keeps the caller's ID / len / BRS through a 29-bit
//     mask filter, the mask admits an ID range and rejects the rest,
//     RecvFrameWithTimeout waits exactly its ppoll timeout (not SO_RCVTIMEO),
//     RecvFrameNonBlocking never waits, a classic can_frame sent through the
//     overload arrives as CAN_MTU beside the FD stream, oversized len is
//     refused with EINVAL, SetPriority applies, and a closed transport reports
//     errors without waiting (no vcan needed for that one)
//   - error_mask: an error message frame (CAN_ERR_FLAG) reaches only the
//     socket that subscribed to its class; a default socket never sees it
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_communication/can/can_transport.hpp"
#include "rtc_communication/can/canfd_transport.hpp"
#include "rtc_communication/testing/can_test_support.hpp"

#include <gtest/gtest.h>
#include <linux/can.h>
#include <linux/can/error.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <thread>

namespace {

using rtc::test::kVcanInterface;
using namespace std::chrono_literals;

constexpr timespec kHalfSecond{0, 500'000'000};
constexpr timespec kTwentyMs{0, 20'000'000};

// 29-bit receiver whose mask drops the low 6 bits, so one filter admits a
// whole ID range (an index field) the way a multi-ID protocol needs. The
// "quiet bus" assertions below hold only while nobody else transmits in this
// range on vcan0: the base is picked outside the ID space of the workspace's
// other vcan0 user (repo_scripts/scripts/setup_vcan.sh names it), whose
// 8-bit node fields never take the values 0x48 / 0xD1 encoded here.
constexpr canid_t kRangeBase = 0x00123440;
constexpr canid_t kRangeMask = 0x1FFFFFC0;

rtc::CanFdTransportConfig RangeReceiverConfig() {
  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  cfg.extended_frame = true;
  cfg.rx_can_id = kRangeBase;
  cfg.rx_can_mask = kRangeMask;
  cfg.recv_timeout_ms = 500;  // SO_RCVTIMEO — the frame API must not depend on it
  return cfg;
}

// A sender whose config tx_can_id / bitrate_switch stay at their defaults:
// the frame API ignores both, every test fills the frame itself.
rtc::CanFdTransportConfig FrameSenderConfig() {
  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  return cfg;
}

canfd_frame MakeFrame(canid_t id, __u8 len, __u8 flags) {
  canfd_frame f{};
  f.can_id = id | CAN_EFF_FLAG;
  f.len = len;
  f.flags = flags;
  std::iota(f.data, f.data + len, uint8_t{0x10});
  return f;
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

// ── Frame API: ID · len · BRS survive the trip through a 29-bit mask filter ──
TEST(CanFdTransportFrameApiTest, SendFrameRoundTripPreservesIdLenAndFlags) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransport receiver(RangeReceiverConfig());
  ASSERT_TRUE(receiver.Open());
  rtc::CanFdTransport sender(FrameSenderConfig());
  ASSERT_TRUE(sender.Open());

  const canfd_frame tx = MakeFrame(kRangeBase | 0x16, CANFD_MAX_DLEN, CANFD_BRS);
  ASSERT_TRUE(sender.SendFrame(tx));

  canfd_frame rx{};
  ASSERT_EQ(receiver.RecvFrameWithTimeout(rx, kHalfSecond), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id, tx.can_id);  // CAN_EFF_FLAG included
  EXPECT_EQ(rx.len, CANFD_MAX_DLEN);
  EXPECT_EQ(rx.flags & CANFD_BRS, CANFD_BRS);
  EXPECT_EQ(0, std::memcmp(rx.data, tx.data, CANFD_MAX_DLEN));
}

// ── Frame API: the mask admits every index of the range, nothing outside ────
TEST(CanFdTransportFrameApiTest, MaskFilterAdmitsIdRangeAndRejectsOthers) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransport receiver(RangeReceiverConfig());
  ASSERT_TRUE(receiver.Open());
  rtc::CanFdTransport sender(FrameSenderConfig());
  ASSERT_TRUE(sender.Open());

  // First an ID that differs above the masked low 6 bits, then the range's
  // highest index. If the filter let the first one through it would arrive
  // first, and the ID check below would see it.
  const canfd_frame outside = MakeFrame(kRangeBase + 0x40, 8, 0);
  const canfd_frame inside = MakeFrame(kRangeBase | 0x3F, 8, 0);
  ASSERT_TRUE(sender.SendFrame(outside));
  ASSERT_TRUE(sender.SendFrame(inside));

  canfd_frame rx{};
  ASSERT_EQ(receiver.RecvFrameWithTimeout(rx, kHalfSecond), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id, inside.can_id);
  EXPECT_EQ(rx.len, 8);
  EXPECT_EQ(receiver.RecvFrameWithTimeout(rx, kTwentyMs), 0);  // nothing else queued
}

// ── Frame API: RecvFrameWithTimeout waits its ppoll timeout, not SO_RCVTIMEO ─
TEST(CanFdTransportFrameApiTest, RecvFrameWithTimeoutExpiresOnQuietBus) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransport receiver(RangeReceiverConfig());  // SO_RCVTIMEO = 500 ms
  ASSERT_TRUE(receiver.Open());

  canfd_frame rx{};
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(receiver.RecvFrameWithTimeout(rx, kTwentyMs), 0);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(elapsed, 20ms);   // ppoll really waited
  EXPECT_LT(elapsed, 250ms);  // and returned long before the 500 ms socket timeout
}

// ── Frame API: RecvFrameNonBlocking never waits, and drains a queued frame ──
TEST(CanFdTransportFrameApiTest, RecvFrameNonBlockingNeverWaits) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransport receiver(RangeReceiverConfig());  // SO_RCVTIMEO = 500 ms
  ASSERT_TRUE(receiver.Open());
  rtc::CanFdTransport sender(FrameSenderConfig());
  ASSERT_TRUE(sender.Open());

  canfd_frame rx{};
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(receiver.RecvFrameNonBlocking(rx), 0);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 100ms);  // did not honour SO_RCVTIMEO

  const canfd_frame tx = MakeFrame(kRangeBase | 0x01, 16, CANFD_BRS);
  ASSERT_TRUE(sender.SendFrame(tx));

  // vcan delivers through the softirq path; poll (sleep-only) until it lands.
  ssize_t n = 0;
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while ((n = receiver.RecvFrameNonBlocking(rx)) == 0 &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(1ms);
  ASSERT_EQ(n, static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id, tx.can_id);
  EXPECT_EQ(rx.len, 16);
  EXPECT_EQ(receiver.RecvFrameNonBlocking(rx), 0);  // queue empty again
}

// ── Frame API: classic overload sends CAN_MTU on the same FD socket ─────────
TEST(CanFdTransportFrameApiTest, ClassicOverloadArrivesAsClassicFrame) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransport receiver(RangeReceiverConfig());
  ASSERT_TRUE(receiver.Open());
  rtc::CanFdTransport sender(FrameSenderConfig());
  ASSERT_TRUE(sender.Open());

  can_frame classic{};
  classic.can_id = (kRangeBase | 0x02) | CAN_EFF_FLAG;
  classic.len = 8;
  std::iota(classic.data, classic.data + 8, uint8_t{0xC0});
  ASSERT_TRUE(sender.SendFrame(classic));

  // An FD frame with the same len is a different thing on the wire (FDF bit):
  // the receiver must tell the two apart by size.
  const canfd_frame fd = MakeFrame(kRangeBase | 0x03, 8, 0);
  ASSERT_TRUE(sender.SendFrame(fd));

  canfd_frame rx{};
  ASSERT_EQ(receiver.RecvFrameWithTimeout(rx, kHalfSecond), static_cast<ssize_t>(CAN_MTU));
  EXPECT_EQ(rx.can_id, classic.can_id);
  EXPECT_EQ(rx.len, 8);
  EXPECT_EQ(0, std::memcmp(rx.data, classic.data, 8));

  ASSERT_EQ(receiver.RecvFrameWithTimeout(rx, kHalfSecond), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id, fd.can_id);
}

// ── Frame API: oversized len is refused by the kernel; SetPriority applies ──
TEST(CanFdTransportFrameApiTest, SendFrameRejectsOversizedLenAndSetPriorityApplies) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransport transport(FrameSenderConfig());
  ASSERT_TRUE(transport.Open());

  canfd_frame tx = MakeFrame(kRangeBase, CANFD_MAX_DLEN, 0);
  tx.len = CANFD_MAX_DLEN + 1;
  errno = 0;
  EXPECT_FALSE(transport.SendFrame(tx));
  EXPECT_EQ(errno, EINVAL);

  tx.len = CANFD_MAX_DLEN;
  EXPECT_TRUE(transport.SendFrame(tx));  // boundary passes

  EXPECT_TRUE(transport.SetPriority(6));  // highest value without CAP_NET_ADMIN
}

// ── error_mask: error message frames reach only a subscribed socket ─────────
// The kernel routes CAN_ERR_FLAG frames through a receiver list of their own
// that CAN_RAW_FILTER never reaches, so without error_mask a bus-off is
// invisible; with it the frame arrives as-is on the frame API (CAN_MTU, the
// error class in can_id) beside the regular stream.
TEST(CanFdTransportFrameApiTest, ErrorMaskDeliversErrorFramesOnlyWhenSubscribed) {
  SKIP_IF_NO_FD_VCAN();

  rtc::CanFdTransportConfig subscribed = FrameSenderConfig();
  subscribed.error_mask = CAN_ERR_BUSOFF | CAN_ERR_CRTL;
  rtc::CanFdTransport rx_subscribed(subscribed);
  ASSERT_TRUE(rx_subscribed.Open());

  rtc::CanFdTransport rx_default(FrameSenderConfig());  // error_mask 0, no filter
  ASSERT_TRUE(rx_default.Open());

  rtc::CanFdTransport tx(FrameSenderConfig());
  ASSERT_TRUE(tx.Open());

  // Error message frames are classic frames: class bits in can_id, 8 data bytes.
  can_frame err{};
  err.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;
  err.len = CAN_ERR_DLC;
  ASSERT_TRUE(tx.SendFrame(err));

  canfd_frame rx{};
  ASSERT_EQ(rx_subscribed.RecvFrameWithTimeout(rx, kHalfSecond), static_cast<ssize_t>(CAN_MTU));
  EXPECT_TRUE(rx.can_id & CAN_ERR_FLAG);
  EXPECT_TRUE(rx.can_id & CAN_ERR_BUSOFF);

  EXPECT_EQ(rx_default.RecvFrameWithTimeout(rx, kTwentyMs), 0);  // never delivered

  // A class outside the mask is not delivered. Error frames have no id
  // separation on the shared vcan0, so a subscribed-class frame injected by
  // another test at this moment is tolerated: only a TX_TIMEOUT frame fails.
  err.can_id = CAN_ERR_FLAG | CAN_ERR_TX_TIMEOUT;
  ASSERT_TRUE(tx.SendFrame(err));
  while (rx_subscribed.RecvFrameWithTimeout(rx, kTwentyMs) > 0)
    EXPECT_EQ(rx.can_id & CAN_ERR_TX_TIMEOUT, 0u);
}

// ── Frame API: a closed transport reports errors and never waits (no vcan) ──
TEST(CanFdTransportFrameApiTest, ClosedTransportReportsErrorsWithoutWaiting) {
  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = "nonexistent999";
  rtc::CanFdTransport transport(cfg);
  ASSERT_FALSE(transport.Open());

  const canfd_frame tx = MakeFrame(kRangeBase, 8, 0);
  EXPECT_FALSE(transport.SendFrame(tx));

  canfd_frame rx{};
  const auto t0 = std::chrono::steady_clock::now();
  errno = 0;
  EXPECT_EQ(transport.RecvFrameWithTimeout(rx, kHalfSecond), -1);
  EXPECT_EQ(errno, EBADF);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 100ms);  // guard, not a 500 ms ppoll on fd -1

  errno = 0;
  EXPECT_EQ(transport.RecvFrameNonBlocking(rx), -1);
  EXPECT_EQ(errno, EBADF);
  EXPECT_FALSE(transport.SetPriority(0));
}

}  // namespace
