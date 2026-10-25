// ── test_canfd_hand_comm.cpp ────────────────────────────────────────────────
// CanFdHandComm (proto2 over CAN FD, #901) against the proto2 fake firmware
// behind a second CAN FD socket on vcan0, plus the Tier 1 cases that need no
// socket. The vcan cases skip (SKIP_IF_NO_FD_VCAN, rtc_communication's shared
// test header) when vcan0 is absent or not FD-capable; a skip is unverified.
//
// The cases use the real hand id space (right 0x01 / left 0x02 nodes), so they
// cannot share vcan0 with a proto2 SIL (fake_hand_firmware protocol_version 2
// + udp_hand_node): its echoes / RESP bursts would be read as stale frames
// here, FakePeer would answer the SIL node's REQs, and the error frames D41
// injects would reach the SIL node's bus_off count. Before each vcan case the
// fixture probes both nodes and listens, and fails the case when hand traffic
// answers or is already on the bus (#927 B).
//
// Covers (decision ids from epic #895):
//   D41 · D50  write + echo round trip reaches the fake, feedforward Nm → A
//   D47 · D49  REQ → RESP set fills the joint triplet and the raw 192 B set
//              that SensorProtocol2 decodes; per-kind stats 1:1 with proto1
//   D42        one deadline per stage: echo timeout, set timeout
//   D43 · D44  a partial set is abandoned, no retry in the tick, next cycle ok
//   D57        a RESP burst in reverse order (2 1 0) still completes the set
//   D41 · D58  echo stage classification: unexpected frames only count — one
//              or three before the echoes, the write still succeeds; with no
//              echo it ends at the deadline; an echo with another payload
//              only counts, an error frame costs nothing (error frames are
//              shared bus-wide, so that case asserts >= 1)
//   D59 · D60  RunCycle = CMD pair + REQ back to back, one receive window:
//              echoes and RESP interleaved (RESP first), no echo → echo
//              timeout but the set completes within ~T, no RESP F2 → write ok
//              but the set times out, the deadline chain (late echoes, later
//              RESP) completes the set, class-④ frames only count, a stale
//              RESP before the echoes is replaced by the real burst (D57),
//              payload mismatch / bus-off / left-right split / recovery on the
//              window path, write without a REQ-side change for the E-Stop
//   D26        E-Stop fire-and-forget: Frame 1 zeros then Frame 0 zeros
//   D51        a left-hand comm never talks to a right-hand fake
//   D63 · D64  cmd_ack none (#945): the write is the send (write_ok, not
//   · D68      write_rx), no echo stage, the set keeps CMD + 2T after a write
//              and REQ + T without one; echoes from an echoing firmware only
//              count (echo_unexpected) — and the CmdAckMismatchDecision table
//   DrainStale bounded drain, stale_drained accounting
//   #956       the shared-request split (D81 · D83): one broadcast REQ
//              (sniffed id 0x007FC000) completes both hands' sets; the bound
//              is anchored at the REQ, not at ReceiveSet's call; queued
//              frames are taken after the bound has passed; 2T after a
//              write; a failed REQ is a set error, not a timeout; the write
//              result survives ReceiveSet
//   Tier 1     tx errno classification, closed link harmless, kRxMask bits,
//              the vcan0 SIL detector's id set (#927 B), SupportsBroadcastReq,
//              SensorProtocol2 decode (signs, metres, length guard, clamp)
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_communication/can/canfd_transport.hpp"
#include "rtc_communication/testing/can_test_support.hpp"
#include "udp_hand_driver/canfd_hand_comm.hpp"
#include "udp_hand_driver/fake_hand_firmware.hpp"
#include "udp_hand_driver/hand_canfd_packets.hpp"
#include "udp_hand_driver/protocol/sensor_protocol.hpp"
#include "udp_hand_driver/protocol/sensor_protocol_canfd.hpp"

#include <gtest/gtest.h>
#include <linux/can.h>
#include <linux/can/error.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace udp_hand_driver::test {

using namespace canfd;

namespace {

using namespace std::chrono_literals;
using rtc::test::kVcanInterface;
using Frames = std::array<FakeHandFirmware::CanFdFrame, kNumRespFrames>;

constexpr double kStageTimeoutMs = 50.0;  // generous for a vcan peer thread
constexpr int kBaseMin = 1000;
constexpr int kBaseMax = 2000;
constexpr double kNoiseStddev = 5.0;
constexpr int kBand = static_cast<int>(6 * kNoiseStddev);  // see test_fake_hand_firmware_canfd
constexpr canid_t kTargetFieldMask = 0xFFu << kTargetShift;

CanFdHandCommConfig CommConfig(HandSide side = HandSide::kRight) {
  CanFdHandCommConfig cfg;
  cfg.interface_name = kVcanInterface;
  cfg.hand_side = side;
  cfg.recv_timeout_ms = kStageTimeoutMs;
  return cfg;
}

FakeHandFirmware::Config FakeConfig(HandSide side, CmdAck cmd_ack = CmdAck::kEcho) {
  FakeHandFirmware::Config cfg;
  cfg.num_fingertips = kDefaultNumFingertips;
  cfg.effort_stiffness = 2.0;
  cfg.sensor_base_min = kBaseMin;
  cfg.sensor_base_max = kBaseMax;
  cfg.sensor_noise_stddev = kNoiseStddev;
  cfg.seed = 7;
  cfg.hand_side = side;
  cfg.cmd_ack = cmd_ack;
  return cfg;
}

// A plain FD socket on vcan0 that receives only frames addressed to `target`
// (the hand's view of the bus) — used by the peer and by the E-Stop sniffer.
rtc::CanFdTransportConfig TargetedConfig(uint8_t target) {
  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  cfg.extended_frame = true;
  cfg.bitrate_switch = true;
  cfg.rx_can_id = static_cast<canid_t>(target) << kTargetShift;
  cfg.rx_can_mask = kTargetFieldMask;
  return cfg;
}

// The peer's view: frames to its node and to the broadcast node 0xFF (#956).
// One kernel filter with id = mask = the node's target bits passes a target
// that has every bit of the node set — the node itself and 0xFF; the host
// (0x00) and the other hand (0x01 vs 0x02) do not pass.
rtc::CanFdTransportConfig PeerConfig(uint8_t node) {
  rtc::CanFdTransportConfig cfg = TargetedConfig(node);
  cfg.rx_can_mask = static_cast<canid_t>(node) << kTargetShift;
  return cfg;
}

// Sender with no filter (its reads are never used).
rtc::CanFdTransportConfig InjectorConfig() {
  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  cfg.extended_frame = true;
  return cfg;
}

// ── vcan0 shared with a running SIL (#927 B) ──────────────────────────────
// with_verify_hold.sh holds only the Stop hook, not a `colcon test` typed by
// hand, so the clash above is detected here instead: a REQ probe to each node
// draws a RESP set from a listening fake_hand_firmware whatever the SIL node
// is doing, and a transmitting SIL node shows its own traffic. A node with no
// firmware that is not transmitting (Inactive, E-Stopped) is not seen — and
// does not touch these cases either.

// Kernel pre-filter: the bits that are zero in every hand-protocol id of nodes
// 1 and 2 — priority, the top two msg-type bits (types 0..2), and target and
// source above bit 1. The transport adds EFF to it, so 11-bit frames never
// reach user space. IsHandTraffic below is the exact test; this only narrows.
constexpr canid_t kHandTrafficFilterMask = (0x7u << kPriorityShift) | (0xCu << kMsgTypeShift) |
                                           (0xFCu << kTargetShift) | (0xFCu << kSourceShift);
static_assert(kHandTrafficFilterMask == 0x1F3F3F00u, "hand-traffic pre-filter mask");
// Lets a frame the previous case sent but the kernel delivers late (a deferred
// softirq) arrive and be discarded before the probe.
constexpr auto kSilSettle = 5ms;
constexpr auto kSilListenWindow = 20ms;

// A data frame of either hand's protocol: CMD, REQ, echo or RESP (any frame
// index) to or from node 1 or 2.
[[nodiscard]] bool IsHandTraffic(canid_t can_id) noexcept {
  if ((can_id & CAN_EFF_FLAG) == 0 || (can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) != 0) {
    return false;
  }
  const uint32_t base = (can_id & kExtIdMask) & ~kFrameIndexMask;
  for (const uint8_t node : {kNodeRightHand, kNodeLeftHand}) {
    if (base == CmdId(node, 0) || base == ReqId(node) || base == EchoId(node, 0) ||
        base == RespId(node, 0)) {
      return true;
    }
  }
  return false;
}

// The id of the first hand frame seen on vcan0 within `window`, 0 when none
// arrived, -1 when the listener failed (EINTR is not a failure).
[[nodiscard]] int64_t FirstHandFrame(rtc::CanFdTransport& listener,
                                     std::chrono::nanoseconds window) {
  const auto deadline = std::chrono::steady_clock::now() + window;
  for (auto now = std::chrono::steady_clock::now(); now < deadline;
       now = std::chrono::steady_clock::now()) {
    const auto left = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
    const timespec wait{0, static_cast<long>(left.count())};
    canfd_frame rx{};
    const ssize_t n = listener.RecvFrameWithTimeout(rx, wait);
    if (n < 0 && errno != EINTR) {
      return -1;
    }
    if (n > 0 && IsHandTraffic(rx.can_id)) {
      return static_cast<int64_t>(rx.can_id & kExtIdMask);
    }
  }
  return 0;
}

// Settles, probes both nodes with a REQ, then listens. Returns the id of the
// first hand frame seen, 0 when the bus stayed quiet, or -1 when the listener
// itself failed (an infrastructure error, reported apart from a SIL). The
// socket does not receive its own REQs.
[[nodiscard]] int64_t FirstHandTrafficOnVcan() {
  rtc::CanFdTransportConfig cfg = InjectorConfig();
  cfg.rx_can_id = 0;
  cfg.rx_can_mask = kHandTrafficFilterMask;
  rtc::CanFdTransport listener(cfg);
  if (!listener.Open() || FirstHandFrame(listener, kSilSettle) < 0) {
    return -1;
  }
  for (const uint8_t node : {kNodeRightHand, kNodeLeftHand}) {
    canfd_frame req{};
    req.can_id = ReqId(node) | CAN_EFF_FLAG;
    req.flags = CANFD_BRS;
    if (!listener.SendFrame(req)) {
      return -1;
    }
  }
  return FirstHandFrame(listener, kSilListenWindow);
}

HandMotorArray Ramp(float start, float step) {
  HandMotorArray m{};
  for (std::size_t i = 0; i < m.size(); ++i)
    m[i] = start + step * static_cast<float>(i);
  return m;
}

canfd_frame DataFrame(uint32_t bare_id, uint8_t fill) {
  canfd_frame f{};
  f.can_id = bare_id | CAN_EFF_FLAG;
  f.len = kFrameLen;
  f.flags = CANFD_BRS;
  std::memset(f.data, fill, kFrameLen);
  return f;
}

// FakeHandFirmware behind its own socket on a thread: the device side of the
// link. Modes beyond kNormal shape the answers:
//   kDropFrame2  withholds RESP idx 2 (a partial set every cycle)
//   kReverse     sends each answer's frames last-first (RESP 2 1 0)
//   kSilent      answers nothing
//   kRespFirst   holds the echoes until the REQ, then RESP 0 · echo 0 · echo 1 ·
//                RESP 1 · RESP 2 — the interleaving a pipelined window must take
//   kNoEcho      answers the REQ, never echoes
//   kLate        holds the echoes until 0.6 x `late_unit` after CMD 0 arrived
//                and the RESP burst until 1.3 x `late_unit` (the D59 chain:
//                within (echo close) + T, outside REQ + T)
//   kDelayedResp answers a REQ `late_unit` after it arrived (#956: a hand
//                whose set lands past one bound and inside another)
// Model reads go through WithModel() (mutex — test code, not RT).
class FakePeer {
 public:
  enum class Mode {
    kNormal,
    kDropFrame2,
    kReverse,
    kSilent,
    kRespFirst,
    kNoEcho,
    kLate,
    kDelayedResp
  };

  // `cmd_ack` selects which firmware the real fake mirrors (#945): under kNone
  // it never yields an echo, whatever `mode` does with the frames it yields.
  explicit FakePeer(HandSide side, Mode mode = Mode::kNormal,
                    std::chrono::milliseconds late_unit = std::chrono::milliseconds(0),
                    CmdAck cmd_ack = CmdAck::kEcho)
      : firmware_(FakeConfig(side, cmd_ack)),
        mode_(mode),
        late_unit_(late_unit),
        transport_(PeerConfig(NodeIdOf(side))) {
    open_ = transport_.Open();
    if (open_)
      thread_ = std::thread([this] { Run(); });
  }

  ~FakePeer() {
    stop_.store(true);
    if (thread_.joinable())
      thread_.join();
  }

  [[nodiscard]] bool open() const noexcept { return open_; }

  template <typename Fn>
  auto WithModel(Fn&& fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    return fn(firmware_);
  }

 private:
  using Clock = std::chrono::steady_clock;

  void Send(const FakeHandFirmware::CanFdFrame& f) {
    canfd_frame tx{};
    tx.can_id = f.can_id | CAN_EFF_FLAG;
    tx.len = static_cast<__u8>(f.len);
    tx.flags = CANFD_BRS;
    std::memcpy(tx.data, f.data.data(), f.len);
    (void)transport_.SendFrame(tx);
  }

  void Run() {
    canfd_frame rx{};
    Frames out{};
    std::vector<FakeHandFirmware::CanFdFrame> held;  // echoes held back (kRespFirst · kLate)
    Clock::time_point t_cmd0{};
    const timespec poll{0, 5'000'000};  // 5 ms, re-checks stop_
    while (!stop_.load()) {
      const ssize_t n = transport_.RecvFrameWithTimeout(rx, poll);
      if (n <= 0)
        continue;
      const Clock::time_point t_rx = Clock::now();
      const bool is_req = MsgTypeOf(rx.can_id) == static_cast<uint8_t>(MsgType::kReq);
      if (MsgTypeOf(rx.can_id) == static_cast<uint8_t>(MsgType::kCmd) &&
          FrameIndexOf(rx.can_id) == 0)
        t_cmd0 = Clock::now();
      std::size_t count = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        count = firmware_.BuildCanFdResponse(rx.can_id, rx.data, rx.len, out.data(), out.size());
      }
      switch (mode_) {
        case Mode::kSilent:
          continue;
        case Mode::kNoEcho:
          if (!is_req)
            continue;
          break;
        case Mode::kRespFirst:
          if (!is_req) {
            held.insert(held.end(), out.begin(), out.begin() + static_cast<std::ptrdiff_t>(count));
            continue;
          }
          Send(out[0]);
          for (const auto& f : held)
            Send(f);
          held.clear();
          for (std::size_t k = 1; k < count; ++k)
            Send(out[k]);
          continue;
        case Mode::kLate:
          if (!is_req) {
            held.insert(held.end(), out.begin(), out.begin() + static_cast<std::ptrdiff_t>(count));
            continue;
          }
          std::this_thread::sleep_until(t_cmd0 + late_unit_ * 6 / 10);
          for (const auto& f : held)
            Send(f);
          held.clear();
          std::this_thread::sleep_until(t_cmd0 + late_unit_ * 13 / 10);
          break;
        case Mode::kDelayedResp:
          if (is_req)
            std::this_thread::sleep_until(t_rx + late_unit_);
          break;
        case Mode::kNormal:
        case Mode::kDropFrame2:
        case Mode::kReverse:
          break;
      }
      for (std::size_t k = 0; k < count; ++k) {
        const std::size_t i = (mode_ == Mode::kReverse) ? count - 1 - k : k;
        if (mode_ == Mode::kDropFrame2 &&
            MsgTypeOf(out[i].can_id) == static_cast<uint8_t>(MsgType::kResp) &&
            FrameIndexOf(out[i].can_id) == 2)
          continue;
        Send(out[i]);
      }
    }
  }

  FakeHandFirmware firmware_;
  Mode mode_;
  std::chrono::milliseconds late_unit_;
  rtc::CanFdTransport transport_;
  bool open_{false};
  std::mutex mutex_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

// Opens a comm, fails the test if vcan refuses.
class CanFdHandCommVcan : public ::testing::Test {
 protected:
  void SetUp() override {
    SKIP_IF_NO_FD_VCAN();
    // FAIL, not skip: a skip reads as "no vcan here" and would hide the clash.
    const int64_t seen = FirstHandTrafficOnVcan();
    if (seen < 0) {
      FAIL() << "could not listen on " << kVcanInterface << " before the case";
    }
    if (seen > 0) {
      FAIL() << "proto2 hand traffic on " << kVcanInterface << " (id 0x" << std::hex << seen
             << ") — a firmware SIL is running there. Stop it before running "
                "test_canfd_hand_comm (docs/testing.md).";
    }
  }

  std::unique_ptr<CanFdHandComm> OpenComm(CanFdHandCommConfig cfg = CommConfig()) {
    auto comm = std::make_unique<CanFdHandComm>(std::move(cfg));
    EXPECT_TRUE(comm->Open());
    return comm;
  }

  std::unique_ptr<FakePeer> StartPeer(HandSide side = HandSide::kRight,
                                      FakePeer::Mode mode = FakePeer::Mode::kNormal,
                                      std::chrono::milliseconds late_unit = 0ms,
                                      CmdAck cmd_ack = CmdAck::kEcho) {
    auto peer = std::make_unique<FakePeer>(side, mode, late_unit, cmd_ack);
    EXPECT_TRUE(peer->open());
    return peer;
  }

  // One cycle transaction with a command (the write tick of the controller).
  HandReadOutcome Cycle(CanFdHandComm& comm, const HandMotorArray& pos, const HandMotorArray& ff,
                        HandReadBuffers& bufs, bool sensors = true) {
    HandReadOutcome out{};
    comm.RunCycle(SensorPlan(sensors), &pos, &ff, bufs, out);
    return out;
  }

  static const UdpHandCommStats::PerKind& Kind(const UdpHandCommStats& s, RequestKind k) {
    return s.per_kind[static_cast<std::size_t>(k)];
  }

  HandReadPlan SensorPlan(bool sensors) {
    HandReadPlan plan{};
    plan.motor_space_read = false;
    plan.sensors = sensors;
    plan.sensor_response_size = sizeof(RespSet);
    return plan;
  }
};

}  // namespace

// ═══════════════════════════ Tier 1 (no socket) ═════════════════════════════

TEST(CanFdHandCommTier1, ClassifyTxErrnoMapsTheFourClasses) {
  using Tx = CanFdHandComm::TxFailure;
  EXPECT_EQ(CanFdHandComm::ClassifyTxErrno(EAGAIN), Tx::kEagain);
  EXPECT_EQ(CanFdHandComm::ClassifyTxErrno(ENOBUFS), Tx::kEnobufs);
  EXPECT_EQ(CanFdHandComm::ClassifyTxErrno(ENETDOWN), Tx::kEnetdown);
  EXPECT_EQ(CanFdHandComm::ClassifyTxErrno(EINVAL), Tx::kOther);
  EXPECT_EQ(CanFdHandComm::ClassifyTxErrno(EBADF), Tx::kOther);
}

TEST(CanFdHandCommTier1, RxMaskSelectsPriorityTargetAndSourceOnly) {
  constexpr uint32_t kPriority = 0x7u << kPriorityShift;
  constexpr uint32_t kTarget = 0xFFu << kTargetShift;
  constexpr uint32_t kSource = 0xFFu << kSourceShift;
  EXPECT_EQ(CanFdHandComm::kRxMask, kPriority | kTarget | kSource);
  // The echo base and every RESP of the same node agree under the mask; the
  // other hand, our own CMD and REQ do not.
  const uint32_t base = EchoId(kNodeRightHand, 0) & CanFdHandComm::kRxMask;
  for (uint8_t idx = 0; idx < kNumRespFrames; ++idx)
    EXPECT_EQ(RespId(kNodeRightHand, idx) & CanFdHandComm::kRxMask, base);
  EXPECT_EQ(EchoId(kNodeRightHand, 1) & CanFdHandComm::kRxMask, base);
  EXPECT_NE(RespId(kNodeLeftHand, 0) & CanFdHandComm::kRxMask, base);
  EXPECT_NE(CmdId(kNodeRightHand, 0) & CanFdHandComm::kRxMask, base);
  EXPECT_NE(ReqId(kNodeRightHand) & CanFdHandComm::kRxMask, base);
}

TEST(CanFdHandCommTier1, ClosedLinkRtMethodsAreHarmlessAndCountNothing) {
  CanFdHandCommConfig cfg = CommConfig();
  cfg.interface_name = "nonexistent999";
  CanFdHandComm comm(cfg);
  EXPECT_FALSE(comm.Open());
  EXPECT_FALSE(comm.transport().is_open());

  EXPECT_EQ(comm.DrainStale(), 0);
  const HandMotorArray zeros{};
  EXPECT_FALSE(comm.WriteCommandWithEcho(zeros, zeros));
  comm.WriteCommandFireAndForget(zeros, zeros);  // must not block or crash
  HandReadPlan plan{};
  plan.sensors = true;
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  comm.ReadCycle(plan, bufs, out);
  EXPECT_FALSE(out.motor_ok);
  EXPECT_FALSE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, -1);
  out.joint_ok = true;  // RunCycle must reset it too
  comm.RunCycle(plan, &zeros, &zeros, bufs, out);
  EXPECT_FALSE(out.write_ok);
  EXPECT_FALSE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, -1);
  EXPECT_NE(out.write_end, HandReadOutcome::Clock::time_point{});  // the stage "ran" and closed
  EXPECT_LE(out.PhaseEnd(ReadPhase::kMotor), out.PhaseEnd(ReadPhase::kJoint));

  comm.PublishCommStats();
  const UdpHandCommStats s = comm.comm_stats();
  EXPECT_EQ(s.recv_ok, 0u);
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(s.recv_error, 0u);
  EXPECT_EQ(comm.recv_error_count(), 0u);
  const CanFdCommStats c = comm.canfd_stats();
  EXPECT_EQ(c.tx_other, 0u);
  EXPECT_EQ(c.error_frames, 0u);

  // #956: the shared-request split on a closed link sends nothing, a failed
  // REQ is not counted as anything the hands did, and ReceiveSet keeps the
  // write result while it resets the read fields.
  HandReadOutcome::Clock::time_point t_req{};
  EXPECT_FALSE(comm.SendCommandOnly(zeros, zeros, out));
  EXPECT_FALSE(comm.SendBroadcastReq(t_req));
  EXPECT_NE(t_req, HandReadOutcome::Clock::time_point{});
  out.write_ok = true;
  out.joint_ok = true;
  comm.ReceiveSet(plan, bufs, out, SharedRequest{t_req, true, false});
  EXPECT_TRUE(out.write_ok);
  EXPECT_FALSE(out.joint_ok);
  EXPECT_EQ(out.PhaseEnd(ReadPhase::kMotor), t_req);

  EXPECT_FALSE(comm.SupportsIndividualMode());
  EXPECT_TRUE(comm.InitializeSensors());
  EXPECT_DOUBLE_EQ(comm.recv_timeout_ms(), kStageTimeoutMs);
  EXPECT_EQ(comm.node(), kNodeRightHand);
}

// D50 / NUM-2: the one predicate Open() and the node's configure check share.
// +inf is the case `kt > 0 && isfinite(1/kt)` alone would let through (1/inf =
// 0 → every feedforward silently 0 A).
TEST(CanFdHandCommTier1, IsValidFeedforwardKtRejectsNonFiniteZeroNegativeAndSubnormal) {
  EXPECT_TRUE(CanFdHandComm::IsValidFeedforwardKt(1.0f));
  EXPECT_TRUE(CanFdHandComm::IsValidFeedforwardKt(0.05f));
  EXPECT_TRUE(CanFdHandComm::IsValidFeedforwardKt(1e30f));
  EXPECT_FALSE(CanFdHandComm::IsValidFeedforwardKt(0.0f));
  EXPECT_FALSE(CanFdHandComm::IsValidFeedforwardKt(-1.0f));
  EXPECT_FALSE(CanFdHandComm::IsValidFeedforwardKt(std::nanf("")));
  EXPECT_FALSE(CanFdHandComm::IsValidFeedforwardKt(std::numeric_limits<float>::infinity()));
  EXPECT_FALSE(CanFdHandComm::IsValidFeedforwardKt(1e-40f));  // 1/kt overflows
}

// #927 B: the vcan0 SIL detector. Every hand-protocol id of both nodes is
// hand traffic AND passes the kernel pre-filter (a filter that dropped one
// would make the exact check unreachable for it); the ids rtc_communication's
// vcan tests use, 11-bit frames, a hand id sent without EFF, error and RTR
// frames are not.
TEST(CanFdHandCommTier1, VcanSilDetectorMatchesExactlyTheHandIds) {
  const uint32_t hand_ids[] = {
      CmdId(kNodeRightHand, 0),  CmdId(kNodeLeftHand, 1),   ReqId(kNodeRightHand),
      ReqId(kNodeLeftHand),      EchoId(kNodeRightHand, 0), EchoId(kNodeLeftHand, 1),
      RespId(kNodeRightHand, 0), RespId(kNodeLeftHand, 2),
  };
  for (const uint32_t id : hand_ids) {
    EXPECT_TRUE(IsHandTraffic(id | CAN_EFF_FLAG)) << std::hex << id;
    EXPECT_EQ(id & kHandTrafficFilterMask, 0u) << std::hex << id;
    EXPECT_FALSE(IsHandTraffic(id)) << "without EFF: " << std::hex << id;
    EXPECT_FALSE(IsHandTraffic(id | CAN_EFF_FLAG | CAN_RTR_FLAG)) << std::hex << id;
  }
  EXPECT_FALSE(IsHandTraffic(CAN_ERR_FLAG | CAN_ERR_BUSOFF));
  // rtc_communication's vcan ids (11-bit ones, then the 29-bit ones).
  for (const canid_t id : {canid_t{0x100}, canid_t{0x123}, canid_t{0x321}, canid_t{0x456}}) {
    EXPECT_FALSE(IsHandTraffic(id)) << std::hex << id;
  }
  for (const uint32_t id : {0x1ABCDEFu, 0x200u, 0x300u, 0x321u, 0x400u}) {
    EXPECT_FALSE(IsHandTraffic(id | CAN_EFF_FLAG)) << std::hex << id;
  }
  // test_canfd_loopback's range filter cases: kRangeBase (0x00123440) | 0..0x3F
  // and the first id past it.
  for (uint32_t id = 0x00123440u; id <= 0x00123480u; ++id) {
    EXPECT_FALSE(IsHandTraffic(id | CAN_EFF_FLAG)) << std::hex << id;
    EXPECT_NE(id & kHandTrafficFilterMask, 0u) << std::hex << id;
  }
  // The broadcast REQ (#956) is left out on purpose: a broadcast SIL is seen
  // through its RESP bursts, and a REQ to 0xFF from this probe would draw both
  // hands' answers from any firmware fake on the bus anyway.
  EXPECT_NE(ReqId(kNodeBroadcast) & kHandTrafficFilterMask, 0u);
  // Same node but another message type (type field 3) is not the protocol.
  EXPECT_FALSE(IsHandTraffic((ReqId(kNodeRightHand) | 0x00C00000u) | CAN_EFF_FLAG));
}

// #945 D63: the WARN predicate. echo → a silent firmware needs enough echo
// timeouts, no echoed write, no late echo and a live set stage; none → any
// unexpected echo.
TEST(CanFdHandCommTier1, CmdAckMismatchDecisionTable) {
  using PerKind = UdpHandCommStats::PerKind;
  const auto echo = [](const PerKind& w, uint64_t joint_ok, uint64_t unexpected) {
    return CmdAckMismatchDecision(CmdAck::kEcho, w, joint_ok, unexpected);
  };
  PerKind all_timeout{};
  all_timeout.timeout = kCmdAckMismatchMinWrites;
  EXPECT_EQ(echo(all_timeout, 1, 0), CmdAckMismatch::kFirmwareSilent);
  EXPECT_EQ(echo(all_timeout, 0, 0), CmdAckMismatch::kNone) << "dead link: the set fails too";
  PerKind too_few{};
  too_few.timeout = kCmdAckMismatchMinWrites - 1;
  EXPECT_EQ(echo(too_few, 1, 0), CmdAckMismatch::kNone);
  PerKind one_ok = all_timeout;
  one_ok.ok = 1;
  EXPECT_EQ(echo(one_ok, 1, 0), CmdAckMismatch::kNone) << "an echo arrived once";
  PerKind one_error = all_timeout;
  one_error.error = 1;
  EXPECT_EQ(echo(one_error, 1, 0), CmdAckMismatch::kFirmwareSilent)
      << "a send error does not hide a silent firmware";
  EXPECT_EQ(echo(all_timeout, 1, 1), CmdAckMismatch::kNone)
      << "late echoes: T is too short, the firmware does echo";
  EXPECT_EQ(echo(PerKind{}, 1, 5), CmdAckMismatch::kNone);

  EXPECT_EQ(CmdAckMismatchDecision(CmdAck::kNone, PerKind{}, 1, 1),
            CmdAckMismatch::kFirmwareEchoes);
  EXPECT_EQ(CmdAckMismatchDecision(CmdAck::kNone, PerKind{}, 1, 0), CmdAckMismatch::kNone);
  EXPECT_EQ(CmdAckMismatchDecision(CmdAck::kNone, all_timeout, 1, 0), CmdAckMismatch::kNone)
      << "none mode ignores the echo-stage record";
}

TEST(SensorProtocol2Test, CapabilitiesMatchTheWire) {
  SensorProtocol2 proto;
  EXPECT_EQ(proto.ResponseSize(), sizeof(RespSet));
  EXPECT_EQ(proto.ResponseSize(), 192u);
  EXPECT_FALSE(proto.HasMotorSpaceRead());
  EXPECT_FALSE(proto.RunsSensorPostProcess());
  EXPECT_FALSE(proto.VerifiesResponseMode());
  EXPECT_FALSE(proto.VerifiesBulkSensorResponseMode());
  EXPECT_EQ(proto.JointIoMode(), packets::JointMode::kMotor);
  EXPECT_STREQ(proto.Version(), "2");
}

// The node selects the protocol by key; "2" must resolve to SensorProtocol2 and
// the proto1 keys must still resolve (the factory is shared).
TEST(SensorProtocol2Test, FactoryBuildsItForKeyTwo) {
  const auto proto = CreateSensorProtocol("2");
  ASSERT_NE(proto, nullptr);
  EXPECT_NE(dynamic_cast<const SensorProtocol2*>(proto.get()), nullptr);
  EXPECT_STREQ(proto->Version(), "2");
  EXPECT_NE(CreateSensorProtocol("1a"), nullptr);
  EXPECT_NE(CreateSensorProtocol("1b"), nullptr);
  EXPECT_EQ(CreateSensorProtocol("3"), nullptr);
}

TEST(SensorProtocol2Test, DecodeAppliesSignsMetresClampAndLengthGuard) {
  RespSet set{};
  // Thumb split across F1 / F2; index whole in F2. Full-scale raws.
  set.frame1.thumb_fx = 32767;
  set.frame1.thumb_fy = -32767;
  set.frame1.thumb_fz = -32767;
  set.frame1.thumb_px = 32767;
  set.frame2.thumb_py = -32767;
  set.frame2.thumb_temp = 25;
  set.frame2.thumb_contact = 1;
  set.frame2.index.fx = -32767;
  set.frame2.index.fy = 32767;
  set.frame2.index.temp = 30;
  set.frame2.middle.fx = 32767;  // must stay untouched with num_fingertips = 2
  std::array<uint8_t, sizeof(RespSet)> raw{};
  std::memcpy(raw.data(), &set, sizeof(set));

  SensorProtocol2 proto;
  UdpHandState state{};
  state.num_fingertips = 2;
  state.sensor_force.fill(-1.0f);
  ASSERT_TRUE(proto.DecodeAllSensors(raw.data(), raw.size(), state));

  constexpr std::size_t kVals = SensorProtocol2::kValuesPerFingertip;
  EXPECT_FLOAT_EQ(state.sensor_force[0 * kVals + 0], 25.0f);     // fx +
  EXPECT_FLOAT_EQ(state.sensor_force[0 * kVals + 1], 25.0f);     // fy flipped
  EXPECT_FLOAT_EQ(state.sensor_force[0 * kVals + 2], 100.0f);    // fz flipped
  EXPECT_FLOAT_EQ(state.sensor_force[0 * kVals + 3], 0.32767f);  // px, metres
  EXPECT_FLOAT_EQ(state.sensor_force[0 * kVals + 4], 0.32767f);  // py flipped
  EXPECT_FLOAT_EQ(state.sensor_force[0 * kVals + 5], 25.0f);     // temp
  EXPECT_FLOAT_EQ(state.sensor_force[1 * kVals + 0], -25.0f);    // index fx
  EXPECT_FLOAT_EQ(state.sensor_force[1 * kVals + 1], -25.0f);    // index fy flipped
  EXPECT_FLOAT_EQ(state.sensor_force[1 * kVals + 5], 30.0f);
  EXPECT_FLOAT_EQ(state.sensor_force[2 * kVals + 0], -1.0f);  // clamp: untouched
  EXPECT_TRUE(proto.telemetry().tactile[0].contact);
  EXPECT_FALSE(proto.telemetry().tactile[1].contact);
  // D29: the contact flag lands in the state field the publish path reads.
  EXPECT_EQ(state.sensor_contact[0], 1);
  EXPECT_EQ(state.sensor_contact[1], 0);
  EXPECT_EQ(state.sensor_contact[2], 0);  // clamped fingertip: untouched (zero-init)

  EXPECT_FALSE(proto.DecodeAllSensors(raw.data(), raw.size() - 1, state));
  EXPECT_FALSE(proto.DecodeAllSensors(nullptr, raw.size(), state));
}

// ═══════════════════════════ vcan0 (skips without) ══════════════════════════

TEST_F(CanFdHandCommVcan, OpenRejectsNonPositiveKtAndScalesFeedforward) {
  CanFdHandCommConfig cfg = CommConfig();
  cfg.feedforward_kt[3] = 0.0f;
  EXPECT_FALSE(CanFdHandComm(cfg).Open());
  cfg.feedforward_kt[3] = -1.0f;
  EXPECT_FALSE(CanFdHandComm(cfg).Open());
  cfg.feedforward_kt[3] = std::nanf("");
  EXPECT_FALSE(CanFdHandComm(cfg).Open());
  cfg.feedforward_kt[3] = 1e-40f;  // subnormal: > 0 but 1/kt = inf → NaN E-Stop ff
  EXPECT_FALSE(CanFdHandComm(cfg).Open());

  cfg.feedforward_kt[3] = 4.0f;
  CanFdHandComm comm(cfg);
  ASSERT_TRUE(comm.Open());
  EXPECT_FLOAT_EQ(comm.FeedforwardToAmps(3, 3.0f), 0.75f);  // Nm / (Nm/A) = A
  EXPECT_FLOAT_EQ(comm.FeedforwardToAmps(0, 3.0f), 3.0f);   // kt 1.0 passes through
}

TEST_F(CanFdHandCommVcan, D41_D50_WriteEchoReachesFakeAndFeedforwardIsInAmps) {
  auto peer = StartPeer();
  CanFdHandCommConfig cfg = CommConfig();
  cfg.feedforward_kt.fill(2.0f);
  auto comm = OpenComm(cfg);

  const HandMotorArray pos = Ramp(0.1f, 0.01f);
  const HandMotorArray ff_nm = Ramp(1.0f, 0.5f);
  EXPECT_TRUE(comm->WriteCommandWithEcho(pos, ff_nm));

  peer->WithModel([&](const FakeHandFirmware& fw) {
    for (std::size_t i = 0; i < pos.size(); ++i) {
      EXPECT_FLOAT_EQ(fw.target()[i], pos[i]);
      EXPECT_FLOAT_EQ(fw.last_feedforward()[i], ff_nm[i] / 2.0f);
    }
    return 0;
  });

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  const auto& echo = s.per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)];
  EXPECT_EQ(echo.ok, 1u);
  EXPECT_EQ(echo.timeout, 0u);
  EXPECT_EQ(echo.cmd_mismatch, 0u);
  EXPECT_EQ(s.recv_ok, 1u);
  EXPECT_EQ(s.cmd_mismatch, 0u);
  EXPECT_EQ(comm->canfd_stats().echo_payload_mismatch, 0u);
  EXPECT_EQ(comm->recv_error_count(), 0u);
}

TEST_F(CanFdHandCommVcan, D47_D49_ReadCycleFillsJointTripletAndRawSet) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  ASSERT_TRUE(comm->WriteCommandWithEcho(Ramp(0.2f, 0.02f), HandMotorArray{}));  // move the model

  HandReadBuffers bufs{};
  HandReadOutcome out{};
  comm->ReadCycle(SensorPlan(true), bufs, out);
  EXPECT_FALSE(out.motor_ok);  // no motor-space read on this wire
  ASSERT_TRUE(out.joint_ok);
  ASSERT_EQ(out.sensor_recvd, static_cast<ssize_t>(sizeof(RespSet)));
  EXPECT_LE(out.PhaseEnd(ReadPhase::kMotor), out.PhaseEnd(ReadPhase::kJoint));

  peer->WithModel([&](const FakeHandFirmware& fw) {
    for (std::size_t i = 0; i < kWireMotorCount; ++i) {
      EXPECT_FLOAT_EQ(bufs.joint.pos[i], fw.pos()[i]);
      EXPECT_FLOAT_EQ(bufs.joint.vel[i], fw.vel()[i]);
      EXPECT_FLOAT_EQ(bufs.joint.cur[i], fw.effort()[i]);
    }
    return 0;
  });
  EXPECT_NE(bufs.joint.pos[9], 0.0f);  // the model did move toward the ramp

  // The raw set decodes through SensorProtocol2 into the fake's band, with the
  // Proto §4.3 sign flips undone (the fake pre-applies them: raw fy fz py < 0).
  SensorProtocol2 proto;
  UdpHandState state{};
  state.num_fingertips = kDefaultNumFingertips;
  ASSERT_TRUE(proto.DecodeAllSensors(bufs.sensor_raw.data(),
                                     static_cast<std::size_t>(out.sensor_recvd), state));
  const float lo_n = static_cast<float>(kBaseMin - kBand) * kForceXyRangeN / kTactileFullScale;
  const float hi_n = static_cast<float>(kBaseMax + kBand) * kForceXyRangeN / kTactileFullScale;
  for (std::size_t f = 0; f < kWireFingertipCount; ++f) {
    const float* slot = &state.sensor_force[f * SensorProtocol2::kValuesPerFingertip];
    EXPECT_GE(slot[0], lo_n) << "finger " << f;
    EXPECT_LE(slot[0], hi_n) << "finger " << f;
    EXPECT_GT(slot[1], 0.0f) << "fy finger " << f;
    EXPECT_GT(slot[2], 0.0f) << "fz finger " << f;
    EXPECT_GT(slot[4], 0.0f) << "py finger " << f;
    EXPECT_TRUE(proto.telemetry().tactile[f].contact);
  }

  // A non-sensor cycle leaves sensor_recvd at -1 and counts only kJointRead.
  comm->ReadCycle(SensorPlan(false), bufs, out);
  EXPECT_TRUE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, -1);

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kJointRead)].ok, 2u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kBulkSensor)].ok, 1u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kMotorRead)].ok, 0u);
  EXPECT_EQ(s.recv_ok, 3u);  // write + 2 reads, one arrival each
  EXPECT_EQ(s.cmd_mismatch, 0u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_completed, 2u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_abandoned, 0u);
}

TEST_F(CanFdHandCommVcan, D42_EchoStageTimesOutWithoutPeer) {
  auto comm = OpenComm();
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(elapsed, 45ms);   // the ppoll deadline, not an early give-up
  EXPECT_LT(elapsed, 500ms);  // and not SO_RCVTIMEO or a hang

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  const auto& echo = s.per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)];
  EXPECT_EQ(echo.timeout, 1u);
  EXPECT_EQ(echo.ok, 0u);
  EXPECT_EQ(s.recv_timeout, 1u);
  EXPECT_EQ(s.recv_ok, 0u);
  EXPECT_EQ(comm->recv_error_count(), 1u);
}

TEST_F(CanFdHandCommVcan, D42_SetStageTimesOutWithoutPeer) {
  auto comm = OpenComm();
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  comm->ReadCycle(SensorPlan(true), bufs, out);
  EXPECT_FALSE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, -1);

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kJointRead)].timeout, 1u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kBulkSensor)].timeout, 1u);
  EXPECT_EQ(s.recv_timeout, 1u);  // one transaction → one aggregate count
  EXPECT_EQ(comm->recv_error_count(), 1u);
}

TEST_F(CanFdHandCommVcan, D43_D44_PartialSetIsAbandonedAndNextCycleRecovers) {
  auto comm = OpenComm();
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  {
    auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kDropFrame2);
    comm->ReadCycle(SensorPlan(true), bufs, out);
    EXPECT_FALSE(out.joint_ok);
    EXPECT_EQ(out.sensor_recvd, -1);
    comm->PublishCommStats();
    EXPECT_EQ(comm->canfd_stats().assembler.sets_abandoned, 1u);
    EXPECT_EQ(comm->canfd_stats().assembler.sets_completed, 0u);
    EXPECT_EQ(
        comm->comm_stats().per_kind[static_cast<std::size_t>(RequestKind::kJointRead)].timeout, 1u);
  }
  auto peer = StartPeer();
  EXPECT_EQ(comm->DrainStale(), 0);  // nothing lingered from the partial set
  comm->ReadCycle(SensorPlan(true), bufs, out);
  EXPECT_TRUE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, static_cast<ssize_t>(sizeof(RespSet)));
  comm->PublishCommStats();
  EXPECT_EQ(comm->canfd_stats().assembler.sets_completed, 1u);
}

TEST_F(CanFdHandCommVcan, D57_ReverseOrderedRespBurstCompletesTheSet) {
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kReverse);
  auto comm = OpenComm();
  ASSERT_TRUE(comm->WriteCommandWithEcho(Ramp(0.2f, 0.02f), HandMotorArray{}));  // move the model

  HandReadBuffers bufs{};
  HandReadOutcome out{};
  comm->ReadCycle(SensorPlan(true), bufs, out);
  ASSERT_TRUE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, static_cast<ssize_t>(sizeof(RespSet)));
  peer->WithModel([&](const FakeHandFirmware& fw) {
    for (std::size_t i = 0; i < kWireMotorCount; ++i) {
      EXPECT_FLOAT_EQ(bufs.joint.pos[i], fw.pos()[i]);     // F0
      EXPECT_FLOAT_EQ(bufs.joint.vel[i], fw.vel()[i]);     // F0 + F1
      EXPECT_FLOAT_EQ(bufs.joint.cur[i], fw.effort()[i]);  // F1
    }
    return 0;
  });

  comm->PublishCommStats();
  EXPECT_EQ(comm->canfd_stats().assembler.sets_completed, 1u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_abandoned, 0u);
  EXPECT_EQ(comm->canfd_stats().assembler.duplicate, 0u);
  EXPECT_EQ(comm->comm_stats().cmd_mismatch, 0u);
}

TEST_F(CanFdHandCommVcan, D58_OneStaleRespOnlyCountsAndTheWriteSucceeds) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  ASSERT_TRUE(injector.SendFrame(DataFrame(RespId(kNodeRightHand, 2), 0xAB)));

  EXPECT_TRUE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(s.cmd_mismatch, 1u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)].cmd_mismatch, 1u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)].ok, 1u);
  EXPECT_EQ(comm->last_unexpected_cmd(),
            static_cast<uint8_t>((static_cast<uint8_t>(MsgType::kResp) << 4) | 2));
  EXPECT_EQ(comm->last_unexpected_len(), kFrameLen);
}

TEST_F(CanFdHandCommVcan, D58_ThreeUnexpectedFramesBeforeTheEchoesDoNotFailTheWrite) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  for (uint8_t i = 0; i < 3; ++i)
    ASSERT_TRUE(injector.SendFrame(DataFrame(RespId(kNodeRightHand, i), 0x11)));

  EXPECT_TRUE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(s.cmd_mismatch, 3u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)].ok, 1u);
  EXPECT_EQ(s.recv_timeout, 0u);
}

TEST_F(CanFdHandCommVcan, D58_ThreeUnexpectedFramesOnlyCountUntilTheDeadline) {
  auto comm = OpenComm();  // no peer: only the injected frames are on the bus
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  for (uint8_t i = 0; i < 3; ++i)
    ASSERT_TRUE(injector.SendFrame(DataFrame(RespId(kNodeRightHand, i), 0x11)));

  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(elapsed, 45ms);  // still waiting for the echoes at the deadline
  EXPECT_LT(elapsed, 500ms);

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(s.cmd_mismatch, 3u);
  EXPECT_EQ(s.recv_timeout, 1u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)].timeout, 1u);
  EXPECT_EQ(s.per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)].ok, 0u);
}

TEST_F(CanFdHandCommVcan, D41_EchoWithAnotherPayloadOnlyCounts) {
  auto comm = OpenComm();  // no peer: the injected frames play the hand
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  // The hand echoes the previous command, idx 1 first — order must not matter.
  ASSERT_TRUE(injector.SendFrame(DataFrame(EchoId(kNodeRightHand, 1), 0x55)));
  ASSERT_TRUE(injector.SendFrame(DataFrame(EchoId(kNodeRightHand, 0), 0x55)));

  EXPECT_TRUE(comm->WriteCommandWithEcho(Ramp(0.3f, 0.1f), Ramp(1.0f, 0.1f)));
  comm->PublishCommStats();
  EXPECT_EQ(comm->canfd_stats().echo_payload_mismatch, 2u);
  EXPECT_EQ(comm->comm_stats().cmd_mismatch, 0u);
  EXPECT_EQ(comm->comm_stats().per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)].ok, 1u);
}

TEST_F(CanFdHandCommVcan, D41_BusOffErrorFrameIsNotAnUnexpectedFrame) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  can_frame err{};
  err.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;
  err.len = CAN_ERR_DLC;
  ASSERT_TRUE(injector.SendFrame(err));

  EXPECT_TRUE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));
  comm->PublishCommStats();
  const CanFdCommStats c = comm->canfd_stats();
  // >= : error frames have no id separation on the shared vcan0, so another
  // package's test injecting one at the same moment must not fail this one.
  EXPECT_GE(c.error_frames, 1u);
  EXPECT_GE(c.bus_off, 1u);
  EXPECT_EQ(c.error_frames, c.bus_off);
  EXPECT_EQ(comm->comm_stats().cmd_mismatch, 0u);
  EXPECT_EQ(comm->comm_stats().per_kind[static_cast<std::size_t>(RequestKind::kWriteEcho)].ok, 1u);
}

// ═══════════════════ D59 · D60 — the pipelined cycle (RunCycle) ═════════════

// The normal cycle: CMD 0 · CMD 1 · REQ on the bus back to back, one window.
// Both echoes and the set validate, each stage counted once, no mismatch.
TEST_F(CanFdHandCommVcan, D59_RunCycleWritesAndReadsInOneWindow) {
  rtc::CanFdTransport sniffer(TargetedConfig(kNodeRightHand));
  ASSERT_TRUE(sniffer.Open());
  auto peer = StartPeer();
  CanFdHandCommConfig cfg = CommConfig();
  cfg.feedforward_kt.fill(2.0f);
  auto comm = OpenComm(cfg);

  const HandMotorArray pos = Ramp(0.1f, 0.01f);
  const HandMotorArray ff_nm = Ramp(1.0f, 0.5f);
  HandReadBuffers bufs{};
  const auto t0 = std::chrono::steady_clock::now();
  const HandReadOutcome out = Cycle(*comm, pos, ff_nm, bufs);
  const auto t_end = std::chrono::steady_clock::now();
  EXPECT_TRUE(out.write_ok);
  EXPECT_FALSE(out.motor_ok);
  ASSERT_TRUE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, static_cast<ssize_t>(sizeof(RespSet)));
  EXPECT_GE(out.write_end, t0);
  EXPECT_LE(out.write_end, t_end);
  EXPECT_LE(out.PhaseEnd(ReadPhase::kMotor), out.PhaseEnd(ReadPhase::kJoint));
  EXPECT_LE(out.PhaseEnd(ReadPhase::kJoint), t_end);

  // Wire order: the REQ follows the CMD pair without waiting for an echo.
  const timespec wait{0, 200'000'000};
  canfd_frame rx{};
  ASSERT_EQ(sniffer.RecvFrameWithTimeout(rx, wait), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id & kExtIdMask, CmdId(kNodeRightHand, 0));
  ASSERT_EQ(sniffer.RecvFrameWithTimeout(rx, wait), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id & kExtIdMask, CmdId(kNodeRightHand, 1));
  ASSERT_GT(sniffer.RecvFrameWithTimeout(rx, wait), 0);
  EXPECT_EQ(rx.can_id & kExtIdMask, ReqId(kNodeRightHand));
  EXPECT_EQ(rx.len, 0);  // D47

  peer->WithModel([&](const FakeHandFirmware& fw) {
    for (std::size_t i = 0; i < pos.size(); ++i) {
      EXPECT_FLOAT_EQ(fw.target()[i], pos[i]);
      EXPECT_FLOAT_EQ(fw.last_feedforward()[i], ff_nm[i] / 2.0f);  // D50
    }
    for (std::size_t i = 0; i < kWireMotorCount; ++i)
      EXPECT_FLOAT_EQ(bufs.joint.pos[i], fw.pos()[i]);
    return 0;
  });

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).timeout, 0u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kBulkSensor).ok, 1u);
  EXPECT_EQ(s.recv_ok, 2u);  // one per stage
  EXPECT_EQ(s.cmd_mismatch, 0u);
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(comm->canfd_stats().assembler.outside_set, 0u);  // class ④ never reaches Feed
  EXPECT_EQ(comm->canfd_stats().echo_payload_mismatch, 0u);
}

// A RESP that arrives before the echoes (the hand answered the REQ first, or
// the burst interleaves with the echoes) is fed to the set, not rejected as a
// stale frame: same outcome as the in-order cycle, no duplicate, no mismatch.
TEST_F(CanFdHandCommVcan, D59_RespBeforeTheEchoesIsFedNotRejected) {
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kRespFirst);
  auto comm = OpenComm();
  HandReadBuffers bufs{};
  const HandReadOutcome out = Cycle(*comm, Ramp(0.2f, 0.02f), HandMotorArray{}, bufs);
  EXPECT_TRUE(out.write_ok);
  ASSERT_TRUE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, static_cast<ssize_t>(sizeof(RespSet)));
  peer->WithModel([&](const FakeHandFirmware& fw) {
    for (std::size_t i = 0; i < kWireMotorCount; ++i) {
      EXPECT_FLOAT_EQ(bufs.joint.pos[i], fw.pos()[i]);
      EXPECT_FLOAT_EQ(bufs.joint.cur[i], fw.effort()[i]);
    }
    return 0;
  });

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(s.cmd_mismatch, 0u);
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(s.recv_ok, 2u);
  EXPECT_EQ(comm->canfd_stats().assembler.duplicate, 0u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_completed, 1u);
}

// No echo at all: the echo stage times out at CMD + T while the set, answered
// at once, completes — the window lasts about T, not 2T, and the set is not
// charged for the missing echoes.
TEST_F(CanFdHandCommVcan, D59_NoEchoTimesOutTheEchoStageOnlyWithinOneT) {
  constexpr auto kT = 100ms;
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kNoEcho);
  CanFdHandCommConfig cfg = CommConfig();
  cfg.recv_timeout_ms = 100.0;
  auto comm = OpenComm(cfg);
  HandReadBuffers bufs{};
  const auto t0 = std::chrono::steady_clock::now();
  const HandReadOutcome out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_FALSE(out.write_ok);
  EXPECT_TRUE(out.joint_ok);
  EXPECT_GE(elapsed, kT * 9 / 10);
  EXPECT_LT(elapsed, kT * 15 / 10);
  EXPECT_GE(out.write_end, t0 + kT * 9 / 10);                 // closed at the deadline, not early
  EXPECT_LE(out.PhaseEnd(ReadPhase::kJoint), out.write_end);  // the set closed first

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).timeout, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 0u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).timeout, 0u);
  EXPECT_EQ(s.recv_timeout, 1u);
  EXPECT_EQ(s.recv_ok, 1u);  // only the set stage saw its class
  EXPECT_EQ(s.cmd_mismatch, 0u);
  EXPECT_EQ(comm->recv_error_count(), 1u);
}

// RESP frame 2 missing: the write validates, the set stage times out at
// (echo close) + T and the partial set is abandoned (D43 · D44) — one
// terminal counter per stage.
TEST_F(CanFdHandCommVcan, D59_MissingRespFrame2FailsOnlyTheSetStage) {
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kDropFrame2);
  auto comm = OpenComm();
  HandReadBuffers bufs{};
  const HandReadOutcome out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  EXPECT_TRUE(out.write_ok);
  EXPECT_FALSE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, -1);
  EXPECT_GE(out.PhaseEnd(ReadPhase::kJoint), out.write_end);

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).timeout, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kBulkSensor).timeout, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 0u);
  EXPECT_EQ(s.recv_timeout, 1u);
  EXPECT_EQ(s.recv_ok, 2u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_abandoned, 1u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_completed, 0u);
}

// The deadline chain (A2): echoes 0.6 T after the CMD, the RESP burst 1.3 T
// after it. A set deadline of REQ + T would miss the burst; the chain's
// (echo close) + T = 1.6 T takes it. Window within 2 T.
TEST_F(CanFdHandCommVcan, D59_SetDeadlineChainsFromTheEchoClose) {
  constexpr auto kT = 100ms;
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kLate, kT);
  CanFdHandCommConfig cfg = CommConfig();
  cfg.recv_timeout_ms = 100.0;
  auto comm = OpenComm(cfg);
  HandReadBuffers bufs{};
  const auto t0 = std::chrono::steady_clock::now();
  const HandReadOutcome out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.joint_ok);
  EXPECT_GE(out.write_end - t0, kT * 5 / 10);
  EXPECT_GE(out.PhaseEnd(ReadPhase::kJoint) - t0, kT * 12 / 10);
  EXPECT_LT(elapsed, kT * 2);

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(s.cmd_mismatch, 0u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_abandoned, 0u);
}

// Class ④ in the window: an echo idx 2, a second copy of echo 0 (the first
// copy, injected with another payload, is the one accepted) and a frame of
// another type that passes the socket filter. Three mismatches, nothing
// fails, nothing reaches the assembler as outside_set.
TEST_F(CanFdHandCommVcan, D59_ClassFourFramesOnlyCount) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  ASSERT_TRUE(injector.SendFrame(DataFrame(EchoId(kNodeRightHand, 2), 0x22)));
  ASSERT_TRUE(injector.SendFrame(DataFrame(EchoId(kNodeRightHand, 0), 0x55)));
  constexpr uint32_t kOtherType =
      MakeCanId(kPriorityRealtime, static_cast<MsgType>(0x3), kNodeHost, kNodeRightHand, 0);
  ASSERT_TRUE(injector.SendFrame(DataFrame(kOtherType, 0x33)));

  HandReadBuffers bufs{};
  const HandReadOutcome out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.joint_ok);

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(s.cmd_mismatch, 3u);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).cmd_mismatch,
            3u);  // all while the echo stage was open
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).cmd_mismatch, 0u);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(comm->canfd_stats().echo_payload_mismatch, 1u);  // the injected echo 0
  EXPECT_EQ(comm->canfd_stats().echo_unexpected, 0u);  // every echo came while its stage was open
  EXPECT_EQ(comm->canfd_stats().assembler.outside_set, 0u);
  EXPECT_EQ(comm->canfd_stats().assembler.bad_index, 0u);
}

// D57 on the window path: a stale RESP 0 of the previous cycle queued before
// the CMD goes out is stored, then replaced by the real burst's frame 0
// (duplicate before completion) — the set completes with the hand's data.
TEST_F(CanFdHandCommVcan, D59_StaleRespBeforeTheEchoesIsReplacedByTheRealBurst) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  ASSERT_TRUE(comm->WriteCommandWithEcho(Ramp(0.2f, 0.02f), HandMotorArray{}));  // move the model
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  ASSERT_TRUE(injector.SendFrame(DataFrame(RespId(kNodeRightHand, 0), 0xAB)));

  HandReadBuffers bufs{};
  const HandReadOutcome out = Cycle(*comm, Ramp(0.2f, 0.02f), HandMotorArray{}, bufs);
  EXPECT_TRUE(out.write_ok);
  ASSERT_TRUE(out.joint_ok);
  peer->WithModel([&](const FakeHandFirmware& fw) {
    for (std::size_t i = 0; i < kWireMotorCount; ++i)
      EXPECT_FLOAT_EQ(bufs.joint.pos[i], fw.pos()[i]);  // not the 0xAB pattern
    return 0;
  });

  comm->PublishCommStats();
  EXPECT_EQ(comm->canfd_stats().assembler.duplicate, 1u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_completed, 1u);
  EXPECT_EQ(comm->canfd_stats().assembler.sets_abandoned, 0u);
  EXPECT_EQ(comm->comm_stats().cmd_mismatch, 0u);
}

// D41 on the window path: echoes with another payload only count; a bus-off
// error frame costs nothing (>= 1: error frames are bus-wide on vcan0).
TEST_F(CanFdHandCommVcan, D59_PayloadMismatchAndErrorFrameOnlyCountInTheWindow) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  ASSERT_TRUE(injector.SendFrame(DataFrame(EchoId(kNodeRightHand, 1), 0x55)));
  can_frame err{};
  err.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;
  err.len = CAN_ERR_DLC;
  ASSERT_TRUE(injector.SendFrame(err));

  HandReadBuffers bufs{};
  const HandReadOutcome out = Cycle(*comm, Ramp(0.3f, 0.1f), Ramp(1.0f, 0.1f), bufs);
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.joint_ok);

  comm->PublishCommStats();
  const CanFdCommStats c = comm->canfd_stats();
  EXPECT_EQ(c.echo_payload_mismatch, 1u);  // the injected idx 1; the real idx 1 is then class ④
  EXPECT_EQ(c.echo_unexpected, 1u);        // … arriving after the echo stage closed
  EXPECT_GE(c.error_frames, 1u);
  EXPECT_GE(c.bus_off, 1u);
  EXPECT_EQ(comm->comm_stats().cmd_mismatch, 1u);
  EXPECT_EQ(Kind(comm->comm_stats(), RequestKind::kWriteEcho).ok, 1u);
}

// D51 on the window path, and recovery: a left comm gets nothing from a right
// fake (both stages time out, one terminal counter each); with a left fake
// the next cycle is clean after DrainStale.
TEST_F(CanFdHandCommVcan, D59_LeftHandWindowSeesOnlyTheLeftFakeAndRecovers) {
  auto right_peer = StartPeer(HandSide::kRight);
  auto comm = OpenComm(CommConfig(HandSide::kLeft));
  HandReadBuffers bufs{};
  HandReadOutcome out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  EXPECT_FALSE(out.write_ok);
  EXPECT_FALSE(out.joint_ok);
  comm->PublishCommStats();
  EXPECT_EQ(Kind(comm->comm_stats(), RequestKind::kWriteEcho).timeout, 1u);
  EXPECT_EQ(Kind(comm->comm_stats(), RequestKind::kJointRead).timeout, 1u);
  EXPECT_EQ(comm->comm_stats().recv_timeout, 2u);
  EXPECT_EQ(comm->comm_stats().recv_ok, 0u);

  auto left_peer = StartPeer(HandSide::kLeft);
  EXPECT_EQ(comm->DrainStale(), 0);
  out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.joint_ok);
  right_peer->WithModel([](const FakeHandFirmware& fw) {
    EXPECT_FLOAT_EQ(fw.target()[9], 0.0f);
    return 0;
  });
  left_peer->WithModel([](const FakeHandFirmware& fw) {
    EXPECT_FLOAT_EQ(fw.target()[9], 0.9f);
    return 0;
  });
  comm->PublishCommStats();
  EXPECT_EQ(comm->comm_stats().recv_timeout, 2u);  // nothing new
  EXPECT_EQ(comm->comm_stats().cmd_mismatch, 0u);
}

// A tick without a command: RunCycle with null pointers is the set stage
// alone — the same as ReadCycle, write fields at their defaults.
TEST_F(CanFdHandCommVcan, D60_RunCycleWithoutCommandIsTheReadCycle) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  out.write_ok = true;
  comm->RunCycle(SensorPlan(false), nullptr, nullptr, bufs, out);
  EXPECT_FALSE(out.write_ok);
  EXPECT_EQ(out.write_end, HandReadOutcome::Clock::time_point{});
  EXPECT_TRUE(out.joint_ok);
  EXPECT_EQ(out.sensor_recvd, -1);
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok + Kind(s, RequestKind::kWriteEcho).timeout, 0u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(s.recv_ok, 1u);
  peer->WithModel([](const FakeHandFirmware& fw) {
    EXPECT_FLOAT_EQ(fw.target()[9], 0.0f);  // no command reached the hand
    return 0;
  });
}

// ── cmd_ack none (#945 D63 · D64 · D68) ────────────────────────────────────

namespace {
CanFdHandCommConfig NoneConfig(double recv_timeout_ms = kStageTimeoutMs) {
  CanFdHandCommConfig cfg = CommConfig();
  cfg.cmd_ack = CmdAck::kNone;
  cfg.recv_timeout_ms = recv_timeout_ms;
  return cfg;
}
}  // namespace

// On an echoing link the write is receive-confirmed: write_rx follows write_ok.
TEST_F(CanFdHandCommVcan, D68_EchoWriteIsReceiveConfirmed) {
  auto peer = StartPeer();
  auto comm = OpenComm();
  HandReadBuffers bufs{};
  const HandReadOutcome out = Cycle(*comm, Ramp(0.1f, 0.01f), HandMotorArray{}, bufs);
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.write_rx);
}

// A silent firmware and cmd_ack none: the write is ok on its send (not a
// receive), the command reached the hand, the set completes, and nothing waits
// for an echo — the window is far shorter than T.
TEST_F(CanFdHandCommVcan, D63_NoneWriteIsSendConfirmed) {
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kNormal, 0ms, CmdAck::kNone);
  auto comm = OpenComm(NoneConfig());
  const HandMotorArray pos = Ramp(0.1f, 0.01f);
  HandReadBuffers bufs{};
  const auto t0 = std::chrono::steady_clock::now();
  const HandReadOutcome out = Cycle(*comm, pos, HandMotorArray{}, bufs);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_TRUE(out.write_ok);
  EXPECT_FALSE(out.write_rx) << "D68: a send is not a receive";
  ASSERT_TRUE(out.joint_ok);
  EXPECT_GE(out.write_end, t0);
  EXPECT_LE(out.write_end, out.PhaseEnd(ReadPhase::kJoint));
  EXPECT_LT(elapsed, std::chrono::milliseconds(static_cast<int>(kStageTimeoutMs / 2)));
  peer->WithModel([&](const FakeHandFirmware& fw) {
    for (std::size_t i = 0; i < pos.size(); ++i)
      EXPECT_FLOAT_EQ(fw.target()[i], pos[i]);
    return 0;
  });

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).timeout, 0u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(s.recv_ok, 1u);  // the set stage only
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(s.cmd_mismatch, 0u);
  EXPECT_EQ(comm->recv_error_count(), 0u);
  EXPECT_EQ(comm->canfd_stats().echo_unexpected, 0u);
}

// cmd_ack none against an echoing firmware: both echoes land in the set stage
// as class ④ (before RESP F2 — the hand answers in order), counted as
// echo_unexpected; nothing fails, and the WARN predicate fires.
TEST_F(CanFdHandCommVcan, D63_NoneCountsUnexpectedEchoes) {
  auto peer = StartPeer();  // echoes
  auto comm = OpenComm(NoneConfig());
  HandReadBuffers bufs{};
  const HandReadOutcome out = Cycle(*comm, Ramp(0.1f, 0.01f), HandMotorArray{}, bufs);
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.joint_ok);

  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  const CanFdCommStats c = comm->canfd_stats();
  EXPECT_EQ(c.echo_unexpected, 2u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).cmd_mismatch, 2u);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).cmd_mismatch, 0u);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
  EXPECT_EQ(CmdAckMismatchDecision(CmdAck::kNone, Kind(s, RequestKind::kWriteEcho),
                                   Kind(s, RequestKind::kJointRead).ok, c.echo_unexpected),
            CmdAckMismatch::kFirmwareEchoes);
}

// cmd_ack echo against a silent firmware (the real fake under none): every
// echo stage times out while the set completes, and after enough writes the
// WARN predicate reads it as a silent firmware.
TEST_F(CanFdHandCommVcan, D63_EchoAgainstSilentFirmwareReadsAsSilent) {
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kNormal, 0ms, CmdAck::kNone);
  CanFdHandCommConfig cfg = CommConfig();
  cfg.recv_timeout_ms = 2.0;
  auto comm = OpenComm(cfg);
  HandReadBuffers bufs{};
  for (uint64_t i = 0; i < kCmdAckMismatchMinWrites; ++i) {
    const HandReadOutcome out = Cycle(*comm, Ramp(0.1f, 0.01f), HandMotorArray{}, bufs);
    EXPECT_FALSE(out.write_ok);
    EXPECT_FALSE(out.write_rx);
  }
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).timeout, kCmdAckMismatchMinWrites);
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 0u);
  EXPECT_GT(Kind(s, RequestKind::kJointRead).ok, 0u);
  EXPECT_EQ(CmdAckMismatchDecision(CmdAck::kEcho, Kind(s, RequestKind::kWriteEcho),
                                   Kind(s, RequestKind::kJointRead).ok,
                                   comm->canfd_stats().echo_unexpected),
            CmdAckMismatch::kFirmwareSilent);
}

// The write alone (no REQ) under none opens no stage at all: true at once,
// even with nobody on the bus.
TEST_F(CanFdHandCommVcan, D63_NoneWriteAloneReturnsAtOnce) {
  auto comm = OpenComm(NoneConfig());
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_TRUE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 10ms);
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok, 1u);
  EXPECT_EQ(s.recv_ok, 0u);
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(comm->recv_error_count(), 0u);
}

// D64: after a write the set stage keeps the nominal CMD + 2T. The hand works
// through the CMD before the REQ, so its burst lands later than REQ + T: here
// 1.3 T after the CMD (kLate; the silent fake holds back no echo) — a REQ + T
// deadline would miss it.
TEST_F(CanFdHandCommVcan, D64_NoneSetDeadlineIsCmdPlusTwoT) {
  constexpr auto kT = 100ms;
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kLate, kT, CmdAck::kNone);
  auto comm = OpenComm(NoneConfig(100.0));
  HandReadBuffers bufs{};
  const auto t0 = std::chrono::steady_clock::now();
  const HandReadOutcome out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.joint_ok);
  EXPECT_LT(out.write_end - t0, kT / 10);  // the write closed on the send
  EXPECT_GE(out.PhaseEnd(ReadPhase::kJoint) - t0, kT * 12 / 10);
  EXPECT_LT(elapsed, kT * 2);
  comm->PublishCommStats();
  EXPECT_EQ(comm->comm_stats().recv_timeout, 0u);
}

// Nobody answers: under none the window after a write is the set stage's 2T.
TEST_F(CanFdHandCommVcan, D64_NoneSilentPeerWindowIsTwoT) {
  const auto k2T = std::chrono::milliseconds(static_cast<int>(2 * kStageTimeoutMs));
  auto comm = OpenComm(NoneConfig());
  HandReadBuffers bufs{};
  const auto t0 = std::chrono::steady_clock::now();
  const HandReadOutcome out = Cycle(*comm, Ramp(0.0f, 0.1f), HandMotorArray{}, bufs);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_TRUE(out.write_ok);
  EXPECT_FALSE(out.joint_ok);
  EXPECT_GE(elapsed, k2T * 9 / 10);
  EXPECT_LT(elapsed, k2T * 15 / 10);
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).timeout, 1u);
  EXPECT_EQ(s.recv_timeout, 1u);  // the set only — there is no echo stage to time out
}

// A tick without a command under none: the set stage alone, REQ + T.
TEST_F(CanFdHandCommVcan, D64_NoneReadOnlyTickKeepsReqPlusT) {
  const auto kT = std::chrono::milliseconds(static_cast<int>(kStageTimeoutMs));
  auto comm = OpenComm(NoneConfig());
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  const auto t0 = std::chrono::steady_clock::now();
  comm->RunCycle(SensorPlan(false), nullptr, nullptr, bufs, out);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_FALSE(out.joint_ok);
  EXPECT_GE(elapsed, kT * 9 / 10);
  EXPECT_LT(elapsed, kT * 15 / 10);
}

// D60 under none: RunCycle without a command is the read cycle, write fields
// at their defaults.
TEST_F(CanFdHandCommVcan, D60_NoneRunCycleWithoutCommandIsTheReadCycle) {
  auto peer = StartPeer(HandSide::kRight, FakePeer::Mode::kNormal, 0ms, CmdAck::kNone);
  auto comm = OpenComm(NoneConfig());
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  out.write_ok = true;
  out.write_rx = true;
  comm->RunCycle(SensorPlan(false), nullptr, nullptr, bufs, out);
  EXPECT_FALSE(out.write_ok);
  EXPECT_FALSE(out.write_rx);
  EXPECT_EQ(out.write_end, HandReadOutcome::Clock::time_point{});
  EXPECT_TRUE(out.joint_ok);
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kWriteEcho).ok + Kind(s, RequestKind::kWriteEcho).timeout, 0u);
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
}

TEST_F(CanFdHandCommVcan, D26_FireAndForgetSendsZeroFrame1ThenZeroFrame0) {
  rtc::CanFdTransport sniffer(TargetedConfig(kNodeRightHand));
  ASSERT_TRUE(sniffer.Open());
  auto comm = OpenComm();
  comm->WriteCommandFireAndForget(HandMotorArray{}, HandMotorArray{});

  const timespec wait{0, 200'000'000};
  canfd_frame rx{};
  ASSERT_EQ(sniffer.RecvFrameWithTimeout(rx, wait), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id & kExtIdMask, CmdId(kNodeRightHand, 1));
  EXPECT_EQ(rx.len, kFrameLen);
  EXPECT_TRUE(rx.flags & CANFD_BRS);
  const std::array<uint8_t, kFrameLen> zeros{};
  EXPECT_EQ(std::memcmp(rx.data, zeros.data(), kFrameLen), 0);

  ASSERT_EQ(sniffer.RecvFrameWithTimeout(rx, wait), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id & kExtIdMask, CmdId(kNodeRightHand, 0));
  EXPECT_EQ(rx.len, kFrameLen);
  EXPECT_EQ(std::memcmp(rx.data, zeros.data(), kFrameLen), 0);

  EXPECT_EQ(sniffer.RecvFrameNonBlocking(rx), 0);  // nothing else was sent
  comm->PublishCommStats();
  EXPECT_EQ(comm->comm_stats().recv_ok, 0u);  // nothing awaited
}

TEST_F(CanFdHandCommVcan, DrainStaleIsBoundedAndCountsStaleDrained) {
  auto comm = OpenComm();
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  for (int i = 0; i < 2; ++i)
    ASSERT_TRUE(injector.SendFrame(DataFrame(RespId(kNodeRightHand, 0), 0x01)));
  EXPECT_EQ(comm->DrainStale(), 2);
  EXPECT_EQ(comm->DrainStale(), 0);

  for (int i = 0; i < CanFdHandComm::kMaxDrainPerCall + 4; ++i)
    ASSERT_TRUE(injector.SendFrame(DataFrame(RespId(kNodeRightHand, 1), 0x02)));
  EXPECT_EQ(comm->DrainStale(), CanFdHandComm::kMaxDrainPerCall);
  EXPECT_EQ(comm->DrainStale(), 4);

  comm->PublishCommStats();
  EXPECT_EQ(comm->comm_stats().stale_drained, 2u + CanFdHandComm::kMaxDrainPerCall + 4u);
  EXPECT_EQ(comm->comm_stats().cmd_mismatch, 0u);  // a drain is not a mismatch
}

TEST_F(CanFdHandCommVcan, D51_LeftHandCommTalksOnlyToTheLeftFake) {
  auto right_peer = StartPeer(HandSide::kRight);
  auto comm = OpenComm(CommConfig(HandSide::kLeft));
  EXPECT_EQ(comm->node(), kNodeLeftHand);
  EXPECT_FALSE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));  // nobody answers

  auto left_peer = StartPeer(HandSide::kLeft);
  EXPECT_TRUE(comm->WriteCommandWithEcho(Ramp(0.0f, 0.1f), HandMotorArray{}));
  right_peer->WithModel([](const FakeHandFirmware& fw) {
    EXPECT_FLOAT_EQ(fw.target()[9], 0.0f);  // the right hand never saw the command
    return 0;
  });
  left_peer->WithModel([](const FakeHandFirmware& fw) {
    EXPECT_FLOAT_EQ(fw.target()[9], 0.9f);
    return 0;
  });
}

// ═══════════════════ #956 shared request (broadcast REQ) ═══════════════════

TEST(CanFdHandCommTier1, SupportsBroadcastReqOnlyWithoutCmdEcho) {
  CanFdHandCommConfig cfg = CommConfig();
  cfg.cmd_ack = CmdAck::kEcho;
  EXPECT_FALSE(CanFdHandComm(cfg).SupportsBroadcastReq());
  cfg.cmd_ack = CmdAck::kNone;
  EXPECT_TRUE(CanFdHandComm(cfg).SupportsBroadcastReq());
}

namespace {
CanFdHandCommConfig BcConfig(HandSide side) {
  CanFdHandCommConfig cfg = NoneConfig();
  cfg.hand_side = side;
  return cfg;
}
}  // namespace

// One REQ to 0xFF, both hands answer, each comm completes its own set — and
// the REQ on the bus is the broadcast id, nothing else to 0xFF.
TEST_F(CanFdHandCommVcan, Bc956_OneBroadcastReqCompletesBothSets) {
  rtc::CanFdTransport sniffer(TargetedConfig(kNodeBroadcast));
  ASSERT_TRUE(sniffer.Open());
  auto right_peer = StartPeer(HandSide::kRight, FakePeer::Mode::kNormal, 0ms, CmdAck::kNone);
  auto left_peer = StartPeer(HandSide::kLeft, FakePeer::Mode::kNormal, 0ms, CmdAck::kNone);
  auto right = OpenComm(BcConfig(HandSide::kRight));
  auto left = OpenComm(BcConfig(HandSide::kLeft));
  EXPECT_EQ(right->DrainStale(), 0);
  EXPECT_EQ(left->DrainStale(), 0);

  HandReadOutcome::Clock::time_point t_req{};
  ASSERT_TRUE(right->SendBroadcastReq(t_req));
  const SharedRequest req{t_req, true, false};
  HandReadBuffers rb{};
  HandReadBuffers lb{};
  HandReadOutcome ro{};
  HandReadOutcome lo{};
  right->ReceiveSet(SensorPlan(true), rb, ro, req);
  left->ReceiveSet(SensorPlan(true), lb, lo, req);
  EXPECT_TRUE(ro.joint_ok);
  EXPECT_TRUE(lo.joint_ok);
  EXPECT_EQ(ro.sensor_recvd, static_cast<ssize_t>(sizeof(RespSet)));
  EXPECT_EQ(lo.sensor_recvd, static_cast<ssize_t>(sizeof(RespSet)));
  EXPECT_EQ(ro.PhaseEnd(ReadPhase::kMotor), t_req);  // S3: kMotor at the anchor
  EXPECT_GE(lo.PhaseEnd(ReadPhase::kJoint), t_req);

  const timespec wait{0, 200'000'000};
  canfd_frame rx{};
  ASSERT_EQ(sniffer.RecvFrameWithTimeout(rx, wait), static_cast<ssize_t>(CANFD_MTU));
  EXPECT_EQ(rx.can_id & kExtIdMask, 0x007FC000u);
  EXPECT_EQ(rx.len, 0);
  EXPECT_EQ(sniffer.RecvFrameNonBlocking(rx), 0);  // one frame to 0xFF, no broadcast CMD

  for (auto* comm : {right.get(), left.get()}) {
    comm->PublishCommStats();
    const UdpHandCommStats s = comm->comm_stats();
    EXPECT_EQ(Kind(s, RequestKind::kJointRead).ok, 1u);
    EXPECT_EQ(Kind(s, RequestKind::kBulkSensor).ok, 1u);
    EXPECT_EQ(s.recv_timeout + s.recv_error + s.cmd_mismatch, 0u);
  }
}

// The bound is REQ + T, not (ReceiveSet call) + T. The right hand's window
// stands in for 0.6 T; the left peer answers 1.5 T after the REQ, so a bound
// anchored at the call (1.6 T) would take it.
TEST_F(CanFdHandCommVcan, Bc956_BoundIsAnchoredAtTheReqNotTheCall) {
  const auto kT = std::chrono::milliseconds(static_cast<int>(kStageTimeoutMs));
  auto left_peer =
      StartPeer(HandSide::kLeft, FakePeer::Mode::kDelayedResp, kT * 3 / 2, CmdAck::kNone);
  auto left = OpenComm(BcConfig(HandSide::kLeft));
  HandReadOutcome::Clock::time_point t_req{};
  ASSERT_TRUE(left->SendBroadcastReq(t_req));
  std::this_thread::sleep_until(t_req + kT * 6 / 10);
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  left->ReceiveSet(SensorPlan(false), bufs, out, SharedRequest{t_req, true, false});
  const auto closed = std::chrono::steady_clock::now() - t_req;
  EXPECT_FALSE(out.joint_ok);
  EXPECT_GE(closed, kT * 9 / 10);
  EXPECT_LT(closed, kT * 14 / 10);
  left->PublishCommStats();
  EXPECT_EQ(Kind(left->comm_stats(), RequestKind::kJointRead).timeout, 1u);
}

// A set that arrived in time is taken even when ReceiveSet starts after the
// bound: the right hand's set never completes (bound T), the left set has
// been queued since long before.
TEST_F(CanFdHandCommVcan, Bc956_QueuedSetIsTakenAfterTheBoundPassed) {
  auto right_peer = StartPeer(HandSide::kRight, FakePeer::Mode::kDropFrame2, 0ms, CmdAck::kNone);
  auto left_peer = StartPeer(HandSide::kLeft, FakePeer::Mode::kNormal, 0ms, CmdAck::kNone);
  auto right = OpenComm(BcConfig(HandSide::kRight));
  auto left = OpenComm(BcConfig(HandSide::kLeft));
  HandReadOutcome::Clock::time_point t_req{};
  ASSERT_TRUE(right->SendBroadcastReq(t_req));
  const SharedRequest req{t_req, true, false};
  HandReadBuffers bufs{};
  HandReadOutcome ro{};
  HandReadOutcome lo{};
  right->ReceiveSet(SensorPlan(false), bufs, ro, req);
  EXPECT_FALSE(ro.joint_ok);
  const auto t_left = std::chrono::steady_clock::now();
  left->ReceiveSet(SensorPlan(false), bufs, lo, req);
  EXPECT_TRUE(lo.joint_ok);
  EXPECT_LT(std::chrono::steady_clock::now() - t_left, 5ms);  // no wait: it was queued
  left->PublishCommStats();
  EXPECT_EQ(Kind(left->comm_stats(), RequestKind::kJointRead).timeout, 0u);
}

// After a write the bound is REQ + 2T for every hand (D83), also for a hand
// that did not write itself; the write result survives ReceiveSet (S1).
TEST_F(CanFdHandCommVcan, Bc956_AfterAWriteEveryHandGetsTwoT) {
  const auto kT = std::chrono::milliseconds(static_cast<int>(kStageTimeoutMs));
  auto right_peer = StartPeer(HandSide::kRight, FakePeer::Mode::kNormal, 0ms, CmdAck::kNone);
  auto left_peer =
      StartPeer(HandSide::kLeft, FakePeer::Mode::kDelayedResp, kT * 3 / 2, CmdAck::kNone);
  auto right = OpenComm(BcConfig(HandSide::kRight));
  auto left = OpenComm(BcConfig(HandSide::kLeft));
  HandReadOutcome ro{};
  HandReadOutcome lo{};
  ASSERT_TRUE(right->SendCommandOnly(Ramp(0.0f, 0.1f), HandMotorArray{}, ro));
  EXPECT_TRUE(ro.write_ok);
  EXPECT_FALSE(ro.write_rx);
  const auto write_end = ro.write_end;
  HandReadOutcome::Clock::time_point t_req{};
  ASSERT_TRUE(right->SendBroadcastReq(t_req));
  const SharedRequest req{t_req, true, true};
  HandReadBuffers bufs{};
  right->ReceiveSet(SensorPlan(false), bufs, ro, req);
  left->ReceiveSet(SensorPlan(false), bufs, lo, req);
  EXPECT_TRUE(ro.joint_ok);
  EXPECT_TRUE(ro.write_ok);  // S1: kept through ReceiveSet
  EXPECT_EQ(ro.write_end, write_end);
  EXPECT_TRUE(lo.joint_ok);
  EXPECT_GE(lo.PhaseEnd(ReadPhase::kJoint) - t_req, kT * 14 / 10);
  right_peer->WithModel([](const FakeHandFirmware& fw) {
    EXPECT_FLOAT_EQ(fw.target()[9], 0.9f);  // the command reached the right hand
    return 0;
  });
  right->PublishCommStats();
  EXPECT_EQ(Kind(right->comm_stats(), RequestKind::kWriteEcho).ok, 1u);
}

// A REQ that never left: every hand's set is an error at once, not a timeout
// (S4), and no window opens.
TEST_F(CanFdHandCommVcan, Bc956_FailedReqIsASetErrorWithoutAWindow) {
  auto right = OpenComm(BcConfig(HandSide::kRight));
  auto left = OpenComm(BcConfig(HandSide::kLeft));
  const SharedRequest req{std::chrono::steady_clock::now(), false, false};
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  const auto t0 = std::chrono::steady_clock::now();
  right->ReceiveSet(SensorPlan(true), bufs, out, req);
  left->ReceiveSet(SensorPlan(true), bufs, out, req);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 5ms);
  EXPECT_FALSE(out.joint_ok);
  for (auto* comm : {right.get(), left.get()}) {
    comm->PublishCommStats();
    const UdpHandCommStats s = comm->comm_stats();
    EXPECT_EQ(Kind(s, RequestKind::kJointRead).error, 1u);
    EXPECT_EQ(Kind(s, RequestKind::kBulkSensor).error, 1u);
    EXPECT_EQ(s.recv_timeout, 0u);
  }
}

// Review #2: the pre-feed reads at most kMaxDrainPerCall queued frames, so a
// stream of duplicates cannot hold ReceiveSet past its bound — it then times
// out and drains the rest.
TEST_F(CanFdHandCommVcan, Bc956_PreFeedIsBoundedByTheDrainCap) {
  auto comm = OpenComm(BcConfig(HandSide::kRight));
  rtc::CanFdTransport injector(InjectorConfig());
  ASSERT_TRUE(injector.Open());
  constexpr int kExtra = 4;
  for (int i = 0; i < CanFdHandComm::kMaxDrainPerCall + kExtra; ++i)
    ASSERT_TRUE(injector.SendFrame(DataFrame(RespId(kNodeRightHand, 0), 0x01)));
  std::this_thread::sleep_for(5ms);  // let every injected frame reach the queue
  const auto t_req = std::chrono::steady_clock::now() - 1s;  // the bound is long gone
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  comm->ReceiveSet(SensorPlan(false), bufs, out, SharedRequest{t_req, true, false});
  EXPECT_FALSE(out.joint_ok);
  comm->PublishCommStats();
  const UdpHandCommStats s = comm->comm_stats();
  EXPECT_EQ(Kind(s, RequestKind::kJointRead).timeout, 1u);
  EXPECT_EQ(s.stale_drained, static_cast<uint64_t>(kExtra)) << "the pre-feed read past its cap";
}

}  // namespace udp_hand_driver::test
