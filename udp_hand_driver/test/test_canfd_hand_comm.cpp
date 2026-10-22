// ── test_canfd_hand_comm.cpp ────────────────────────────────────────────────
// CanFdHandComm (proto2 over CAN FD, #901) against the proto2 fake firmware
// behind a second CAN FD socket on vcan0, plus the Tier 1 cases that need no
// socket. The vcan cases skip (SKIP_IF_NO_FD_VCAN, rtc_communication's shared
// test header) when vcan0 is absent or not FD-capable; a skip is unverified.
//
// The cases use the real hand id space (right 0x01 / left 0x02 nodes), so they
// must not run while a proto2 SIL (fake_hand_firmware protocol_version 2) is
// on vcan0 — its echoes / RESP bursts would be read as stale frames here.
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
//   D26        E-Stop fire-and-forget: Frame 1 zeros then Frame 0 zeros
//   D51        a left-hand comm never talks to a right-hand fake
//   DrainStale bounded drain, stale_drained accounting
//   Tier 1     tx errno classification, closed link harmless, kRxMask bits,
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

FakeHandFirmware::Config FakeConfig(HandSide side) {
  FakeHandFirmware::Config cfg;
  cfg.num_fingertips = kDefaultNumFingertips;
  cfg.effort_stiffness = 2.0;
  cfg.sensor_base_min = kBaseMin;
  cfg.sensor_base_max = kBaseMax;
  cfg.sensor_noise_stddev = kNoiseStddev;
  cfg.seed = 7;
  cfg.hand_side = side;
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

// Sender with no filter (its reads are never used).
rtc::CanFdTransportConfig InjectorConfig() {
  rtc::CanFdTransportConfig cfg{};
  cfg.interface_name = kVcanInterface;
  cfg.extended_frame = true;
  return cfg;
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
// link. Mode kDropFrame2 withholds RESP idx 2 (a partial set every cycle);
// kReverse sends each answer's frames last-first (RESP 2 1 0); kSilent
// answers nothing. Model reads go through WithModel() (mutex — test
// code, not RT).
class FakePeer {
 public:
  enum class Mode { kNormal, kDropFrame2, kReverse, kSilent };

  explicit FakePeer(HandSide side, Mode mode = Mode::kNormal)
      : firmware_(FakeConfig(side)), mode_(mode), transport_(TargetedConfig(NodeIdOf(side))) {
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
  void Run() {
    canfd_frame rx{};
    Frames out{};
    const timespec poll{0, 5'000'000};  // 5 ms, re-checks stop_
    while (!stop_.load()) {
      const ssize_t n = transport_.RecvFrameWithTimeout(rx, poll);
      if (n <= 0)
        continue;
      std::size_t count = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        count = firmware_.BuildCanFdResponse(rx.can_id, rx.data, rx.len, out.data(), out.size());
      }
      if (mode_ == Mode::kSilent)
        continue;
      for (std::size_t k = 0; k < count; ++k) {
        const std::size_t i = (mode_ == Mode::kReverse) ? count - 1 - k : k;
        if (mode_ == Mode::kDropFrame2 &&
            MsgTypeOf(out[i].can_id) == static_cast<uint8_t>(MsgType::kResp) &&
            FrameIndexOf(out[i].can_id) == 2)
          continue;
        canfd_frame tx{};
        tx.can_id = out[i].can_id | CAN_EFF_FLAG;
        tx.len = static_cast<__u8>(out[i].len);
        tx.flags = CANFD_BRS;
        std::memcpy(tx.data, out[i].data.data(), out[i].len);
        (void)transport_.SendFrame(tx);
      }
    }
  }

  FakeHandFirmware firmware_;
  Mode mode_;
  rtc::CanFdTransport transport_;
  bool open_{false};
  std::mutex mutex_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

// Opens a comm, fails the test if vcan refuses.
class CanFdHandCommVcan : public ::testing::Test {
 protected:
  void SetUp() override { SKIP_IF_NO_FD_VCAN(); }

  std::unique_ptr<CanFdHandComm> OpenComm(CanFdHandCommConfig cfg = CommConfig()) {
    auto comm = std::make_unique<CanFdHandComm>(std::move(cfg));
    EXPECT_TRUE(comm->Open());
    return comm;
  }

  std::unique_ptr<FakePeer> StartPeer(HandSide side = HandSide::kRight,
                                      FakePeer::Mode mode = FakePeer::Mode::kNormal) {
    auto peer = std::make_unique<FakePeer>(side, mode);
    EXPECT_TRUE(peer->open());
    return peer;
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

  comm.PublishCommStats();
  const UdpHandCommStats s = comm.comm_stats();
  EXPECT_EQ(s.recv_ok, 0u);
  EXPECT_EQ(s.recv_timeout, 0u);
  EXPECT_EQ(s.recv_error, 0u);
  EXPECT_EQ(comm.recv_error_count(), 0u);
  const CanFdCommStats c = comm.canfd_stats();
  EXPECT_EQ(c.tx_other, 0u);
  EXPECT_EQ(c.error_frames, 0u);

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

}  // namespace udp_hand_driver::test
