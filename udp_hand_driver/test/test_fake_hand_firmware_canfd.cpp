// Unit tests for the proto2 (CAN FD) mirror of fake_hand_firmware.hpp —
// BuildCanFdResponse(): CMD echo + model latch (D41), feedforward keep (D50),
// REQ → RESP burst (D47 · D43), hand side → node id (D51), broadcast and the
// drop counters (D45). Tier 1: no socket, no vcan — frames are crafted with
// hand_canfd_packets.hpp and the burst is read back through the driver's own
// RespSetAssembler / DecodeTelemetry, so the mirror is checked against the
// decoder it exists to exercise.

#include "udp_hand_driver/fake_hand_firmware.hpp"
#include "udp_hand_driver/hand_canfd_packets.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace udp_hand_driver::test {

using namespace canfd;

namespace {

using Bytes = std::array<uint8_t, kFrameLen>;
using Motors = std::array<float, kNumHandMotors>;
using Frames = std::array<FakeHandFirmware::CanFdFrame, kNumRespFrames>;
using namespace std::chrono_literals;

constexpr int kBaseMin = 1000;
constexpr int kBaseMax = 2000;
constexpr double kNoiseStddev = 5.0;
// Every tactile count must land in base ± 6σ; the test fails long before a
// genuine outlier would (6σ at σ = 5 is one draw in ~5e8).
constexpr int kBand = static_cast<int>(6 * kNoiseStddev);
// Socket-flavoured can_id: the EFF flag the fake must strip (CAN_EFF_FLAG).
constexpr uint32_t kEffFlag = 0x80000000u;

FakeHandFirmware::Config DefaultConfig() {
  FakeHandFirmware::Config cfg;
  cfg.num_fingertips = kDefaultNumFingertips;
  cfg.lpf_time_constant_s = 0.1;
  cfg.effort_stiffness = 2.0;
  cfg.effort_damping = 0.1;
  cfg.step_rate_hz = 500.0;
  cfg.sensor_base_min = kBaseMin;
  cfg.sensor_base_max = kBaseMax;
  cfg.sensor_noise_stddev = kNoiseStddev;
  cfg.seed = 7;
  cfg.hand_side = HandSide::kRight;
  return cfg;
}

Motors Ramp(float start, float step) {
  Motors m{};
  for (std::size_t i = 0; i < m.size(); ++i)
    m[i] = start + step * static_cast<float>(i);
  return m;
}

// 64-byte CMD Frame 0 / 1 payload carrying `values`, reserved bytes zero.
Bytes CmdBytes(const Motors& values) {
  Bytes b{};
  SerializeFrame(MakeCmdMotorFrame(values), b.data());
  return b;
}

std::size_t Send(FakeHandFirmware& fw, uint32_t id, const uint8_t* data, std::size_t len,
                 Frames& out) {
  return fw.BuildCanFdResponse(id, data, len, out.data(), out.size());
}

std::size_t SendCmd(FakeHandFirmware& fw, uint8_t node, uint8_t idx, const Bytes& b, Frames& out) {
  return Send(fw, CmdId(node, idx), b.data(), b.size(), out);
}

std::size_t SendReq(FakeHandFirmware& fw, uint8_t node, Frames& out) {
  return Send(fw, ReqId(node), nullptr, 0, out);
}

// Feed a burst to the driver's assembler and decode it.
void ReadBack(uint8_t node, const Frames& burst, Telemetry& t) {
  RespSetAssembler a(node);
  a.Begin(RespSetAssembler::Clock::time_point{} + 1ms);
  for (const auto& f : burst)
    (void)a.Feed(f.can_id, f.data.data(), f.len);
  ASSERT_TRUE(a.complete());
  DecodeTelemetry(a.set(), t);
}

void ExpectBareId(const FakeHandFirmware::CanFdFrame& f, uint32_t id) {
  EXPECT_EQ(f.can_id, id);
  EXPECT_EQ(f.can_id & ~kExtIdMask, 0u) << "bare 29-bit id — the socket shell adds the flags";
  EXPECT_EQ(f.len, kFrameLen);
}

// Raw counts of an active fingertip: base ± noise on fx / px / temp, the
// negated band on fy / fz / py (the fake pre-applies the Proto §4.3 flips).
void ExpectInBand(const TactileRaw& r) {
  for (int v : {r.fx, r.px, r.temp}) {
    EXPECT_GE(v, kBaseMin - kBand);
    EXPECT_LE(v, kBaseMax + kBand);
  }
  for (int v : {r.fy, r.fz, r.py}) {
    EXPECT_GE(v, -(kBaseMax + kBand));
    EXPECT_LE(v, -(kBaseMin - kBand));
  }
  EXPECT_EQ(r.contact, 1u);
}

void ExpectZero(const TactileRaw& r) {
  EXPECT_EQ(r.fx, 0);
  EXPECT_EQ(r.fy, 0);
  EXPECT_EQ(r.fz, 0);
  EXPECT_EQ(r.px, 0);
  EXPECT_EQ(r.py, 0);
  EXPECT_EQ(r.temp, 0);
  EXPECT_EQ(r.contact, 0u);
}

void ExpectNoDrops(const FakeHandFirmware::CanFdCounters& c) {
  EXPECT_EQ(c.peer_traffic, 0u);
  EXPECT_EQ(c.req_with_payload, 0u);
  EXPECT_EQ(c.error_frames, 0u);
  EXPECT_EQ(c.other_target, 0u);
  EXPECT_EQ(c.unknown_type, 0u);
  EXPECT_EQ(c.bad_length, 0u);
  EXPECT_EQ(c.cmd_ignored, 0u);
}

}  // namespace

// ── CMD Frame 0: echo + latch + one model step (D41) ────────────────────────

TEST(FakeHandFirmwareCanFd, CmdFrame0_EchoesVerbatim_LatchesTarget_StepsOnce) {
  FakeHandFirmware fw(DefaultConfig());
  const Motors target = Ramp(0.1f, 0.1f);
  const Bytes cmd = CmdBytes(target);
  Frames out{};

  ASSERT_EQ(SendCmd(fw, kNodeRightHand, 0, cmd, out), 1u);
  ExpectBareId(out[0], EchoId(kNodeRightHand, 0));
  EXPECT_EQ(std::memcmp(out[0].data.data(), cmd.data(), kFrameLen), 0);

  EXPECT_EQ(fw.target(), target);
  // Exactly one LPF step from rest: pos = alpha · target.
  const float alpha = static_cast<float>(fw.alpha());
  for (std::size_t i = 0; i < kNumHandMotors; ++i)
    EXPECT_FLOAT_EQ(fw.pos()[i], alpha * target[i]);
  EXPECT_EQ(fw.feedforward_count(), 0u);
  ExpectNoDrops(fw.canfd_counters());
}

TEST(FakeHandFirmwareCanFd, CmdFrame0_WithSocketEffFlag_StillAccepted) {
  FakeHandFirmware fw(DefaultConfig());
  const Bytes cmd = CmdBytes(Ramp(0.5f, 0.0f));
  Frames out{};
  ASSERT_EQ(Send(fw, kEffFlag | CmdId(kNodeRightHand, 0), cmd.data(), cmd.size(), out), 1u);
  ExpectBareId(out[0], EchoId(kNodeRightHand, 0));
  EXPECT_FLOAT_EQ(fw.target()[0], 0.5f);
}

// ── CMD Frame 1: echo + keep, no model effect (D50) ─────────────────────────

TEST(FakeHandFirmwareCanFd, CmdFrame1_EchoesVerbatim_KeepsFeedforward_ModelUnchanged) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  ASSERT_EQ(SendCmd(fw, kNodeRightHand, 0, CmdBytes(Ramp(1.0f, 0.0f)), out), 1u);
  const Motors pos_before = fw.pos();
  const Motors vel_before = fw.vel();
  const Motors target_before = fw.target();

  const Motors ff = Ramp(0.05f, 0.01f);
  const Bytes cmd = CmdBytes(ff);
  ASSERT_EQ(SendCmd(fw, kNodeRightHand, 1, cmd, out), 1u);
  ExpectBareId(out[0], EchoId(kNodeRightHand, 1));
  EXPECT_EQ(std::memcmp(out[0].data.data(), cmd.data(), kFrameLen), 0);

  EXPECT_EQ(fw.last_feedforward(), ff);
  EXPECT_EQ(fw.feedforward_count(), 1u);
  EXPECT_EQ(fw.pos(), pos_before) << "Frame 1 must not step the model";
  EXPECT_EQ(fw.vel(), vel_before);
  EXPECT_EQ(fw.target(), target_before);

  ASSERT_EQ(SendCmd(fw, kNodeRightHand, 1, CmdBytes(Ramp(0.0f, 0.0f)), out), 1u);
  EXPECT_EQ(fw.feedforward_count(), 2u);
  EXPECT_FLOAT_EQ(fw.last_feedforward()[9], 0.0f);
}

// ── cmd_ack none: applied, nothing sent back (#945 D65) ─────────────────────

TEST(FakeHandFirmwareCanFd, CmdFrame0_NoneAck_AppliesWithoutEcho) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.cmd_ack = CmdAck::kNone;
  FakeHandFirmware fw(cfg);
  const Motors target = Ramp(0.1f, 0.1f);
  Frames out{};

  EXPECT_EQ(SendCmd(fw, kNodeRightHand, 0, CmdBytes(target), out), 0u);
  EXPECT_EQ(out[0].len, 0u) << "no echo written";
  EXPECT_EQ(fw.target(), target);
  const float alpha = static_cast<float>(fw.alpha());
  for (std::size_t i = 0; i < kNumHandMotors; ++i)
    EXPECT_FLOAT_EQ(fw.pos()[i], alpha * target[i]);
  ExpectNoDrops(fw.canfd_counters());
  // The REQ is answered as under echo.
  EXPECT_EQ(SendReq(fw, kNodeRightHand, out), kNumRespFrames);
}

TEST(FakeHandFirmwareCanFd, CmdFrame1_NoneAck_KeepsFeedforwardWithoutEcho) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.cmd_ack = CmdAck::kNone;
  FakeHandFirmware fw(cfg);
  const Motors ff = Ramp(0.05f, 0.01f);
  Frames out{};

  EXPECT_EQ(SendCmd(fw, kNodeRightHand, 1, CmdBytes(ff), out), 0u);
  EXPECT_EQ(out[0].len, 0u) << "no echo written";
  EXPECT_EQ(fw.last_feedforward(), ff);
  EXPECT_EQ(fw.feedforward_count(), 1u);
  ExpectNoDrops(fw.canfd_counters());
}

TEST(FakeHandFirmwareCanFd, Cmd_ZeroCapacity_StillApplies) {
  FakeHandFirmware fw(DefaultConfig());
  const Motors target = Ramp(0.2f, 0.0f);
  const Bytes cmd = CmdBytes(target);
  Frames out{};
  EXPECT_EQ(fw.BuildCanFdResponse(CmdId(kNodeRightHand, 0), cmd.data(), cmd.size(), out.data(), 0),
            0u);
  EXPECT_EQ(out[0].len, 0u);
  EXPECT_EQ(fw.target(), target) << "the command is applied though no echo fits";
}

// ── Hand side → node id (D51) · broadcast · other target ────────────────────

TEST(FakeHandFirmwareCanFd, LeftHand_AnswersAsNode2_IgnoresRightTarget) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.hand_side = HandSide::kLeft;
  FakeHandFirmware fw(cfg);
  const Bytes cmd = CmdBytes(Ramp(0.3f, 0.0f));
  Frames out{};

  // Addressed to the right hand: not ours.
  EXPECT_EQ(SendCmd(fw, kNodeRightHand, 0, cmd, out), 0u);
  EXPECT_EQ(fw.canfd_counters().other_target, 1u);
  EXPECT_FLOAT_EQ(fw.target()[0], 0.0f);
  EXPECT_EQ(SendReq(fw, kNodeRightHand, out), 0u);
  EXPECT_EQ(fw.canfd_counters().other_target, 2u);

  // Addressed to the left hand: echo and RESP carry source = node 2.
  ASSERT_EQ(SendCmd(fw, kNodeLeftHand, 0, cmd, out), 1u);
  ExpectBareId(out[0], EchoId(kNodeLeftHand, 0));
  EXPECT_EQ(SourceOf(out[0].can_id), kNodeLeftHand);
  EXPECT_EQ(TargetOf(out[0].can_id), kNodeHost);
  ASSERT_EQ(SendReq(fw, kNodeLeftHand, out), kNumRespFrames);
  for (std::size_t i = 0; i < kNumRespFrames; ++i)
    ExpectBareId(out[i], RespId(kNodeLeftHand, static_cast<uint8_t>(i)));
}

TEST(FakeHandFirmwareCanFd, Broadcast_IsAnswered_WithOwnNodeAsSource) {
  FakeHandFirmware fw(DefaultConfig());
  const Bytes cmd = CmdBytes(Ramp(0.2f, 0.0f));
  Frames out{};
  ASSERT_EQ(SendCmd(fw, kNodeBroadcast, 0, cmd, out), 1u);
  ExpectBareId(out[0], EchoId(kNodeRightHand, 0));
  EXPECT_FLOAT_EQ(fw.target()[3], 0.2f);
  ASSERT_EQ(SendReq(fw, kNodeBroadcast, out), kNumRespFrames);
  ExpectBareId(out[0], RespId(kNodeRightHand, 0));
  ExpectNoDrops(fw.canfd_counters());
}

// ── REQ → RESP burst (D47 · D43) read back through the driver's decoder ─────

TEST(FakeHandFirmwareCanFd, Req_ThreeFrames_DecodeMatchesJointModel) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  // Give the model something non-trivial: two steps toward a ramp.
  const Bytes cmd = CmdBytes(Ramp(0.5f, 0.1f));
  ASSERT_EQ(SendCmd(fw, kNodeRightHand, 0, cmd, out), 1u);
  ASSERT_EQ(SendCmd(fw, kNodeRightHand, 0, cmd, out), 1u);
  const Motors pos = fw.pos();
  const Motors vel = fw.vel();
  const Motors effort = fw.effort();

  ASSERT_EQ(SendReq(fw, kNodeRightHand, out), kNumRespFrames);
  for (std::size_t i = 0; i < kNumRespFrames; ++i)
    ExpectBareId(out[i], RespId(kNodeRightHand, static_cast<uint8_t>(i)));

  Telemetry t{};
  ASSERT_NO_FATAL_FAILURE(ReadBack(kNodeRightHand, out, t));
  for (std::size_t i = 0; i < kWireMotorCount; ++i) {
    EXPECT_FLOAT_EQ(t.pos[i], pos[i]) << "motor " << i;
    EXPECT_FLOAT_EQ(t.vel[i], vel[i]) << "motor " << i;
    EXPECT_FLOAT_EQ(t.cur[i], effort[i]) << "motor " << i;
  }
  EXPECT_NE(t.vel[kVelLoCount], 0.0f) << "the F0/F1 velocity split carries M6..M9";

  // A read leaves the model where it was.
  EXPECT_EQ(fw.pos(), pos);
  EXPECT_EQ(fw.vel(), vel);
  ExpectNoDrops(fw.canfd_counters());
}

TEST(FakeHandFirmwareCanFd, Req_ActiveFingertips_TactileInBand_WithContact) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  ASSERT_EQ(SendReq(fw, kNodeRightHand, out), kNumRespFrames);

  RespFrame1 f1{};
  RespFrame2 f2{};
  ASSERT_TRUE(DecodeFrame(out[1].data.data(), out[1].len, f1));
  ASSERT_TRUE(DecodeFrame(out[2].data.data(), out[2].len, f2));
  ExpectInBand(ThumbRaw(f1, f2));
  ExpectInBand(f2.index);
  ExpectInBand(f2.middle);
  ExpectInBand(f2.ring);
  for (uint8_t b : f2.reserved)
    EXPECT_EQ(b, 0u);

  // After the host's Proto §4.3 decode every channel is positive, like the
  // 1b fake's — a SIL force threshold behaves the same on both wires.
  Telemetry t{};
  ASSERT_NO_FATAL_FAILURE(ReadBack(kNodeRightHand, out, t));
  const float fx_lo = static_cast<float>(kBaseMin - kBand) * kForceXyRangeN / kTactileFullScale;
  for (const Tactile& tac : t.tactile) {
    EXPECT_GE(tac.fx_n, fx_lo);
    EXPECT_GT(tac.fy_n, 0.0f);
    EXPECT_GT(tac.fz_n, 0.0f);
    EXPECT_GT(tac.px_m, 0.0f);
    EXPECT_GT(tac.py_m, 0.0f);
    EXPECT_GT(tac.temp_c, 0.0f);
    EXPECT_TRUE(tac.contact);
  }
}

TEST(FakeHandFirmwareCanFd, Req_InactiveFingertips_AllZero_NoContact) {
  FakeHandFirmware::Config cfg = DefaultConfig();
  cfg.num_fingertips = 2;  // thumb + index live, middle + ring off
  FakeHandFirmware fw(cfg);
  Frames out{};
  ASSERT_EQ(SendReq(fw, kNodeRightHand, out), kNumRespFrames);

  RespFrame1 f1{};
  RespFrame2 f2{};
  ASSERT_TRUE(DecodeFrame(out[1].data.data(), out[1].len, f1));
  ASSERT_TRUE(DecodeFrame(out[2].data.data(), out[2].len, f2));
  ExpectInBand(ThumbRaw(f1, f2));
  ExpectInBand(f2.index);
  ExpectZero(f2.middle);
  ExpectZero(f2.ring);

  Telemetry t{};
  ASSERT_NO_FATAL_FAILURE(ReadBack(kNodeRightHand, out, t));
  EXPECT_FALSE(t.tactile[static_cast<std::size_t>(Fingertip::kMiddle)].contact);
  EXPECT_FLOAT_EQ(t.tactile[static_cast<std::size_t>(Fingertip::kRing)].fz_n, 0.0f);
}

TEST(FakeHandFirmwareCanFd, Req_LengthIgnored_ClassicFallbackStillAnswered) {
  // D47: the REQ is DLC 0, but a classic can_frame fallback or a stray
  // payload must not silence the hand.
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  const Bytes junk = CmdBytes(Ramp(9.0f, 0.0f));
  EXPECT_EQ(Send(fw, ReqId(kNodeRightHand), junk.data(), 8, out), kNumRespFrames);
  EXPECT_EQ(Send(fw, ReqId(kNodeRightHand), junk.data(), junk.size(), out), kNumRespFrames);
  EXPECT_FLOAT_EQ(fw.target()[0], 0.0f) << "a REQ payload is not a command";
  EXPECT_EQ(fw.canfd_counters().req_with_payload, 2u) << "…but the harness can see it";
  EXPECT_EQ(SendReq(fw, kNodeRightHand, out), kNumRespFrames);
  EXPECT_EQ(fw.canfd_counters().req_with_payload, 2u);
}

TEST(FakeHandFirmwareCanFd, Req_CapBelowBurst_WritesNothing) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  EXPECT_EQ(fw.BuildCanFdResponse(ReqId(kNodeRightHand), nullptr, 0, out.data(), 2), 0u);
  EXPECT_EQ(out[0].len, 0u) << "no partial set (D43)";
  EXPECT_EQ(fw.BuildCanFdResponse(ReqId(kNodeRightHand), nullptr, 0, nullptr, 0), 0u);
  ExpectNoDrops(fw.canfd_counters());  // capacity is the caller's bug, not a wire drop
  EXPECT_EQ(SendReq(fw, kNodeRightHand, out), kNumRespFrames);
}

TEST(FakeHandFirmwareCanFd, SameSeed_SameBurst) {
  FakeHandFirmware a(DefaultConfig());
  FakeHandFirmware b(DefaultConfig());
  Frames fa{};
  Frames fb{};
  for (int k = 0; k < 3; ++k) {
    ASSERT_EQ(SendReq(a, kNodeRightHand, fa), kNumRespFrames);
    ASSERT_EQ(SendReq(b, kNodeRightHand, fb), kNumRespFrames);
    for (std::size_t i = 0; i < kNumRespFrames; ++i)
      EXPECT_EQ(fa[i].data, fb[i].data) << "burst " << k << " frame " << i;
  }
  // Noise is live: two bursts of one hand differ somewhere in the tactile bytes.
  Frames fa2{};
  ASSERT_EQ(SendReq(a, kNodeRightHand, fa2), kNumRespFrames);
  EXPECT_NE(fa2[2].data, fa[2].data);
}

// ── Drops: Frame 2 / 3, error frames, bad length, unknown type (D45) ────────

TEST(FakeHandFirmwareCanFd, CmdFrame2And3_NoEcho_NoModel_Counted) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  const Bytes cmd = CmdBytes(Ramp(0.7f, 0.0f));
  EXPECT_EQ(SendCmd(fw, kNodeRightHand, static_cast<uint8_t>(CmdFrame::kTargetCurrent), cmd, out),
            0u);
  EXPECT_EQ(SendCmd(fw, kNodeRightHand, static_cast<uint8_t>(CmdFrame::kHandSide), cmd, out), 0u);
  EXPECT_EQ(SendCmd(fw, kNodeRightHand, 17, cmd, out), 0u);
  EXPECT_EQ(fw.canfd_counters().cmd_ignored, 3u);
  EXPECT_EQ(fw.canfd_counters().bad_length, 0u);
  EXPECT_FLOAT_EQ(fw.target()[0], 0.0f);
  EXPECT_FLOAT_EQ(fw.pos()[0], 0.0f);
  EXPECT_EQ(fw.feedforward_count(), 0u);
}

TEST(FakeHandFirmwareCanFd, ErrorFrame_RejectedBeforeIdIsRead) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  const Bytes cmd = CmdBytes(Ramp(0.7f, 0.0f));
  // Bus-off error frame whose low bits look like our own echo id.
  EXPECT_EQ(Send(fw, kErrorFrameFlag | EchoId(kNodeRightHand, 0), cmd.data(), 8, out), 0u);
  // …and one whose low bits look like a CMD addressed to us.
  EXPECT_EQ(Send(fw, kErrorFrameFlag | CmdId(kNodeRightHand, 0), cmd.data(), cmd.size(), out), 0u);
  EXPECT_EQ(fw.canfd_counters().error_frames, 2u);
  EXPECT_EQ(fw.canfd_counters().other_target, 0u);
  EXPECT_FLOAT_EQ(fw.target()[0], 0.0f);
}

TEST(FakeHandFirmwareCanFd, CmdBadLength_Dropped_Counted) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  const Bytes cmd = CmdBytes(Ramp(0.7f, 0.0f));
  EXPECT_EQ(Send(fw, CmdId(kNodeRightHand, 0), cmd.data(), 8, out), 0u);
  EXPECT_EQ(Send(fw, CmdId(kNodeRightHand, 1), cmd.data(), 0, out), 0u);
  EXPECT_EQ(Send(fw, CmdId(kNodeRightHand, 0), nullptr, kFrameLen, out), 0u);
  EXPECT_EQ(fw.canfd_counters().bad_length, 3u);
  EXPECT_FLOAT_EQ(fw.target()[0], 0.0f);
  EXPECT_EQ(fw.feedforward_count(), 0u);
}

TEST(FakeHandFirmwareCanFd, UnknownType_Dropped_Counted) {
  FakeHandFirmware fw(DefaultConfig());
  Frames out{};
  const Bytes cmd = CmdBytes(Ramp(0.7f, 0.0f));
  // The other hand's RESP and echo, addressed to the host: normal bus traffic.
  EXPECT_EQ(Send(fw, RespId(kNodeLeftHand, 0), cmd.data(), cmd.size(), out), 0u);
  EXPECT_EQ(Send(fw, EchoId(kNodeLeftHand, 1), cmd.data(), cmd.size(), out), 0u);
  EXPECT_EQ(fw.canfd_counters().peer_traffic, 2u);
  EXPECT_EQ(fw.canfd_counters().other_target, 0u);
  // A RESP (type 2) and an undefined type (0xF) addressed to us.
  EXPECT_EQ(Send(fw, MakeCanId(0, MsgType::kResp, kNodeRightHand, kNodeHost, 0), cmd.data(),
                 cmd.size(), out),
            0u);
  EXPECT_EQ(Send(fw, MakeCanId(0, static_cast<MsgType>(0xF), kNodeRightHand, kNodeHost, 0),
                 cmd.data(), cmd.size(), out),
            0u);
  EXPECT_EQ(fw.canfd_counters().unknown_type, 2u);
  EXPECT_FLOAT_EQ(fw.target()[0], 0.0f);
}

// ── G-1: the UDP path is untouched by the proto2 additions ──────────────────

TEST(FakeHandFirmwareCanFd, UdpBuildResponse_StillAnswersProto1) {
  FakeHandFirmware fw(DefaultConfig());
  const std::array<uint8_t, packets::kHeaderSize> req = {
      packets::kDeviceId, static_cast<uint8_t>(packets::Command::kReadAllMotors), 0};
  std::array<uint8_t, packets::kMaxPacketSize> out{};
  EXPECT_EQ(fw.BuildResponse(req.data(), req.size(), out.data(), out.size()),
            packets::kAllMotorResponseSize);
  ExpectNoDrops(fw.canfd_counters());
}

}  // namespace udp_hand_driver::test
