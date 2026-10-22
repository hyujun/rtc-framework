// Unit tests for hand_canfd_packets.hpp — proto2 CAN ids, 64-byte frame
// layouts, tactile scaling (Proto §4.3) and the RESP set assembler.
//
// Tier 1: pure computation — no ROS 2, no socket, no vcan. The vendor
// documents' worked examples (Proto §5.2 Frame 0, §5.4 RESP F0/F1/F2) are
// the byte-level oracles.

#include "udp_hand_driver/hand_canfd_packets.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace udp_hand_driver::canfd::test {

// The wire header pins its own counts (byte pure, no driver include); the
// driver's hand constants must agree.
static_assert(kWireMotorCount == static_cast<std::size_t>(kNumHandMotors),
              "proto2 wire motor count == driver kNumHandMotors");
static_assert(kWireFingertipCount == static_cast<std::size_t>(kDefaultNumFingertips),
              "proto2 wire fingertip count == driver kDefaultNumFingertips");

namespace {

using Bytes = std::array<uint8_t, kFrameLen>;
using Clock = RespSetAssembler::Clock;
using namespace std::chrono_literals;

// A frame whose every byte is `fill` — distinct per frame so a mis-slotted
// store is visible.
Bytes Filled(uint8_t fill) {
  Bytes b{};
  b.fill(fill);
  return b;
}

void PutF32(Bytes& b, std::size_t at, float v) {
  std::memcpy(b.data() + at, &v, sizeof(v));
}

void PutI16(Bytes& b, std::size_t at, int16_t v) {
  std::memcpy(b.data() + at, &v, sizeof(v));
}

// Proto §5.4 (1) RESP F0: M0=0.5, M1=-1.0, M2=1.57, rest 0; velocities 0.
Bytes SpecRespF0() {
  Bytes b{};
  PutF32(b, 0, 0.5f);
  PutF32(b, 4, -1.0f);
  PutF32(b, 8, 1.57f);
  return b;
}

// Proto §5.4 (2) RESP F1: M0=1.2 A, M1=0.8 A; thumb Fx=100 Fy=-100 Fz=1000 Px=288.
Bytes SpecRespF1() {
  Bytes b{};
  PutF32(b, 16, 1.2f);
  PutF32(b, 20, 0.8f);
  const uint8_t thumb[8] = {0x64, 0x00, 0x9C, 0xFF, 0xE8, 0x03, 0x20, 0x01};
  std::memcpy(b.data() + 56, thumb, sizeof(thumb));
  return b;
}

// Proto §5.4 (3) RESP F2: thumb Py=16 Temp=24 Contact=1; index Temp=23;
// middle / ring Temp=24; contacts 0; reserved 0.
Bytes SpecRespF2() {
  Bytes b{};
  PutI16(b, 0, 16);
  PutI16(b, 2, 24);
  b[4] = 1;
  PutI16(b, 5 + 10, 23);   // index temp
  PutI16(b, 18 + 10, 24);  // middle temp
  PutI16(b, 31 + 10, 24);  // ring temp
  return b;
}

FeedResult FeedRight(RespSetAssembler& a, uint8_t idx, const Bytes& b) {
  return a.Feed(RespId(kNodeRightHand, idx), b.data(), b.size());
}

}  // namespace

// ── CAN identifier ──────────────────────────────────────────────────────────

TEST(HandCanFdId, SpecWorkedExamples) {
  // Proto §3 / Frame §2.2 — host → hand
  EXPECT_EQ(CmdId(kNodeRightHand, 0), 0x00004000u);
  EXPECT_EQ(CmdId(kNodeRightHand, 1), 0x00004001u);
  EXPECT_EQ(CmdId(kNodeRightHand, 2), 0x00004002u);
  EXPECT_EQ(CmdId(kNodeRightHand, 3), 0x00004003u);
  EXPECT_EQ(CmdId(kNodeLeftHand, 0), 0x00008000u);
  EXPECT_EQ(CmdId(kNodeLeftHand, 3), 0x00008003u);
  EXPECT_EQ(CmdId(kNodeBroadcast, 0), 0x003FC000u);
  EXPECT_EQ(CmdId(kNodeBroadcast, 3), 0x003FC003u);
  EXPECT_EQ(ReqId(kNodeRightHand), 0x00404000u);
  EXPECT_EQ(ReqId(kNodeLeftHand), 0x00408000u);
  // Proto §3.6 — echo
  EXPECT_EQ(EchoId(kNodeRightHand, 0), 0x00000040u);
  EXPECT_EQ(EchoId(kNodeRightHand, 3), 0x00000043u);
  EXPECT_EQ(EchoId(kNodeLeftHand, 0), 0x00000080u);
  EXPECT_EQ(EchoId(kNodeLeftHand, 3), 0x00000083u);
  // Proto §4 / Frame §4.1 — RESP
  EXPECT_EQ(RespId(kNodeRightHand, 0), 0x00800040u);
  EXPECT_EQ(RespId(kNodeRightHand, 1), 0x00800041u);
  EXPECT_EQ(RespId(kNodeRightHand, 2), 0x00800042u);
  EXPECT_EQ(RespId(kNodeLeftHand, 0), 0x00800080u);
  EXPECT_EQ(RespId(kNodeLeftHand, 2), 0x00800082u);
}

TEST(HandCanFdId, FieldExtractorsRoundTrip) {
  const uint32_t id = MakeCanId(5, MsgType::kResp, 0xAB, 0xCD, 0x2A);
  EXPECT_EQ(PriorityOf(id), 5u);
  EXPECT_EQ(MsgTypeOf(id), static_cast<uint8_t>(MsgType::kResp));
  EXPECT_EQ(TargetOf(id), 0xABu);
  EXPECT_EQ(SourceOf(id), 0xCDu);
  EXPECT_EQ(FrameIndexOf(id), 0x2Au);
  EXPECT_EQ(id & ~kExtIdMask, 0u) << "29 bits only";
}

TEST(HandCanFdId, FieldsAreMaskedToWidth) {
  // Over-wide inputs do not bleed into neighbouring fields (Proto §2.2 macro).
  EXPECT_EQ(MakeCanId(0xFF, MsgType::kCmd, 0, 0, 0), 0x07u << kPriorityShift);
  EXPECT_EQ(MakeCanId(0, MsgType::kCmd, 0, 0, 0xFF), kFrameIndexMask);
}

TEST(HandCanFdId, RespMaskSelectsOneNodeBurst) {
  EXPECT_EQ(kRespIdMask, 0x1FFFFFC0u);
  for (uint8_t idx = 0; idx < kNumRespFrames; ++idx) {
    EXPECT_EQ(RespId(kNodeRightHand, idx) & kRespIdMask, RespId(kNodeRightHand, 0));
    EXPECT_NE(RespId(kNodeLeftHand, idx) & kRespIdMask, RespId(kNodeRightHand, 0));
  }
  EXPECT_NE(EchoId(kNodeRightHand, 0) & kRespIdMask, RespId(kNodeRightHand, 0));
  EXPECT_NE(CmdId(kNodeRightHand, 0) & kRespIdMask, RespId(kNodeRightHand, 0));
}

TEST(HandCanFdId, ErrorFrameFlagAndMaskDerivation) {
  EXPECT_EQ(kErrorFrameFlag, 0x20000000u);
  EXPECT_TRUE(IsErrorFrame(kErrorFrameFlag | 0x40u));
  EXPECT_FALSE(IsErrorFrame(0x80000000u | RespId(kNodeRightHand, 0)));
  // A bus-off error frame masked to 29 bits would read as the Frame 0 echo —
  // the reason the flag is tested first.
  EXPECT_EQ((kErrorFrameFlag | 0x40u) & kExtIdMask, EchoId(kNodeRightHand, 0));
  EXPECT_EQ(kRespSetMask, (1u << kNumRespFrames) - 1u);
  EXPECT_EQ(FrameBit(0) | FrameBit(1) | FrameBit(2), kRespSetMask);
}

TEST(HandCanFdId, FrameBitHasNoBitPastTheBurst) {
  EXPECT_EQ(FrameBit(0), 0x01u);
  EXPECT_EQ(FrameBit(2), 0x04u);
  EXPECT_EQ(FrameBit(3), 0u);
  EXPECT_EQ(FrameBit(10), 0u);
  EXPECT_EQ(FrameBit(63), 0u);
}

TEST(HandCanFdId, ArbitrationOrderW2) {
  // echo < CMD < REQ < RESP right < RESP left (lower id wins)
  EXPECT_LT(EchoId(kNodeLeftHand, 3), CmdId(kNodeRightHand, 0));
  EXPECT_LT(CmdId(kNodeLeftHand, 3), ReqId(kNodeRightHand));
  EXPECT_LT(ReqId(kNodeLeftHand), RespId(kNodeRightHand, 0));
  EXPECT_LT(RespId(kNodeRightHand, 2), RespId(kNodeLeftHand, 0));
}

// ── Hand side → node id (D51) ───────────────────────────────────────────────

TEST(HandCanFdId, HandSide_MapsToNodeId) {
  EXPECT_EQ(NodeIdOf(HandSide::kRight), kNodeRightHand);
  EXPECT_EQ(NodeIdOf(HandSide::kLeft), kNodeLeftHand);
  EXPECT_EQ(CmdId(NodeIdOf(HandSide::kLeft), 0), 0x00008000u);
}

TEST(HandCanFdId, ParseHandSide_ExactSpellingsOnly) {
  HandSide side = HandSide::kRight;
  EXPECT_TRUE(ParseHandSide("left", side));
  EXPECT_EQ(side, HandSide::kLeft);
  EXPECT_TRUE(ParseHandSide("right", side));
  EXPECT_EQ(side, HandSide::kRight);
  side = HandSide::kLeft;
  EXPECT_FALSE(ParseHandSide("Right", side));
  EXPECT_FALSE(ParseHandSide("1", side));
  EXPECT_FALSE(ParseHandSide("", side));
  EXPECT_EQ(side, HandSide::kLeft) << "a failed parse leaves `out` alone";
}

// ── Frame layouts ───────────────────────────────────────────────────────────

TEST(HandCanFdLayout, Sizes) {
  EXPECT_EQ(kFrameLen, 64u);
  EXPECT_EQ(sizeof(CmdMotorFrame), 64u);
  EXPECT_EQ(sizeof(RespFrame0), 64u);
  EXPECT_EQ(sizeof(RespFrame1), 64u);
  EXPECT_EQ(sizeof(RespFrame2), 64u);
  EXPECT_EQ(sizeof(TactileRaw), 13u);
  EXPECT_EQ(sizeof(RespSet), 192u);
  EXPECT_EQ(kWireMotorCount, 10u);
  EXPECT_EQ(kWireFingertipCount, 4u);
  EXPECT_EQ(kVelLoCount + kVelHiCount, kWireMotorCount);
}

TEST(HandCanFdLayout, Offsets) {
  EXPECT_EQ(offsetof(CmdMotorFrame, reserved), 40u);
  EXPECT_EQ(offsetof(RespFrame0, vel_lo), 40u);
  EXPECT_EQ(offsetof(RespFrame1, vel_hi), 0u);
  EXPECT_EQ(offsetof(RespFrame1, cur), 16u);
  EXPECT_EQ(offsetof(RespFrame1, thumb_fx), 56u);
  EXPECT_EQ(offsetof(RespFrame1, thumb_px), 62u);
  EXPECT_EQ(offsetof(RespFrame2, thumb_py), 0u);
  EXPECT_EQ(offsetof(RespFrame2, thumb_temp), 2u);
  EXPECT_EQ(offsetof(RespFrame2, thumb_contact), 4u);
  EXPECT_EQ(offsetof(RespFrame2, index), 5u);
  EXPECT_EQ(offsetof(RespFrame2, middle), 18u);
  EXPECT_EQ(offsetof(RespFrame2, ring), 31u);
  EXPECT_EQ(offsetof(RespFrame2, reserved), 44u);
  EXPECT_EQ(offsetof(TactileRaw, temp), 10u);
  EXPECT_EQ(offsetof(TactileRaw, contact), 12u);
}

TEST(HandCanFdLayout, FingertipOrderMatchesDefaultNames) {
  ASSERT_EQ(kDefaultFingertipNames.size(), kWireFingertipCount);
  EXPECT_EQ(kDefaultFingertipNames[static_cast<std::size_t>(Fingertip::kThumb)], "thumb");
  EXPECT_EQ(kDefaultFingertipNames[static_cast<std::size_t>(Fingertip::kIndex)], "index");
  EXPECT_EQ(kDefaultFingertipNames[static_cast<std::size_t>(Fingertip::kMiddle)], "middle");
  EXPECT_EQ(kDefaultFingertipNames[static_cast<std::size_t>(Fingertip::kRing)], "ring");
}

// ── CMD encode (Proto §5.2) ─────────────────────────────────────────────────

TEST(HandCanFdCmd, SpecFrame0HexDump) {
  std::array<float, kWireMotorCount> pos{};
  pos[0] = 0.5f;
  pos[1] = -1.0f;
  pos[2] = 1.57f;
  const CmdMotorFrame frame = MakeCmdMotorFrame(pos);

  Bytes wire{};
  SerializeFrame(frame, wire.data());
  const uint8_t expect_head[12] = {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00,
                                   0x80, 0xBF, 0xC3, 0xF5, 0xC8, 0x3F};
  EXPECT_EQ(std::memcmp(wire.data(), expect_head, sizeof(expect_head)), 0);
  for (std::size_t i = 12; i < kFrameLen; ++i) {
    EXPECT_EQ(wire[i], 0u) << "byte " << i;
  }
}

TEST(HandCanFdCmd, SerializeDecodeRoundTrip) {
  std::array<float, kWireMotorCount> values{};
  for (std::size_t i = 0; i < kWireMotorCount; ++i) {
    values[i] = static_cast<float>(i) * -0.25f;
  }
  Bytes wire{};
  SerializeFrame(MakeCmdMotorFrame(values), wire.data());

  CmdMotorFrame back{};
  ASSERT_TRUE(DecodeFrame(wire.data(), wire.size(), back));
  for (std::size_t i = 0; i < kWireMotorCount; ++i) {
    EXPECT_FLOAT_EQ(back.values[i], values[i]);
  }
}

TEST(HandCanFdCmd, DecodeFrameRejectsWrongLength) {
  Bytes wire{};
  RespFrame0 f0{};
  EXPECT_FALSE(DecodeFrame(wire.data(), 63, f0));
  EXPECT_FALSE(DecodeFrame(wire.data(), 0, f0));
  EXPECT_TRUE(DecodeFrame(wire.data(), 64, f0));
}

// ── Tactile scaling (Proto §4.3) ────────────────────────────────────────────

TEST(HandCanFdTactile, FullScaleAndSigns) {
  TactileRaw raw{};
  raw.fx = 32767;
  raw.fy = 32767;
  raw.fz = 32767;
  raw.px = 32767;
  raw.py = 32767;
  raw.temp = -40;
  raw.contact = 7;
  const Tactile t = DecodeTactile(raw);
  EXPECT_FLOAT_EQ(t.fx_n, 25.0f);
  EXPECT_FLOAT_EQ(t.fy_n, -25.0f);
  EXPECT_FLOAT_EQ(t.fz_n, -100.0f);
  EXPECT_FLOAT_EQ(t.px_m, 0.32767f);
  EXPECT_FLOAT_EQ(t.py_m, -0.32767f);
  EXPECT_FLOAT_EQ(t.temp_c, -40.0f);
  EXPECT_TRUE(t.contact);
}

TEST(HandCanFdTactile, ZeroIsZeroAndNoContact) {
  const Tactile t = DecodeTactile(TactileRaw{});
  EXPECT_FLOAT_EQ(t.fx_n, 0.0f);
  EXPECT_FLOAT_EQ(t.fy_n, 0.0f);
  EXPECT_FLOAT_EQ(t.fz_n, 0.0f);
  EXPECT_FLOAT_EQ(t.px_m, 0.0f);
  EXPECT_FLOAT_EQ(t.py_m, 0.0f);
  EXPECT_FLOAT_EQ(t.temp_c, 0.0f);
  EXPECT_FALSE(t.contact);
}

TEST(HandCanFdTactile, SpecThumbExample) {
  // Proto §5.4: Fx=100 → +0.076 N, Fy=-100 → +0.076 N, Fz=1000 → -3.052 N
  RespFrame1 f1{};
  RespFrame2 f2{};
  ASSERT_TRUE(DecodeFrame(SpecRespF1().data(), kFrameLen, f1));
  ASSERT_TRUE(DecodeFrame(SpecRespF2().data(), kFrameLen, f2));
  const TactileRaw raw = ThumbRaw(f1, f2);
  EXPECT_EQ(raw.fx, 100);
  EXPECT_EQ(raw.fy, -100);
  EXPECT_EQ(raw.fz, 1000);
  EXPECT_EQ(raw.px, 288);
  EXPECT_EQ(raw.py, 16);
  EXPECT_EQ(raw.temp, 24);
  EXPECT_EQ(raw.contact, 1u);

  const Tactile t = DecodeTactile(raw);
  EXPECT_NEAR(t.fx_n, 0.076f, 1e-3f);
  EXPECT_NEAR(t.fy_n, 0.076f, 1e-3f);
  EXPECT_NEAR(t.fz_n, -3.052f, 1e-3f);
  EXPECT_NEAR(t.px_m, 2.88e-3f, 1e-6f);   // wire 2.88 mm
  EXPECT_NEAR(t.py_m, -0.16e-3f, 1e-6f);  // wire 0.16 mm, flipped
  EXPECT_FLOAT_EQ(t.temp_c, 24.0f);
  EXPECT_TRUE(t.contact);
}

// ── Telemetry decode (Proto §5.4) ───────────────────────────────────────────

TEST(HandCanFdTelemetry, SpecRespBurst) {
  RespSet set{};
  ASSERT_TRUE(DecodeFrame(SpecRespF0().data(), kFrameLen, set.frame0));
  ASSERT_TRUE(DecodeFrame(SpecRespF1().data(), kFrameLen, set.frame1));
  ASSERT_TRUE(DecodeFrame(SpecRespF2().data(), kFrameLen, set.frame2));

  Telemetry t{};
  DecodeTelemetry(set, t);
  EXPECT_FLOAT_EQ(t.pos[0], 0.5f);
  EXPECT_FLOAT_EQ(t.pos[1], -1.0f);
  EXPECT_FLOAT_EQ(t.pos[2], 1.57f);
  EXPECT_FLOAT_EQ(t.pos[9], 0.0f);
  EXPECT_FLOAT_EQ(t.cur[0], 1.2f);
  EXPECT_FLOAT_EQ(t.cur[1], 0.8f);
  EXPECT_FLOAT_EQ(t.cur[9], 0.0f);
  for (std::size_t i = 0; i < kWireMotorCount; ++i) {
    EXPECT_FLOAT_EQ(t.vel[i], 0.0f);
  }
  const auto& thumb = t.tactile[static_cast<std::size_t>(Fingertip::kThumb)];
  const auto& index = t.tactile[static_cast<std::size_t>(Fingertip::kIndex)];
  const auto& middle = t.tactile[static_cast<std::size_t>(Fingertip::kMiddle)];
  const auto& ring = t.tactile[static_cast<std::size_t>(Fingertip::kRing)];
  EXPECT_TRUE(thumb.contact);
  EXPECT_FLOAT_EQ(thumb.temp_c, 24.0f);
  EXPECT_FALSE(index.contact);
  EXPECT_FLOAT_EQ(index.temp_c, 23.0f);
  EXPECT_FLOAT_EQ(index.fx_n, 0.0f);
  EXPECT_FLOAT_EQ(middle.temp_c, 24.0f);
  EXPECT_FLOAT_EQ(ring.temp_c, 24.0f);
}

TEST(HandCanFdTelemetry, StitchesVelocityHalvesAndFingerSlots) {
  Bytes f0{}, f1{}, f2{};
  for (std::size_t i = 0; i < kVelLoCount; ++i) {
    PutF32(f0, 40 + 4 * i, static_cast<float>(i + 1));
  }
  for (std::size_t i = 0; i < kVelHiCount; ++i) {
    PutF32(f1, 4 * i, static_cast<float>(kVelLoCount + i + 1));
  }
  // Per-finger fx: index 1000, middle 2000, ring 3000 (fy flips sign — check it too)
  PutI16(f2, 5, 1000);
  PutI16(f2, 5 + 2, 1000);
  PutI16(f2, 18, 2000);
  PutI16(f2, 31, 3000);

  RespSet set{};
  ASSERT_TRUE(DecodeFrame(f0.data(), kFrameLen, set.frame0));
  ASSERT_TRUE(DecodeFrame(f1.data(), kFrameLen, set.frame1));
  ASSERT_TRUE(DecodeFrame(f2.data(), kFrameLen, set.frame2));
  Telemetry t{};
  DecodeTelemetry(set, t);
  for (std::size_t i = 0; i < kWireMotorCount; ++i) {
    EXPECT_FLOAT_EQ(t.vel[i], static_cast<float>(i + 1)) << "motor " << i;
  }
  const float unit = kForceXyRangeN / kTactileFullScale;
  EXPECT_FLOAT_EQ(t.tactile[1].fx_n, 1000.0f * unit);
  EXPECT_FLOAT_EQ(t.tactile[1].fy_n, -1000.0f * unit);
  EXPECT_FLOAT_EQ(t.tactile[2].fx_n, 2000.0f * unit);
  EXPECT_FLOAT_EQ(t.tactile[3].fx_n, 3000.0f * unit);
  EXPECT_FLOAT_EQ(t.tactile[0].fx_n, 0.0f);
}

// ── RESP set assembler ──────────────────────────────────────────────────────

TEST(HandCanFdAssembler, InOrderBurstCompletes) {
  RespSetAssembler a(kNodeRightHand);
  EXPECT_EQ(a.node(), kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  EXPECT_EQ(FeedRight(a, 0, Filled(0x10)), FeedResult::kStored);
  EXPECT_EQ(a.mask(), FrameBit(0));
  EXPECT_FALSE(a.complete());
  EXPECT_EQ(FeedRight(a, 1, Filled(0x11)), FeedResult::kStored);
  EXPECT_EQ(a.mask(), FrameBit(0) | FrameBit(1));
  EXPECT_EQ(FeedRight(a, 2, Filled(0x12)), FeedResult::kComplete);
  EXPECT_EQ(a.mask(), kRespSetMask);
  EXPECT_TRUE(a.complete());
  EXPECT_EQ(a.stats().sets_completed, 1u);
  EXPECT_EQ(a.stats().sets_abandoned, 0u);

  // Each payload landed in its own slot.
  Bytes slot{};
  SerializeFrame(a.set().frame0, slot.data());
  EXPECT_EQ(slot, Filled(0x10));
  SerializeFrame(a.set().frame1, slot.data());
  EXPECT_EQ(slot, Filled(0x11));
  SerializeFrame(a.set().frame2, slot.data());
  EXPECT_EQ(slot, Filled(0x12));
}

TEST(HandCanFdAssembler, EveryArrivalOrderCompletes) {
  // D57: the three bits decide, not the order — all 3! orders, each payload
  // in its own slot.
  std::array<uint8_t, kNumRespFrames> order{0, 1, 2};
  int orders = 0;
  do {
    RespSetAssembler a(kNodeRightHand);
    a.Begin(Clock::time_point{} + 1ms);
    for (std::size_t k = 0; k < order.size(); ++k) {
      const uint8_t idx = order[k];
      const FeedResult want = (k + 1 == order.size()) ? FeedResult::kComplete : FeedResult::kStored;
      EXPECT_EQ(FeedRight(a, idx, Filled(static_cast<uint8_t>(0x20 + idx))), want)
          << "order " << int{order[0]} << int{order[1]} << int{order[2]} << " step " << k;
    }
    ASSERT_TRUE(a.complete());
    EXPECT_EQ(a.stats().sets_completed, 1u);
    Bytes slot{};
    SerializeFrame(a.set().frame0, slot.data());
    EXPECT_EQ(slot, Filled(0x20));
    SerializeFrame(a.set().frame1, slot.data());
    EXPECT_EQ(slot, Filled(0x21));
    SerializeFrame(a.set().frame2, slot.data());
    EXPECT_EQ(slot, Filled(0x22));
    ++orders;
  } while (std::next_permutation(order.begin(), order.end()));
  EXPECT_EQ(orders, 6);
}

TEST(HandCanFdAssembler, FramesAfterCompleteChangeNothing) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  (void)FeedRight(a, 2, Filled(0x32));
  (void)FeedRight(a, 0, Filled(0x30));
  ASSERT_EQ(FeedRight(a, 1, Filled(0x31)), FeedResult::kComplete);
  for (uint8_t idx = 0; idx < kNumRespFrames; ++idx)
    EXPECT_EQ(FeedRight(a, idx, Filled(0xEE)), FeedResult::kDuplicate) << "idx " << int{idx};
  EXPECT_TRUE(a.complete());
  EXPECT_EQ(a.stats().sets_completed, 1u) << "no second completion";
  EXPECT_EQ(a.stats().duplicate, 3u);
  Bytes slot{};
  SerializeFrame(a.set().frame0, slot.data());
  EXPECT_EQ(slot, Filled(0x30));
  SerializeFrame(a.set().frame1, slot.data());
  EXPECT_EQ(slot, Filled(0x31));
  SerializeFrame(a.set().frame2, slot.data());
  EXPECT_EQ(slot, Filled(0x32));
}

TEST(HandCanFdAssembler, StaleTailBeforeIdx0CompletesAMixedSet) {
  // Late F1 / F2 of the previous burst arrive, then the real burst. With no
  // counter on the wire (D31) the set completes on the first F0 — the risk
  // D57 accepts; the real F1 / F2 arrive after completion and are dropped.
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  EXPECT_EQ(FeedRight(a, 1, Filled(0xA1)), FeedResult::kStored);
  EXPECT_EQ(FeedRight(a, 2, Filled(0xA2)), FeedResult::kStored);
  EXPECT_FALSE(a.complete());
  EXPECT_EQ(FeedRight(a, 0, Filled(0xB0)), FeedResult::kComplete);
  EXPECT_EQ(a.mask(), kRespSetMask);
  EXPECT_EQ(a.stats().sets_abandoned, 0u);
  EXPECT_EQ(FeedRight(a, 1, Filled(0xB1)), FeedResult::kDuplicate);
  EXPECT_EQ(FeedRight(a, 2, Filled(0xB2)), FeedResult::kDuplicate);

  Bytes slot{};
  SerializeFrame(a.set().frame1, slot.data());
  EXPECT_EQ(slot, Filled(0xA1)) << "the stale F1 (D57 accepted risk)";
  SerializeFrame(a.set().frame2, slot.data());
  EXPECT_EQ(slot, Filled(0xA2));
}

TEST(HandCanFdAssembler, Idx0AfterCompleteLeavesTheSetWithoutAbandon) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  (void)FeedRight(a, 0, Filled(1));
  (void)FeedRight(a, 1, Filled(2));
  ASSERT_EQ(FeedRight(a, 2, Filled(3)), FeedResult::kComplete);
  EXPECT_EQ(FeedRight(a, 0, Filled(4)), FeedResult::kDuplicate);
  EXPECT_TRUE(a.complete());
  EXPECT_EQ(a.mask(), kRespSetMask);
  EXPECT_EQ(a.stats().duplicate, 1u);
  EXPECT_EQ(a.stats().sets_abandoned, 0u) << "the finished set was not partial";
}

TEST(HandCanFdAssembler, RepeatedIdx0ReplacesWithoutAbandon) {
  // F0 twice, then F1 F2: the later F0 replaces the earlier one and the cycle
  // still completes — nothing was abandoned (the counter means a dropped
  // partial set, D43).
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  EXPECT_EQ(FeedRight(a, 0, Filled(0xC0)), FeedResult::kStored);
  EXPECT_EQ(FeedRight(a, 0, Filled(0xD0)), FeedResult::kDuplicate);
  EXPECT_EQ(a.mask(), FrameBit(0));
  EXPECT_EQ(a.stats().duplicate, 1u);
  EXPECT_EQ(a.stats().sets_abandoned, 0u);
  EXPECT_EQ(FeedRight(a, 1, Filled(0xD1)), FeedResult::kStored);
  EXPECT_EQ(FeedRight(a, 2, Filled(0xD2)), FeedResult::kComplete);
  EXPECT_EQ(a.stats().sets_completed, 1u);
  Bytes slot{};
  SerializeFrame(a.set().frame0, slot.data());
  EXPECT_EQ(slot, Filled(0xD0));
}

TEST(HandCanFdAssembler, ErrorFrameIsOutsideSetEvenWhenItsBitsLookLikeAnId) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  const Bytes b = Filled(0x00);
  // CAN_ERR_FLAG with error-class bits that alias RESP Frame 0 after masking.
  const uint32_t aliasing = kErrorFrameFlag | RespId(kNodeRightHand, 0);
  EXPECT_EQ(a.Feed(aliasing, b.data(), b.size()), FeedResult::kOutsideSet);
  EXPECT_EQ(a.Feed(kErrorFrameFlag | 0x40u, b.data(), 8), FeedResult::kOutsideSet)
      << "bus-off error frame, not an echo";
  EXPECT_EQ(a.stats().outside_set, 2u);
  EXPECT_EQ(a.mask(), 0u);
}

TEST(HandCanFdAssembler, DuplicateBeforeCompleteReplacesTheCopy) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  (void)FeedRight(a, 0, Filled(0));
  EXPECT_EQ(FeedRight(a, 1, Filled(0x55)), FeedResult::kStored);
  EXPECT_EQ(FeedRight(a, 1, Filled(0x66)), FeedResult::kDuplicate);
  EXPECT_EQ(a.stats().duplicate, 1u);
  EXPECT_EQ(a.mask(), FrameBit(0) | FrameBit(1));
  EXPECT_EQ(FeedRight(a, 2, Filled(0)), FeedResult::kComplete);
  Bytes slot{};
  SerializeFrame(a.set().frame1, slot.data());
  EXPECT_EQ(slot, Filled(0x66));
}

TEST(HandCanFdAssembler, FramesOutsideTheSetAreCountedNotStored) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  const Bytes b = Filled(0xEE);
  EXPECT_EQ(a.Feed(EchoId(kNodeRightHand, 0), b.data(), b.size()), FeedResult::kOutsideSet);
  EXPECT_EQ(a.Feed(EchoId(kNodeRightHand, 1), b.data(), b.size()), FeedResult::kOutsideSet);
  EXPECT_EQ(a.Feed(RespId(kNodeLeftHand, 0), b.data(), b.size()), FeedResult::kOutsideSet);
  EXPECT_EQ(a.Feed(CmdId(kNodeRightHand, 0), b.data(), b.size()), FeedResult::kOutsideSet);
  EXPECT_EQ(a.Feed(ReqId(kNodeRightHand), b.data(), 0), FeedResult::kOutsideSet);
  EXPECT_EQ(a.Feed(0x00000001u, b.data(), b.size()), FeedResult::kOutsideSet)
      << "an error-class id is not a RESP";
  EXPECT_EQ(a.stats().outside_set, 6u);
  EXPECT_EQ(a.mask(), 0u);
}

TEST(HandCanFdAssembler, SocketFlagBitsAreIgnored) {
  // CAN_EFF_FLAG (bit 31) as the socket delivers it on a 29-bit id.
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  const Bytes b = Filled(0x01);
  EXPECT_EQ(a.Feed(0x80000000u | RespId(kNodeRightHand, 0), b.data(), b.size()),
            FeedResult::kStored);
  EXPECT_EQ(a.mask(), FrameBit(0));
}

TEST(HandCanFdAssembler, BadIndexAndBadLengthAreDropped) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  const Bytes b = Filled(0x01);
  EXPECT_EQ(a.Feed(RespId(kNodeRightHand, 3), b.data(), b.size()), FeedResult::kBadIndex);
  EXPECT_EQ(a.Feed(RespId(kNodeRightHand, 63), b.data(), b.size()), FeedResult::kBadIndex);
  EXPECT_EQ(a.stats().bad_index, 2u);
  EXPECT_EQ(FeedRight(a, 0, b), FeedResult::kStored);
  EXPECT_EQ(a.Feed(RespId(kNodeRightHand, 1), b.data(), 8), FeedResult::kBadLength);
  EXPECT_EQ(a.Feed(RespId(kNodeRightHand, 1), b.data(), 63), FeedResult::kBadLength);
  EXPECT_EQ(a.Feed(RespId(kNodeRightHand, 1), b.data(), 0), FeedResult::kBadLength);
  EXPECT_EQ(a.stats().bad_length, 3u);
  EXPECT_EQ(a.mask(), FrameBit(0)) << "a bad frame stores nothing";
  // A short idx 0 must not touch a pending set either.
  EXPECT_EQ(a.Feed(RespId(kNodeRightHand, 0), b.data(), 8), FeedResult::kBadLength);
  EXPECT_EQ(a.mask(), FrameBit(0));
  EXPECT_EQ(a.stats().duplicate, 0u);
}

TEST(HandCanFdAssembler, LeftHandInstanceIgnoresRightHand) {
  RespSetAssembler a(kNodeLeftHand);
  EXPECT_EQ(a.node(), kNodeLeftHand);
  a.Begin(Clock::time_point{} + 1ms);
  const Bytes b = Filled(0x02);
  EXPECT_EQ(a.Feed(RespId(kNodeRightHand, 0), b.data(), b.size()), FeedResult::kOutsideSet);
  EXPECT_EQ(a.Feed(RespId(kNodeLeftHand, 0), b.data(), b.size()), FeedResult::kStored);
  EXPECT_EQ(a.Feed(RespId(kNodeLeftHand, 1), b.data(), b.size()), FeedResult::kStored);
  EXPECT_EQ(a.Feed(RespId(kNodeLeftHand, 2), b.data(), b.size()), FeedResult::kComplete);
}

TEST(HandCanFdAssembler, DeadlineIsCallerClock) {
  RespSetAssembler a(kNodeRightHand);
  const Clock::time_point t0{};
  a.Begin(t0 + 400us);
  EXPECT_EQ(a.deadline(), t0 + 400us);
  EXPECT_FALSE(a.Expired(t0));
  EXPECT_FALSE(a.Expired(t0 + 399us));
  EXPECT_TRUE(a.Expired(t0 + 400us));
  EXPECT_TRUE(a.Expired(t0 + 1s));
  EXPECT_EQ(a.Remaining(t0 + 100us), 300us);
  EXPECT_EQ(a.Remaining(t0 + 400us), Clock::duration::zero());
  EXPECT_EQ(a.Remaining(t0 + 1s), Clock::duration::zero());
}

TEST(HandCanFdAssembler, ResetAndBeginAbandonOnlyPartialSets) {
  RespSetAssembler a(kNodeRightHand);
  const Clock::time_point t0{};
  a.Begin(t0 + 1ms);
  a.Reset();  // nothing pending
  EXPECT_EQ(a.stats().sets_abandoned, 0u);

  (void)FeedRight(a, 0, Filled(1));
  (void)FeedRight(a, 1, Filled(2));
  a.Reset();  // 2 of 3 → abandoned (D43)
  EXPECT_EQ(a.mask(), 0u);
  EXPECT_EQ(a.stats().sets_abandoned, 1u);
  EXPECT_EQ(a.deadline(), t0 + 1ms) << "Reset keeps the deadline";

  (void)FeedRight(a, 2, Filled(3));
  a.Begin(t0 + 2ms);  // 1 of 3 → abandoned, deadline moves
  EXPECT_EQ(a.mask(), 0u);
  EXPECT_EQ(a.stats().sets_abandoned, 2u);
  EXPECT_EQ(a.deadline(), t0 + 2ms);

  (void)FeedRight(a, 0, Filled(1));
  (void)FeedRight(a, 1, Filled(2));
  (void)FeedRight(a, 2, Filled(3));
  ASSERT_TRUE(a.complete());
  a.Begin(t0 + 3ms);  // complete set is not "abandoned"
  EXPECT_EQ(a.stats().sets_abandoned, 2u);
  EXPECT_EQ(a.stats().sets_completed, 1u);
  EXPECT_FALSE(a.complete());
}

TEST(HandCanFdAssembler, ResetStatsClearsCounters) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  const Bytes b = Filled(0);
  (void)a.Feed(EchoId(kNodeRightHand, 0), b.data(), b.size());
  (void)FeedRight(a, 1, b);
  (void)FeedRight(a, 1, b);
  a.Reset();
  EXPECT_EQ(a.stats().outside_set, 1u);
  EXPECT_EQ(a.stats().duplicate, 1u);
  EXPECT_EQ(a.stats().sets_abandoned, 1u);
  a.ResetStats();
  EXPECT_EQ(a.stats().outside_set, 0u);
  EXPECT_EQ(a.stats().duplicate, 0u);
  EXPECT_EQ(a.stats().sets_abandoned, 0u);
  EXPECT_EQ(a.stats().sets_completed, 0u);
}

TEST(HandCanFdAssembler, CompleteSetDecodesToSpecTelemetry) {
  RespSetAssembler a(kNodeRightHand);
  a.Begin(Clock::time_point{} + 1ms);
  ASSERT_EQ(FeedRight(a, 0, SpecRespF0()), FeedResult::kStored);
  ASSERT_EQ(FeedRight(a, 1, SpecRespF1()), FeedResult::kStored);
  ASSERT_EQ(FeedRight(a, 2, SpecRespF2()), FeedResult::kComplete);
  Telemetry t{};
  DecodeTelemetry(a.set(), t);
  EXPECT_FLOAT_EQ(t.pos[2], 1.57f);
  EXPECT_FLOAT_EQ(t.cur[1], 0.8f);
  EXPECT_NEAR(t.tactile[0].fz_n, -3.052f, 1e-3f);
  EXPECT_TRUE(t.tactile[0].contact);
}

}  // namespace udp_hand_driver::canfd::test
